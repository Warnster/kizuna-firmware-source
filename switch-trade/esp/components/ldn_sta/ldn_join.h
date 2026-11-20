/* SPDX-License-Identifier: AGPL-3.0-only
 * Derived from frlg-ldn-trade-esp32 (https://github.com/easyworld/frlg-ldn-trade-esp32), Copyright (c) Easy World and the frlg-ldn-trade-esp32 authors.
 * Derived from LDN (https://github.com/kinnay/LDN), Copyright (c) kinnay and the LDN authors.
 * Changes Copyright (c) 2026 Kizuna. See NOTICE.
 */
/* ldn_join — join a Nintendo Switch's FRLG LDN group on the ESP32-C6 (v5.3, single-chip, no PC).
 *
 * Ported from the easyworld esp32-c6 join glue (frlg-ldn-trade-esp32) but targeting the REAL v5.3
 * wpa_supplicant ABI (we #include esp_wifi_driver.h directly — no hand-mirrored struct, no v6.1 pin,
 * no libpp/raw-TX binary surgery). The nasty raw-TX diagnostic path is dropped; the normal trade flow
 * uses hardware CCMP via esp_wifi_set_sta_key_internal, which exists on v5.3.
 *
 * Join sequence (all inputs from the LIVE decrypted advert + fixed FRLG constants, the
 * auth-handshake research, and cport/ldn_auth.c):
 *   decrypt advert -> derive CCMP key (server_random||FRLG_PASSWORD) -> associate (open 802.11 mgmt)
 *   -> on assoc-resp: install CCMP key + esp_wifi_auth_done_internal -> send AuthenticationRequest
 *   (CCMP 0x88b7) -> verify AuthenticationResponse -> our MAC appears in the re-broadcast advert = MEMBER.
 *
 * GBA-INDEPENDENT: none of this touches the PARLIO GBA link, so it cannot regress the login gate. */
#ifndef LDN_JOIN_H
#define LDN_JOIN_H
#include <stdint.h>
#include <stdbool.h>
#include "esp_netif.h"
#include "ldn_advert.h"   /* ldn_keys */

/* Target LDN group, filled from a live decrypted advert (trade_scan). */
typedef struct {
    uint8_t  bssid[6];              /* host (Switch) MAC = advert frame src */
    uint8_t  ssid[16];             /* 16-byte LDN ssid (from NetworkId)     */
    uint8_t  channel;              /* 1/6/11                                */
    uint64_t comm_id;             /* local_comm_id (FRLG = 0x01006fa0233f8000) */
    uint16_t scene_id;
    uint8_t  server_random[16];    /* decrypted advert plaintext[0:16]      */
    uint64_t challenge_token;      /* decrypted advert plaintext[16:24] (BE) */
    uint16_t app_version;          /* decrypted advert plaintext[26:28] (BE) */
} ldn_join_params_t;

/* Key material (provisioned at runtime by the app; switch-trade keeps a write-only copy in NVS). ldn_join_run() refuses to start without it.
 * The pointer must stay valid for as long as a join can run. */
void ldn_join_set_keys(const ldn_keys *k);
bool ldn_join_have_keys(void);

/* One-time init: install the WPA callback hook + register the Wi-Fi/data RX callbacks. Call once
 * AFTER esp_wifi_start(). Benign on its own (passthrough) — does not associate. */
void ldn_join_init(void);

/* Derive the CCMP key from p, associate to the group, install the key, and send the auth request.
 * Returns true if the association + key install started OK (membership is async — poll below). */
bool ldn_join_run(const ldn_join_params_t *p);

/* Drive the async join steps (install keys on assoc-resp, then send the auth request). Call from a
 * task loop after ldn_join_run(). */
void ldn_join_poll(void);

/* State the join reached (for logging / the relay gate). */
bool ldn_join_associated(void);   /* 802.11 assoc-resp seen + CCMP key installed + port authorized */
bool ldn_join_member(void);       /* our MAC seen in the Switch's re-broadcast advert (has an IP)   */
bool ldn_join_auth_responded(void); /* the host answered our AuthenticationRequest with a 0x88b7 frame */
bool ldn_join_auth_exhausted(void); /* the 12-attempt auth loop gave up without becoming a member       */
uint32_t ldn_join_ip_frames(void);  /* count of host IP (0x0800) frames forwarded to lwip (RX-path diag) */
bool ldn_join_host_ip(uint8_t out[4]); /* host IP learned from the first inbound Pia frame (true if known) */
void ldn_join_set_ip_forward(bool en); /* enable forwarding Pia frames into lwIP — only once the brain runs
                                        * (before that the Switch's broadcast flood would WDT the Wi-Fi task) */

/* Tear the join back down (disconnect + clear all state) so trade_scan can re-scan + re-latch the
 * CURRENT room. Use when the latched room is stale/gone (auth exhausted, or association never seen). */
void ldn_join_reset(void);

/* Membership plumbing (set by trade_scan once our IP is parsed from the re-broadcast advert). */
void ldn_join_set_member(bool member);
void ldn_join_station_mac(uint8_t out[6]);   /* our real STA MAC (valid after ldn_join_run) */

/* Hand the join the static-IP STA netif so its RX callback can forward the Switch's Pia IP frames
 * into lwip (the UDP socket in ldn_udp.c). Call once, after the netif is created. */
/* true: behave as an ordinary WPA2 station (4-way handshake, stock RSN IE) -- for the host role's uplink. */
void ldn_join_set_passthrough(bool on);
void ldn_join_clear_disc_reasons(void); /* call at session start so the next session's end reasons are its own */
int  ldn_join_last_disc_reason(void);   /* last WIFI_EVENT_STA_DISCONNECTED reason (-1 none) */
int  ldn_join_host_disc_reason(void);   /* last LDN Disconnect (0x0103) reason from the host, 3 = network destroyed (-1 none) */
void ldn_join_set_netif(esp_netif_t *netif);

/* Feed each promiscuous mgmt frame here so the join can watch for the assoc-response and the
 * re-broadcast advert listing our MAC (trade_scan already runs promiscuous; it forwards here). */
void ldn_join_sniff(const uint8_t *frame, uint32_t len, uint8_t channel);

#endif
