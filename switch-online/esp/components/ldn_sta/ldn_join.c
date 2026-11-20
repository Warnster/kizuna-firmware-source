/* SPDX-License-Identifier: AGPL-3.0-only
 * Derived from frlg-ldn-trade-esp32 (https://github.com/easyworld/frlg-ldn-trade-esp32), Copyright (c) Easy World and the frlg-ldn-trade-esp32 authors.
 * Derived from LDN (https://github.com/kinnay/LDN), Copyright (c) kinnay and the LDN authors.
 * Changes Copyright (c) 2026 Kizuna. See NOTICE.
 */
/* ldn_join — see ldn_join.h. v5.3 LDN group join for the ESP32-C6 (single-chip, no PC).
 * Ported from easyworld's esp32-c6 join glue to the real v5.3 wpa_supplicant ABI. Raw-TX path dropped;
 * uses hardware CCMP (esp_wifi_set_sta_key_internal). GBA-independent -> cannot touch the login gate. */
#include "ldn_join.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_coexist.h"
#include "esp_mac.h"
#include "esp_random.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_netif.h"            /* esp_netif_receive — hand IP (0x0800) frames to lwip so UDP works */
#include "esp_private/wifi.h"      /* esp_wifi_internal_tx / _reg_rxcb / _free_rx_buffer */
/* esp_wifi_driver.h is a private wpa_supplicant header meant to be included after wpa's common.h,
 * so it uses wpa's short integer typedefs. Provide them so we can include the REAL header (exact ABI). */
typedef uint8_t  u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;
#include "esp_wifi_driver.h"      /* REAL v5.3 wpa_supplicant ABI: struct wpa_funcs, enums, *_internal */
#include "ldn_advert.h"
#include "ldn_auth.h"

static const char *TAG = "ldn_join";

/* Key material arrives at runtime (app/bench -> RAM) and is never compiled in. */
static const ldn_keys *s_keys;
void ldn_join_set_keys(const ldn_keys *k) { s_keys = k; }
bool ldn_join_have_keys(void) { return s_keys != NULL; }

/* The supplicant's global callback table (defined in libwpa_supplicant; declared extern where used). */
extern struct wpa_funcs *wpa_cb;

/* 64-byte FRLG network password appended to server_random for the CCMP data-key derivation
 * (host/core/Ldn.cs:29-30, verified from source). */
static const uint8_t FRLG_PASSWORD[64] = {
    0xfc,0xb6,0xf6,0xad,0xb9,0xdf,0xea,0x66,0xac,0xa9,0xc3,0x26,0x14,0x9d,0x2b,0x3b,
    0x08,0xa7,0x81,0x89,0x5c,0xbf,0x78,0xf7,0x20,0xd7,0x8b,0x85,0xa5,0x75,0x84,0xa9,
    0x96,0x65,0xd2,0x37,0x79,0x7b,0x2a,0x41,0xdd,0xef,0x14,0x06,0x3e,0xc2,0x8d,0x25,
    0x91,0x43,0xaf,0x78,0x32,0xfb,0x3c,0xbc,0xf2,0x75,0x9c,0xbf,0xbd,0xc8,0x1d,0x8c,
};

/* Exact RSN element kinnay/LDN emits: CCMP pairwise+group, PSK AKM, caps 0x000c. */
static uint8_t s_ldn_rsn_ie[] = {
    0x30, 0x14, 0x01, 0x00,
    0x00, 0x0f, 0xac, 0x04,
    0x01, 0x00, 0x00, 0x0f, 0xac, 0x04,
    0x01, 0x00, 0x00, 0x0f, 0xac, 0x02,
    0x0c, 0x00,
};

static ldn_join_params_t s_params;
static uint8_t  s_target_bssid[6];
static uint8_t  s_station_mac[6];
static uint8_t  s_ccmp_key[16];
static uint16_t s_app_version;
static volatile bool s_joining;
static volatile int s_last_sta_disc = -1, s_host_disc_reason = -1;
static volatile bool s_association_seen;
static volatile bool s_authorized;
static volatile bool s_member;
static volatile bool s_auth_response;   /* set by data_rx when a 0x88b7 frame from the host arrives */
static volatile uint32_t s_ip_frames_seen;   /* count of host IP (0x0800) frames forwarded to lwip */
static int (*s_original_sta_connect)(uint8_t *bssid);
static int (*s_original_rx_eapol)(uint8_t *src, uint8_t *buf, uint32_t len);
static void (*s_original_connected_cb)(uint8_t *bssid);
static bool (*s_original_in_4way)(void);
static volatile bool s_passthrough;     /* host-role uplink: ordinary WPA2 STA, the LDN join overrides stand down */
static int64_t s_connect_us;
static esp_netif_t *s_ip_netif;         /* static-IP STA netif; IP frames are handed here for lwip */

/* ldn_join_poll progress statics — file-scope so ldn_join_reset() can clear them for a fresh re-join. */
static bool s_keyed, s_promisc_back;
static int  s_auth_tries;
static int64_t s_last_auth_us;

/* Host IP learned from the first inbound Pia (0x0800) frame's source — lets trade_scan derive our own
 * IP (host+1) and start the brain WITHOUT waiting for the advert participant list (which the Switch does
 * not always publish on a fresh room, deadlocking the old membership-list-gated brain start). */
static uint8_t s_host_ip[4];
static bool    s_have_host_ip;
static volatile bool s_ip_forward;   /* gate lwIP forwarding until the brain is running (anti-flood WDT) */

bool ldn_join_associated(void)     { return s_authorized; }
bool ldn_join_member(void)         { return s_member; }
bool ldn_join_auth_responded(void) { return s_auth_response; }
bool ldn_join_auth_exhausted(void) { return s_auth_tries >= 12; }
uint32_t ldn_join_ip_frames(void) { return s_ip_frames_seen; }
bool ldn_join_host_ip(uint8_t out[4]) { if (!s_have_host_ip) return false; memcpy(out, s_host_ip, 4); return true; }
void ldn_join_set_ip_forward(bool en) { s_ip_forward = en; }
void ldn_join_set_member(bool m)
{
    s_member = m;
    if (m) esp_coex_preference_set(ESP_COEX_PREFER_BALANCE);   /* seated: the association handshake is over */
}

/* Tear the join back down to the pre-run state so trade_scan can re-scan + re-latch the CURRENT room.
 * Needed because the Switch reuses the LDN ssid across room recreations while rotating the per-session
 * challenge (a stale challenge => the Switch's short non-seating auth reply), and a torn-down room makes
 * esp_wifi_connect fail "not suitable AP" forever. GBA-INDEPENDENT — does not touch PARLIO/login. */
void ldn_join_reset(void)
{
    esp_wifi_disconnect();
    s_joining = false; s_association_seen = false; s_authorized = false;
    s_member = false; s_auth_response = false;
    s_keyed = false; s_promisc_back = false;
    s_auth_tries = 0; s_last_auth_us = 0; s_connect_us = 0;
    s_have_host_ip = false;
    s_ip_forward = false;
    printf("  [join] RESET — re-scanning for the current room (stale/gone room recovery)\n");
}
void ldn_join_set_netif(esp_netif_t *n) { s_ip_netif = n; }
void ldn_join_station_mac(uint8_t out[6]) { memcpy(out, s_station_mac, 6); }

/* ---- WPA callback overrides (mirror easyworld; real v5.3 struct so field offsets are exact) ---- */
static int probe_sta_connect(uint8_t *bssid)
{
    if (s_passthrough) return s_original_sta_connect ? s_original_sta_connect(bssid) : 0;
    /* Install the LDN RSN IE both before and after the stock connect (it rebuilds the IE). */
    int a = esp_wifi_set_appie_internal(WIFI_APPIE_RSN, s_ldn_rsn_ie, sizeof(s_ldn_rsn_ie), 0);
    int r = s_original_sta_connect ? s_original_sta_connect(bssid) : 0;
    esp_wifi_set_appie_internal(WIFI_APPIE_RSN, s_ldn_rsn_ie, sizeof(s_ldn_rsn_ie), 0);
    printf("  [join] probe_sta_connect: appie=%d orig_connect=%d (hook active)\n", a, r);
    return r;
}
static void probe_sta_connected(uint8_t *bssid)
{
    if (s_passthrough && s_original_connected_cb) s_original_connected_cb(bssid);
}
static int  probe_rx_eapol(uint8_t *src, uint8_t *buf, uint32_t len)
{
    if (s_passthrough && s_original_rx_eapol) return s_original_rx_eapol(src, buf, len);
    return 0;
}
static bool probe_in_4way_handshake(void)
{
    return s_passthrough && s_original_in_4way ? s_original_in_4way() : false;
}
void ldn_join_set_passthrough(bool on) { s_passthrough = on; }

static void install_wpa_hook(void)
{
    if (wpa_cb == NULL) { ESP_LOGE(TAG, "wpa_cb NULL — call after esp_wifi_start()"); return; }
    struct wpa_funcs *probe = malloc(sizeof(*probe));
    if (!probe) { ESP_LOGE(TAG, "no mem for wpa hook"); return; }
    memcpy(probe, wpa_cb, sizeof(*probe));
    s_original_sta_connect          = probe->wpa_sta_connect;
    s_original_rx_eapol             = probe->wpa_sta_rx_eapol;
    s_original_connected_cb         = probe->wpa_sta_connected_cb;
    s_original_in_4way              = probe->wpa_sta_in_4way_handshake;
    probe->wpa_sta_connect          = probe_sta_connect;
    probe->wpa_sta_connected_cb     = probe_sta_connected;
    probe->wpa_sta_rx_eapol         = probe_rx_eapol;
    probe->wpa_sta_in_4way_handshake= probe_in_4way_handshake;
    if (esp_wifi_register_wpa_cb_internal(probe) != 0) { free(probe); printf("  [join] wpa cb register FAILED\n"); return; }
    wpa_cb = probe;   /* register frees the old table; keep the supplicant owner in sync */
    printf("  [join] v5.3 WPA hook installed\n");
}

static esp_err_t data_rx(void *buf, uint16_t len, void *eb);   /* fwd decl (re-registered in install_ldn_keys) */

/* ---- CCMP key install (after assoc-resp) + port authorize ---- */
static bool read_key_back(int idx, enum key_flag flag, const uint8_t *expect)
{
    uint8_t ifx = WIFI_IF_STA; int alg = WIFI_WPA_ALG_NONE; int index = idx;
    uint8_t addr[6]; uint8_t key[16];
    memcpy(addr, s_target_bssid, 6); memset(key, 0, 16);
    int r = esp_wifi_get_sta_key_internal(&ifx, &alg, addr, &index, key, sizeof(key), flag);
    return r == 0 && alg == WIFI_WPA_ALG_CCMP && index == idx && memcmp(key, expect, 16) == 0;
}

static void install_ldn_keys(void)
{
    uint8_t seq[8] = {0};
    /* CRITICAL ORDER (from the S3's working install_ldn_keys, ldn_probe.c): call auth_done_internal FIRST,
     * THEN inject the CCMP keys. The C6 Wi-Fi driver only finalizes the station context / CAM key slots once
     * the handshake is reported done — injecting keys BEFORE authorizing leaves the GROUP key in a slot that
     * cannot decrypt the host's (broadcast) Pia frames, so RX stays EMPTY (our data_rx=0 symptom). */
    s_authorized = esp_wifi_auth_done_internal();
    printf("  [join] auth_done=%s (BEFORE key install — S3 order)\n", s_authorized ? "true" : "false");
    int pw = esp_wifi_set_sta_key_internal(WIFI_WPA_ALG_CCMP, s_target_bssid, 0, 1, seq, sizeof(seq),
                                           s_ccmp_key, sizeof(s_ccmp_key),
                                           KEY_FLAG_PAIRWISE | KEY_FLAG_RX | KEY_FLAG_TX);
    int gr = esp_wifi_set_sta_key_internal(WIFI_WPA_ALG_CCMP, s_target_bssid, 1, 0, seq, sizeof(seq),
                                           s_ccmp_key, sizeof(s_ccmp_key),
                                           KEY_FLAG_GROUP | KEY_FLAG_RX);
    bool group_ok = read_key_back(1, KEY_FLAG_GROUP, s_ccmp_key);
    printf("  [join] CCMP install pairwise=%d group=%d group_readback=%s\n", pw, gr, group_ok ? "ok" : "no");
    /* Re-register the 0x88b7 data RX callback AFTER key install — esp_wifi_connect/assoc can reset the rxcb. */
    esp_wifi_internal_reg_rxcb(WIFI_IF_STA, data_rx);
}

/* ---- Send the AuthenticationRequest as an Ethernet-II 0x88b7 CCMP data frame ---- */
static bool send_auth_request(void)
{
    uint8_t client_random[16];
    esp_fill_random(client_random, sizeof(client_random));
    uint64_t nonce = ((uint64_t)esp_random() << 32) | esp_random();

    /* STATIC (not stack): the auth frame is ~2KB and this runs in the 4KB "scan" task — a stack copy
     * overflowed it (Stack protection fault right after auth_done). Single-threaded join, so static is safe. */
    static uint8_t body[1024];
    int n = ldn_build_auth_frame(s_keys, 3 /*protocol*/, 4 /*version*/,
                                 s_params.comm_id, s_params.scene_id, s_params.ssid,
                                 s_params.server_random, client_random,
                                 s_params.challenge_token, nonce,
                                 "Kizuna", s_app_version, body, sizeof(body));
    if (n < 0) { printf("  [join] auth frame build FAILED (%d)\n", n); return false; }

    static uint8_t frame[14 + 1024];
    memcpy(frame + 0, s_target_bssid, 6);   /* dst = host */
    memcpy(frame + 6, s_station_mac, 6);     /* src = us   */
    frame[12] = 0x88; frame[13] = 0xb7;      /* EtherType 0x88b7 (LDN) */
    memcpy(frame + 14, body, n);
    esp_err_t e = esp_wifi_internal_tx(WIFI_IF_STA, frame, (uint16_t)(14 + n));
    printf("  [join] AUTH REQUEST tx (%d B) = %s\n", 14 + n, esp_err_to_name(e));
    return e == ESP_OK;
}

/* ---- 0x88b7 data RX: watch for the AuthenticationResponse from the host ---- */
static esp_err_t data_rx(void *buf, uint16_t len, void *eb)
{
    const uint8_t *f = buf;
    /* DIAGNOSTIC: log EVERY frame reaching this cb (first ~8) so we know the RX path works at all after
     * association — ethertype + src, to distinguish "Switch not responding" from "response not delivered". */
    static int rx_seen = 0;
    if (rx_seen < 8 && len >= 14) {
        rx_seen++;
        printf("  [join] data_rx #%d len=%u ethertype=%02x%02x src=%02x:%02x:%02x:%02x:%02x:%02x\n",
               rx_seen, len, f[12], f[13], f[6], f[7], f[8], f[9], f[10], f[11]);
    }
    if (len >= 14 && f[12] == 0x88 && f[13] == 0xb7 && memcmp(f + 6, s_target_bssid, 6) == 0) {
        /* tr-1.18: LDN payload starts at f[14] and is  u24 OUI(0x0022AA) | u16be frame_type | pad(1)  then a
         * TYPE-SPECIFIC body. The old code hardcoded the AuthenticationFrame layout and applied it to EVERY
         * 0x88b7 frame, so Disconnect frames printed garbage ("status=0 is_resp=0" was really just padding) --
         * which is exactly what disguised the mid-trade teardown as an unanswered request all night.
         * Frame types (pokeldn vendor/LDN/ldn/__init__.py:449/792/872 — the COMPLETE type space; there is no
         * keepalive/heartbeat type in LDN at all):
         *   0x0101 Advertisement | 0x0102 Authentication | 0x0103 Disconnect
         * Authentication: 72B header at f[20] -> status @f[22], is_response @f[23].
         * Disconnect: u8 reason @f[20] then 31 pad bytes (38B payload -> 52B frame). reason 3 =
         * NETWORK_DESTROYED (host tore the network down; NOT a station kick, and NO reply is expected or even
         * possible -- the reference STA path raises an event and sends nothing back). */
        uint16_t ftype = (len >= 19) ? (uint16_t)((f[17] << 8) | f[18]) : 0;
        printf("  [join] 0x88b7 RX from host, %u B type=0x%04x — hdr:", len, ftype);
        for (int i = 14; i < 14 + 12 && i < len; i++) printf(" %02x", f[i]);
        if (ftype == 0x0103) {
            if (len >= 21) s_host_disc_reason = f[20];
            if (len >= 21) printf("  DISCONNECT reason=%u%s", f[20],
                                  f[20] == 3 ? " (NETWORK_DESTROYED by host)" : "");
        } else if (ftype == 0x0102) {
            s_auth_response = true;   /* only a real Authentication frame may satisfy the join wait */
            if (len >= 24) printf("  status=%u is_resp=%u", f[22], f[23]);
        }
        printf("\n");
        if (eb) esp_wifi_internal_free_rx_buffer(eb);
        return ESP_OK;
    }
    /* Everything else (the Switch's Pia IP frames, ethertype 0x0800, and ARP) must reach lwip so the
     * UDP socket in ldn_udp.c can receive them. We own the STA rxcb (registered over esp_netif's), so
     * mirror the S3's control_rx: forward non-auth frames to the netif. esp_netif takes ownership of eb. */
    if (len >= 34 && f[12] == 0x08 && f[13] == 0x00) {   /* IP frame from the host -> count + fwd */
        s_ip_frames_seen++;
        if (!s_have_host_ip) {                            /* learn the host IP from the first Pia frame's src */
            memcpy(s_host_ip, f + 26, 4);
            s_have_host_ip = true;
        }
        if (s_ip_frames_seen <= 6)
            printf("  [join] IP frame #%u len=%u dst=%u.%u.%u.%u src=%u.%u.%u.%u proto=%u\n",
                   (unsigned)s_ip_frames_seen, len, f[30],f[31],f[32],f[33], f[26],f[27],f[28],f[29], f[23]);
    }
    /* Only hand frames to lwIP once the trade brain is running (post room-selection). Before that we're
     * beacon-only: the Switch floods Pia broadcasts we can't yet use, and forwarding them all into lwIP with
     * no reader pegs the Wi-Fi/TCP tasks -> IDLE starves -> task-WDT reset (~24s). We still captured the host
     * IP + counted above, which is all STAGE A needs. Drop (free) the flood until forwarding is enabled. */
    if (s_ip_forward && s_ip_netif) return esp_netif_receive(s_ip_netif, buf, len, eb);
    if (eb) esp_wifi_internal_free_rx_buffer(eb);
    return ESP_OK;
}

/* ---- assoc-response watcher (fed from promiscuous mgmt frames) ---- */
void ldn_join_sniff(const uint8_t *frame, uint32_t len, uint8_t channel)
{
    (void)channel;
    if (!s_joining || len < 34) return;
    uint8_t subtype = frame[0] & 0xfcU;
    if ((subtype == 0x10 || subtype == 0x30) &&                 /* assoc-resp / reassoc-resp */
        memcmp(frame + 4, s_station_mac, 6) == 0 &&
        memcmp(frame + 10, s_target_bssid, 6) == 0 &&
        memcmp(frame + 16, s_target_bssid, 6) == 0 &&
        frame[26] == 0 && frame[27] == 0) {                     /* status = success */
        if (!s_association_seen) { s_association_seen = true; ESP_LOGI(TAG, "assoc-response seen"); }
    }
}

/* Wi-Fi events: STA_CONNECTED is a robust assoc signal (in addition to the mgmt sniff); DISCONNECTED
 * carries the reason code — the key diagnostic for why an LDN association is rejected. */
void ldn_join_clear_disc_reasons(void) { s_last_sta_disc = s_host_disc_reason = -1; }
int ldn_join_last_disc_reason(void) { return s_last_sta_disc; }
int ldn_join_host_disc_reason(void) { return s_host_disc_reason; }

static void wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg; (void)base;
    if (id == WIFI_EVENT_STA_CONNECTED) {
        if (!s_association_seen) { s_association_seen = true; printf("  [join] STA_CONNECTED (assoc ok)\n"); }
    } else if (id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t *d = data;
        s_last_sta_disc = d ? d->reason : -1;
        printf("  [join] STA_DISCONNECTED reason=%d\n", d ? d->reason : -1);
        /* tr-1.31: the association is GONE — say so. Previously these flags were left stale-true, so after
         * the Switch tore its network down (post-death teardown, or room recreate) the join-hb kept printing
         * assoc=1 member=1 forever, scan_task's self-heal (gated on !member) could never fire, and the only
         * recovery was a reboot/reflash (hw:
         * STA_DISCONNECTED reason=5 followed by 20+ lying join-hb lines). Clearing here makes the state
         * truthful; the scan_task session re-arm (tr-1.31) does the orderly rescan/rejoin. */
        s_association_seen = false; s_authorized = false; s_member = false; s_keyed = false;
    }
}

void ldn_join_init(void)
{
    esp_log_level_set(TAG, ESP_LOG_INFO);   /* default level is WARN; we want the join diagnostics */
    /* No modem power-save: default WIFI_PS_MIN_MODEM makes the STA doze through beacons during the
     * connect scan -> NO_AP_FOUND even though the AP beacons 10x/s. easyworld sets this too. */
    esp_wifi_set_ps(WIFI_PS_NONE);
    install_wpa_hook();
    esp_wifi_internal_reg_rxcb(WIFI_IF_STA, data_rx);
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event, NULL);
    printf("  [join] init: wpa hook + data rx + event handler\n");
}

bool ldn_join_run(const ldn_join_params_t *p)
{
    if (!s_keys) { ESP_LOGE(TAG, "no LDN keys provisioned - refusing to join"); return false; }
    s_params = *p;
    s_app_version = p->app_version;
    memcpy(s_target_bssid, p->bssid, 6);

    /* CCMP session key from the advert alone: derive(server_random || FRLG_PASSWORD). */
    if (ldn_derive_data_key(s_keys, p->server_random, 16, FRLG_PASSWORD, sizeof(FRLG_PASSWORD), s_ccmp_key) != 0) {
        ESP_LOGE(TAG, "CCMP key derive failed"); return false;
    }

    /* Use the real STA MAC (esp_wifi_set_mac needs the iface stopped; easyworld's fresh-MAC is only for
     * CCMP-replay avoidance on RECONNECT — not needed for the first attempt). We need our MAC to match
     * the assoc-response and to set the auth frame src. */
    esp_wifi_get_mac(WIFI_IF_STA, s_station_mac);

    wifi_config_t cfg = {0};
    /* tr-0.15: match the WORKING easyworld firmware EXACTLY — it sets sta.ssid to the 32-char ASCII-HEX
     * string of the 16-byte LDN ssid (NOT empty, NOT the raw bytes). With bssid_set this shouldn't matter
     * for matching, but the LDN association appears to require the client to present this exact ssid string. */
    {
        char hex[33];
        for (int i = 0; i < 16; i++) sprintf(hex + i * 2, "%02x", p->ssid[i]);
        memcpy(cfg.sta.ssid, hex, 32);
    }
    memcpy(cfg.sta.bssid, p->bssid, 6);
    cfg.sta.bssid_set = true;
    cfg.sta.channel = p->channel;
    cfg.sta.scan_method = WIFI_FAST_SCAN;             /* find the AP by bssid quickly */
    cfg.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;
    /* The AP IS found (active=1,passive=1) but connect rejects it as "not suitable" with a WPA2_PSK
     * threshold — the LDN beacon's security parses as incompatible. We inject the RSN IE + install the
     * CCMP key manually, so drop the scan-time security filter (OPEN = accept the AP regardless). */
    cfg.sta.threshold.authmode = WIFI_AUTH_OPEN;
    cfg.sta.pmf_cfg.capable = false;
    cfg.sta.pmf_cfg.required = false;
    strcpy((char *)cfg.sta.password, "00000000");    /* dummy; real key injected manually (4-way skipped) */
    if (esp_wifi_set_config(WIFI_IF_STA, &cfg) != ESP_OK) { ESP_LOGE(TAG, "set_config failed"); return false; }
    printf("  [join] sta mac %02x:%02x:%02x:%02x:%02x:%02x, connecting...\n",
           s_station_mac[0],s_station_mac[1],s_station_mac[2],s_station_mac[3],s_station_mac[4],s_station_mac[5]);

    s_association_seen = false; s_authorized = false; s_member = false; s_joining = true;

    /* Promiscuous must be OFF for the connect scan (trade_scan runs it); the poll re-enables it after. */
    esp_wifi_set_promiscuous(false);
    s_connect_us = esp_timer_get_time();
    /* The association/auth handshake must not lose frames to a connected phone's Bluetooth events; rebalanced once seated. */
    esp_coex_preference_set(ESP_COEX_PREFER_WIFI);
    esp_err_t e = esp_wifi_connect();
    ESP_LOGI(TAG, "esp_wifi_connect = %s (bssid %02x:%02x:%02x:%02x:%02x:%02x ch%d)",
             esp_err_to_name(e), p->bssid[0],p->bssid[1],p->bssid[2],p->bssid[3],p->bssid[4],p->bssid[5], p->channel);
    return e == ESP_OK;
}

/* Poll from the join task: once assoc-resp is seen, install keys + authorize, then send auth request. */
void ldn_join_poll(void)
{
    /* Re-enable promiscuous ~1.5s after connect: the scan needed it OFF, but now we need it ON to sniff
     * the assoc-response (STA_CONNECTED may not fire since LDN has no 4-way handshake) + the re-broadcast. */
    if (s_joining && !s_promisc_back && s_connect_us && esp_timer_get_time() - s_connect_us > 1500000) {
        s_promisc_back = true;
        esp_wifi_set_promiscuous(true);
        printf("  [join] promiscuous re-enabled post-scan (to sniff assoc-resp)\n");
    }
    /* Fallback association detector: ap_info returns OK once the STA has associated to the BSSID. */
    if (s_joining && !s_association_seen) {
        wifi_ap_record_t ap;
        if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK && memcmp(ap.bssid, s_target_bssid, 6) == 0) {
            s_association_seen = true;
            printf("  [join] ap_info shows associated to target bssid\n");
        }
    }
    if (s_joining && s_association_seen && !s_keyed && esp_wifi_sta_is_running_internal()) {
        s_keyed = true; s_joining = false;
        install_ldn_keys();
    }
    /* Re-send the AuthenticationRequest every ~1.5s until we appear as a MEMBER (our participant IP shows
     * in the re-broadcast advert), up to 12 attempts — this is the S3's AUTO_AUTH<->AUTO_GETIP loop. A single
     * auth response arriving does NOT mean the Switch registered us; only the participant IP does. */
    if (s_authorized && !s_member && s_auth_tries < 12) {
        int64_t now = esp_timer_get_time();
        if (s_last_auth_us == 0 || now - s_last_auth_us > 1500000) {
            s_auth_tries++;
            printf("  [join] AuthenticationRequest attempt %d/12 (auth_resp=%d member=%d)\n",
                   s_auth_tries, s_auth_response, s_member);
            send_auth_request();
            s_last_auth_us = now;
        }
    }
}
