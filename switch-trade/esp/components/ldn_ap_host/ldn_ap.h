/* SPDX-License-Identifier: AGPL-3.0-only
 * Derived from GB-Link-Switch-LDN (https://github.com/GB-Link/GB-Link-Switch-LDN), Copyright (c) the GB-Link-Switch-LDN authors.
 * Changes Copyright (c) 2026 Kizuna. See NOTICE.
 */
/* ldn_ap.h — the adapter as the ACCESS POINT of an LDN network: a Switch on Mystery Gift → Wonder
 * Cards → Friend finds the room and joins it. The ESP-IDF glue around cport/ldn_host.c (the pure
 * advert/auth/member logic): a hidden WPA2 AP whose 4-way handshake is replaced through the private
 * wpa_supplicant hooks (LDN authenticates in its own 0x88b7 exchange and both sides derive the CCMP
 * key from the advertised server random), the 100 ms advertisement, and the station bookkeeping.
 *
 * Pinned to ESP-IDF v6.1: it uses esp_wifi_driver.h (private) and three hostapd functions.
 * Keys are borrowed for the life of the room and never stored. */
#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
#include "esp_netif.h"
#include "ldn_advert.h"

typedef struct {
    int            channel;         /* 1, 6 or 11 */
    const char    *host_name;       /* our LDN nickname, e.g. "KIZUNA" */
    const uint8_t *app_data;        /* FRLG room (ldn_frlg_app_data_build) */
    size_t         app_data_len;
    const char    *sta_ssid;        /* optional: also join this network as an uplink (AP+STA, DHCP) */
    const char    *sta_pass;
} ldn_ap_config;

typedef struct {
    uint8_t mac[6];
    uint8_t ip[4];
    uint8_t platform;               /* 0 Switch, 1 Switch 2 */
    uint8_t index;                  /* LDN participant slot, 1.. (0 is us); Pia's station record names it */
    char    name[33];
} ldn_ap_member;

/* Called from ldn_ap_poll() (the session task) when an authenticated member joins or leaves. */
typedef void (*ldn_ap_member_fn)(const ldn_ap_member *m, bool joined);

/* Wi-Fi must be initialised (esp_wifi_init) but not started. `keys` must stay valid until stop. */
esp_err_t ldn_ap_start(const ldn_keys *keys, const ldn_ap_config *cfg, ldn_ap_member_fn fn);
/* Call every few ms from one task: key install, auth, advert, ARP probes, timeouts. */
void      ldn_ap_poll(void);
void      ldn_ap_stop(void);
bool      ldn_ap_running(void);
esp_err_t ldn_ap_set_app_data(const uint8_t *data, size_t len);

/* Room identity, valid while running: the session id (Pia keys its crypto with it), our AP MAC
 * and IP (169.254.N.1), and the AP netif for the UDP socket. */
const uint8_t *ldn_ap_ssid(void);
void           ldn_ap_identity(uint8_t mac[6], uint8_t ip[4]);
esp_netif_t   *ldn_ap_netif(void);

/* One JSON object: the uplink's state (sta on/off, connected, ip, rssi, channel actually in use). */
void           ldn_ap_uplink_status(char *buf, size_t n);
