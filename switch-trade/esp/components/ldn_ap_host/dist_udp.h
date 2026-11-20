/* SPDX-License-Identifier: AGPL-3.0-only
 * Derived from frlg-ldn-trade-esp32 (https://github.com/easyworld/frlg-ldn-trade-esp32), Copyright (c) Easy World and the frlg-ldn-trade-esp32 authors.
 * Changes Copyright (c) 2026 Kizuna. See NOTICE.
 */
/* dist_udp.h — the Pia datagram path on the HOST side of an LDN room: a non-blocking UDP socket on
 * port 12345 of the AP interface (169.254.N.1) and a static ARP entry per member (the AP path never
 * delivers a station's ARP for us). The AP-side twin of components/ldn_join/ldn_udp.c.
 * Requires CONFIG_LWIP_DHCPS_STATIC_ENTRIES=y. */
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

typedef void (*dist_udp_rx_fn)(const uint8_t src_ip[4], const uint8_t *data, int len);

/* Open the socket once the AP (ldn_ap_start) has its address. */
esp_err_t dist_udp_start(dist_udp_rx_fn rx);
void      dist_udp_stop(void);
/* A member's IP and MAC (octets, network order): its static ARP entry; NULL mac removes it. */
esp_err_t dist_udp_peer(const uint8_t ip[4], const uint8_t *mac);
/* dst may be a member or the room broadcast 169.254.N.255. */
void      dist_udp_send(const uint8_t dst_ip[4], const uint8_t *data, int len);
/* Drain waiting datagrams into the rx callback; call at the session tick rate. */
void      dist_udp_poll(void);
/* Send failures by cause: out of buffers (ENOMEM/ENOBUFS) vs anything else, and the last errno. */
void      dist_udp_send_errors(unsigned *nomem, unsigned *other, int *last_errno);
void      dist_udp_counters(unsigned *tx, unsigned *rx, unsigned *rejected);
