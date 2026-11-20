/* SPDX-License-Identifier: AGPL-3.0-only
 * Derived from frlg-ldn-trade-esp32 (https://github.com/easyworld/frlg-ldn-trade-esp32), Copyright (c) Easy World and the frlg-ldn-trade-esp32 authors.
 * Changes Copyright (c) 2026 Kizuna. See NOTICE.
 */
/* ldn_udp — static-IP UDP transport for the LDN Pia datagram path on the ESP32-C6.
 *
 * Ported from the S3 firmware's ldn_udp.c, adapted for the single-chip C6 (no PC / serial control
 * port). After the LDN association+auth the host re-advertises us with an assigned IP; we bring up a
 * static-IP STA netif (169.254.x.x/24, DHCP off), install a static ARP entry for the host, and open a
 * non-blocking UDP socket on port 12345 (SO_BROADCAST). ldn_udp_poll() drains inbound Pia datagrams
 * straight into the trade brain (ldn_brain_on_datagram); ldn_udp_send() is the brain's transport out.
 *
 * Requires CONFIG_LWIP_DHCPS_STATIC_ENTRIES=y (for etharp_add_static_entry). GBA-independent: none of
 * this touches the PARLIO GBA link, so it cannot regress the login gate. */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "esp_netif.h"
#include "esp_err.h"

/* Inbound datagram sink (the trade brain: ldn_brain_on_datagram). Set once before ldn_udp_start(). */
typedef void (*ldn_udp_rx_fn)(const uint8_t src_ip[4], const uint8_t *data, int len);
void ldn_udp_set_rx(ldn_udp_rx_fn fn);

/* One-time: hand the module the static-IP STA netif (created in app_main). */
void ldn_udp_init(esp_netif_t *netif);

/* Configure the static IP + static ARP for the host and open the UDP socket. All args are 4/6 raw
 * octets (network order, as they appear in the advert). Returns ESP_OK on success. */
esp_err_t ldn_udp_start(const uint8_t our_ip[4], const uint8_t host_ip[4], const uint8_t host_mac[6]);

/* Tear down the socket + static ARP + clear the interface IP. */
void ldn_udp_stop(void);

/* Direct datagram send (sim's transport callback). dst_ip = 4 octets (network order). */
void ldn_udp_send(const uint8_t dst_ip[4], const uint8_t *data, int len);

/* Drain any waiting inbound datagrams -> the rx callback. Call at the brain tick rate. */
void ldn_udp_poll(void);

/* Report the session IPs (octets): our static IP + the first known peer. false if not up. */
bool ldn_udp_ips(uint8_t our_ip[4], uint8_t host_ip[4]);
