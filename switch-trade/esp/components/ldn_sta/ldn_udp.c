/* SPDX-License-Identifier: AGPL-3.0-only
 * Derived from frlg-ldn-trade-esp32 (https://github.com/easyworld/frlg-ldn-trade-esp32), Copyright (c) Easy World and the frlg-ldn-trade-esp32 authors.
 * Changes Copyright (c) 2026 Kizuna. See NOTICE.
 */
/* ldn_udp — see ldn_udp.h. Static-IP UDP transport for the LDN Pia path on the ESP32-C6.
 * Ported from the S3 firmware (main/ldn_udp.c); the serial LDN_* command parser is dropped (no PC on
 * the C6) — ldn_udp_start() takes the resolved IPs directly from the membership parse instead. */
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include "esp_timer.h"
#include "esp_idf_version.h"
#include "lwip/def.h"          /* PP_HTONL */
#include "lwip/ip4_addr.h"     /* IP4_ADDR, ip4_addr_t */
#include "lwip/etharp.h"
#include "lwip/sockets.h"
#include "lwip/tcpip.h"
#include "ldn_udp.h"

#if !ETHARP_SUPPORT_STATIC_ENTRIES
#error "LDN requires CONFIG_LWIP_DHCPS_STATIC_ENTRIES for static neighbor support"
#endif

#define PIA_PORT    12345
#define PAYLOAD_MAX 1472
#define PEERS_MAX   8

static esp_netif_t *s_netif;
static int          s_socket = -1;
static uint8_t      s_host[6];
static ip4_addr_t   s_ip, s_peers[PEERS_MAX];
static unsigned     s_tx, s_rx, s_rejected;
static ldn_udp_rx_fn s_rx_fn;

void ldn_udp_set_rx(ldn_udp_rx_fn fn) { s_rx_fn = fn; }

/* ---- static ARP neighbor (added on the tcpip thread, as etharp requires) ---- */
typedef struct {
    ip4_addr_t ip;
    struct eth_addr mac;
    bool remove;
    err_t result;
} neighbor_call_t;

static void neighbor_callback(void *arg)
{
    neighbor_call_t *call = arg;
    call->result = call->remove ? etharp_remove_static_entry(&call->ip)
                                : etharp_add_static_entry(&call->ip, &call->mac);
}

static int neighbor(ip4_addr_t ip, const uint8_t *mac)
{
    int slot = -1;
    for (int i = 0; i < PEERS_MAX; ++i) {
        if (s_peers[i].addr == ip.addr) { slot = i; break; }
        if (!s_peers[i].addr && slot < 0) slot = i;
    }
    if (slot < 0) return ESP_ERR_NO_MEM;
    neighbor_call_t call = {.ip = ip, .remove = mac == NULL};
    if (mac) memcpy(call.mac.addr, mac, 6);
    /* Run the etharp mutation on/under the tcpip core. tcpip_callback_wait() exists only in newer lwip
     * (IDF >= 5.4 / v6.1); on v5.3 (the login-safe GBA baseline, [[v61-breaks-gba-timing]]) use the
     * core-lock equivalent (CONFIG_LWIP_TCPIP_CORE_LOCKING=y). Non-login LDN path. */
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 4, 0)
    if (tcpip_callback_wait(neighbor_callback, &call) != ERR_OK) return ESP_FAIL;
#else
    LOCK_TCPIP_CORE();
    neighbor_callback(&call);
    UNLOCK_TCPIP_CORE();
#endif
    if (call.result != ERR_OK && !(call.remove && call.result == ERR_ARG)) return ESP_FAIL;
    s_peers[slot].addr = mac ? ip.addr : 0;
    return ESP_OK;
}

void ldn_udp_init(esp_netif_t *netif)
{
    s_netif = netif;
}

void ldn_udp_stop(void)
{
    if (s_socket >= 0) close(s_socket);
    s_socket = -1;
    for (int i = 0; i < PEERS_MAX; ++i) {
        if (s_peers[i].addr) neighbor(s_peers[i], NULL);
    }
    s_ip.addr = 0;
    if (s_netif) {
        esp_netif_ip_info_t empty = {0};
        esp_netif_set_ip_info(s_netif, &empty);
    }
}

esp_err_t ldn_udp_start(const uint8_t our_ip[4], const uint8_t host_ip[4], const uint8_t host_mac[6])
{
    if (!s_netif) return ESP_ERR_INVALID_STATE;
    ldn_udp_stop();
    memcpy(s_host, host_mac, 6);

    ip4_addr_t peer;
    IP4_ADDR(&s_ip, our_ip[0],  our_ip[1],  our_ip[2],  our_ip[3]);
    IP4_ADDR(&peer,  host_ip[0], host_ip[1], host_ip[2], host_ip[3]);
    if (s_ip.addr == peer.addr) return ESP_ERR_INVALID_ARG;

    esp_netif_ip_info_t info = {0};
    info.ip.addr      = s_ip.addr;
    info.netmask.addr = PP_HTONL(0xffffff00u);   /* /24 */
    info.gw.addr      = 0;
    esp_err_t r = esp_netif_set_ip_info(s_netif, &info);
    if (r != ESP_OK) return r;

    if (neighbor(peer, s_host) != ESP_OK) goto failed;

    s_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (s_socket < 0) goto failed;
    int yes = 1;
    struct sockaddr_in address = {.sin_family = AF_INET, .sin_port = htons(PIA_PORT),
                                  .sin_addr.s_addr = INADDR_ANY};
    if (setsockopt(s_socket, SOL_SOCKET, SO_BROADCAST, &yes, sizeof(yes)) ||
        bind(s_socket, (struct sockaddr *)&address, sizeof(address)) ||
        fcntl(s_socket, F_SETFL, O_NONBLOCK)) goto failed;
    s_tx = s_rx = s_rejected = 0;
    return ESP_OK;
failed:
    ldn_udp_stop();
    return ESP_FAIL;
}

void ldn_udp_send(const uint8_t dst_ip[4], const uint8_t *data, int len)
{
    if (s_socket < 0 || len <= 0 || len > PAYLOAD_MAX) return;
    struct sockaddr_in dest = {.sin_family = AF_INET, .sin_port = htons(PIA_PORT)};
    memcpy(&dest.sin_addr.s_addr, dst_ip, 4);   /* octets are already network order */
    if (sendto(s_socket, data, len, 0, (struct sockaddr *)&dest, sizeof(dest)) == len) ++s_tx;
    else ++s_rejected;
}

bool ldn_udp_ips(uint8_t our_ip[4], uint8_t host_ip[4])
{
    if (!s_ip.addr) return false;
    memcpy(our_ip, &s_ip.addr, 4);
    for (int i = 0; i < PEERS_MAX; ++i) {
        if (s_peers[i].addr) { memcpy(host_ip, &s_peers[i].addr, 4); return true; }
    }
    return false;
}

void ldn_udp_poll(void)
{
    if (s_socket < 0) return;
    static uint8_t bytes[PAYLOAD_MAX + 1];
    /* Bound the drain so the tick doesn't stall if the Switch bursts. */
    for (int count = 0; count < 24; ++count) {   /* drain more per inline poll (single loop polls in GBA gaps, less often than the old 60Hz task) */
        struct sockaddr_in source;
        socklen_t size = sizeof(source);
        int n = recvfrom(s_socket, bytes, sizeof(bytes), 0, (struct sockaddr *)&source, &size);
        if (n < 0) {
            if (errno != EAGAIN && errno != EWOULDBLOCK) printf("  [udp] recv errno=%d\n", errno);
            break;
        }
        bool known = false;
        for (int i = 0; i < PEERS_MAX; ++i) known |= s_peers[i].addr == source.sin_addr.s_addr;
        static int dbg;
        if (dbg < 12) { dbg++;
            uint8_t *ip = (uint8_t *)&source.sin_addr.s_addr;
            printf("  [udp] rx n=%d from=%u.%u.%u.%u:%u known=%d (tx=%u rx=%u rej=%u)\n",
                   n, ip[0], ip[1], ip[2], ip[3], ntohs(source.sin_port), known, s_tx, s_rx, s_rejected);
        }
        if (n > PAYLOAD_MAX || source.sin_port != htons(PIA_PORT) || !known) { ++s_rejected; continue; }
        ++s_rx;
        uint8_t src[4]; memcpy(src, &source.sin_addr.s_addr, 4);
        if (s_rx_fn) s_rx_fn(src, bytes, n);
    }
}
