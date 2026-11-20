/* SPDX-License-Identifier: AGPL-3.0-only
 * Derived from frlg-ldn-trade-esp32 (https://github.com/easyworld/frlg-ldn-trade-esp32), Copyright (c) Easy World and the frlg-ldn-trade-esp32 authors.
 * Changes Copyright (c) 2026 Kizuna. See NOTICE.
 */
/* dist_udp.c — see dist_udp.h. */
#include "dist_udp.h"
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include "esp_log.h"
#include "lwip/etharp.h"
#include "lwip/ip4_addr.h"
#include "lwip/sockets.h"
#include "lwip/tcpip.h"

#if !ETHARP_SUPPORT_STATIC_ENTRIES
#error "the distributor needs CONFIG_LWIP_DHCPS_STATIC_ENTRIES for static ARP entries"
#endif

#define PIA_PORT    12345
#define PAYLOAD_MAX 1472
#define PEERS_MAX   8

static const char *TAG = "dist_udp";
static int            s_socket = -1;
static ip4_addr_t     s_peers[PEERS_MAX];
static dist_udp_rx_fn s_rx;
static unsigned       s_tx_n, s_rx_n, s_rejected;
static unsigned       s_send_nomem, s_send_other;
static int            s_send_errno;

typedef struct { ip4_addr_t ip; struct eth_addr mac; bool remove; err_t result; } neighbor_call_t;

static void neighbor_callback(void *arg)   /* etharp is mutated on the tcpip thread */
{
    neighbor_call_t *c = arg;
    c->result = c->remove ? etharp_remove_static_entry(&c->ip) : etharp_add_static_entry(&c->ip, &c->mac);
}

esp_err_t dist_udp_peer(const uint8_t ip[4], const uint8_t *mac)
{
    ip4_addr_t a;
    IP4_ADDR(&a, ip[0], ip[1], ip[2], ip[3]);
    int slot = -1;
    for (int i = 0; i < PEERS_MAX; i++) {
        if (s_peers[i].addr == a.addr) { slot = i; break; }
        if (!s_peers[i].addr && slot < 0) slot = i;
    }
    if (slot < 0) return ESP_ERR_NO_MEM;
    neighbor_call_t call = { .ip = a, .remove = mac == NULL };
    if (mac) memcpy(call.mac.addr, mac, 6);
    if (tcpip_callback_wait(neighbor_callback, &call) != ERR_OK) return ESP_FAIL;
    if (call.result != ERR_OK && !(call.remove && call.result == ERR_ARG)) return ESP_FAIL;
    s_peers[slot].addr = mac ? a.addr : 0;
    return ESP_OK;
}

esp_err_t dist_udp_start(dist_udp_rx_fn rx)
{
    dist_udp_stop();
    s_rx = rx;
    s_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (s_socket < 0) return ESP_FAIL;
    int yes = 1;
    struct sockaddr_in addr = { .sin_family = AF_INET, .sin_port = htons(PIA_PORT), .sin_addr.s_addr = INADDR_ANY };
    if (setsockopt(s_socket, SOL_SOCKET, SO_BROADCAST, &yes, sizeof yes)
        || bind(s_socket, (struct sockaddr *)&addr, sizeof addr)
        || fcntl(s_socket, F_SETFL, O_NONBLOCK)) {
        dist_udp_stop();
        return ESP_FAIL;
    }
    s_tx_n = s_rx_n = s_rejected = s_send_nomem = s_send_other = 0; s_send_errno = 0;
    return ESP_OK;
}

void dist_udp_stop(void)
{
    if (s_socket >= 0) close(s_socket);
    s_socket = -1;
    for (int i = 0; i < PEERS_MAX; i++) {
        if (!s_peers[i].addr) continue;
        uint8_t ip[4];
        memcpy(ip, &s_peers[i].addr, 4);
        dist_udp_peer(ip, NULL);
    }
}

void dist_udp_send(const uint8_t dst_ip[4], const uint8_t *data, int len)
{
    if (s_socket < 0 || len <= 0 || len > PAYLOAD_MAX) return;
    struct sockaddr_in dst = { .sin_family = AF_INET, .sin_port = htons(PIA_PORT) };
    memcpy(&dst.sin_addr.s_addr, dst_ip, 4);
    if (sendto(s_socket, data, len, 0, (struct sockaddr *)&dst, sizeof dst) == len) s_tx_n++;
    else {
        s_rejected++;
        s_send_errno = errno;
        if (errno == ENOMEM || errno == ENOBUFS) s_send_nomem++; else s_send_other++;
    }
}

void dist_udp_poll(void)
{
    if (s_socket < 0) return;
    static uint8_t buf[PAYLOAD_MAX + 1];
    for (int count = 0; count < 24; count++) {   /* bounded: a burst must not stall the tick */
        struct sockaddr_in src;
        socklen_t size = sizeof src;
        int n = recvfrom(s_socket, buf, sizeof buf, 0, (struct sockaddr *)&src, &size);
        if (n < 0) {
            if (errno != EAGAIN && errno != EWOULDBLOCK) ESP_LOGW(TAG, "recv errno %d", errno);
            return;
        }
        bool known = false;
        for (int i = 0; i < PEERS_MAX; i++) known |= s_peers[i].addr == src.sin_addr.s_addr;
        if (n > PAYLOAD_MAX || src.sin_port != htons(PIA_PORT) || !known) { s_rejected++; continue; }
        s_rx_n++;
        uint8_t ip[4];
        memcpy(ip, &src.sin_addr.s_addr, 4);
        if (s_rx) s_rx(ip, buf, n);
    }
}

void dist_udp_send_errors(unsigned *nomem, unsigned *other, int *last_errno)
{
    if (nomem) *nomem = s_send_nomem;
    if (other) *other = s_send_other;
    if (last_errno) *last_errno = s_send_errno;
}

void dist_udp_counters(unsigned *tx, unsigned *rx, unsigned *rejected)
{
    if (tx) *tx = s_tx_n;
    if (rx) *rx = s_rx_n;
    if (rejected) *rejected = s_rejected;
}
