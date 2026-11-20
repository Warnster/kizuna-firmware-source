/* rfu1.c — see include/rfu1.h. Pure C, no ESP dependencies (host-tested in test/). */
#include "rfu1.h"
#include <string.h>

static const uint16_t s_len[RFU1_NUM_TYPES] = { 36, 16, 16, 16, 16, 104, 104, 16, 16 };
static const char *s_name[RFU1_NUM_TYPES] = {
    "BROADCAST", "CONNECT_REQ", "CONNECT_ACK", "CONNECT_NACK", "DISCONNECT",
    "HOST_SEND", "CLIENT_SEND", "CLIENT_ACK", "FLOWCTL" };

uint32_t rfu1_len_for(uint32_t ptype) { return ptype < RFU1_NUM_TYPES ? s_len[ptype] : 0; }
const char *rfu1_name(uint32_t ptype) { return ptype < RFU1_NUM_TYPES ? s_name[ptype] : "?"; }

static inline uint32_t get_u32be(const uint8_t *p) {
    return (uint32_t)p[0] << 24 | (uint32_t)p[1] << 16 | (uint32_t)p[2] << 8 | p[3];
}
uint32_t rfu1_put_u32be(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)(v >> 24); p[1] = (uint8_t)(v >> 16); p[2] = (uint8_t)(v >> 8); p[3] = (uint8_t)v;
    return 4;
}

uint32_t rfu1_build(uint8_t *out, uint32_t ptype, uint32_t hdata, const uint8_t *payload, uint32_t plen) {
    uint32_t total = rfu1_len_for(ptype);
    if (!total) return 0;
    uint32_t room = total - RFU1_HDR_LEN;
    if (plen > room) plen = room;
    rfu1_put_u32be(out, RFU1_MAGIC);
    rfu1_put_u32be(out + 4, ptype);
    rfu1_put_u32be(out + 8, hdata);
    memset(out + RFU1_HDR_LEN, 0, room);
    if (payload && plen) memcpy(out + RFU1_HDR_LEN, payload, plen);
    return total;
}

uint32_t rfu1_build_broadcast(uint8_t *out, uint16_t devid, uint8_t next_slot, const uint32_t words[6]) {
    uint8_t p[24];
    for (int i = 0; i < 6; i++) rfu1_put_u32be(p + 4 * i, words[i]);
    return rfu1_build(out, RFU1_BROADCAST, (uint32_t)devid | ((uint32_t)next_slot << 16), p, sizeof p);
}
uint32_t rfu1_build_connect_ack(uint8_t *out, uint16_t devid, uint8_t clnum, bool ack) {
    return rfu1_build(out, ack ? RFU1_CONNECT_ACK : RFU1_CONNECT_NACK,
                      (uint32_t)devid | ((uint32_t)(clnum & 3) << 16), NULL, 0);
}
uint32_t rfu1_build_host_send(uint8_t *out, const uint8_t *data, uint32_t len) {
    if (len > RFU1_SEND_DATA) len = RFU1_SEND_DATA;
    return rfu1_build(out, RFU1_HOST_SEND, len & 0x7F, data, len);
}
uint32_t rfu1_build_client_ack(uint8_t *out, uint16_t devid, uint8_t clnum) {
    return rfu1_build(out, RFU1_CLIENT_ACK, (uint32_t)devid | ((uint32_t)(clnum & 3) << 16), NULL, 0);
}
uint32_t rfu1_build_flowctl(uint8_t *out, bool hold, uint8_t depth) {
    return rfu1_build(out, RFU1_FLOWCTL, (hold ? 1u : 0u) | ((uint32_t)depth << 8), NULL, 0);
}
uint32_t rfu1_build_disconnect(uint8_t *out, uint16_t devid) {
    return rfu1_build(out, RFU1_DISCONNECT, devid, NULL, 0);
}

uint32_t rfu1_parse(const uint8_t *d, size_t n, rfu1_pkt_t *out) {
    if (n < RFU1_HDR_LEN || get_u32be(d) != RFU1_MAGIC) return 0;
    uint32_t ptype = get_u32be(d + 4), total = rfu1_len_for(ptype);
    if (!total || n < total) return 0;
    out->ptype = ptype; out->hdata = get_u32be(d + 8); out->len = total;
    memcpy(out->payload, d + RFU1_HDR_LEN, total - RFU1_HDR_LEN);
    return total;
}

void rfu1_rx_init(rfu1_rx_t *rx) { memset(rx, 0, sizeof *rx); }

int rfu1_rx_feed(rfu1_rx_t *rx, const uint8_t *data, size_t n, rfu1_pkt_cb_t cb, void *ctx) {
    int delivered = 0;
    size_t i = 0;
    while (i < n) {
        /* Hunting for the magic: skip zero padding and anything else, one byte at a time, keeping a
         * partial magic prefix so a split across UART frames still locks. */
        if (rx->have < 4) {
            static const uint8_t magic[4] = { 'R', 'F', 'U', '1' };
            uint8_t b = data[i++];
            if (b == magic[rx->have]) { rx->buf[rx->have++] = b; }
            else {
                if (rx->have || b) rx->resyncs++;     /* zero padding between packets is not a resync */
                rx->have = (b == 'R') ? 1 : 0;
                if (rx->have) rx->buf[0] = b;
            }
            continue;
        }
        /* Have the magic: collect the rest of the header, then the fixed payload. */
        uint32_t need = RFU1_HDR_LEN;
        if (rx->have >= 8) {
            uint32_t ptype = get_u32be(rx->buf + 4);
            need = rfu1_len_for(ptype);
            if (!need) { rx->bad_len++; rx->have = 0; continue; }   /* unknown type -> resync */
        }
        size_t take = need - rx->have;
        if (take > n - i) take = n - i;
        memcpy(rx->buf + rx->have, data + i, take);
        rx->have += (uint32_t)take; i += take;
        if (rx->have == need && need > RFU1_HDR_LEN) {
            rfu1_pkt_t pkt;
            if (rfu1_parse(rx->buf, rx->have, &pkt)) { rx->packets++; delivered++; if (cb) cb(&pkt, ctx); }
            rx->have = 0;
        } else if (rx->have == need) {
            /* header complete, loop recomputes need from ptype */
        }
    }
    return delivered;
}
