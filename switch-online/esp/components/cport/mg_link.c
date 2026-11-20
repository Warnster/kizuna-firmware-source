/* SPDX-License-Identifier: AGPL-3.0-only
 * Derived from pokeldn (https://github.com/Decryptu/pokeldn), Copyright (c) the pokeldn authors.
 * Changes Copyright (c) 2026 Kizuna. See NOTICE.
 */
/* mg_link.c — portable C port of pokeldn frlg/gift/mg_link.py. See mg_link.h. */
#include "mg_link.h"
#include <string.h>

uint16_t mg_crc16(const uint8_t *data, size_t len) {
    uint16_t crc = 0x1121;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++)
            crc = (crc & 1) ? (uint16_t)((crc >> 1) ^ 0x8408) : (uint16_t)(crc >> 1);
    }
    return (uint16_t)~crc;
}

static void put16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static uint16_t get16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }

int mg_msg_build(mg_msg *m, uint16_t ident, const uint8_t *payload, size_t len, size_t size) {
    if (size == 0) size = MG_LINK_BUFFER_SIZE;
    if (size > MG_LINK_BUFFER_SIZE || len > size) return -1;
    memset(m, 0, sizeof(*m));
    if (len) memcpy(m->buf, payload, len);
    m->ident = ident;
    m->size = (uint16_t)size;
    m->crc = mg_crc16(m->buf, size);
    put16(m->header, ident);
    put16(m->header + 2, m->crc);
    put16(m->header + 4, m->size);
    /* As MGL_Send walks it: an exact multiple of 252 ends on a full chunk, no trailing block. */
    m->nblocks = 1 + (int)((size + MG_LINK_MAX_CHUNK - 1) / MG_LINK_MAX_CHUNK);
    return 0;
}

size_t mg_msg_block(const mg_msg *m, int i, const uint8_t **out) {
    if (i < 0 || i >= m->nblocks) return 0;
    if (i == 0) { *out = m->header; return MG_LINK_HEADER_SIZE; }
    size_t off = (size_t)(i - 1) * MG_LINK_MAX_CHUNK;
    size_t n = m->size - off;
    *out = m->buf + off;
    return n < MG_LINK_MAX_CHUNK ? n : MG_LINK_MAX_CHUNK;
}

void mg_rx_init(mg_rx *rx) {
    memset(rx, 0, sizeof(*rx));
    rx->expected_ident = -1;
}

void mg_rx_expect(mg_rx *rx, uint16_t ident) {
    mg_rx_init(rx);
    rx->expected_ident = ident;
    rx->in_header = 1;
}

int mg_rx_active(const mg_rx *rx) { return rx->expected_ident >= 0; }

mg_rx_result mg_rx_feed(mg_rx *rx, const uint8_t *block, size_t len) {
    if (!mg_rx_active(rx)) return MG_RX_ERR_IDLE;
    rx->blocks++;
    if (rx->in_header) {
        if (len < MG_LINK_HEADER_SIZE) return MG_RX_ERR_SHORT;
        uint16_t ident = get16(block), crc = get16(block + 2), size = get16(block + 4);
        /* MGL_Receive checks size before ident; keep that order. A zero is never a real header:
         * InitSend maps 0 -> 1024 before sending. */
        if (size == 0 || size > MG_LINK_BUFFER_SIZE) return MG_RX_ERR_SIZE;
        if (ident != rx->expected_ident) return MG_RX_ERR_IDENT;
        rx->ident = ident;
        rx->crc = crc;
        rx->size = size;
        rx->in_header = 0;
        return MG_RX_PENDING;
    }
    size_t take = rx->size - rx->len;
    if (take > MG_LINK_MAX_CHUNK) take = MG_LINK_MAX_CHUNK;
    if (len < take) return MG_RX_ERR_SHORT;
    memcpy(rx->buf + rx->len, block, take);
    rx->len += take;
    if (rx->len < rx->size) return MG_RX_PENDING;
    if (mg_crc16(rx->buf, rx->size) != rx->crc) return MG_RX_ERR_CRC;
    rx->expected_ident = -1;
    return MG_RX_COMPLETE;
}
