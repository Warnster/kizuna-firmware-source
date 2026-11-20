/* SPDX-License-Identifier: AGPL-3.0-only
 * Derived from frlg-ldn-trade (https://github.com/tornadus/frlg-ldn-trade), Copyright (c) the frlg-ldn-trade authors.
 * Changes Copyright (c) 2026 Kizuna. See NOTICE.
 */
/* gbaframe.c — portable C port of frlgsim/gbaframe.py. See gbaframe.h. */
#include "gbaframe.h"
#include <string.h>

static void put_u16le(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static void put_u32le(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}
static uint16_t get_u16le(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static uint32_t get_u32le(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static size_t roundup4(size_t n) { return (n + 3u) & ~(size_t)3u; }

size_t gba_build_frame(uint8_t ftype, const uint8_t *data, size_t dlen, uint8_t *out) {
    out[0] = GBA_MARKER; out[1] = ftype;
    put_u16le(out + 2, (uint16_t)dlen);
    if (dlen) memcpy(out + 4, data, dlen);
    return 4 + dlen;
}

size_t gba_build_connect(const uint8_t *connect_id, size_t idlen, uint8_t *out) {
    return gba_build_frame(GBA_TYPE_C, connect_id, idlen, out);
}

size_t gba_wrap_t(const uint8_t *slot, size_t slot_len, uint32_t ts, uint8_t *out) {
    size_t padded = roundup4(slot_len);
    size_t blen = 8 + padded;                 /* ts(4) + [00 slot_len 00 00](4) + padded slot */
    out[0] = GBA_MARKER; out[1] = GBA_TYPE_T;
    put_u16le(out + 2, (uint16_t)blen);
    put_u32le(out + 4, ts);
    out[8] = 0; out[9] = (uint8_t)(slot_len & 0xFF); out[10] = 0; out[11] = 0;
    memcpy(out + 12, slot, slot_len);
    memset(out + 12 + slot_len, 0, padded - slot_len);
    return 4 + blen;
}

size_t gba_build_k(uint32_t k_seq, uint32_t mid, uint32_t acked_ts, uint8_t out[16]) {
    out[0] = GBA_MARKER; out[1] = GBA_TYPE_K;
    put_u16le(out + 2, 12);
    put_u32le(out + 4, k_seq);
    put_u32le(out + 8, mid);
    put_u32le(out + 12, acked_ts);
    return 16;
}

size_t gba_wrap_t_parent(const uint8_t *slot, size_t slot_len, uint32_t ts, uint8_t *out) {
    size_t n = gba_wrap_t(slot, slot_len, ts, out);
    out[8] = (uint8_t)(slot_len & 0xFF); out[9] = 0;   /* host layout: slot_len at body[4] */
    return n;
}

size_t gba_build_accept(const uint8_t host_session_id[2], const uint8_t connect_id[2], uint8_t out[10]) {
    uint8_t body[6] = { host_session_id[0], host_session_id[1], connect_id[0], connect_id[1], 0, 0 };
    return gba_build_frame(GBA_TYPE_A, body, sizeof body, out);
}

size_t gba_build_link_state(uint32_t value, uint8_t out[8]) {
    uint8_t body[4];
    put_u32le(body, value);
    return gba_build_frame(GBA_TYPE_G, body, sizeof body, out);
}

size_t gba_build_disconnect(const uint8_t connect_id[2], uint8_t out[6]) {
    return gba_build_frame(GBA_TYPE_D, connect_id, 2, out);
}

int gba_parse_in(const uint8_t *payload, size_t len, gba_in *out) {
    memset(out, 0, sizeof(*out));
    out->llsf_state = -1;
    if (len < 4 || payload[0] != GBA_MARKER) return 0;
    out->valid = 1;
    out->type = payload[1];
    uint16_t ln = get_u16le(payload + 2);
    const uint8_t *body = payload + 4;
    size_t blen = len - 4;
    if (ln < blen) blen = ln;

    if (out->type == GBA_TYPE_T) {
        if (blen < 5) return 1;               /* degenerate T */
        out->ts = get_u32le(body); out->ts_valid = 1;
        out->slot_len = body[4];              /* HOST slot_len @ body[4] */
        if (out->slot_len <= 1) return 1;     /* host idle keepalive */
        if ((size_t)8 + out->slot_len > blen) return 1;
        const uint8_t *slot = body + 8;
        uint32_t llsf3 = (uint32_t)slot[0] | ((uint32_t)slot[1] << 8) | ((uint32_t)slot[2] << 16);
        out->llsf_state = (int)((llsf3 >> 14) & 0xF);
        const uint8_t *pl = slot + 3;
        size_t plen = (size_t)out->slot_len - 3;
        if (out->llsf_state == 4) {           /* LCOM_UNI: N x 14-byte gRecvCmds */
            for (size_t off = 0; off + GBA_SLOT_LEN <= plen && out->nslots < GBA_MAX_SLOTS;
                 off += GBA_SLOT_LEN) {
                memcpy(out->slots[out->nslots], pl + off, GBA_SLOT_LEN);
                out->nslots++;
            }
        } else {                              /* NI window */
            out->ni_ack   = (int)((llsf3 >> 13) & 1);
            out->ni_n     = (int)((llsf3 >> 11) & 3);
            out->ni_phase = (int)((llsf3 >> 9) & 3);
            out->ni_size  = (int)(llsf3 & 0x7F);
            out->has_ni   = 1;
            size_t pcopy = plen > sizeof(out->ni_payload) ? sizeof(out->ni_payload) : plen;
            memcpy(out->ni_payload, pl, pcopy);
            out->ni_payload_len = (int)pcopy;
        }
        return 1;
    }
    if (out->type == GBA_TYPE_A) {
        if (blen >= 4) { memcpy(out->host_session_id, body, 2); memcpy(out->connect_id, body + 2, 2); }
        return 1;
    }
    if (out->type == GBA_TYPE_K) {
        if (blen >= 4) out->k_seq = get_u32le(body);
        if (blen >= 12) { out->acked_ts = get_u32le(body + 8); out->acked_ts_valid = 1; }
        return 1;
    }
    return 1;
}

int gba_parse_out(const uint8_t *payload, size_t len, gba_out *out) {
    memset(out, 0, sizeof(*out));
    if (len < 4 || payload[0] != GBA_MARKER || payload[1] != GBA_TYPE_T) return 0;
    uint16_t ln = get_u16le(payload + 2);
    const uint8_t *body = payload + 4;
    size_t blen = len - 4;
    if (ln < blen) blen = ln;
    if (blen < 8) return 0;
    out->valid = 1;
    out->ts = get_u32le(body);
    out->slot_len = body[5];                  /* CHILD slot_len @ body[5] */
    size_t sl = out->slot_len;
    if (sl > sizeof(out->slot)) sl = sizeof(out->slot);
    if (8 + sl > blen) sl = (blen > 8) ? blen - 8 : 0;
    memcpy(out->slot, body + 8, sl);
    if (out->slot_len >= 2) {
        rfu_parse_llsf_child(out->slot, &out->llsf);
        out->have_llsf = 1;
        if (out->llsf.state == RFU_LCOM_UNI && sl >= 2 + GBA_SLOT_LEN) {
            memcpy(out->cmd, out->slot + 2, GBA_SLOT_LEN);
            out->have_cmd = 1;
        }
    }
    return 1;
}
