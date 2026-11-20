/* SPDX-License-Identifier: AGPL-3.0-only
 * Derived from frlg-ldn-trade (https://github.com/tornadus/frlg-ldn-trade), Copyright (c) the frlg-ldn-trade authors.
 * Changes Copyright (c) 2026 Kizuna. See NOTICE.
 */
/* ni.c — portable C port of frlgsim/ni.py. See ni.h. */
#include "ni.h"
#include "charmap.h"
#include <string.h>

/* Shared builder: `uname9` is the already-encoded 9-byte uname field (8 name bytes + terminator/pad
 * as the caller prepared it). Both the ASCII and raw-bytes entry points below feed this. */
static size_t ni_build_game_data_u(int version_low, uint16_t trainer_id, const uint8_t uname9[9],
                                   int language, int activity, int started,
                                   const uint8_t partner_info[4], uint8_t out[26]) {
    uint16_t compat = (uint16_t)((language & 0xF) | ((version_low & 0xF) << 10));
    uint8_t rgd[13];
    memset(rgd, 0, sizeof(rgd));
    rgd[0] = (uint8_t)compat;         rgd[1] = (uint8_t)(compat >> 8);
    rgd[2] = (uint8_t)trainer_id;     rgd[3] = (uint8_t)(trainer_id >> 8);
    if (partner_info) memcpy(rgd + 4, partner_info, 4);
    rgd[10] = (uint8_t)((activity & 0x7F) | ((started ? 1 : 0) << 7));
    uint8_t gname[15];
    memset(gname, 0, sizeof(gname));
    memcpy(gname, rgd, 13);
    out[0] = (uint8_t)NI_RFU_SERIAL_GAME; out[1] = (uint8_t)(NI_RFU_SERIAL_GAME >> 8);
    memcpy(out + 2, gname, 15);
    memcpy(out + 17, uname9, 9);
    return 26;
}

size_t ni_build_game_data_raw(int version_low, uint16_t trainer_id, const uint8_t ot_name8[8],
                              int language, int activity, int started,
                              const uint8_t partner_info[4], uint8_t out[26]) {
    uint8_t uname[9];
    memcpy(uname, ot_name8, 8);   /* raw FRLG-charmap bytes (kana-safe), verbatim */
    uname[8] = 0x00;              /* 9th byte: terminator/pad, same as charmap_encode's pad */
    return ni_build_game_data_u(version_low, trainer_id, uname, language, activity, started,
                                partner_info, out);
}

size_t ni_build_game_data(int version_low, uint16_t trainer_id, const char *ot_name,
                          int language, int activity, int started,
                          const uint8_t partner_info[4], uint8_t out[26]) {
    uint8_t uname[9];
    charmap_encode(ot_name, 9, 0x00, uname);   /* ASCII -> FRLG charmap (English names only) */
    return ni_build_game_data_u(version_low, trainer_id, uname, language, activity, started,
                                partner_info, out);
}

void ni_header(int data_type, uint16_t payload_size, uint32_t data_size, uint8_t out7[7]) {
    out7[0] = (uint8_t)data_type;
    out7[1] = (uint8_t)payload_size;       out7[2] = (uint8_t)(payload_size >> 8);
    out7[3] = (uint8_t)data_size;          out7[4] = (uint8_t)(data_size >> 8);
    out7[5] = (uint8_t)(data_size >> 16);  out7[6] = (uint8_t)(data_size >> 24);
}

void ni_sender_init(ni_sender *s, const uint8_t *src, size_t src_len,
                    int sub_frame_size, int data_type) {
    memset(s, 0, sizeof(*s));
    if (src_len > sizeof(s->src)) src_len = sizeof(s->src);
    memcpy(s->src, src, src_len);
    s->data_size = src_len;
    s->data_type = data_type & 0xFF;
    s->payload_size = sub_frame_size - NI_CHILD_FRAME_SIZE;   /* 12 for 14 */
    s->state = NI_SEND_START;
    s->phase = 0;
    for (int i = 0; i < NI_WINDOW_COUNT; i++) s->n[i] = 1;
    s->remain = NI_HEADER_SIZE;
    ni_header(s->data_type, (uint16_t)s->payload_size, (uint32_t)s->data_size, s->header);
}

int ni_sender_done(const ni_sender *s) { return s->state == NI_DONE; }

/* Retain every emitted sub-frame (state,n,phase + bytes) so the orchestrator can re-send one
 * the host lost. Mirrors NISender._emit's self.emitted.append. */
static void ni_record(ni_sender *s, int state, int n, int phase, const uint8_t *slot, size_t len) {
    if (s->emitted_n >= NI_EMIT_MAX) return;   /* game data never tiles this far */
    ni_emit_t *e = &s->emitted[s->emitted_n++];
    e->state = state; e->n = n; e->phase = phase;
    if (len > NI_SLOT_MAX) len = NI_SLOT_MAX;
    memcpy(e->slot, slot, len); e->slot_len = (int)len;
}

int ni_sender_next_slot(ni_sender *s, uint8_t *out, size_t *out_len) {
    uint8_t llsf[2];
    if (s->state == NI_SEND_START) {
        int size = (s->remain < s->payload_size) ? s->remain : s->payload_size;   /* 7 */
        int en = s->n[0];
        rfu_child_ni_llsf(RFU_LCOM_NI_START, en, 0, 0, size, llsf);
        out[0] = llsf[0]; out[1] = llsf[1];
        memcpy(out + 2, s->header, (size_t)size);
        *out_len = 2 + (size_t)size;
        ni_record(s, RFU_LCOM_NI_START, en, 0, out, *out_len);
        s->state = NI_SENDING;
        s->phase = 0;
        for (int i = 0; i < NI_WINDOW_COUNT; i++) { s->n[i] = 1; s->now[i] = s->payload_size * i; }
        s->remain = (int)s->data_size;
        return 1;
    }
    if (s->state == NI_SENDING) {
        while (s->now[s->phase] >= (int)s->data_size) s->phase = (s->phase + 1) % NI_WINDOW_COUNT;
        int off = s->now[s->phase];
        int avail = (int)s->data_size - off;
        int size = (s->payload_size < avail) ? s->payload_size : avail;
        int en = s->n[s->phase], eph = s->phase;
        rfu_child_ni_llsf(RFU_LCOM_NI, en, eph, 0, size, llsf);
        out[0] = llsf[0]; out[1] = llsf[1];
        memcpy(out + 2, s->src + off, (size_t)size);
        *out_len = 2 + (size_t)size;
        ni_record(s, RFU_LCOM_NI, en, eph, out, *out_len);
        s->remain -= size;
        s->now[s->phase] += s->payload_size << 2;
        s->phase = (s->phase + 1) % NI_WINDOW_COUNT;
        if (s->remain <= 0) { s->state = NI_SEND_LAST; s->phase = 0; }
        return 1;
    }
    if (s->state == NI_SEND_LAST) {
        rfu_child_ni_llsf(RFU_LCOM_NI_END, 0, 0, 0, 0, llsf);
        out[0] = llsf[0]; out[1] = llsf[1]; *out_len = 2;
        ni_record(s, RFU_LCOM_NI_END, 0, 0, out, *out_len);
        s->state = NI_SEND_NULL;
        return 1;
    }
    if (s->state == NI_SEND_NULL) {
        rfu_child_ni_llsf(RFU_LCOM_NULL, 1, 0, 0, 0, llsf);
        out[0] = llsf[0]; out[1] = llsf[1]; *out_len = 2;
        ni_record(s, RFU_LCOM_NULL, 1, 0, out, *out_len);
        s->state = NI_DONE;
        return 1;
    }
    return 0;
}

int ni_sender_last_emitted(const ni_sender *s, int *state, int *n, int *phase,
                           uint8_t *out, size_t *out_len) {
    if (s->emitted_n <= 0) return 0;
    const ni_emit_t *e = &s->emitted[s->emitted_n - 1];
    if (state) *state = e->state;
    if (n) *n = e->n;
    if (phase) *phase = e->phase;
    if (out) memcpy(out, e->slot, (size_t)e->slot_len);
    if (out_len) *out_len = (size_t)e->slot_len;
    return 1;
}

int ni_sender_resend_after(const ni_sender *s, int state, int n, int phase,
                           uint8_t *out, size_t *out_len) {
    for (int i = 0; i < s->emitted_n; i++) {
        const ni_emit_t *e = &s->emitted[i];
        if (e->state == state && e->n == n && e->phase == phase) {
            if (i + 1 < s->emitted_n) {
                const ni_emit_t *nx = &s->emitted[i + 1];
                if (out) memcpy(out, nx->slot, (size_t)nx->slot_len);
                if (out_len) *out_len = (size_t)nx->slot_len;
                return 1;
            }
            return 0;
        }
    }
    return 0;
}

size_t ni_recv_ack_slot(int state, int n, int phase, uint8_t out2[2]) {
    rfu_child_ni_llsf(state, n, phase, 1, 0, out2);
    return 2;
}

void ni_receiver_init(ni_receiver *r) { r->complete = 0; r->status = 0; r->has_status = 0; }

size_t ni_receiver_on_host_ni(ni_receiver *r, int state, int ack, int n, int phase,
                              const uint8_t *payload, size_t payload_len, uint8_t out2[2]) {
    if (ack != 0) return 0;                          /* only host outgoing NI (ack=0) is ack-able */
    if (state == RFU_LCOM_NI && payload_len > 0) { r->status = payload[0]; r->has_status = 1; }
    if (state == RFU_LCOM_NULL) { r->complete = 1; return 0; }
    if (state == RFU_LCOM_NI_END) r->complete = 1;
    if (state == RFU_LCOM_NI_START || state == RFU_LCOM_NI || state == RFU_LCOM_NI_END)
        return ni_recv_ack_slot(state, n, phase, out2);
    return 0;
}
