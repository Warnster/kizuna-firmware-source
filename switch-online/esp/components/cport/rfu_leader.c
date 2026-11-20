/* SPDX-License-Identifier: AGPL-3.0-only
 * Derived from pokeldn (https://github.com/Decryptu/pokeldn), Copyright (c) the pokeldn authors.
 * Changes Copyright (c) 2026 Kizuna. See NOTICE.
 */
/* rfu_leader.c — portable C port of pokeldn gba/rfu_leader.py + the parent halves of gba/ni.py.
 * See rfu_leader.h. */
#include "rfu_leader.h"
#include "gbaframe.h"
#include <string.h>

#define NI_HEADER_SIZE   7    /* dataType(1) + payloadSize(2) + dataSize(4) */
#define NI_WINDOW_COUNT  4
#define PARENT_FRAME_SIZE 3
#define COMM_TABLE_LEN   70   /* 5 rows of 14: gRecvCmds */
#define STATUS_SUBFRAME  8    /* parent payloadSize 5, under the 7-byte header: two NI_STARTs */

static uint16_t rd16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static uint32_t rd32(const uint8_t *p) { return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24; }

/* Parent LLSF [llsf_struct[MODE_PARENT], librfu_rfu.c:95-110]: 3 bytes LE,
 * state<<14 | bmSlot<<18 | ack<<13 | n<<11 | phase<<9 | size&0x7f. */
static void parent_llsf(int state, int size, int ack, int n, int phase, int bm_slot, uint8_t out[3]) {
    uint32_t f = ((uint32_t)(state & 0xF) << 14) | ((uint32_t)(ack & 1) << 13) | ((uint32_t)(n & 3) << 11)
               | ((uint32_t)(phase & 3) << 9) | ((uint32_t)(bm_slot & 0xF) << 18) | (uint32_t)(size & 0x7F);
    out[0] = (uint8_t)f; out[1] = (uint8_t)(f >> 8); out[2] = (uint8_t)(f >> 16);
}

static void push_pending(rfu_leader *l, const uint8_t *b, size_t n) {
    if (l->p_len >= RFUL_PENDING_MAX || n > RFUL_FRAME_MAX) return;
    rful_frame *f = &l->pending[(l->p_head + l->p_len++) % RFUL_PENDING_MAX];
    f->len = (uint8_t)n;
    memcpy(f->b, b, n);
}

static size_t wrap_t(rfu_leader *l, const uint8_t *slot, size_t n, uint8_t *out) {
    size_t len = gba_wrap_t_parent(slot, n, l->ts, out);
    l->ts++;
    return len;
}

static void push_wrapped(rfu_leader *l, const uint8_t *slot, size_t n) {
    uint8_t f[RFUL_FRAME_MAX];
    push_pending(l, f, wrap_t(l, slot, n, f));
}

/* ---- row one (ChildEcho) ---- */
static void echo_append(rfu_leader *l, const uint8_t cmd[RFU_SLOT_LEN]) {
    for (int i = 0; i < l->echo_len; i++)
        if (!memcmp(l->echo_q[(l->echo_head + i) % RFUL_ECHO_MAX], cmd, RFU_SLOT_LEN)) {
            l->echo_coalesced++;
            return;
        }
    if (l->echo_len == RFUL_ECHO_MAX) {         /* safety valve; must stay unused */
        l->echo_head = (l->echo_head + 1) % RFUL_ECHO_MAX;
        l->echo_len--;
        l->echo_dropped++;
    }
    memcpy(l->echo_q[(l->echo_head + l->echo_len++) % RFUL_ECHO_MAX], cmd, RFU_SLOT_LEN);
    if (l->echo_len > l->echo_backlog_peak) l->echo_backlog_peak = l->echo_len;
}

/* With nothing queued the last row stands: the console's RFU does not clear a mirrored command. */
static const uint8_t *echo_next_row(rfu_leader *l) {
    if (l->echo_len) {
        memcpy(l->echo_row, l->echo_q[l->echo_head], RFU_SLOT_LEN);
        l->echo_head = (l->echo_head + 1) % RFUL_ECHO_MAX;
        l->echo_len--;
    }
    return l->echo_row;
}

int rfu_leader_echo_backlog(const rfu_leader *l) { return l->echo_len; }

/* ---- our NI: the librfu sender sequence for the 1-byte join status [ni.py _ni_send_sequence] ---- */
static void build_parent_ni(rfu_leader *l) {
    uint8_t src[1] = { l->join_status };
    const int data_size = 1, payload_size = STATUS_SUBFRAME - PARENT_FRAME_SIZE;
    /* dataType 0 on the sending path of setSendData_org [librfu_rfu.c:1477] */
    uint8_t hdr[NI_HEADER_SIZE] = { 0, (uint8_t)payload_size, 0, (uint8_t)data_size, 0, 0, 0 };
    l->pni_len = 0;
    for (int n0 = 1, off = 0, remain = NI_HEADER_SIZE; remain > 0; n0 = (n0 + 1) & 3,
         off += payload_size, remain -= payload_size) {
        rful_ni_step *s = &l->pni[l->pni_len++];
        s->state = RFU_LCOM_NI_START; s->n = (uint8_t)n0; s->phase = 0;
        s->size = (uint8_t)(remain < payload_size ? remain : payload_size);
        memcpy(s->payload, hdr + off, s->size);
    }
    int now[NI_WINDOW_COUNT], n[NI_WINDOW_COUNT];
    for (int i = 0; i < NI_WINDOW_COUNT; i++) { now[i] = payload_size * i; n[i] = 1; }
    for (int phase = 0, remain = data_size; remain > 0; remain -= payload_size) {
        while (now[phase] >= data_size) phase = (phase + 1) % NI_WINDOW_COUNT;
        int o = now[phase], size = data_size - o < payload_size ? data_size - o : payload_size;
        rful_ni_step *s = &l->pni[l->pni_len++];
        s->state = RFU_LCOM_NI; s->n = (uint8_t)n[phase]; s->phase = (uint8_t)phase; s->size = (uint8_t)size;
        memcpy(s->payload, src + o, (size_t)size);
        now[phase] += payload_size << 2;
        n[phase] = (n[phase] + 1) & 3;
        phase = (phase + 1) % NI_WINDOW_COUNT;
    }
    l->pni[l->pni_len++] = (rful_ni_step){ RFU_LCOM_NI_END, 0, 0, 0, {0} };
    l->pni[l->pni_len++] = (rful_ni_step){ RFU_LCOM_NULL, 1, 0, 0, {0} };
    l->pni_i = 0;
}

/* ---- the child's NI (ParentNIReceiver) ---- */
static void ni_finalize(rfu_leader *l) {
    if (l->child_game_data_len) return;
    int n = l->ni_buf_len;
    if (l->ni_data_size >= 0 && l->ni_data_size < n) n = l->ni_data_size;
    memcpy(l->child_game_data, l->ni_buf, (size_t)n);
    l->child_game_data_len = n;
}

/* Returns 1 and fills ack3 when the sub-frame must be acked. */
static int on_child_ni(rfu_leader *l, int state, int n, int phase, const uint8_t *payload, int size,
                       uint8_t ack3[3]) {
    if (state == RFU_LCOM_NI_START) {
        for (int i = 0; i < size && l->ni_hdr_len < (int)sizeof l->ni_hdr; i++) l->ni_hdr[l->ni_hdr_len++] = payload[i];
        if (l->ni_data_size < 0 && l->ni_hdr_len >= NI_HEADER_SIZE) {
            int ps = rd16(l->ni_hdr + 1);
            if (ps) l->ni_payload_size = ps;
            l->ni_data_size = (int)rd32(l->ni_hdr + 3);
        }
    } else if (state == RFU_LCOM_NI) {
        int ph = phase & 3;
        int off = ph * l->ni_payload_size + l->ni_phase_count[ph] * NI_WINDOW_COUNT * l->ni_payload_size;
        if (off + size <= RFUL_GAME_DATA_MAX) {
            memcpy(l->ni_buf + off, payload, (size_t)size);
            if (off + size > l->ni_buf_len) l->ni_buf_len = off + size;
        }
        l->ni_phase_count[ph]++;
    } else if (state == RFU_LCOM_NULL) {
        l->ni_complete = 1;
        ni_finalize(l);
        return 0;
    }
    if (state == RFU_LCOM_NI_END) {
        l->ni_complete = 1;
        ni_finalize(l);
    }
    if (state == RFU_LCOM_NI_START || state == RFU_LCOM_NI || state == RFU_LCOM_NI_END) {
        parent_llsf(state, 0, 1, n, phase, l->bm_slot, ack3);
        return 1;
    }
    return 0;
}

void rfu_leader_init(rfu_leader *l, const uint8_t host_session_id[2], uint32_t start_ts) {
    memset(l, 0, sizeof(*l));
    memcpy(l->host_session_id, host_session_id, 2);
    l->bm_slot = 1;
    l->join_status = RFU_STATUS_JOIN_GROUP_OK;
    l->ts = start_ts;
    l->state = RFUL_WAIT_CONNECT;
    l->ni_payload_size = 12;
    l->ni_data_size = -1;
}

static int connected(const rfu_leader *l) { return l->have_connect && l->state != RFUL_DISCONNECTED; }

rful_event rfu_leader_receive(rfu_leader *l, const uint8_t *inner, size_t len) {
    if (len < 4 || inner[0] != GBA_MARKER) return RFUL_EV_NONE;
    size_t size = rd16(inner + 2);
    if (len < 4 + size) return RFUL_EV_NONE;
    uint8_t typ = inner[1];
    const uint8_t *body = inner + 4;

    if (typ == GBA_TYPE_C) {
        if (size < 2) return RFUL_EV_NONE;
        if (!l->have_connect) {
            uint8_t f[10];
            memcpy(l->connect_id, body, 2);
            l->have_connect = 1;
            l->state = RFUL_CHILD_NI;
            push_pending(l, f, gba_build_accept(l->host_session_id, l->connect_id, f));
            push_pending(l, f, gba_build_link_state(0, f));
            return RFUL_EV_CONNECT;
        }
        /* The child sends C every VBlank until it sees A; A's retransmits belong to Reliable. */
        if (!memcmp(body, l->connect_id, 2) && l->state != RFUL_DISCONNECTED) return RFUL_EV_CONNECT_DUPLICATE;
        return RFUL_EV_CONNECT_REJECTED;
    }
    if (typ == GBA_TYPE_D) {
        l->state = RFUL_DISCONNECTED;
        l->p_len = 0;
        return RFUL_EV_DISCONNECT;
    }
    if (typ == GBA_TYPE_K) { l->k_acks++; return RFUL_EV_K_ACK; }
    if (typ != GBA_TYPE_T || !connected(l)) return RFUL_EV_NONE;

    gba_out rec;
    if (!gba_parse_out(inner, len, &rec) || !rec.have_llsf) return RFUL_EV_NONE;
    const rfu_child_llsf *ll = &rec.llsf;

    if (ll->ack == 1) {
        if (l->state == RFUL_PARENT_NI && l->parent_waiting && ll->state == l->pw_state
            && ll->n == l->pw_n && ll->phase == l->pw_phase) {
            l->parent_waiting = 0;
            return RFUL_EV_PARENT_NI_ACK;
        }
        return RFUL_EV_NI_ACK_IGNORED;
    }

    if (ll->state != RFU_LCOM_UNI) {
        if (l->state != RFUL_CHILD_NI && l->state != RFUL_PARENT_NI) return RFUL_EV_NI_OUT_OF_PHASE;
        int klen = rec.slot_len > 2 ? rec.slot_len - 2 : 0;
        if (klen > RFU_SLOT_LEN) klen = RFU_SLOT_LEN;
        for (int i = 0; i < l->seen_ni_len; i++) {
            const rful_ni_key *k = &l->seen_ni[i];
            if (k->state == ll->state && k->n == ll->n && k->phase == ll->phase && k->len == klen
                && !memcmp(k->payload, rec.slot + 2, (size_t)klen)) {
                /* A layer failed to de-duplicate: ack again, never append the payload twice. */
                if (ll->state == RFU_LCOM_NI_START || ll->state == RFU_LCOM_NI || ll->state == RFU_LCOM_NI_END) {
                    uint8_t ack[3];
                    parent_llsf(ll->state, 0, 1, ll->n, ll->phase, l->bm_slot, ack);
                    push_wrapped(l, ack, 3);
                }
                return RFUL_EV_CHILD_NI_DUPLICATE;
            }
        }
        if (l->seen_ni_len < RFUL_SEEN_NI_MAX) {
            rful_ni_key *k = &l->seen_ni[l->seen_ni_len++];
            k->state = (uint8_t)ll->state; k->n = (uint8_t)ll->n; k->phase = (uint8_t)ll->phase;
            k->len = (uint8_t)klen;
            memcpy(k->payload, rec.slot + 2, (size_t)klen);
        }
        int psize = ll->size;
        if (psize > (int)sizeof(rec.slot) - 2) psize = (int)sizeof(rec.slot) - 2;
        uint8_t ack[3];
        if (on_child_ni(l, ll->state, ll->n, ll->phase, rec.slot + 2, psize, ack)) push_wrapped(l, ack, 3);
        /* Native order: the child's END, its unacked terminal NULL, then our NI. */
        if (ll->state == RFU_LCOM_NULL && l->ni_complete && l->state == RFUL_CHILD_NI) {
            uint8_t f[8];
            push_pending(l, f, gba_build_link_state(1, f));
            build_parent_ni(l);
            l->state = RFUL_PARENT_NI;
            return RFUL_EV_CHILD_NI_COMPLETE;
        }
        return RFUL_EV_CHILD_NI;
    }

    /* RfuMain2_Parent strips childSendCmdId before publishing via gRecvCmds. */
    if (l->state != RFUL_UNI || !rec.have_cmd) return RFUL_EV_UNI_EARLY;
    memcpy(l->child_cmd, rec.cmd, RFU_SLOT_LEN);
    l->child_cmd[0] &= RFU_FRAG_INDEX_MASK;
    echo_append(l, l->child_cmd);
    l->uni_in++;
    return RFUL_EV_UNI;
}

size_t rfu_leader_tick(rfu_leader *l, const uint8_t *parent_cmd, uint8_t out[RFUL_FRAME_MAX]) {
    if (l->p_len) {
        rful_frame *f = &l->pending[l->p_head];
        l->p_head = (l->p_head + 1) % RFUL_PENDING_MAX;
        l->p_len--;
        memcpy(out, f->b, f->len);
        return f->len;
    }
    if (l->state == RFUL_PARENT_NI) {
        /* Re-present the current NI sub-frame every VBlank until the child mirrors it. */
        if (l->parent_waiting) return wrap_t(l, l->pcur_slot, l->pcur_len, out);
        if (l->pni_i < l->pni_len) {
            const rful_ni_step *s = &l->pni[l->pni_i++];
            uint8_t slot[16];
            parent_llsf(s->state, s->size, 0, s->n, s->phase, l->bm_slot, slot);
            memcpy(slot + 3, s->payload, s->size);
            size_t n = wrap_t(l, slot, 3u + s->size, out);
            if (s->state == RFU_LCOM_NULL) {
                l->state = RFUL_UNI;          /* the terminal NULL is not acked */
            } else {
                l->parent_waiting = 1;
                l->pw_state = s->state; l->pw_n = s->n; l->pw_phase = s->phase;
                memcpy(l->pcur_slot, slot, 3u + s->size);
                l->pcur_len = (uint8_t)(3u + s->size);
            }
            return n;
        }
    }
    if (l->state == RFUL_UNI) {
        uint8_t slot[PARENT_FRAME_SIZE + COMM_TABLE_LEN] = {0};
        parent_llsf(RFU_LCOM_UNI, COMM_TABLE_LEN, 0, 0, 0, l->bm_slot, slot);
        if (parent_cmd) memcpy(slot + PARENT_FRAME_SIZE, parent_cmd, RFU_SLOT_LEN);
        memcpy(slot + PARENT_FRAME_SIZE + RFU_SLOT_LEN, echo_next_row(l), RFU_SLOT_LEN);
        l->uni_out++;
        return wrap_t(l, slot, sizeof slot, out);
    }
    return 0;
}

size_t rfu_leader_disconnect_frame(rfu_leader *l, uint8_t out[8]) {
    if (!l->have_connect) return 0;
    l->state = RFUL_DISCONNECTED;
    l->p_len = 0;
    return gba_build_disconnect(l->connect_id, out);
}

void rfu_leader_on_ldn_leave(rfu_leader *l) {
    l->state = RFUL_DISCONNECTED;
    l->p_len = 0;
}
