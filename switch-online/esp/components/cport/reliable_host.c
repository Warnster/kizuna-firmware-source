/* SPDX-License-Identifier: AGPL-3.0-only
 * Derived from pokeldn (https://github.com/Decryptu/pokeldn), Copyright (c) the pokeldn authors.
 * Changes Copyright (c) 2026 Kizuna. See NOTICE.
 */
/* reliable_host.c — portable C port of pokeldn ldn/reliable.py HostReliableSession + ReliableLink.
 * See reliable_host.h. */
#include "reliable_host.h"
#include "reliable.h"
#include <string.h>

const relh_cfg RELH_DEFAULT_CFG = {
    .ack_period_ms = RELH_RTO_BASE_MS, .max_inflight = RELH_WINDOW,
    .has_rto_bootstrap = 1, .rto_bootstrap_ms = 200.0, .dup_nack_threshold = 1, .retransmit_limit = -1,
};

static int seq_lt(uint16_t a, uint16_t b) { uint16_t d = (uint16_t)(b - a); return d != 0 && d < 0x8000; }
static uint16_t rd16(const uint8_t *p) { return (uint16_t)((p[0] << 8) | p[1]); }

int relh_message_flags(const relh_emission *e) {
    if (e->flagsA == RELH_FLAGSA_CTRL) return 0x40;
    return e->retransmitted ? 0x20 : -1;
}

int relh_serialize(const relh_emission *e, uint8_t *out) {
    return rel_build_reliable(e->seq, e->ack, e->payload, e->len, e->flagsA, out);
}

void relh_init(relh_t *r, const relh_cfg *cfg) {
    memset(r, 0, sizeof(*r));
    r->cfg = cfg ? *cfg : RELH_DEFAULT_CFG;
    if (r->cfg.max_inflight <= 0 || r->cfg.max_inflight > RELH_WINDOW) r->cfg.max_inflight = RELH_WINDOW;
    if (r->cfg.dup_nack_threshold < 1) r->cfg.dup_nack_threshold = 1;
    r->start_seq = RELH_START;
    r->out_seq = r->window_lo = r->recv_next = RELH_START;
}

int relh_inflight(const relh_t *r) { return (uint16_t)(r->out_seq - r->window_lo); }

static relh_tx *at(relh_t *r, int k) { return &r->tx[(r->head + k) % RELH_WINDOW]; }

int relh_outstanding(const relh_t *r) {
    int n = 0, inflight = relh_inflight(r);
    for (int k = 0; k < inflight; k++) if (!r->tx[(r->head + k) % RELH_WINDOW].acked) n++;
    return n;
}

uint16_t relh_send_low(const relh_t *r) { return relh_inflight(r) ? r->window_lo : r->out_seq; }

const relh_tx *relh_find(const relh_t *r, uint16_t seq) {
    int k = (uint16_t)(seq - r->window_lo);
    if (k >= relh_inflight(r)) return NULL;
    return &r->tx[(r->head + k) % RELH_WINDOW];
}

static void emission(relh_t *r, uint16_t seq, const relh_tx *e, int retransmitted, relh_emission *out) {
    out->seq = seq; out->flagsA = e->flagsA; out->ack = relh_send_low(r);
    out->retransmitted = (uint8_t)retransmitted;
    out->payload = e->inner; out->len = e->len;
}

/* ReliableLink.queue. The caller has checked the window (inflight < max_inflight <= RELH_WINDOW). */
static uint16_t queue(relh_t *r, const uint8_t *p, int len, uint8_t flagsA, double now_ms) {
    uint16_t seq = r->out_seq;
    relh_tx *e = at(r, relh_inflight(r));
    if (len > RELH_INNER_MAX) len = RELH_INNER_MAX;
    e->flagsA = flagsA; e->len = (uint8_t)len; e->acked = 0; e->resends = 0; e->last_tx = now_ms;
    if (len) memcpy(e->inner, p, (size_t)len);
    r->out_seq = (uint16_t)(r->out_seq + 1);
    return seq;
}

int relh_open(relh_t *r, const uint8_t *payload, int len, double now_ms, uint8_t flagsA, relh_emission *out) {
    if (r->local_opened || relh_inflight(r) >= RELH_WINDOW) return 0;
    flagsA = (uint8_t)(flagsA | RELH_FLAGSA_INIT);
    uint16_t seq = queue(r, payload, len, flagsA, now_ms);
    r->local_opened = 1;
    emission(r, seq, relh_find(r, seq), 0, out);
    return 1;
}

int relh_send(relh_t *r, const uint8_t *payload, int len, double now_ms, uint8_t flagsA, relh_emission *out) {
    if (!r->local_opened) return -1;
    if (relh_inflight(r) >= r->cfg.max_inflight) return -2;
    if (!(flagsA & 0x01)) return -3;
    uint16_t seq = queue(r, payload, len, flagsA, now_ms);
    emission(r, seq, relh_find(r, seq), 0, out);
    return 1;
}

void relh_note_rtt(relh_t *r, double rtt_ms) {
    if (rtt_ms < 0) return;
    if (r->rtt_n == RELH_RTT_WINDOW) { memmove(r->rtt, r->rtt + 1, (RELH_RTT_WINDOW - 1) * sizeof(double)); r->rtt_n--; }
    r->rtt[r->rtt_n++] = rtt_ms;
}

static double median(const relh_t *r) {
    double s[RELH_RTT_WINDOW];
    int n = r->rtt_n;
    memcpy(s, r->rtt, (size_t)n * sizeof(double));
    for (int i = 1; i < n; i++) { double v = s[i]; int j = i - 1; while (j >= 0 && s[j] > v) { s[j + 1] = s[j]; j--; } s[j + 1] = v; }
    return s[n / 2];
}

/* BASE + FACTOR * median(RTT); the bootstrap before a sample; 0 = None. */
static int rto(const relh_t *r, double *out) {
    if (r->rtt_n) { *out = RELH_RTO_BASE_MS + RELH_RTO_RTT_FACTOR * median(r); return 1; }
    if (r->cfg.has_rto_bootstrap) { *out = r->cfg.rto_bootstrap_ms; return 1; }
    return 0;
}

static double fast_retx_gap(const relh_t *r) {
    if (!r->rtt_n) return RELH_FAST_RETX_MIN_MS;
    double m = median(r);
    return m > RELH_FAST_RETX_MIN_MS ? m : RELH_FAST_RETX_MIN_MS;
}

/* ReliableLink.on_ack: the window base advances over the contiguous acked run only; a first ack
 * of an unretransmitted frame is an RTT sample, mask included. Bit i of the mask is ack_id+i+1. */
static void on_ack(relh_t *r, uint16_t ack_id, const uint8_t mask[16], double now_ms) {
    int masked = 0, inflight = relh_inflight(r);
    for (int i = 0; i < 16; i++) if (mask[i]) masked = 1;
    for (int k = 0; k < inflight; k++) {
        relh_tx *e = at(r, k);
        uint16_t seq = (uint16_t)(r->window_lo + k);
        int arrived = seq_lt(seq, ack_id);
        if (!arrived && masked) {
            uint16_t i = (uint16_t)(seq - ack_id - 1);
            arrived = i < 128 && ((mask[i >> 3] >> (i & 7)) & 1);
        }
        if (arrived && !e->acked) {
            if (e->resends == 0) relh_note_rtt(r, now_ms - e->last_tx);
            e->acked = 1;
        }
    }
    while (relh_inflight(r) && at(r, 0)->acked) {
        r->head = (r->head + 1) % RELH_WINDOW;
        r->window_lo = (uint16_t)(r->window_lo + 1);
    }
    const relh_tx *gap = relh_find(r, ack_id);
    if (masked && gap && !gap->acked) {
        if (r->has_peer_gap && r->peer_gap == ack_id) r->gap_nacks++;
        else { r->has_peer_gap = 1; r->peer_gap = ack_id; r->gap_nacks = 1; }
    } else {
        r->has_peer_gap = 0; r->gap_nacks = 0;
    }
}

/* Out-of-order store: the slot holding seq, or NULL. */
static relh_rx *rx_find(relh_t *r, uint16_t seq) {
    for (int i = 0; i < RELH_RX_SLOTS; i++) if (r->rx[i].used && r->rx[i].seq == seq) return &r->rx[i];
    return NULL;
}

int relh_receive(relh_t *r, const uint8_t *wire, int len, double now_ms, relh_deliver_fn deliver, void *ctx) {
    if (len < 8) return 0;
    int ln = rd16(wire + 1);
    if (ln > len - 8) return 0;
    uint8_t flagsA = wire[0];
    uint16_t seq = rd16(wire + 3);
    const uint8_t *p = wire + 8;

    if (!(flagsA & 0x01)) {
        uint16_t ack_id; uint8_t mask[16];
        if (rel_parse_bulk_ack(p, ln, &ack_id, mask) == 0) on_ack(r, ack_id, mask, now_ms);
        return 0;
    }
    /* The FFF0 opening DATA must carry Initialized; a plain frame taking that slot makes a later
     * INIT look like a duplicate. */
    if (!r->peer_opened && seq == r->start_seq && !(flagsA & 0x08)) return 0;
    int ahead = seq_lt(r->recv_next, seq);
    if (ahead && !rx_find(r, seq) && (ln > RELH_RX_INNER || r->rx_n == RELH_RX_SLOTS)) {
        r->rx_unheld++;              /* no room to wait: unacked, so the peer resends it */
        return 0;
    }
    if (flagsA & 0x08) r->peer_opened = 1;
    r->ack_owed = 1;
    if (!r->has_next_ack) { r->has_next_ack = 1; r->next_ack_ms = now_ms + r->cfg.ack_period_ms; }

    if (seq == r->recv_next) {
        int n = 1;
        r->recv_next = (uint16_t)(r->recv_next + 1);
        if (deliver) deliver(ctx, seq, flagsA, p, ln);
        relh_rx *b;
        while ((b = rx_find(r, r->recv_next)) != NULL) {
            b->used = 0; r->rx_n--;
            r->recv_next = (uint16_t)(r->recv_next + 1);
            if (deliver) deliver(ctx, b->seq, b->flagsA, b->payload, b->len);
            n++;
        }
        return n;
    }
    if (ahead && !rx_find(r, seq)) {
        for (int i = 0; i < RELH_RX_SLOTS; i++) if (!r->rx[i].used) {
            relh_rx *b = &r->rx[i];
            b->used = 1; b->seq = seq; b->flagsA = flagsA; b->len = (uint8_t)ln;
            memcpy(b->payload, p, (size_t)ln);
            r->rx_n++;
            break;
        }
    }
    return 0;
}

int relh_poll(relh_t *r, double now_ms, relh_emission *out, int max_out) {
    int n = 0, limit = r->cfg.retransmit_limit, inflight = relh_inflight(r);
    double base; int have_rto = rto(r, &base);
    /* due_retransmits: oldest first; `limit` also stops at the first frame not yet due */
    for (int k = 0; k < inflight && n < max_out; k++) {
        relh_tx *e = at(r, k);
        uint16_t seq = (uint16_t)(r->window_lo + k);
        int due;
        if (e->acked) continue;
        if (r->has_peer_gap && seq == r->peer_gap && r->gap_nacks >= r->cfg.dup_nack_threshold
            && (e->resends == 0 || now_ms - e->last_tx >= fast_retx_gap(r))) {
            due = 1;
            r->gap_nacks = 0;
        } else if (!have_rto) {
            due = 0;
        } else {
            due = now_ms - e->last_tx >= base;   /* backoff 1.0, no ceiling */
        }
        if (due) {
            e->last_tx = now_ms;
            e->resends++;
            emission(r, seq, e, 1, &out[n++]);
            if (limit >= 0 && n >= limit) break;
        } else if (limit >= 0) {
            break;
        }
    }
    int ack_due = r->has_next_ack && now_ms >= r->next_ack_ms;
    if (ack_due && (r->ack_owed || r->rx_n) && n < max_out) {
        uint8_t mask[16] = {0};
        for (int i = 0; i < RELH_RX_SLOTS; i++) if (r->rx[i].used) {
            uint16_t b = (uint16_t)(r->rx[i].seq - r->recv_next - 1);
            if (b < 128) mask[b >> 3] |= (uint8_t)(1 << (b & 7));
        }
        rel_build_bulk_ack(r->recv_next, mask, 0, r->ack_buf);
        relh_emission *a = &out[n++];
        a->seq = r->start_seq; a->flagsA = RELH_FLAGSA_CTRL; a->ack = relh_send_low(r);
        a->retransmitted = 0; a->payload = r->ack_buf; a->len = 20;
        r->ack_owed = 0;
        /* keep NACKing a persistent gap at the delayed-ack cadence */
        if (r->rx_n) r->next_ack_ms = now_ms + r->cfg.ack_period_ms;
        else r->has_next_ack = 0;
    }
    return n;
}
