/* SPDX-License-Identifier: AGPL-3.0-only
 * Derived from frlg-ldn-trade (https://github.com/tornadus/frlg-ldn-trade), Copyright (c) the frlg-ldn-trade authors.
 * Changes Copyright (c) 2026 Kizuna. See NOTICE.
 */
/* C port of frlgsim/reliable.py — Pia message tiling, Reliable(10) sub-header, and the
 * selective-repeat sliding window. Verified byte/behaviour-identical to the Python oracle by
 * test_reliable.c. Times in ms. Defaults console-faithful. */
#include "reliable.h"
#include <string.h>

/* ---- helpers ---- */
static int seq_lt(uint16_t a, uint16_t b) {   /* a<b in 16-bit wrap space (RFC1982) */
    uint16_t d = (uint16_t)(b - a);
    return d != 0 && d < 0x8000;
}
static void put_be16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)(v >> 8); p[1] = (uint8_t)v; }
static uint16_t get_be16(const uint8_t *p) { return (uint16_t)((p[0] << 8) | p[1]); }
static int mask_nonzero(const uint8_t *m) { for (int i = 0; i < 16; i++) if (m[i]) return 1; return 0; }
static int mask_bit(const uint8_t *m, int i) { return (m[i >> 3] >> (i & 7)) & 1; }

/* ---- serialization ---- */
int rel_build_message(uint8_t proto, const uint8_t *payload, int plen, int msgflags, uint8_t *out) {
    int n = 0;
    uint8_t flags = 0x02 | 0x04;
    if (msgflags >= 0) flags |= 0x01;
    out[n++] = flags;
    if (msgflags >= 0) out[n++] = (uint8_t)msgflags;
    put_be16(out + n, (uint16_t)plen); n += 2;
    out[n++] = proto;
    if (plen) { memcpy(out + n, payload, plen); n += plen; }
    return n;
}

int rel_build_reliable(uint16_t seq, uint16_t ack, const uint8_t *inner, int inner_len,
                       uint8_t flagsA, uint8_t *out) {
    int n = 0;
    out[n++] = flagsA;
    put_be16(out + n, (uint16_t)inner_len); n += 2;
    put_be16(out + n, seq); n += 2;
    put_be16(out + n, ack); n += 2;
    out[n++] = 0x00;
    if (inner_len) { memcpy(out + n, inner, inner_len); n += inner_len; }
    return n;
}

int rel_build_bulk_ack(uint16_t next_expected, const uint8_t mask[16], uint8_t stream_id, uint8_t *out) {
    out[0] = stream_id; out[1] = 1;
    put_be16(out + 2, next_expected);
    if (mask) memcpy(out + 4, mask, 16); else memset(out + 4, 0, 16);
    return 20;
}

int rel_parse_bulk_ack(const uint8_t *payload, int len, uint16_t *ack_id, uint8_t mask[16]) {
    if (len < 4) return -1;
    *ack_id = get_be16(payload + 2);
    memset(mask, 0, 16);
    int m = len - 4; if (m > 16) m = 16; if (m > 0) memcpy(mask, payload + 4, m);
    return 0;
}

int rel_parse_reliable(const uint8_t *payload, int len, rel_reliable_t *out) {
    if (len < 8) return -1;
    int ln = get_be16(payload + 1);
    out->flagsA = payload[0];
    out->seq = get_be16(payload + 3);
    out->ack = get_be16(payload + 5);
    out->payload = payload + 8;
    int avail = len - 8;
    out->payload_len = ln < avail ? ln : avail;
    return 0;
}

int rel_build_app(const uint8_t *msgs, int msgs_len, uint16_t station_id, int pad, uint8_t *out) {
    int n = 0;
    if (msgs_len) { memcpy(out, msgs, msgs_len); n = msgs_len; }
    put_be16(out + n, station_id); n += 2;
    for (int i = 0; i < pad; i++) out[n++] = 0xff;
    return n;
}

int rel_parse_messages(const uint8_t *data, int n, rel_message_t *msgs, int max_msgs, int *consumed) {
    int i = 0, count = 0;
    int mf = 0, size = -1, proto = -1;
    while (i < n) {
        uint8_t fl = data[i];
        if (fl == 0xff || (fl & 0xF0)) break;
        if (fl == 0 && size < 0) break;
        int hdr_start = i, j = i + 1;
        if (fl & 1) { if (j >= n) break; mf = data[j]; j += 1; }
        if (fl & 2) { if (j + 2 > n) break; size = get_be16(data + j); j += 2; }
        if (fl & 4) { if (j >= n) break; proto = data[j]; j += 1; }
        if (fl & 8) { if (j + 1 > n) break; j += 1; }   /* 1-byte dest port (6.32+ format) */
        if (size < 0 || j + size > n) break;
        if (count < max_msgs) {
            msgs[count].flags = fl; msgs[count].msgflags = (uint8_t)mf;
            msgs[count].proto = proto; msgs[count].size = size;
            msgs[count].payload = data + j; msgs[count].hdr_len = j - hdr_start;
        }
        count++;
        i = j + size;
    }
    if (consumed) *consumed = i;
    /* Never report more messages than were actually stored — the loop above keeps counting past
     * max_msgs for `consumed` accuracy, but returning count>max_msgs makes callers iterate past the
     * end of their msgs[] array and dereference garbage (LoadProhibited on m->payload). Real trade
     * datagrams tile into more messages than the handshake did, which is why this only bit past
     * host_accepted. */
    return count > max_msgs ? max_msgs : count;
}

int rel_parse_app(const uint8_t *data, int n, rel_message_t *msgs, int max_msgs,
                  int *station, int *tail_len) {
    int consumed = 0;
    int count = rel_parse_messages(data, n, msgs, max_msgs, &consumed);
    int tl = n - consumed;
    /* tail = data[consumed:]; strip trailing 0xff; a 2-byte remainder is the BE station id */
    int end = n;
    while (end > consumed && data[end - 1] == 0xff) end--;
    if (station) *station = (end - consumed == 2) ? get_be16(data + consumed) : -1;
    if (tail_len) *tail_len = tl;
    return count;
}

/* ---- ReliableLink ---- */
void rel_link_init(rel_link_t *L, uint16_t start) {
    memset(L, 0, sizeof(*L));
    L->out_seq = start; L->window_lo = start; L->recv_next = start;
    L->dup_nack_threshold = 1;
    L->rto_backoff = 1.0;
    /* has_rto_ceil / has_rto_bootstrap default 0 (console: None) */
}

static rel_entry_t *find_entry(rel_link_t *L, uint16_t seq) {
    for (int k = 0; k < REL_MAX_INFLIGHT; k++)
        if (L->unacked[k].used && L->unacked[k].seq == seq) return &L->unacked[k];
    return NULL;
}

int rel_inflight(const rel_link_t *L) {
    int c = 0; for (int k = 0; k < REL_MAX_INFLIGHT; k++) if (L->unacked[k].used) c++; return c;
}

/* Python ReliableLink.outstanding(): unacked entries not yet peer-acked. */
int rel_outstanding(const rel_link_t *L) {
    int c = 0; for (int k = 0; k < REL_MAX_INFLIGHT; k++)
        if (L->unacked[k].used && !L->unacked[k].acked) c++;
    return c;
}

/* Carry-forward eligibility: an entry for `seq` exists and is not yet acked. */
int rel_entry_active(const rel_link_t *L, uint16_t seq) {
    for (int k = 0; k < REL_MAX_INFLIGHT; k++)
        if (L->unacked[k].used && L->unacked[k].seq == seq) return !L->unacked[k].acked;
    return 0;
}

/* Const lookup of an unacked entry by seq (NULL if absent). For carry-forward inner bytes. */
const rel_entry_t *rel_find(const rel_link_t *L, uint16_t seq) {
    for (int k = 0; k < REL_MAX_INFLIGHT; k++)
        if (L->unacked[k].used && L->unacked[k].seq == seq) return &L->unacked[k];
    return NULL;
}

uint16_t rel_send_low(const rel_link_t *L) {
    return rel_inflight(L) ? L->window_lo : L->out_seq;
}

uint16_t rel_queue(rel_link_t *L, const uint8_t *inner, int inner_len, uint8_t flagsA, uint32_t now_ms) {
    uint16_t seq = L->out_seq;
    L->out_seq = (uint16_t)(L->out_seq + 1);
    for (int k = 0; k < REL_MAX_INFLIGHT; k++) {
        if (!L->unacked[k].used) {
            rel_entry_t *e = &L->unacked[k];
            e->used = 1; e->seq = seq; e->flagsA = flagsA;
            if (inner_len > REL_MAXINNER) inner_len = REL_MAXINNER;
            e->inner_len = inner_len; if (inner_len) memcpy(e->inner, inner, inner_len);
            e->last_tx = now_ms; e->resends = 0; e->acked = 0;
            break;
        }
    }
    return seq;
}

void rel_add_rtt_sample(rel_link_t *L, double rtt_ms) {
    if (rtt_ms < 0) return;
    if (L->rtt_n < REL_RTT_WINDOW) {
        L->rtt[L->rtt_n++] = rtt_ms;
    } else {
        for (int k = 1; k < REL_RTT_WINDOW; k++) L->rtt[k - 1] = L->rtt[k];
        L->rtt[REL_RTT_WINDOW - 1] = rtt_ms;
    }
}

static double dmedian(const double *a, int n) {
    double s[REL_RTT_WINDOW];
    memcpy(s, a, n * sizeof(double));
    for (int i = 0; i < n; i++) for (int j = i + 1; j < n; j++) if (s[j] < s[i]) { double t = s[i]; s[i] = s[j]; s[j] = t; }
    return s[n / 2];
}

int rel_rto(const rel_link_t *L, double *out_rto) {
    double rto;
    if (L->rtt_n > 0) {
        double median = dmedian(L->rtt, L->rtt_n);
        rto = 33.0 + 1.4 * median;
        if (L->rtt_jitter_k) {
            double mad = 0; for (int i = 0; i < L->rtt_n; i++) { double d = L->rtt[i] - median; mad += d < 0 ? -d : d; }
            mad /= L->rtt_n; rto += L->rtt_jitter_k * mad;
        }
    } else if (L->has_rto_bootstrap) {
        rto = L->rto_bootstrap_ms;
    } else {
        return 0;   /* None */
    }
    if (L->has_rto_ceil && rto > L->rto_ceil_ms) rto = L->rto_ceil_ms;
    *out_rto = rto;
    return 1;
}

void rel_on_ack(rel_link_t *L, uint16_t ack_id, const uint8_t mask[16], int has_now, uint32_t now_ms) {
    uint8_t zero[16] = {0};
    const uint8_t *m = mask ? mask : zero;
    int mnz = mask_nonzero(m);
    for (int k = 0; k < REL_MAX_INFLIGHT; k++) {
        rel_entry_t *e = &L->unacked[k];
        if (!e->used) continue;
        int arrived = seq_lt(e->seq, ack_id);
        if (!arrived && mnz) {
            uint16_t i = (uint16_t)(e->seq - ack_id - 1);
            arrived = i < 128 && mask_bit(m, i);
        }
        if (arrived && !e->acked) {
            if (has_now && e->resends == 0) rel_add_rtt_sample(L, (double)now_ms - (double)e->last_tx);
            e->acked = 1;
        }
    }
    /* advance window base over the contiguous acked run only */
    rel_entry_t *e;
    while ((e = find_entry(L, L->window_lo)) != NULL && e->acked) {
        e->used = 0;
        L->window_lo = (uint16_t)(L->window_lo + 1);
    }
    /* peer-gap dup-NACK tracking */
    rel_entry_t *ge = find_entry(L, ack_id);
    if (mnz && ge && !ge->acked) {
        if (L->has_peer_gap && L->peer_gap == ack_id) {
            L->gap_nacks += 1;
        } else {
            L->has_peer_gap = 1; L->peer_gap = ack_id; L->gap_nacks = 1;
        }
    } else {
        L->has_peer_gap = 0; L->gap_nacks = 0;
    }
}

/* sort helper: order used unacked indices by (seq-window_lo)&0xffff */
static int cmp_key(const rel_link_t *L, uint16_t seq) { return (uint16_t)(seq - L->window_lo); }

int rel_due_retransmits(rel_link_t *L, uint32_t now_ms, int limit, rel_due_t *out, int max_out) {
    double base_rto; int have_rto = rel_rto(L, &base_rto);
    /* collect + sort used indices by gap key */
    int idx[REL_MAX_INFLIGHT], ni = 0;
    for (int k = 0; k < REL_MAX_INFLIGHT; k++) if (L->unacked[k].used) idx[ni++] = k;
    for (int i = 0; i < ni; i++) for (int j = i + 1; j < ni; j++)
        if (cmp_key(L, L->unacked[idx[j]].seq) < cmp_key(L, L->unacked[idx[i]].seq)) { int t = idx[i]; idx[i] = idx[j]; idx[j] = t; }
    int nout = 0;
    for (int ii = 0; ii < ni; ii++) {
        rel_entry_t *e = &L->unacked[idx[ii]];
        if (e->acked) continue;
        int due;
        if (L->has_peer_gap && e->seq == L->peer_gap && e->resends == 0 && L->gap_nacks >= L->dup_nack_threshold) {
            due = 1;
        } else if (!have_rto) {
            due = 0;
        } else {
            double eff = base_rto;
            for (int r = 0; r < e->resends; r++) eff *= L->rto_backoff;
            if (L->has_rto_ceil && eff > L->rto_ceil_ms) eff = L->rto_ceil_ms;
            due = ((double)now_ms - (double)e->last_tx) >= eff;
        }
        if (due) {
            e->last_tx = now_ms; e->resends += 1;
            if (nout < max_out) { out[nout].seq = e->seq; out[nout].flagsA = e->flagsA; out[nout].inner = e->inner; out[nout].inner_len = e->inner_len; }
            nout++;
            if (limit >= 0 && nout >= limit) break;
        } else if (limit >= 0) {
            break;   /* gap-targeted: stop at first not-yet-due */
        }
    }
    return nout;
}

static rel_recv_t *find_recvbuf(rel_link_t *L, uint16_t seq) {
    for (int k = 0; k < REL_MAX_INFLIGHT; k++) if (L->recv_buf[k].used && L->recv_buf[k].seq == seq) return &L->recv_buf[k];
    return NULL;
}

int rel_on_data(rel_link_t *L, uint16_t seq, const uint8_t *payload, int plen,
                rel_recv_t *deliv, int max_deliv) {
    int nd = 0;
    if (plen > REL_MAXINNER) plen = REL_MAXINNER;
    if (seq == L->recv_next) {
        if (nd < max_deliv) { deliv[nd].len = plen; memcpy(deliv[nd].payload, payload, plen); }
        nd++;
        L->recv_next = (uint16_t)(L->recv_next + 1);
        rel_recv_t *b;
        while ((b = find_recvbuf(L, L->recv_next)) != NULL) {
            if (nd < max_deliv) { deliv[nd].len = b->len; memcpy(deliv[nd].payload, b->payload, b->len); }
            nd++;
            b->used = 0;
            L->recv_next = (uint16_t)(L->recv_next + 1);
        }
        return nd;
    }
    if (seq_lt(L->recv_next, seq) && !find_recvbuf(L, seq)) {
        for (int k = 0; k < REL_MAX_INFLIGHT; k++) if (!L->recv_buf[k].used) {
            L->recv_buf[k].used = 1; L->recv_buf[k].seq = seq; L->recv_buf[k].len = plen;
            memcpy(L->recv_buf[k].payload, payload, plen); break;
        }
    }
    return 0;
}

static int ooo_has(rel_link_t *L, uint16_t seq) {
    for (int k = 0; k < L->recv_ooo_n; k++) if (L->recv_ooo[k] == seq) return 1;
    return 0;
}
static void ooo_del(rel_link_t *L, uint16_t seq) {
    for (int k = 0; k < L->recv_ooo_n; k++) if (L->recv_ooo[k] == seq) {
        L->recv_ooo[k] = L->recv_ooo[--L->recv_ooo_n]; return;
    }
}

void rel_note_received(rel_link_t *L, uint16_t seq) {
    if (seq == L->recv_next) {
        L->recv_next = (uint16_t)(L->recv_next + 1);
        while (ooo_has(L, L->recv_next)) { ooo_del(L, L->recv_next); L->recv_next = (uint16_t)(L->recv_next + 1); }
    } else if (seq_lt(L->recv_next, seq)) {
        if (!ooo_has(L, seq) && L->recv_ooo_n < REL_MAX_INFLIGHT) L->recv_ooo[L->recv_ooo_n++] = seq;
    }
}

int rel_ack_payload(rel_link_t *L, uint8_t *out) {
    uint8_t mask[16] = {0};
    for (int k = 0; k < L->recv_ooo_n; k++) {
        uint16_t i = (uint16_t)(L->recv_ooo[k] - L->recv_next - 1);
        if (i < 128) mask[i >> 3] |= (uint8_t)(1 << (i & 7));
    }
    return rel_build_bulk_ack(L->recv_next, mask, 0, out);
}
