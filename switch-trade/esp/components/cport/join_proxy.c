/* join_proxy.c — see join_proxy.h. */
#include "join_proxy.h"
#include <string.h>

#define LD(p)     __atomic_load_n((p), __ATOMIC_ACQUIRE)
#define ST(p, v)  __atomic_store_n((p), (v), __ATOMIC_RELEASE)
#define BIT(c)    (1u << (c))

#define OP_FEED 0xF0
#define OP_TICK 0xF1
#define OP_PSD  0xF2

void jp_init(jp_activity *jp, jp_send_fn send, void *send_ctx)
{
    memset(jp, 0, sizeof *jp);
    jp->send = send;
    jp->send_ctx = send_ctx;
    jp->flush_ms = JP_FLUSH_MS_DEFAULT;
    jp->match_itvl = true;
}

void jp_session_begin(jp_activity *jp)
{
    jp->ops_head = jp->ops_tail = 0;
    jp->feed_seq = jp->tick_next_idx = jp->sent_feed_seq = jp->sent_tick_idx = 0;
    jp->credits_sent = 0;
    jp->ctrl_pending = BIT(JP_CTRL_JOIN_HELLO);
    jp->hello_sent = jp->retry = jp->primed = jp->uni_seen = false;
    jp->prefill_wait = 0;
    jp->stall_ticks = 0;
    jp->played_idx = 0;
    ST(&jp->down_head, 0); ST(&jp->down_tail, 0); ST(&jp->flags, 0); ST(&jp->down_started, 0);
    jp->down_expected = 0;
    ST(&jp->down_lost, 0);
}

void jp_set_trace(jp_activity *jp, jp_trace_fn fn, void *ctx, uint32_t host_n, uint32_t child_n)
{
    jp->trace = fn; jp->trace_ctx = ctx;
    jp->trace_host_left = fn ? host_n : 0;
    jp->trace_child_left = fn ? child_n : 0;
}

void jp_set_flush_ms(jp_activity *jp, uint32_t ms) { jp->flush_ms = ms > 1000 ? 1000 : ms; }
void jp_set_link_itvl_ms(jp_activity *jp, uint32_t ms) { jp->link_itvl_ms = ms > 4000 ? 4000 : ms; }
void jp_set_match_itvl(jp_activity *jp, bool on) { jp->match_itvl = on; }

uint32_t jp_effective_flush_ms(const jp_activity *jp)
{
    uint32_t base = jp->flush_ms;
    if (jp->match_itvl && jp->link_itvl_ms > base) base = jp->link_itvl_ms;
    return base + jp->flush_extra;
}

bool jp_flag(const jp_activity *jp, uint8_t code) { return code < 32 && (LD(&jp->flags) & BIT(code)) != 0; }
int  jp_down_depth(const jp_activity *jp) { return (int)(LD(&jp->down_head) - LD(&jp->down_tail)); }

void jp_ctrl(jp_activity *jp, uint8_t code)
{
    if (code >= 2 && code <= JP_CTRL_MAX) jp->ctrl_pending |= BIT(code);
}

/* ---- BLE task side ---- */

static void down_rows(jp_activity *jp, uint16_t seq, unsigned count, const uint8_t *rows)
{
    for (unsigned r = 0; r < count; r++) {
        uint16_t idx = (uint16_t)(seq + r);
        if (LD(&jp->down_started)) {
            int16_t diff = (int16_t)(idx - jp->down_expected);
            if (diff < 0) { jp->dups++; continue; }
            if (diff > 0) { jp->gaps++; __atomic_fetch_add(&jp->down_lost, (uint32_t)diff, __ATOMIC_RELEASE); }
        }
        jp->down_expected = (uint16_t)(idx + 1);
        uint32_t head = jp->down_head;
        const uint8_t *row = rows + r * JP_ROW_LEN;
        if (head - LD(&jp->down_tail) >= JP_DOWN_SHED_AT) {
            bool idle = true;
            for (int i = 0; i < JP_ROW_LEN; i++) if (row[i]) { idle = false; break; }
            if (idle) {
                jp->idle_shed++;
                __atomic_fetch_add(&jp->down_lost, 1u, __ATOMIC_RELEASE);
                ST(&jp->down_started, 1);
                continue;
            }
        }
        if (head - LD(&jp->down_tail) >= JP_DOWN_RING) { jp->down_overflow++; ST(&jp->down_started, 1); continue; }
        memcpy(jp->down[head % JP_DOWN_RING], row, JP_ROW_LEN);
        ST(&jp->down_head, head + 1);
        ST(&jp->down_started, 1);
        jp->down_rows++;
    }
}

bool jp_on_data(jp_activity *jp, const uint8_t *d, size_t len)
{
    if (len == 4 || len == 0) return false;
    if (d[0] != JP_MSG_JDOWN && d[0] != JP_MSG_CTRL) return false;
    jp->bundles_rx++;
    if (len > JP_BUNDLE_MAX) jp->oversize_rx++;
    size_t off = 0;
    while (off < len) {
        const uint8_t *m = d + off;
        size_t left = len - off;
        if (m[0] == JP_MSG_CTRL) {
            if (left < 5 || m[1] < 1 || m[1] > JP_CTRL_MAX || m[2] || m[3] || m[4]) { jp->bad_msgs++; break; }
            __atomic_fetch_or(&jp->flags, BIT(m[1]), __ATOMIC_RELEASE);
            jp->ctrl_rx++;
            off += 5;
        } else if (m[0] == JP_MSG_JDOWN) {
            unsigned count = left >= 4 ? m[3] : 0;
            if (left < 4 || count < 1 || count > JP_MAX_DOWN || left < 4 + count * JP_ROW_LEN) { jp->bad_msgs++; break; }
            down_rows(jp, (uint16_t)(m[1] | (m[2] << 8)), count, m + 4);
            off += 4 + count * JP_ROW_LEN;
        } else { jp->bad_msgs++; break; }
    }
    return true;
}

/* ---- brain task side: the op log ---- */

static unsigned ops_used(const jp_activity *jp) { return (unsigned)((jp->ops_head - jp->ops_tail + JP_OPS_BYTES) % JP_OPS_BYTES); }
static unsigned ops_free(const jp_activity *jp) { return JP_OPS_BYTES - 1 - ops_used(jp); }

static void ops_put(jp_activity *jp, const uint8_t *b, size_t n)
{
    for (size_t i = 0; i < n; i++) { jp->ops[jp->ops_head] = b[i]; jp->ops_head = (uint16_t)((jp->ops_head + 1) % JP_OPS_BYTES); }
}

static uint8_t ops_at(const jp_activity *jp, unsigned pos) { return jp->ops[(jp->ops_tail + pos) % JP_OPS_BYTES]; }

size_t jp_encode_record(const gba_in *rec, uint8_t *out)
{
    int n = rec->nslots < 0 ? 0 : rec->nslots > JP_MAX_SLOTS ? JP_MAX_SLOTS : rec->nslots;
    uint8_t mask = 0;
    size_t off = 2;
    for (int i = 0; i < n; i++) {
        bool nz = false;
        for (int k = 0; k < JP_ROW_LEN; k++) if (rec->slots[i][k]) { nz = true; break; }
        if (!nz) continue;
        mask |= (uint8_t)(1u << i);
        memcpy(out + off, rec->slots[i], JP_ROW_LEN);
        off += JP_ROW_LEN;
    }
    out[0] = (uint8_t)n; out[1] = mask;
    return off;
}

static void jp_feed(void *ctx, const gba_in *rec)
{
    jp_activity *jp = ctx;
    bool uni = rec->llsf_state == 4;
    bool keepalive = rec->slot_len <= 1;
    jp->host_frames++;
    jp->last_state = rec->llsf_state; jp->last_slot_len = rec->slot_len;
    if (!jp->uni_seen) {
        if (!uni) return;                                        /* the NI window is the sim's own business */
        jp->uni_seen = true;
        jp_ctrl(jp, JP_CTRL_JOIN_UNI_UP);
    } else if (!uni && !keepalive) {
        /* The host left UNI for something we do not proxy (NI/NULL after a close, ...): count it and record each CHANGE of shape. */
        jp->nonuni_drop++;
        int sig = (rec->llsf_state & 0xFF) | (rec->slot_len << 8) | (rec->has_ni << 16) | (rec->ni_ack << 17);
        if (jp->trace && sig != jp->last_drop_sig && jp->trace_child_left) {
            uint8_t b[12] = { (uint8_t)rec->llsf_state, rec->slot_len, (uint8_t)rec->has_ni, (uint8_t)rec->ni_ack, (uint8_t)rec->ni_n, (uint8_t)rec->ni_phase };
            for (int i = 0; i < 6 && i < rec->ni_payload_len; i++) b[6 + i] = rec->ni_payload[i];
            jp->trace(jp->trace_ctx, 'x', jp->feed_seq, b, sizeof b);
        }
        jp->last_drop_sig = sig;
        return;
    }
    uint8_t item[3 + 2 + JP_MAX_SLOTS * JP_ROW_LEN];
    item[0] = OP_FEED;
    size_t n = 1 + jp_encode_record(rec, item + 1);
    if (n > ops_free(jp)) { jp->ops_overflow++; return; }
    ops_put(jp, item, n);
    if (jp->trace && jp->trace_host_left && item[2]) {      /* mask != 0: a non-idle record */
        jp->trace_host_left--;
        jp->trace(jp->trace_ctx, 'h', jp->feed_seq, item + 1, n - 1);
    }
    jp->feed_seq++;
    jp->rec_fed++;
}

static uint32_t outstanding(const jp_activity *jp)
{
    uint32_t used = jp->down_tail + LD(&jp->down_lost);
    return jp->credits_sent > used ? jp->credits_sent - used : 0;
}

static void idle_row(uint16_t words[7]) { memset(words, 0, 7 * sizeof words[0]); }

static void stall_watch(jp_activity *jp)
{
    if (outstanding(jp) >= JP_CREDIT_WINDOW && ++jp->stall_ticks >= JP_CREDIT_STALL_TICKS) {
        __atomic_fetch_add(&jp->down_lost, outstanding(jp), __ATOMIC_RELEASE);
        jp->credit_resets++;
        jp->stall_ticks = 0;
    }
}

static void jp_tick(void *ctx, uint16_t words[7])
{
    jp_activity *jp = ctx;
    jp->ticks++;
    if (outstanding(jp) >= JP_CREDIT_WINDOW || ops_free(jp) < 1) {
        jp->tick_skipped++;                                       /* the remote cannot keep up: no credit, no index, an idle row below */
    } else {
        uint8_t op = OP_TICK;
        ops_put(jp, &op, 1);
        jp->tick_next_idx++;
        jp->credits_sent++;
    }

    if (!LD(&jp->down_started)) { idle_row(words); return; }
    uint32_t tail = jp->down_tail;
    uint32_t depth = LD(&jp->down_head) - tail;
    if (!jp->primed) {
        if (depth >= JP_PREFILL || (depth > 0 && jp->prefill_wait >= JP_PREFILL_MAX_WAIT)) {
            jp->primed = true;
            jp->prefill_wait = 0;
        } else {
            if (depth > 0) jp->prefill_wait++;
            jp->prefill_idle++;
            idle_row(words);
            if (depth == 0) stall_watch(jp);
            return;
        }
    }
    if (depth == 0) {
        jp->underruns++;
        jp->primed = false;
        jp->prefill_wait = 0;
        idle_row(words);
        stall_watch(jp);
        return;
    }
    jp->stall_ticks = 0;
    const uint8_t *r = jp->down[tail % JP_DOWN_RING];
    for (int i = 0; i < 7; i++) words[i] = (uint16_t)(r[2 * i] | (r[2 * i + 1] << 8));
    if (jp->trace && jp->trace_child_left) {
        bool nz = false;
        for (int i = 0; i < JP_ROW_LEN; i++) if (r[i]) { nz = true; break; }
        if (nz) { jp->trace_child_left--; jp->trace(jp->trace_ctx, 'c', jp->played_idx, r, JP_ROW_LEN); }
    }
    jp->played_idx++;
    ST(&jp->down_tail, tail + 1);
    jp->rows_played++;
}

static void jp_poll_send_done(void *ctx)
{
    jp_activity *jp = ctx;
    if (!jp->uni_seen || ops_free(jp) < 1) return;
    uint8_t op = OP_PSD;
    ops_put(jp, &op, 1);
    jp->psds++;
}

static int jp_has_backlog(void *ctx) { return jp_down_depth(ctx) > JP_BACKLOG_AT; }

void jp_as_engine(jp_activity *jp, sim_engine *e)
{
    memset(e, 0, sizeof *e);
    e->ctx = jp;
    e->tick = jp_tick;
    e->feed_in_frame = jp_feed;
    e->poll_send_done = jp_poll_send_done;
    e->has_backlog = jp_has_backlog;
    e->ni_started = 1;
    e->leave_d = 1;
    e->ni_activity = NI_ACTIVITY_TRADE;
}

/* ---- the bundle ---- */

typedef struct {
    bool     hello, full;
    unsigned ops_bytes;           /* op-log bytes consumed */
    unsigned recs, ticks, psds;
    uint32_t ctrl_mask;
} take_t;

static size_t put_ctrl(uint8_t *out, size_t off, uint8_t code)
{
    out[off] = JP_MSG_CTRL; out[off + 1] = code; out[off + 2] = out[off + 3] = out[off + 4] = 0;
    return off + 5;
}

static size_t build(const jp_activity *jp, uint8_t *out, take_t *t)
{
    memset(t, 0, sizeof *t);
    size_t off = 0;
    if (jp->ctrl_pending & BIT(JP_CTRL_JOIN_HELLO)) { off = put_ctrl(out, off, JP_CTRL_JOIN_HELLO); t->hello = true; t->ctrl_mask |= BIT(JP_CTRL_JOIN_HELLO); }
    if (!jp->hello_sent && !t->hello) {
        if (jp->ctrl_pending & BIT(JP_CTRL_CLOSE)) { off = put_ctrl(out, off, JP_CTRL_CLOSE); t->ctrl_mask |= BIT(JP_CTRL_CLOSE); }
        return off;
    }

    unsigned used = ops_used(jp), pos = 0;
    uint16_t feed_idx = jp->sent_feed_seq, tick_idx = jp->sent_tick_idx;
    while (pos < used && off < JP_BUNDLE_FLUSH_AT) {
        uint8_t k = ops_at(jp, pos);
        if (k == OP_FEED) {
            /* a JUP message: the run of consecutive FEED items, up to 8 records, as many as fit under the flush threshold */
            size_t hdr = off;
            unsigned count = 0;
            size_t o = off + 4;
            unsigned p = pos;
            while (p < used && ops_at(jp, p) == OP_FEED && count < JP_MAX_REC) {
                uint8_t nsl = ops_at(jp, p + 1), mask = ops_at(jp, p + 2);
                unsigned pc = 0;
                for (int b = 0; b < JP_MAX_SLOTS; b++) if (mask & (1u << b)) pc++;
                size_t rl = 2 + (size_t)pc * JP_ROW_LEN;
                if (o + rl > JP_BUNDLE_FLUSH_AT && count > 0) break;
                if (o + rl > JP_BUNDLE_MAX) break;
                out[o] = nsl; out[o + 1] = mask;
                for (unsigned i = 0; i < pc * JP_ROW_LEN; i++) out[o + 2 + i] = ops_at(jp, p + 3 + i);
                o += rl;
                p += 3 + pc * JP_ROW_LEN;
                count++;
            }
            if (!count) break;
            out[hdr] = JP_MSG_JUP; out[hdr + 1] = (uint8_t)feed_idx; out[hdr + 2] = (uint8_t)(feed_idx >> 8); out[hdr + 3] = (uint8_t)count;
            feed_idx = (uint16_t)(feed_idx + count);
            t->recs += count;
            off = o; pos = p;
        } else {
            /* a JTICK message: a tick run, then optionally the psd run that directly follows it */
            if (off + 5 > JP_BUNDLE_MAX) break;
            unsigned nt = 0, np = 0, p = pos;
            while (p < used && ops_at(jp, p) == OP_TICK && nt < 8) { nt++; p++; }
            while (p < used && ops_at(jp, p) == OP_PSD && np < 8) { np++; p++; }
            out[off] = JP_MSG_JTICK; out[off + 1] = (uint8_t)tick_idx; out[off + 2] = (uint8_t)(tick_idx >> 8);
            out[off + 3] = (uint8_t)nt; out[off + 4] = (uint8_t)np;
            off += 5;
            tick_idx = (uint16_t)(tick_idx + nt);
            t->ticks += nt; t->psds += np; pos = p;
        }
    }
    t->ops_bytes = pos;
    if (pos < used) { t->full = true; return off; }
    for (uint8_t code = 2; code <= JP_CTRL_MAX; code++) {
        if (code == JP_CTRL_JOIN_HELLO || !(jp->ctrl_pending & BIT(code)) || off + 5 > JP_BUNDLE_MAX) continue;
        off = put_ctrl(out, off, code);
        t->ctrl_mask |= BIT(code);
    }
    if (off >= JP_BUNDLE_FLUSH_AT) t->full = true;
    return off;
}

static void commit(jp_activity *jp, const take_t *t, size_t bytes, uint32_t now)
{
    if (t->hello) jp->hello_sent = true;
    jp->ops_tail = (uint16_t)((jp->ops_tail + t->ops_bytes) % JP_OPS_BYTES);
    jp->sent_feed_seq = (uint16_t)(jp->sent_feed_seq + t->recs);
    jp->sent_tick_idx = (uint16_t)(jp->sent_tick_idx + t->ticks);
    for (uint8_t code = 1; code <= JP_CTRL_MAX; code++) {
        if (!(t->ctrl_mask & BIT(code))) continue;
        jp->ctrl_pending &= ~BIT(code);
        jp->ctrl_tx++;
    }
    jp->bundles_tx++;
    jp->bundle_bytes_tx += (uint32_t)bytes;
    jp->retry = false;
    jp->flushed_once = true;
    jp->last_flush_ms = now;
}

void jp_flush(jp_activity *jp, uint32_t now)
{
    if (!jp->send) return;
    for (int guard = 0; guard < 4; guard++) {
        uint8_t bundle[JP_BUNDLE_MAX];
        take_t t;
        size_t n = build(jp, bundle, &t);
        if (!n) return;
        uint32_t eff = jp_effective_flush_ms(jp);
        bool due = jp->retry || !jp->flushed_once || t.full || eff == 0 || (uint32_t)(now - jp->last_flush_ms) >= eff;
        if (!due) return;
        if (!jp->send(jp->send_ctx, bundle, n)) {
            if (!jp->retry) {
                jp->flush_extra += JP_EXTRA_STEP_MS;
                if (jp->flush_extra > JP_EXTRA_MAX_MS) jp->flush_extra = JP_EXTRA_MAX_MS;
            }
            jp->retry = true; jp->ok_streak = 0; jp->ble_retries++;
            return;
        }
        if (jp->flush_extra && ++jp->ok_streak >= 16) { jp->flush_extra -= jp->flush_extra < 2 ? jp->flush_extra : 2; jp->ok_streak = 0; }
        commit(jp, &t, n, now);
    }
}

void jp_flush_now(jp_activity *jp) { jp_flush(jp, jp->last_flush_ms + jp_effective_flush_ms(jp)); }
