/* proxy_activity.c — see proxy_activity.h. */
#include "proxy_activity.h"
#include <string.h>

#define LD(p)     __atomic_load_n((p), __ATOMIC_ACQUIRE)
#define ST(p, v)  __atomic_store_n((p), (v), __ATOMIC_RELEASE)
#define BIT(c)    (1u << (c))

void px_init(px_activity *px, px_send_fn send, void *send_ctx)
{
    memset(px, 0, sizeof *px);
    px->send = send;
    px->send_ctx = send_ctx;
    px->flush_ms = PX_FLUSH_MS_DEFAULT;
    px->match_itvl = true;
}

void px_set_flush_ms(px_activity *px, uint32_t ms) { px->flush_ms = ms > 1000 ? 1000 : ms; }
void px_set_link_itvl_ms(px_activity *px, uint32_t ms) { px->link_itvl_ms = ms > 4000 ? 4000 : ms; }
void px_set_match_itvl(px_activity *px, bool on) { px->match_itvl = on; }

uint32_t px_effective_flush_ms(const px_activity *px)
{
    uint32_t base = px->flush_ms;
    if (px->match_itvl && px->link_itvl_ms > base) base = px->link_itvl_ms;
    return base + px->flush_extra;
}

bool px_flag(const px_activity *px, uint8_t code)
{
    return code < 32 && (LD(&px->flags) & BIT(code)) != 0;
}

int px_down_depth(const px_activity *px)
{
    return (int)(LD(&px->down_head) - LD(&px->down_tail));
}

void px_ctrl(px_activity *px, uint8_t code)
{
    if (code >= 1 && code <= 7) px->ctrl_pending |= BIT(code);
}

/* ---- BLE task side: DOWN rows and CTRL from the phone ---- */

static void down_rows(px_activity *px, uint16_t seq, unsigned count, const uint8_t *rows)
{
    for (unsigned r = 0; r < count; r++) {
        uint16_t idx = (uint16_t)(seq + r);
        if (LD(&px->down_started)) {
            int16_t diff = (int16_t)(idx - px->down_expected);
            if (diff < 0) { px->dups++; continue; }               /* already seen (a replay after a relay reconnect) */
            if (diff > 0) { px->gaps++; __atomic_fetch_add(&px->down_lost, (uint32_t)diff, __ATOMIC_RELEASE); }   /* rows lost: accept, resync, write the credits off */
        }
        px->down_expected = (uint16_t)(idx + 1);
        uint32_t head = px->down_head;
        const uint8_t *row = rows + r * PX_ROW_LEN;
        if (head - LD(&px->down_tail) >= PX_DOWN_SHED_AT) {
            bool idle = true;
            for (int i = 0; i < PX_ROW_LEN; i++) if (row[i]) { idle = false; break; }
            if (idle) {                                       /* an all-zero parent row says nothing: shed it, write its credit off */
                px->idle_shed++;
                __atomic_fetch_add(&px->down_lost, 1u, __ATOMIC_RELEASE);
                ST(&px->down_started, 1);
                continue;
            }
        }
        if (head - LD(&px->down_tail) >= PX_DOWN_RING) { px->down_overflow++; ST(&px->down_started, 1); continue; }
        memcpy(px->down[head % PX_DOWN_RING], row, PX_ROW_LEN);
        ST(&px->down_head, head + 1);
        ST(&px->down_started, 1);
        px->down_rows++;
    }
}

bool px_on_data(px_activity *px, const uint8_t *d, size_t len)
{
    if (len == 4 || len == 0) return false;                       /* the relay_ping echo */
    if (d[0] != PX_MSG_DOWN && d[0] != PX_MSG_CTRL) return false; /* UP/TICK are outbound only; anything else is not ours */
    px->bundles_rx++;
    if (len > PX_BUNDLE_MAX) px->oversize_rx++;
    size_t off = 0;
    while (off < len) {
        const uint8_t *m = d + off;
        size_t left = len - off;
        if (m[0] == PX_MSG_CTRL) {
            if (left < 5 || m[1] < 1 || m[1] > 7 || m[2] || m[3] || m[4]) { px->bad_msgs++; break; }
            __atomic_fetch_or(&px->flags, BIT(m[1]), __ATOMIC_RELEASE);
            px->ctrl_rx++;
            off += 5;
        } else if (m[0] == PX_MSG_DOWN) {
            unsigned count = left >= 4 ? m[3] : 0;
            if (left < 4 || count < 1 || count > PX_MAX_ROWS || left < 4 + count * PX_ROW_LEN) { px->bad_msgs++; break; }
            down_rows(px, (uint16_t)(m[1] | (m[2] << 8)), count, m + 4);
            off += 4 + count * PX_ROW_LEN;
        } else { px->bad_msgs++; break; }                         /* unknown kind (or a stray UP/TICK): drop the rest */
    }
    return true;
}

/* ---- session task side: the pia_activity ---- */

static void px_feed(px_activity *px, const uint8_t slot[PX_ROW_LEN])
{
    if (px->up_count >= PX_UP_RING) {                              /* the phone is not draining: drop the oldest */
        px->up_head = (uint16_t)((px->up_head + 1) % PX_UP_RING);
        px->up_first_seq++;
        px->up_count--;
        px->up_overflow++;
        px->ble_drops++;
    }
    memcpy(px->up[(px->up_head + px->up_count) % PX_UP_RING], slot, PX_ROW_LEN);
    px->up_count++;
    px->up_next_seq++;
    px->up_rows++;
}

/* Credits sent whose DOWN row has been neither played nor written off. */
static uint32_t outstanding(const px_activity *px)
{
    uint32_t used = px->down_tail + LD(&px->down_lost);
    return px->credits_sent > used ? px->credits_sent - used : 0;
}

static void idle_row(uint16_t words[7]) { memset(words, 0, 7 * sizeof words[0]); }

/* The ring is empty: if the credit window is full and nothing has come back for ~10 s the answers were lost; write the credits off. */
static void stall_watch(px_activity *px)
{
    if (outstanding(px) >= PX_CREDIT_WINDOW && ++px->stall_ticks >= PX_CREDIT_STALL_TICKS) {
        __atomic_fetch_add(&px->down_lost, outstanding(px), __ATOMIC_RELEASE);
        px->credit_resets++;
        px->stall_ticks = 0;
    }
}

static void px_tick(px_activity *px, uint16_t words[7])
{
    /* Every local tick is one credit for the remote engine; its answer comes back as the DOWN row with this tick's index. */
    px->ticks++;
    if (px->tick_pending < PX_TICK_PENDING_MAX) { px->tick_pending++; px->tick_next_idx++; }
    else px->tick_overflow++;            /* the remote cannot keep up: this tick makes no credit, so indexes stay contiguous (no phantom gap) */

    if (!LD(&px->down_started)) { idle_row(words); return; }       /* the stream has not begun: idle is what the engine would say anyway */

    uint32_t tail = px->down_tail;
    uint32_t depth = LD(&px->down_head) - tail;
    if (!px->primed) {                                             /* (re)filling: play only once a few rows are queued, so a single late */
        if (depth >= PX_PREFILL || (depth > 0 && px->prefill_wait >= PX_PREFILL_MAX_WAIT)) {   /* bundle does not inject an idle row per gap */
            px->primed = true;
            px->prefill_wait = 0;
        } else {
            if (depth > 0) px->prefill_wait++;
            px->prefill_idle++;
            idle_row(words);
            if (depth == 0) stall_watch(px);
            return;
        }
    }
    if (depth == 0) {
        px->underruns++;                                           /* the ring ran dry mid-stream: an idle row goes out, refill before playing on */
        px->primed = false;
        px->prefill_wait = 0;
        idle_row(words);
        stall_watch(px);
        return;
    }
    px->stall_ticks = 0;
    const uint8_t *r = px->down[tail % PX_DOWN_RING];
    for (int i = 0; i < 7; i++) words[i] = (uint16_t)(r[2 * i] | (r[2 * i + 1] << 8));
    ST(&px->down_tail, tail + 1);
}

/* ---- the bundle ---- */

typedef struct {
    bool     hello;
    unsigned up_rows;
    unsigned credits, tick_msgs;
    uint32_t ctrl_mask;       /* CTRL codes in this bundle (HELLO included) */
    bool     full;            /* reached PX_BUNDLE_FLUSH_AT, or rows are left over: send without waiting */
} take_t;

static size_t put_ctrl(uint8_t *out, size_t off, uint8_t code)
{
    out[off] = PX_MSG_CTRL; out[off + 1] = code; out[off + 2] = out[off + 3] = out[off + 4] = 0;
    return off + 5;
}

/* Lay out the next bundle without consuming anything: px_flush commits only after the transport accepted it. */
static size_t build(const px_activity *px, uint8_t *out, take_t *t)
{
    memset(t, 0, sizeof *t);
    size_t off = 0;
    if (px->ctrl_pending & BIT(PX_CTRL_HELLO)) { off = put_ctrl(out, off, PX_CTRL_HELLO); t->hello = true; t->ctrl_mask |= BIT(PX_CTRL_HELLO); }
    if (!px->hello_sent && !t->hello) {                            /* before HELLO only CLOSE may go (a session that ended before it began) */
        if (px->ctrl_pending & BIT(PX_CTRL_CLOSE)) { off = put_ctrl(out, off, PX_CTRL_CLOSE); t->ctrl_mask |= BIT(PX_CTRL_CLOSE); }
        return off;
    }

    unsigned rows_left = px->up_count, taken = 0;
    uint16_t seq = px->up_first_seq;
    while (rows_left && off + 4 + PX_ROW_LEN <= PX_BUNDLE_FLUSH_AT) {
        unsigned n = rows_left < PX_MAX_ROWS ? rows_left : PX_MAX_ROWS;
        unsigned fit = (PX_BUNDLE_FLUSH_AT - off - 4) / PX_ROW_LEN;
        if (n > fit) n = fit;
        out[off] = PX_MSG_UP; out[off + 1] = (uint8_t)seq; out[off + 2] = (uint8_t)(seq >> 8); out[off + 3] = (uint8_t)n;
        for (unsigned i = 0; i < n; i++) memcpy(out + off + 4 + i * PX_ROW_LEN, px->up[(px->up_head + taken + i) % PX_UP_RING], PX_ROW_LEN);
        off += 4 + n * PX_ROW_LEN;
        taken += n; rows_left -= n; seq = (uint16_t)(seq + n);
    }
    t->up_rows = taken;
    if (rows_left) { t->full = true; return off; }                 /* more rows than a bundle holds: send this one now, the next follows */

    /* The credits for this pass's ticks go after its rows: the remote engine must see feed-then-tick, as pia_hsess does. */
    uint32_t out_credits = outstanding(px);
    unsigned room = out_credits >= PX_CREDIT_WINDOW ? 0 : PX_CREDIT_WINDOW - out_credits;
    unsigned avail = px->tick_pending < room ? px->tick_pending : room;
    bool window_closed = px->tick_pending > 0 && room == 0;
    while (t->credits < avail && off + 5 + 5 * 6 <= PX_BUNDLE_MAX) {   /* leave room for the CTRL codes that may follow */
        unsigned n = avail - t->credits < PX_MAX_ROWS ? avail - t->credits : PX_MAX_ROWS;
        uint16_t idx = (uint16_t)(px->tick_first_idx + t->credits);
        out[off] = PX_MSG_TICK; out[off + 1] = (uint8_t)idx; out[off + 2] = (uint8_t)(idx >> 8); out[off + 3] = (uint8_t)n; out[off + 4] = 0;
        off += 5; t->credits += n; t->tick_msgs++;
    }
    /* CLOSE / DISCONNECT_SENT follow the last rows and credits; a closed window must not hold them back (after the final tick the
     * unplayed answers keep it closed for good). */
    if (t->credits == px->tick_pending || window_closed) {
        for (uint8_t code = 2; code <= 7; code++) {
            if (!(px->ctrl_pending & BIT(code)) || off + 5 > PX_BUNDLE_MAX) continue;
            off = put_ctrl(out, off, code);
            t->ctrl_mask |= BIT(code);
        }
    }
    if (off >= PX_BUNDLE_FLUSH_AT) t->full = true;
    return off;
}

static void commit(px_activity *px, const take_t *t, size_t bytes, uint32_t now)
{
    if (t->hello) px->hello_sent = true;
    px->up_head = (uint16_t)((px->up_head + t->up_rows) % PX_UP_RING);
    px->up_first_seq = (uint16_t)(px->up_first_seq + t->up_rows);
    px->up_count = (uint16_t)(px->up_count - t->up_rows);
    px->tick_first_idx = (uint16_t)(px->tick_first_idx + t->credits);
    px->tick_pending = (uint16_t)(px->tick_pending - t->credits);
    px->credits_sent += t->credits;
    px->tick_msgs += t->tick_msgs;
    for (uint8_t code = 1; code <= 7; code++) {
        if (!(t->ctrl_mask & BIT(code))) continue;
        px->ctrl_pending &= ~BIT(code);
        px->ctrl_tx++;
    }
    px->bundles_tx++;
    px->bundle_bytes_tx += (uint32_t)bytes;
    px->retry = false;
    px->flushed_once = true;
    px->last_flush_ms = now;
}

void px_flush(px_activity *px, uint32_t now)
{
    if (!px->send) return;
    for (int guard = 0; guard < 4; guard++) {
        uint8_t bundle[PX_BUNDLE_MAX];
        take_t t;
        size_t n = build(px, bundle, &t);
        if (!n) return;
        uint32_t eff = px_effective_flush_ms(px);
        bool due = px->retry || !px->flushed_once || t.full || eff == 0 || (uint32_t)(now - px->last_flush_ms) >= eff;
        if (!due) return;
        if (!px->send(px->send_ctx, bundle, n)) {                  /* keep it pending; next pass tries again */
            if (!px->retry) {                                      /* a new congestion episode: pace out a little more from now on */
                px->flush_extra += PX_EXTRA_STEP_MS;
                if (px->flush_extra > PX_EXTRA_MAX_MS) px->flush_extra = PX_EXTRA_MAX_MS;
            }
            px->retry = true; px->ok_streak = 0; px->ble_retries++;
            return;
        }
        if (px->flush_extra && ++px->ok_streak >= 16) { px->flush_extra -= px->flush_extra < 2 ? px->flush_extra : 2; px->ok_streak = 0; }
        commit(px, &t, n, now);
    }
}

void px_flush_now(px_activity *px) { px_flush(px, px->last_flush_ms + px_effective_flush_ms(px)); }

static void a_feed(void *ctx, const uint8_t slot[14]) { px_feed((px_activity *)ctx, slot); }
static void a_tick(void *ctx, uint16_t words[7]) { px_tick((px_activity *)ctx, words); }
static int  a_disc(void *ctx) { return px_flag((px_activity *)ctx, PX_CTRL_DISCONNECT_REQUESTED); }
static void a_sent(void *ctx) { px_ctrl((px_activity *)ctx, PX_CTRL_DISCONNECT_SENT); }
static int  a_est(void *ctx)  { return px_flag((px_activity *)ctx, PX_CTRL_ESTABLISHED); }

pia_activity px_as_activity(px_activity *px)
{
    pia_activity a = {
        .ctx = px, .feed_child_slot = a_feed, .tick = a_tick,
        .disconnect_requested = a_disc, .mark_disconnect_sent = a_sent, .established = a_est,
    };
    return a;
}
