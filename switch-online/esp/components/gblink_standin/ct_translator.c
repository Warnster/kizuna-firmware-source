/* SPDX-License-Identifier: AGPL-3.0-only */
/* ct_translator.c -- CableTranslator, ported from GB-Link-Switch-LDN web/js/cable/translator.js (commit c972db4,
 * https://github.com/GB-Link/GB-Link-Switch-LDN). Copyright (c) the GB-Link-Switch-LDN authors. Modified for kizuna
 * 2026-11-02: C port with fixed storage; Ruby/Sapphire dressing, refusal, Emerald bypass and several-rooms handling
 * dropped; battle gated (see ct_translator.h). AGPL-3.0, see NOTICE. Function names follow the JS methods. */
#include "ct_translator.h"

#include <string.h>

enum { LCOM_NULL = 0, LCOM_NI_START = 1, LCOM_NI = 2, LCOM_NI_END = 3, LCOM_UNI = 4 };
enum { HOST_FRAME_BYTES = 3 + 5 * CT_SLOT_BYTES };
enum { LP_BLOCK = 60, LP_VERSION_OFFSET = 16, LP_TRAINER_ID_OFFSET = 20, LP_NAME_OFFSET = 24, LP_LINK_TYPE_OFFSET = 36,
       LP_ID_OFFSET = 40, LP_BUFFER_SIZE = 200, CARD_SIZE = 100, LINK_PLAYER_SIZE = 28 };
enum { OWNER_FLAG = 0x80, ANNOUNCE_PACKETS = 30, SESSION_SETTLE_PACKETS = 150, BARRIER_EMITS = 6, IDLE_TIMEOUT = 90,
       HOST_SILENT_FRAMES = 600, LP_EARLY_PACKETS = 20, REPAIR_QUIET_FRAMES = 30, REPAIRS = 3, REPEAT_FRAMES = 120,
       PULL_WITH_BLOCK_FRAMES = 8 };
enum { AIR_NONE = -1, AIR_SEARCH = 0, AIR_CONNECTING = 1, AIR_NI = 2, AIR_UNI = 3 };
enum { ACT_TRADE = 4, ACT_SINGLE = 1, ACT_DOUBLE = 2 };
enum { LT_SINGLE = 0x2233, LT_DOUBLE = 0x2244, LT_BATTLE = 0x2211, LT_TRADE_SETUP = 0x1133 };
enum { B_ROOM, B_STARTING, B_FIGHTING, B_RETURNING };
enum { EXPECT_NONE, EXPECT_LP, EXPECT_PULL, EXPECT_CARD };
enum { BAR_IDLE, BAR_STANDBY, BAR_CLOSE };
enum { P_NONE, P_ROUND0, P_GAME_STANDBY, P_GAME_CLOSE, P_EXIT_CLOSE, P_BATTLE };
enum { SEND_INIT, SEND_STREAM, SEND_HOLD, SEND_DONE };
enum { BQ_LP = 1, BQ_CARD = 2, BQ_BATTLE_LP = 4 };
#define NEVER (-((int64_t)1 << 40))

static const uint8_t REQUEST_FRAGMENTS[5] = { 17, 17, 9, 19, 4 };
static const uint8_t PLACEHOLDER_NAME[6] = { 0xbd, 0xbf, 0xc6, 0xc3, 0xc9, 0xff };   /* "CELIO" */
#define VERSION_FIRE_RED 4
#define COMPAT_CAN_LINK_NATIONALLY (1 << 7)

static unsigned frag_count(unsigned bytes) { return bytes ? (bytes + CT_FRAG_BYTES - 1) / CT_FRAG_BYTES : 1; }
/* [4, 2, 1, 3].find((type) => REQUEST_FRAGMENTS[type] >= count) ?? null */
static int repair_type(unsigned count)
{
    static const uint8_t order[4] = { 4, 2, 1, 3 };
    for (unsigned i = 0; i < 4; i++) if (REQUEST_FRAGMENTS[order[i]] >= count) return order[i];
    return -1;
}
/* REQUEST_TYPES = { 17: 1, 9: 2, 19: 3, 4: 4 } */
static int request_type(unsigned frags) { return frags == 17 ? 1 : frags == 9 ? 2 : frags == 19 ? 3 : frags == 4 ? 4 : -1; }

static bool is_battle(const ct_translator_t *t) { return t->activity != ACT_TRADE; }
static bool battle_blocks(const ct_translator_t *t) { return is_battle(t) && t->battle_state != B_ROOM; }
static void event(ct_translator_t *t, int ev, uint32_t arg) { if (t->cfg.event) t->cfg.event(t->cfg.event_ctx, ev, arg); }

/* ---- block queues (arena FIFOs) */
static bool bq_push(ct_blockq_t *q, const uint8_t *data, size_t len, uint8_t flags)
{
    if (q->n >= CT_TR_BLOCKS || q->used + len > CT_TR_ARENA) return false;
    memcpy(q->buf + q->used, data, len);
    q->len[q->n] = (uint16_t)len;
    q->flags[q->n] = flags;
    q->n++;
    q->used = (uint16_t)(q->used + len);
    return true;
}
static size_t bq_shift(ct_blockq_t *q, uint8_t *out, uint8_t *flags)
{
    size_t len = q->len[0];
    memcpy(out, q->buf, len);
    *flags = q->flags[0];
    memmove(q->buf, q->buf + len, q->used - len);
    memmove(q->len, q->len + 1, (q->n - 1) * sizeof q->len[0]);
    memmove(q->flags, q->flags + 1, (q->n - 1) * sizeof q->flags[0]);
    q->n--;
    q->used = (uint16_t)(q->used - len);
    return len;
}
static void bq_clear(ct_blockq_t *q) { q->n = 0; q->used = 0; }

/* ---- recv: the leader's own blocks (rx0, with a buffer) and its reflection of ours (rx1, flags only) */
static void recv_reset(ct_recv_t *r, uint8_t *buf)
{
    memset(r, 0, sizeof *r);
    r->last_index = -1;
    if (buf) memset(buf, 0, CT_MAX_BLOCK_BYTES);
}

static void recv_init(ct_recv_t *r, uint8_t *buf, unsigned count)
{
    if (count == 0 || count > CT_MAX_FRAGS) return;
    /* The repeat a repair asked for keeps the fragments already here. */
    if (r->receiving && !r->done && r->repairs) {
        int rt = repair_type(r->need);
        if (rt >= 0 && count == REQUEST_FRAGMENTS[rt]) { r->count = (uint8_t)count; return; }
    }
    /* A repeated INIT of the block being received keeps its fragments. */
    if (!r->receiving || r->done || count != r->count || r->flags) {
        recv_reset(r, buf);
        r->count = (uint8_t)count;
        r->receiving = true;
    }
}

static bool recv_block(ct_recv_t *r, uint8_t *buf, unsigned index, const uint8_t *slot)
{
    if (!r->receiving || index >= r->count) return false;
    unsigned need = r->need ? r->need : r->count;
    if (r->repairs && index >= need) return false;
    /* A repeat that differs from a fragment already here is another block: start it over. */
    if (r->repairs && (r->flags & (1u << index)) && buf && memcmp(slot + 2, buf + index * CT_FRAG_BYTES, CT_FRAG_BYTES) != 0) r->flags = 0;
    bool was_done = r->done;
    r->last_index = (int8_t)index;
    r->flags |= 1u << index;
    if (buf) memcpy(buf + index * CT_FRAG_BYTES, slot + 2, CT_FRAG_BYTES);
    if (r->flags == ((1u << need) - 1)) r->done = true;
    return r->done && !was_done;
}

static bool feed_recv(ct_recv_t *r, uint8_t *buf, const uint16_t *w, const uint8_t *slot)
{
    unsigned op = w[0] & CT_RFUCMD_MASK;
    if (op == CT_RFUCMD_SEND_BLOCK_INIT) recv_init(r, buf, w[1]);
    else if (op == CT_RFUCMD_SEND_BLOCK) return recv_block(r, buf, w[0] & 0x1f, slot);
    return false;
}

/* ---- the child's block sender */
static void send_reset(ct_translator_t *t) { memset(&t->snd, 0, sizeof t->snd); }

static void send_start(ct_translator_t *t, const uint8_t *data, size_t size, bool is_lp)
{
    send_reset(t);
    t->snd.active = true;
    t->snd.count = (uint8_t)frag_count((unsigned)size);
    memcpy(t->snd.data, data, size);
    t->snd.is_lp = is_lp;
}

static void block_words(const ct_translator_t *t, unsigned index, uint16_t *w)
{
    w[0] = (uint16_t)(CT_RFUCMD_SEND_BLOCK | (index & 0x1f));
    for (unsigned i = 0; i < 6; i++) w[1 + i] = ct_le16(t->snd.data + index * CT_FRAG_BYTES + i * 2);
}

/* One frame of the child's block sender, paced by the leader's reflection of our fragments. `hold` keeps the last
 * fragment back: the leader moves on once it has our block. */
static void send_tick(ct_translator_t *t, uint16_t *w, bool hold)
{
    const ct_recv_t *ack = &t->rx1;
    memset(w, 0, 7 * sizeof w[0]);
    if (!t->snd.active) return;
    if (t->snd.state == SEND_INIT) {
        /* INIT repeats until the reflection shows the leader armed its receive side. */
        if (ack->receiving && ack->count == t->snd.count) {
            t->snd.state = SEND_STREAM;
            t->snd.index = 0;
        } else {
            w[0] = CT_RFUCMD_SEND_BLOCK_INIT;
            w[1] = t->snd.count;
            w[2] = 1 | OWNER_FLAG;
            return;
        }
    }
    if (t->snd.state == SEND_STREAM) {
        unsigned index = t->snd.index;
        if (hold && index >= (unsigned)t->snd.count - 1) return;
        block_words(t, index, w);
        if (index >= (unsigned)t->snd.count - 1) t->snd.state = SEND_HOLD;
        else t->snd.index++;
        return;
    }
    if (t->snd.state == SEND_HOLD) {
        unsigned last = t->snd.count - 1;
        uint32_t full = (1u << t->snd.count) - 1;
        if (ack->last_index == (int)last && ack->count == t->snd.count) {
            if (ack->flags == full) { t->snd.state = SEND_DONE; t->snd.active = false; return; }
            uint8_t missing[CT_MAX_FRAGS];
            unsigned m = 0;
            for (unsigned i = 0; i < t->snd.count; i++) if (!((ack->flags >> i) & 1)) missing[m++] = (uint8_t)i;
            if (m) {
                t->snd.rr = (uint8_t)((t->snd.rr + 1) % m);
                block_words(t, missing[t->snd.rr], w);
                return;
            }
        }
        block_words(t, last, w);
    }
}

/* ---- barrier */
static void barrier_reset(ct_translator_t *t)
{
    memset(&t->bar, 0, sizeof t->bar);
    t->bar.host_count = -1;
    t->bar.burst_for = -1;
}

static void barrier_initiate(ct_translator_t *t, uint8_t kind)
{
    if (t->bar.mode == kind) return;
    t->bar.mode = kind;
    t->bar.initiated = true;
    t->bar.host_count = -1;
    t->bar.since_host = 0;
    t->bar.since_initiate = 0;
    t->bar.burst_for = -1;
}

static bool barrier_want(ct_translator_t *t, uint16_t *w)
{
    if (t->bar.mode == BAR_IDLE) return false;
    if (t->bar.burst_for != t->bar.local_count) { t->bar.burst_for = t->bar.local_count; t->bar.burst_n = 0; }
    if (t->bar.burst_n >= BARRIER_EMITS) return false;
    t->bar.burst_n++;
    memset(w, 0, 7 * sizeof w[0]);
    w[0] = t->bar.mode == BAR_STANDBY ? CT_RFUCMD_READY_EXIT_STANDBY : CT_RFUCMD_READY_CLOSE_LINK;
    w[1] = (uint16_t)t->bar.local_count;
    return true;
}

/* ---- NI */
static uint16_t child_llsf(unsigned state, unsigned n, unsigned phase, unsigned ack, unsigned size)
{
    return (uint16_t)(((state & 0xf) << 10) | ((ack & 1) << 9) | ((n & 3) << 7) | ((phase & 3) << 5) | (size & 0x1f));
}

static void new_ni(ct_translator_t *t, const uint8_t *src)
{
    memset(&t->ni, 0, sizeof t->ni);
    t->ni.remain = 7;
    if (src) memcpy(t->ni.src, src, 26);
    /* NI_START header: data type 1 (game data), payload size 12, data size 26. */
    t->ni.header[0] = 1;
    ct_put16(t->ni.header + 1, 12);
    t->ni.header[3] = 26;
}

/* One child NI sub-frame; 0 once the transfer is finished. */
static size_t ni_next(ct_ni_t *ni, uint8_t *out)
{
    const unsigned payload = 12;
    if (ni->state == 0) {
        unsigned size = (unsigned)ni->remain < payload ? (unsigned)ni->remain : payload;
        ct_put16(out, child_llsf(LCOM_NI_START, 1, 0, 0, size));
        memcpy(out + 2, ni->header, size);
        ni->state = 1;
        ni->phase = 0;
        for (unsigned i = 0; i < 4; i++) { ni->n[i] = 1; ni->now[i] = (uint8_t)(payload * i); }
        ni->remain = 26;
        return 2 + size;
    }
    if (ni->state == 1) {
        while (ni->now[ni->phase] >= 26) ni->phase = (ni->phase + 1) % 4;
        unsigned off = ni->now[ni->phase];
        unsigned size = 26 - off < payload ? 26 - off : payload;
        ct_put16(out, child_llsf(LCOM_NI, ni->n[ni->phase], ni->phase, 0, size));
        memcpy(out + 2, ni->src + off, size);
        ni->remain = (int16_t)(ni->remain - size);
        ni->now[ni->phase] = (uint8_t)(ni->now[ni->phase] + (payload << 2));
        ni->phase = (ni->phase + 1) % 4;
        if (ni->remain <= 0) { ni->state = 2; ni->phase = 0; }
        return 2 + size;
    }
    if (ni->state == 2) { ct_put16(out, child_llsf(LCOM_NI_END, 0, 0, 0, 0)); ni->state = 3; return 2; }
    if (ni->state == 3) { ct_put16(out, child_llsf(LCOM_NULL, 1, 0, 0, 0)); ni->state = 4; return 2; }
    return 0;
}

/* ---- frames out (celio.js glue: clientFrame / command) */
static void air_send(ct_translator_t *t, const uint8_t *payload, size_t len)
{
    uint8_t f[CT_RFU1_FRAME_MAX];
    if (len > CT_RFU1_SEND_DATA) len = CT_RFU1_SEND_DATA;
    ct_rfu1_frame(f, CT_RFU1_CLIENT_SEND, (uint32_t)len << 24, sizeof f);
    memcpy(f + 12, payload, len);
    t->send(t->send_ctx, f, sizeof f);
    t->child_frames++;
}
static void board_cmd(ct_translator_t *t, uint32_t type, uint32_t header)
{
    uint8_t f[16];
    ct_rfu1_frame(f, type, header, sizeof f);
    t->send(t->send_ctx, f, sizeof f);
}

/* ---- cable queue */
static void cable_push(ct_translator_t *t, const uint16_t *cmd)
{
    if (t->cq_n >= CT_TR_CABLE_QUEUE) { t->cq_drops++; return; }
    memcpy(t->cq[(t->cq_head + t->cq_n) % CT_TR_CABLE_QUEUE], cmd, CT_CMD_WORDS * sizeof cmd[0]);
    t->cq_n++;
}
static void cable_push_cmd(ct_translator_t *t, uint16_t c0, uint16_t c1, uint16_t c2)
{
    /* Once this side has closed the link, blocks wait for the next one. */
    if (c0 == CT_LINKCMD_READY_CLOSE_LINK) t->link_closing = true;
    uint16_t w[CT_CMD_WORDS] = { c0, c1, c2 };
    cable_push(t, w);
}
/* A block for the game as player 0: INIT_BLOCK, then CONT_BLOCK chunks of seven words. */
static void cable_push_block(ct_translator_t *t, const uint8_t *data, size_t size)
{
    cable_push_cmd(t, CT_LINKCMD_INIT_BLOCK, (uint16_t)size, 0 + 128);
    for (size_t pos = 0; pos < size; pos += (CT_CMD_WORDS - 1) * 2) {
        uint16_t w[CT_CMD_WORDS] = { CT_LINKCMD_CONT_BLOCK };
        for (unsigned i = 0; i < CT_CMD_WORDS - 1; i++) {
            size_t at = pos + i * 2;
            unsigned lo = at < size ? data[at] : 0, hi = at + 1 < size ? data[at + 1] : 0;
            w[1 + i] = (uint16_t)(lo | (hi << 8));
        }
        cable_push(t, w);
    }
}

/* ---- translator */
static void child_queue_push(ct_translator_t *t, const uint8_t *data, size_t size, uint8_t flags)
{
    if (t->child_queue.n >= CT_TR_BLOCKS || size > CT_MAX_BLOCK_BYTES) { t->child_drops++; return; }   /* the JS rule */
    if (!bq_push(&t->child_queue, data, size, flags)) t->arena_drops++;
}

/* The game's LinkPlayer with link type 0, what a wireless link uses (200-byte buffer). */
static void build_lp_for_host(const ct_translator_t *t, uint8_t out[LP_BUFFER_SIZE])
{
    memset(out, 0, LP_BUFFER_SIZE);
    memcpy(out, t->ruby_lp, LP_BLOCK);
    memset(out + LP_LINK_TYPE_OFFSET, 0, 4);
}

static void due_round(ct_translator_t *t) { t->battle_round = true; }

/* The leader's LinkPlayer with the game's own link type. early: before the other GBA's, with the link type of the
 * leader's room and the first spot. */
static void deliver_lp_to_game(ct_translator_t *t, bool early)
{
    if (!t->have_host_lp || t->p0_lp_delivered || (!t->session_ruby_lp && !early)) return;
    uint8_t block[LP_BLOCK];
    memcpy(block, t->host_lp, LP_BLOCK);
    if (early) {
        uint16_t type = t->activity == ACT_SINGLE ? LT_SINGLE : t->activity == ACT_DOUBLE ? LT_DOUBLE : LT_TRADE_SETUP;
        ct_put16(block + LP_LINK_TYPE_OFFSET, type);
        ct_put16(block + LP_LINK_TYPE_OFFSET + 2, 0);
        if (is_battle(t)) block[LP_ID_OFFSET] = 0;
    } else {
        memcpy(block + LP_LINK_TYPE_OFFSET, t->ruby_lp + LP_LINK_TYPE_OFFSET, 4);
    }
    /* In the Colosseum the id is the spot each player stands on; there are two. */
    if (is_battle(t) && !early) block[LP_ID_OFFSET] = t->ruby_lp[LP_ID_OFFSET] ^ 1;
    cable_push_block(t, block, sizeof block);
    t->p0_lp_delivered = true;
    t->ready_at = t->cable_packets + SESSION_SETTLE_PACKETS;
    if (!t->card_requested && !t->room_closed && t->close_count == 0 && !t->cancel_return) {
        t->card_armed = true;
        t->card_at = t->cable_packets + 15;
    }
    if (t->battle_state == B_RETURNING) {
        /* Back in the Colosseum after a battle: keys without a standby from the game. */
        t->battle_state = B_ROOM;
        t->exit_key_seen = false;
        ct_keyruns_clear(&t->host_keys);
        t->keys_active = true;
        due_round(t);
    }
    if (t->cancel_return) {
        /* Back in the room after a cancelled trade: the game sends keys without a standby. */
        t->cancel_return = false;
        t->room_closed = false;
        t->exit_key_seen = false;
        t->keys_active = true;
    }
}

static void maybe_start_round0(ct_translator_t *t)
{
    if (t->round0_started || !t->lp_send_done || !t->p0_lp_delivered) return;
    t->round0_started = true;
    t->purpose = P_ROUND0;
    barrier_initiate(t, BAR_STANDBY);
}

/* Blocks for the game wait while its link is between sessions. */
static bool session_ready(const ct_translator_t *t)
{
    if (is_battle(t) && (t->battle_state == B_STARTING || t->battle_state == B_RETURNING)) return false;
    return !t->link_closing && t->session_ruby_lp && t->p0_lp_delivered && t->cable_packets >= t->ready_at;
}

static void request_from_game(ct_translator_t *t, unsigned type)
{
    t->expect = EXPECT_PULL;
    t->pull_type = (uint8_t)type;
    cable_push_cmd(t, CT_LINKCMD_SEND_BLOCK_REQ, (uint16_t)type, 0);
}

static void release_held(ct_translator_t *t)
{
    if (!session_ready(t)) return;
    if (t->pending_pull >= 0 && t->expect != EXPECT_PULL) {
        request_from_game(t, (unsigned)t->pending_pull);
        t->pending_pull = -1;
    }
    unsigned off = 0;
    for (unsigned i = 0; i < t->held.n; i++) { cable_push_block(t, t->held.buf + off, t->held.len[i]); off += t->held.len[i]; }
    bq_clear(&t->held);
}

/* The game's card is asked for right after the LinkPlayers, ready for the leader's pull. */
static void request_early_card(ct_translator_t *t)
{
    if (t->expect != EXPECT_NONE) {
        t->card_armed = true;
        t->card_at = t->cable_packets + 10;
        return;
    }
    t->card_requested = true;
    t->expect = EXPECT_CARD;
    cable_push_cmd(t, CT_LINKCMD_SEND_BLOCK_REQ, 2, 0);
}

static void queue_card(ct_translator_t *t)
{
    child_queue_push(t, t->ruby_card, CARD_SIZE, BQ_CARD);
    t->card_queued = true;
}

/* The leader pulled a block. The first pull is the LinkPlayer; any other is forwarded to the game. */
static void host_pull(ct_translator_t *t, unsigned type)
{
    t->pull_at = t->host_frames;
    if (type == 2 && t->card_requested) {
        if (t->card_queued) return;
        if (t->have_ruby_card) queue_card(t);
        else t->card_pull_waiting = true;
        return;
    }
    if (!t->lp_sent_to_host && type <= 1) {
        if (!t->have_ruby_lp) { t->lp_pull_waiting = true; return; }
        uint8_t lp[LP_BUFFER_SIZE];
        build_lp_for_host(t, lp);
        child_queue_push(t, lp, LP_BUFFER_SIZE, BQ_LP);
        t->lp_sent_to_host = true;
        return;
    }
    if (t->expect == EXPECT_PULL || t->pending_pull >= 0) return;
    /* The leader repeats a pull until its block arrives: one answered but not yet streamed is a repeat. */
    if (t->child_queue.n || (t->snd.active && t->snd.state != SEND_HOLD)) return;
    if (!session_ready(t)) { t->pending_pull = (int32_t)type; return; }
    request_from_game(t, type);
}

/* The leader's LinkPlayer at a battle's start: answered with the game's on the other spot. */
static void host_battle_lp(ct_translator_t *t, const uint8_t *data)
{
    uint8_t out[LINK_PLAYER_SIZE];
    memcpy(out, t->ruby_lp + 16, LINK_PLAYER_SIZE);
    ct_put16(out + 0x14, LT_BATTLE);
    out[0x16] = 0; out[0x17] = 0;
    out[0x18] = data[0x18] ^ 1;
    out[0x19] = 0;
    child_queue_push(t, out, LINK_PLAYER_SIZE, 0);
    if (t->child_queue.n) t->child_queue.flags[t->child_queue.n - 1] |= BQ_BATTLE_LP;   /* JS: childQueue.at(-1) */
    if (t->battle_state == B_ROOM) t->battle_state = B_STARTING;
    t->keys_active = false;
}

/* A block the leader finished sending. */
static void host_block(ct_translator_t *t, unsigned count, const uint8_t *data)
{
    if (is_battle(t) && count == frag_count(LINK_PLAYER_SIZE) && data[1] == 0x40 && data[0] >= 1 && data[0] <= 5) {
        host_battle_lp(t, data);
        return;
    }
    size_t size = battle_blocks(t) ? count * CT_FRAG_BYTES : ct_size_from_count(count);
    if (count == 17 && !t->have_host_lp && ct_is_magic(data)) {
        memcpy(t->host_lp, data, LP_BLOCK);
        t->have_host_lp = true;
        deliver_lp_to_game(t, false);
        maybe_start_round0(t);
        return;
    }
    if (!battle_blocks(t) && count == 2 && ct_le16(data) == 0xeebb) t->cancel_pending = true;
    if (!session_ready(t)) {
        if (t->held.n < CT_TR_BLOCKS && size <= CT_MAX_BLOCK_BYTES && !bq_push(&t->held, data, size, 0)) t->arena_drops++;
        else if (t->held.n >= CT_TR_BLOCKS) t->held_drops++;   /* the JS rule */
        return;
    }
    cable_push_block(t, data, size);
}

/* A block the game finished sending on the cable. */
static void game_block(ct_translator_t *t, size_t size, const uint8_t *data)
{
    if (t->expect == EXPECT_LP && size >= LP_BLOCK) {
        bool first = !t->have_ruby_lp;
        memcpy(t->ruby_lp, data, LP_BLOCK);
        t->have_ruby_lp = true;
        uint16_t type = ct_le16(data + LP_LINK_TYPE_OFFSET);
        uint8_t chosen = !t->cfg.battle ? ACT_TRADE : type == LT_SINGLE ? ACT_SINGLE : type == LT_DOUBLE ? ACT_DOUBLE : ACT_TRADE;
        if (!ct_translator_joined(t)) {
            t->activity = chosen;
            t->battle_state = B_ROOM;
        } else if (t->cfg.hold_join && first) {
            t->battle_state = B_ROOM;   /* joined first: the room is the leader's */
        }
        if (t->battle_state == B_STARTING && type == LT_BATTLE) t->battle_state = B_FIGHTING;
        t->session_ruby_lp = true;
        t->expect = EXPECT_NONE;
        deliver_lp_to_game(t, false);
        if (t->lp_pull_waiting && !t->lp_sent_to_host && ct_translator_joined(t)) {
            t->lp_pull_waiting = false;
            uint8_t lp[LP_BUFFER_SIZE];
            build_lp_for_host(t, lp);
            child_queue_push(t, lp, LP_BUFFER_SIZE, BQ_LP);
            t->lp_sent_to_host = true;
        }
        maybe_start_round0(t);
        return;
    }
    if (t->expect == EXPECT_CARD && size >= CARD_SIZE) {
        memcpy(t->ruby_card, data, CARD_SIZE);
        t->have_ruby_card = true;
        t->expect = EXPECT_NONE;
        if (t->card_pull_waiting) { t->card_pull_waiting = false; queue_card(t); }
        return;
    }
    uint8_t block[CT_MAX_BLOCK_BYTES] = { 0 };
    size_t use = size;
    if (t->expect == EXPECT_PULL) {
        use = ct_size_from_request(t->pull_type);
        t->expect = EXPECT_NONE;
    }
    memcpy(block, data, size < CT_MAX_BLOCK_BYTES ? size : CT_MAX_BLOCK_BYTES);
    child_queue_push(t, block, use, 0);
}

static void flush_pending_standby(ct_translator_t *t);

static void game_standby(ct_translator_t *t)
{
    if (t->cancel_return) {
        /* The game's standby on returning to the room; the leader's round was part of the exit. */
        t->cancel_return = false;
        t->room_closed = false;
        t->exit_key_seen = false;
        t->keys_active = true;
        cable_push_cmd(t, CT_LINKCMD_READY_EXIT_STANDBY, 0, 0);
        return;
    }
    if (t->card_requested && !t->card_send_done && !t->room_closed) {
        /* A standby and a block never share the wire: wait until the leader has our card. */
        t->pending_game_standby = true;
        return;
    }
    t->purpose = P_GAME_STANDBY;
    barrier_initiate(t, BAR_STANDBY);
}

static void flush_pending_standby(ct_translator_t *t)
{
    if (t->pending_game_standby && t->card_send_done && t->bar.mode == BAR_IDLE) {
        t->pending_game_standby = false;
        game_standby(t);
    }
}

static void reset_link(ct_translator_t *t);
static void set_link(ct_translator_t *t, int8_t link)
{
    bool was = ct_translator_linked(t);
    t->link = link;
    if (was != ct_translator_linked(t)) event(t, CT_TR_EV_LINKED, ct_translator_linked(t));
}

static void exit_close_passed(ct_translator_t *t)
{
    cable_push_cmd(t, CT_LINKCMD_READY_CLOSE_LINK, 0, 0);
    board_cmd(t, CT_RFU1_DISCONNECT, 0);
    reset_link(t);
    set_link(t, AIR_NONE);
    t->have_ruby_lp = false;   /* the next cable club visit starts a new search */
    t->group_seen = false;
    t->join_allowed = !t->cfg.hold_join;
    t->close_count = 0;
    t->room_closed = false;
    t->cancel_pending = false;
    t->cancel_close = false;
    t->cancel_return = false;
    t->exit_key_seen = false;
    t->host_closed_seen = false;
    t->purpose = P_NONE;
}

/* The Colosseum: the game closes its cable link to start a battle and again after it. */
static void battle_close(ct_translator_t *t)
{
    cable_push_cmd(t, CT_LINKCMD_READY_CLOSE_LINK, 0, 0);
    t->keys_active = false;
    if (t->battle_state == B_ROOM || t->battle_state == B_STARTING) {
        t->battle_state = B_STARTING;
    } else {
        t->battle_state = B_RETURNING;
        due_round(t);
    }
}

static void game_close(ct_translator_t *t)
{
    if (is_battle(t) && !(t->exit_key_seen && t->battle_state == B_ROOM)) { battle_close(t); return; }
    if (t->exit_key_seen && !t->cancel_pending && !t->room_closed) {
        /* Leaving the room: the leader closes rather than standing by. */
        t->keys_active = false;
        t->purpose = P_EXIT_CLOSE;
        if (t->host_closed_seen) exit_close_passed(t);
        else barrier_initiate(t, BAR_CLOSE);
        return;
    }
    t->keys_active = false;
    t->room_closed = true;
    t->purpose = P_GAME_CLOSE;
    t->cancel_close = t->cancel_pending;
    t->cancel_pending = false;
    /* The leader's standby rounds around the game's closes alternate: leaving the room two, ending the trade menu for
     * the trade scene one, ending the trade two, and so on. */
    t->close_rounds_left = t->close_count % 2 == 0 ? 2 : 1;
    if (t->cancel_close) {
        t->close_rounds_left = 2;
        t->close_count = 0;
    } else {
        t->close_count++;
    }
    barrier_initiate(t, BAR_STANDBY);
}

static void start_due_round(ct_translator_t *t)
{
    if (!t->battle_round || t->bar.mode != BAR_IDLE || t->snd.active || t->child_queue.n) return;
    t->battle_round = false;
    t->purpose = P_BATTLE;
    barrier_initiate(t, BAR_STANDBY);
}

static void round_passed(ct_translator_t *t)
{
    switch (t->purpose) {
    case P_EXIT_CLOSE:
        exit_close_passed(t);
        break;
    case P_GAME_STANDBY:
        if (!t->room_closed) t->keys_active = true;
        t->purpose = P_NONE;
        cable_push_cmd(t, CT_LINKCMD_READY_EXIT_STANDBY, 0, 0);
        break;
    case P_GAME_CLOSE:
        if (t->close_rounds_left > 1) {
            t->close_rounds_left--;
            barrier_initiate(t, BAR_STANDBY);
        } else {
            t->close_rounds_left = 0;
            t->purpose = P_NONE;
            cable_push_cmd(t, CT_LINKCMD_READY_CLOSE_LINK, 0, 0);
            if (t->cancel_close) {
                t->cancel_close = false;
                t->cancel_return = true;
            }
        }
        break;
    default:   /* P_BATTLE: bar.timedOut is never set in the JS, so no repeat round */
        t->purpose = P_NONE;
        break;
    }
    flush_pending_standby(t);
}

/* ---- barriers against the leader's words */

/* The leader's standby word. True when a round we started has passed. */
static bool barrier_host_standby(ct_translator_t *t, int32_t count)
{
    t->bar.since_host = 0;
    int32_t prev = t->bar.host_count;
    t->bar.host_count = count;
    if (t->bar.initiated && t->bar.mode == BAR_STANDBY) {
        if (count == t->bar.local_count) {
            t->bar.local_count++;
            t->bar.rounds++;
            t->bar.mode = BAR_IDLE;
            t->bar.initiated = false;
            return true;
        }
        return false;
    }
    if (count < t->bar.local_count) return false;
    if (t->bar.mode != BAR_STANDBY) {
        t->bar.mode = BAR_STANDBY;
        t->bar.initiated = false;
    } else if (prev >= 0 && count != prev) {
        t->bar.rounds++;
    }
    t->bar.local_count = count;
    t->bar.since_initiate = 0;
    return false;
}

static void barrier_host_close(ct_translator_t *t, int32_t count)
{
    t->bar.since_host = 0;
    t->bar.host_count = count;
    t->bar.local_count = count;
    if (t->bar.mode != BAR_CLOSE) {
        t->bar.mode = BAR_CLOSE;
        t->bar.initiated = false;
    }
}

/* Once per host frame. True when a round ended without the leader's echo. */
static bool barrier_observe(ct_translator_t *t, bool saw)
{
    if (t->bar.mode == BAR_CLOSE && t->bar.initiated) {
        /* Our own close: repeat every 60 frames, give up after 5 seconds. */
        if (t->bar.since_initiate && t->bar.since_initiate % 60 == 0) t->bar.burst_n = 0;
        if (++t->bar.since_initiate > 300) {
            t->bar.mode = BAR_IDLE;
            t->bar.initiated = false;
            return true;
        }
        return false;
    }
    if (t->bar.mode != BAR_STANDBY) return false;
    if (saw) { t->bar.since_host = 0; t->bar.since_initiate = 0; return false; }
    t->bar.since_host++;
    if (t->bar.initiated) {
        /* Repeat every 60 frames; the round is not passed until the leader gets there (no timeout). */
        if (t->bar.since_initiate && t->bar.since_initiate % 60 == 0) t->bar.burst_n = 0;
        ++t->bar.since_initiate;
    } else if (t->bar.since_host > IDLE_TIMEOUT) {
        t->bar.local_count++;
        t->bar.rounds++;
        t->bar.mode = BAR_IDLE;
        return true;
    }
    return false;
}

/* ---- wireless side */

/* RfuGameData for the NI exchange. */
static void build_game_data(const ct_translator_t *t, uint8_t out[26])
{
    memset(out, 0, 26);
    ct_put16(out, 2);
    out[12] = (uint8_t)(t->activity | 0x80);
    if (!t->have_ruby_lp) {
        /* Shown as FireRed so the leader takes it without its checks for other versions. */
        ct_put16(out + 2, 2 | COMPAT_CAN_LINK_NATIONALLY | (1 << 8) | (1 << 9) | (VERSION_FIRE_RED << 10));
        memcpy(out + 17, PLACEHOLDER_NAME, sizeof PLACEHOLDER_NAME);
        return;
    }
    unsigned version = t->ruby_lp[LP_VERSION_OFFSET];
    ct_put16(out + 2, (uint16_t)(2 | COMPAT_CAN_LINK_NATIONALLY | (1 << 8) | (1 << 9) | (version << 10)));
    ct_put16(out + 4, ct_le16(t->ruby_lp + LP_TRAINER_ID_OFFSET));
    unsigned i = 0;
    for (; i < 7 && t->ruby_lp[LP_NAME_OFFSET + i] != 0xff; i++) out[17 + i] = t->ruby_lp[LP_NAME_OFFSET + i];
    out[17 + i] = 0xff;
}

/* Everything that belongs to one wireless link. */
static void reset_link(ct_translator_t *t)
{
    t->have_host_lp = false;
    memset(t->host_lp, 0, sizeof t->host_lp);
    bq_clear(&t->held);
    t->pending_pull = -1;
    t->tag = 0;
    t->key_count = 0;
    recv_reset(&t->rx0, t->rx0_buf);
    recv_reset(&t->rx1, NULL);
    t->repair_type = -1;
    t->repeat_due.valid = false;
    send_reset(t);
    bq_clear(&t->child_queue);
    barrier_reset(t);
    t->lp_sent_to_host = false;
    t->lp_send_done = false;
    t->lp_pull_waiting = false;
    t->card_armed = false;
    t->card_requested = false;
    t->have_ruby_card = false;
    t->card_queued = false;
    t->card_pull_waiting = false;
    t->card_send_done = false;
    t->pending_game_standby = false;
    t->round0_started = false;
    t->keys_active = false;
    t->battle_round = false;
    t->battle_state = B_ROOM;
    ct_keyqueue_clear(&t->ruby_keys);
    ct_keyruns_clear(&t->host_keys);
    t->last_host_key_count = -1;
    t->purpose = P_NONE;
    t->host_frames = 0;
    t->child_frames = 0;
    t->silent_frames = 0;
}

static void start_search(ct_translator_t *t)
{
    set_link(t, AIR_SEARCH);
    t->link_frames = 0;
}

/* The child's slot for this frame; all zero is an idle frame. */
static void choose_slot(ct_translator_t *t, uint16_t *w)
{
    memset(w, 0, 7 * sizeof w[0]);
    start_due_round(t);
    if (barrier_want(t, w)) return;
    if (t->bar.mode != BAR_IDLE) return;
    if (t->repair_type >= 0) {
        w[0] = CT_RFUCMD_SEND_BLOCK_REQ;
        w[1] = (uint16_t)t->repair_type;
        t->repair_type = -1;
        return;
    }
    if (!t->snd.active && t->child_queue.n) {
        uint8_t data[CT_MAX_BLOCK_BYTES];
        uint8_t flags;
        size_t size = bq_shift(&t->child_queue, data, &flags);
        send_start(t, data, size, flags & BQ_LP);
        t->snd.is_card = flags & BQ_CARD;
        t->snd.is_battle_lp = flags & BQ_BATTLE_LP;
        recv_reset(&t->rx1, NULL);
    }
    if (t->snd.active) {
        bool was_lp = t->snd.is_lp, was_card = t->snd.is_card;
        /* While the leader's own block is short of fragments, ours stays one short too. */
        const ct_recv_t *rx0 = &t->rx0;
        send_tick(t, w, !battle_blocks(t) && rx0->receiving && !rx0->done && rx0->repairs < REPAIRS);
        if (!t->snd.active && was_card) {
            t->card_send_done = true;
            flush_pending_standby(t);
        } else if (!t->snd.active && was_lp) {
            t->lp_send_done = true;
            maybe_start_round0(t);
        } else if (!t->snd.active && t->snd.is_battle_lp) {
            due_round(t);
        }
        if (w[0]) return;
    }
    if (t->keys_active) {
        w[0] = CT_RFUCMD_SEND_HELD_KEYS;
        w[1] = (uint16_t)((t->key_count++ << 8) | ct_keyqueue_pop(&t->ruby_keys));
    }
}

static void send_uni(ct_translator_t *t, uint16_t *w)
{
    uint8_t frame[2 + CT_SLOT_BYTES];
    if (w[0]) {
        w[0] = (uint16_t)(w[0] | (t->tag << 5));
        t->tag = (t->tag + 1) & 7;
    }
    ct_put16(frame, (LCOM_UNI << 10) | CT_SLOT_BYTES);
    for (unsigned i = 0; i < 7; i++) ct_put16(frame + 2 + i * 2, w[i]);
    air_send(t, frame, sizeof frame);
}

static void push_host_key(ct_translator_t *t, uint8_t code)
{
    ct_keyruns_push(&t->host_keys, code);
    if (t->keys_active) t->key_pushes++;
}

static void handle_host_frame(ct_translator_t *t, const uint8_t *data, size_t len)
{
    t->host_frames++;
    if (len < 3) {
        /* A bare frame still asks the child to speak; our game data goes first. */
        if (t->link == AIR_NI && t->ni.state < 4) {
            uint8_t out[16];
            size_t n = ni_next(&t->ni, out);
            if (n) air_send(t, out, n);
        }
        return;
    }
    uint32_t header = data[0] | (data[1] << 8) | ((uint32_t)data[2] << 16);
    unsigned state = (header >> 14) & 0xf, ack = (header >> 13) & 1, n = (header >> 11) & 3, phase = (header >> 9) & 3;

    if (state != LCOM_UNI) {
        /* The leader's own NI transfer: acknowledge each sub-frame by mirroring it. */
        if (t->link != AIR_NI) return;
        uint8_t out[16];
        size_t o = 0;
        if (t->ni.state < 4) o = ni_next(&t->ni, out);
        else if (!ack && (state == LCOM_NI_START || state == LCOM_NI || state == LCOM_NI_END)) {
            ct_put16(out, child_llsf(state, n, phase, 1, 0));
            o = 2;
        }
        if (o) air_send(t, out, o);
        return;
    }

    if (len < HOST_FRAME_BYTES) return;
    if (t->link == AIR_NI) {
        if (t->ni.state < 4) return;
        set_link(t, AIR_UNI);
    }

    /* Slot 0 is the leader's own command, slot 1 the reflection of ours. */
    const uint8_t *slot0 = data + 3, *slot1 = data + 3 + CT_SLOT_BYTES;
    bool saw = false;
    bool idle0 = true, idle1 = true;
    for (unsigned i = 0; i < CT_SLOT_BYTES; i++) { if (slot0[i]) idle0 = false; if (slot1[i]) idle1 = false; }
    if (!idle0) {
        uint16_t w0[7];
        for (unsigned i = 0; i < 7; i++) w0[i] = ct_le16(slot0 + i * 2);
        unsigned op = w0[0] & CT_RFUCMD_MASK;
        switch (op) {
        case CT_RFUCMD_SEND_BLOCK_REQ:
            host_pull(t, w0[1]);
            break;
        case CT_RFUCMD_SEND_BLOCK_INIT:
        case CT_RFUCMD_SEND_BLOCK: {
            /* A block of a request's size with no request just before it is answered as that request. */
            if (op == CT_RFUCMD_SEND_BLOCK_INIT && (!t->rx0.receiving || t->rx0.done) && !battle_blocks(t)
                && request_type(w0[1]) >= 0 && (int64_t)t->host_frames - t->pull_at > PULL_WITH_BLOCK_FRAMES)
                host_pull(t, (unsigned)request_type(w0[1]));
            bool whole = feed_recv(&t->rx0, t->rx0_buf, w0, slot0);
            t->rx0.heard_at = t->host_frames;
            if (whole) {
                unsigned count = t->rx0.need ? t->rx0.need : t->rx0.count;
                size_t blen = count * CT_FRAG_BYTES;
                bool drop = t->repeat_due.valid && !t->rx0.repairs && (int64_t)t->host_frames <= t->repeat_due.until
                            && t->rx0.count == t->repeat_due.count && memcmp(t->repeat_due.data, t->rx0_buf, t->repeat_due.len) == 0;
                if (t->rx0.repairs) {
                    int rt = repair_type(count);
                    t->repeat_due.valid = true;
                    t->repeat_due.count = rt >= 0 ? REQUEST_FRAGMENTS[rt] : 0;   /* undefined never matches */
                    t->repeat_due.len = (uint16_t)blen;
                    t->repeat_due.until = (int64_t)t->host_frames + REPEAT_FRAMES;
                    memcpy(t->repeat_due.data, t->rx0_buf, blen);
                } else {
                    t->repeat_due.valid = false;
                }
                /* The repeat a repair asked for, after the block came whole anyway, is dropped. */
                if (!drop) host_block(t, count, t->rx0_buf);
                t->rx0.receiving = false;
                t->rx0.done = false;
                t->rx0.flags = 0;
            }
            break;
        }
        case CT_RFUCMD_SEND_HELD_KEYS:
            /* The high byte counts the leader's frames; a repeated count is the same event. */
            if (t->last_host_key_count != (w0[1] >> 8)) {
                t->last_host_key_count = w0[1] >> 8;
                push_host_key(t, (uint8_t)(w0[1] & 0xff));
                if ((w0[1] & 0xff) == CT_KEY_EXIT_ROOM) t->exit_key_seen = true;
            }
            break;
        case CT_RFUCMD_READY_EXIT_STANDBY:
            saw = true;
            if (barrier_host_standby(t, w0[1])) round_passed(t);
            break;
        case CT_RFUCMD_READY_CLOSE_LINK:
            saw = true;
            barrier_host_close(t, w0[1]);
            t->host_closed_seen = true;
            if (t->purpose == P_EXIT_CLOSE) exit_close_passed(t);
            break;
        }
    }
    if (!idle1) {
        uint16_t w1[7];
        for (unsigned i = 0; i < 7; i++) w1[i] = ct_le16(slot1 + i * 2);
        feed_recv(&t->rx1, NULL, w1, slot1);
    }
    ct_recv_t *rx0 = &t->rx0;
    /* Not in a battle: a block that stopped short is asked for again. */
    if (!battle_blocks(t) && rx0->receiving && !rx0->done && repair_type(rx0->need ? rx0->need : rx0->count) >= 0
        && rx0->repairs < REPAIRS && t->host_frames - rx0->heard_at >= REPAIR_QUIET_FRAMES) {
        if (!rx0->need) rx0->need = rx0->count;
        rx0->repairs++;
        rx0->heard_at = t->host_frames;
        t->repair_type = (int8_t)repair_type(rx0->need);
    }
    if (barrier_observe(t, saw)) round_passed(t);
    if (t->link != AIR_UNI) return;   /* the exit above ended the link */

    uint16_t out[7];
    choose_slot(t, out);
    send_uni(t, out);
}

void ct_translator_board_frame(ct_translator_t *t, const uint8_t *frame, size_t len)
{
    if (len < 12) return;
    uint32_t type = ct_be32(frame + 4), header = ct_be32(frame + 8);
    if (type == CT_RFU1_BROADCAST) {
        if (t->link != AIR_SEARCH || len < 28) return;
        unsigned occupied = (header >> 16) & 1;
        /* RfuGameData: word 3's low byte is the activity (trade = 4). */
        unsigned room = ct_be32(frame + 24) & 0x7f;
        if (occupied) return;
        if (t->cfg.hold_join && !t->have_ruby_lp) {
            if (room != ACT_TRADE && (!t->cfg.battle || (room != ACT_SINGLE && room != ACT_DOUBLE))) return;
            t->activity = (uint8_t)room;
        } else if (room != t->activity) return;
        t->host_id = header & 0xffff;
        if (!t->join_allowed) {
            if (!t->group_seen) {
                t->group_seen = true;
                event(t, CT_TR_EV_GROUP_SEEN, ct_translator_linked(t));
            }
            return;
        }
        set_link(t, AIR_CONNECTING);
        t->link_frames = 0;
        t->connect_frames = 0;
        board_cmd(t, CT_RFU1_CONNECT_REQ, t->host_id);
        return;
    }
    if (type == CT_RFU1_CONNECT_ACK) {
        if (t->link != AIR_CONNECTING) return;
        uint8_t src[26];
        build_game_data(t, src);
        reset_link(t);
        new_ni(t, src);
        set_link(t, AIR_NI);
        t->link_frames = 0;
        return;
    }
    if (type == CT_RFU1_DISCONNECT) {
        if (t->link == AIR_NI || t->link == AIR_UNI) { reset_link(t); start_search(t); }
        return;
    }
    if (type == CT_RFU1_HOST_SEND && (t->link == AIR_NI || t->link == AIR_UNI)) {
        size_t length = header & 0x7f;
        if (length > CT_RFU1_SEND_DATA) length = CT_RFU1_SEND_DATA;
        if (length > len - 12) length = len - 12;
        if (length == 0) return;
        t->silent_frames = 0;
        handle_host_frame(t, frame + 12, length);
    }
}

void ct_translator_frame(ct_translator_t *t)
{
    if (t->link < 0 && (t->have_ruby_lp || t->cfg.hold_join)) start_search(t);
    if (t->link == AIR_CONNECTING && ++t->connect_frames > 8 * 60) start_search(t);
    /* The child speaks first after the connect: repeat the NI_START every few frames until the leader's first frame. */
    if (t->link == AIR_NI && t->host_frames == 0 && t->ni.state == 0 && ++t->link_frames % 8 == 0) {
        ct_ni_t copy = t->ni;
        uint8_t out[16];
        size_t n = ni_next(&copy, out);
        if (n) air_send(t, out, n);
    }
    if (t->link == AIR_NI || t->link == AIR_UNI) {
        if (++t->silent_frames > HOST_SILENT_FRAMES) {
            reset_link(t);
            board_cmd(t, CT_RFU1_DISCONNECT, 0);
            start_search(t);
        }
    }
}

/* ---- cable side */

void ct_translator_cable_reset(ct_translator_t *t)
{
    t->join_allowed = true;
    t->link_closing = false;
    t->cable_packets = 0;
    t->ready_at = 0;
    t->announced = false;
    t->expect = EXPECT_NONE;
    t->session_ruby_lp = false;
    t->p0_lp_delivered = false;
    t->rx_size = 0;
    t->rx_pos = 0;
    t->cq_n = t->cq_head = 0;
    t->sessions++;
}

void ct_translator_game_command(ct_translator_t *t, const uint16_t w[CT_CMD_WORDS])
{
    switch (w[0]) {
    case CT_LINKCMD_INIT_BLOCK:
        t->rx_size = w[1] <= CT_MAX_BLOCK_BYTES ? w[1] : 0;
        t->rx_pos = 0;
        break;
    case CT_LINKCMD_CONT_BLOCK:
        if (t->rx_size) {
            for (unsigned i = 0; i < CT_CMD_WORDS - 1; i++) {
                unsigned at = t->rx_pos + i * 2;
                if (at + 1 < sizeof t->rx_buf) ct_put16(t->rx_buf + at, w[1 + i]);
            }
            t->rx_pos = (uint16_t)(t->rx_pos + (CT_CMD_WORDS - 1) * 2);
            if (t->rx_pos >= t->rx_size) {
                uint16_t size = t->rx_size;
                t->rx_size = 0;
                game_block(t, size, t->rx_buf);
            }
        }
        break;
    case CT_LINKCMD_SEND_HELD_KEYS:
        ct_keyqueue_push(&t->ruby_keys, (uint8_t)(w[1] & 0xff));
        if ((w[1] & 0xff) == CT_KEY_EXIT_ROOM) t->exit_key_seen = true;
        break;
    case CT_LINKCMD_READY_EXIT_STANDBY:
        game_standby(t);
        break;
    case CT_LINKCMD_READY_CLOSE_LINK:
        game_close(t);
        break;
    }
}

bool ct_translator_next_command(ct_translator_t *t, uint16_t w[CT_CMD_WORDS])
{
    t->cable_packets++;
    if (t->held.n || t->pending_pull >= 0) release_held(t);
    if (t->card_armed && t->cable_packets >= t->card_at) {
        t->card_armed = false;
        request_early_card(t);
    }
    if (!t->announced && t->cable_packets > ANNOUNCE_PACKETS) {
        /* The master asks for the player data exchange. */
        t->announced = true;
        t->expect = EXPECT_LP;
        t->lp_early_wait = 0;
        memset(w, 0, CT_CMD_WORDS * sizeof w[0]);
        w[0] = CT_LINKCMD_SEND_LINK_TYPE;
        w[1] = 0x1133;
        return true;
    }
    if (t->cfg.hold_join && t->expect == EXPECT_LP && t->have_host_lp && !t->session_ruby_lp && !t->p0_lp_delivered
        && ++t->lp_early_wait > LP_EARLY_PACKETS)
        deliver_lp_to_game(t, true);
    if (t->cq_n) {
        memcpy(w, t->cq[t->cq_head], CT_CMD_WORDS * sizeof w[0]);
        t->cq_head = (uint8_t)((t->cq_head + 1) % CT_TR_CABLE_QUEUE);
        t->cq_n--;
        return true;
    }
    if (t->keys_active) {
        if (++t->key_pops >= CT_KEY_RATE_WINDOW) {
            double s = (double)t->key_pushes / (double)t->key_pops;
            t->key_step = s < 1 ? 1 : s > 3 ? 3 : s;
            t->key_pushes = t->key_pops = 0;
        }
        memset(w, 0, CT_CMD_WORDS * sizeof w[0]);
        w[0] = CT_LINKCMD_SEND_HELD_KEYS;
        w[1] = ct_keyruns_pop(&t->host_keys, t->key_step);
        return true;
    }
    return false;
}

/* ---- setup */

static void game_command_cb(void *ctx, const uint16_t w[CT_CMD_WORDS]) { ct_translator_game_command(ctx, w); }
static bool next_command_cb(void *ctx, uint16_t w[CT_CMD_WORDS]) { return ct_translator_next_command(ctx, w); }
static void reset_cb(void *ctx) { ct_translator_cable_reset(ctx); }
static bool ready_cb(void *ctx) { return ct_translator_ready(ctx); }

ct_game_t ct_translator_game(ct_translator_t *t)
{
    ct_game_t g = { t, game_command_cb, next_command_cb, reset_cb, ready_cb };
    return g;
}

void ct_translator_init(ct_translator_t *t, ct_frame_fn send, void *send_ctx, const ct_translator_cfg_t *cfg)
{
    memset(t, 0, sizeof *t);
    t->send = send;
    t->send_ctx = send_ctx;
    t->cfg = *cfg;
    t->join_allowed = !cfg->hold_join;
    t->link = AIR_NONE;
    barrier_reset(t);
    recv_reset(&t->rx0, t->rx0_buf);
    recv_reset(&t->rx1, NULL);
    t->repair_type = -1;
    t->pull_at = NEVER;
    send_reset(t);
    new_ni(t, NULL);
    t->key_step = 1.5;
    t->last_host_key_count = -1;
    t->pending_pull = -1;
    t->activity = ACT_TRADE;
    t->battle_state = B_ROOM;
}
