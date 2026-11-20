/* SPDX-License-Identifier: AGPL-3.0-only */
/* ct_reverse.c -- ReverseTranslator, ported from GB-Link-Switch-LDN web/js/cable/reverse.js (commit c972db4,
 * https://github.com/GB-Link/GB-Link-Switch-LDN). Copyright (c) the GB-Link-Switch-LDN authors. Modified for kizuna
 * 2026-11-02: C port, fixed storage, Ruby/Sapphire dressing / Emerald refusal and bypass dropped, battle gated (see
 * ct_reverse.h). AGPL-3.0, see NOTICE. Function names follow the JS methods. */
#include "ct_reverse.h"

#include <string.h>

enum { LP_SIZE = 60, LP_ID = 16 + 0x18, LP_LINK_TYPE = 16 + 0x14, BATTLE_LP_SIZE = 28 };
enum { LT_TRADE_ROOM = 0x1133, LT_ROOM_AGAIN = 0x1111, LT_TRADE_MENU = 0x1122, LT_TRADE_SCENE = 0x1144,
       LT_SINGLE = 0x2233, LT_DOUBLE = 0x2244, LT_BATTLE = 0x2211 };
enum { MENU_START = 0xccdd, MENU_BOTH_CANCEL = 0xeebb };
enum { SAVE_STANDBYS = 3, AFTER_EXTRA_FRAMES = 30, EXTRA_ROUND_DEADLINE = 360, MENU_BLOCK_SIZE = 20, MENU_BLOCK_SENT = 36,
       LP_WAIT_PACKETS = 20, REQUEST_GAP_FRAMES = 15, REREQUEST_FRAMES = 60, REREQUESTS = 3 };
enum { NEXT_LOCAL, NEXT_STANDBY, NEXT_CLOSE, NEXT_REOPEN, NEXT_EXTRA };
enum { ACT_EXIT, ACT_BATTLE, ACT_SYNC };
enum { DEF_REQUEST, DEF_BLOCK };
#define NEVER_BLOCK (-((int64_t)1 << 40))   /* -Infinity */
#define FOREVER INT64_MAX                    /* Infinity */

static const uint8_t PLACEHOLDER_NAME[8] = { 0xbd, 0xbf, 0xc6, 0xc3, 0xc9, 0xff, 0, 0 };   /* "CELIO" */

static uint16_t activity_link_type(uint8_t a) { return a == 1 ? LT_SINGLE : a == 2 ? LT_DOUBLE : LT_TRADE_ROOM; }
static bool is_battle(const ct_reverse_t *r) { return r->activity != 4; }
static bool is_link_player(const uint8_t *b, size_t len) { return len >= LP_SIZE && ct_is_magic(b) && ct_is_magic(b + 44); }
static void event(ct_reverse_t *r, int ev, uint32_t arg) { if (r->cfg.event) r->cfg.event(r->cfg.event_ctx, ev, arg); }

/* ---- small rings */
static void ring_push(ct_reverse_t *r, ct_u16ring_t *q, uint16_t v)
{
    if (q->n >= CT_REVERSE_ROUNDS) { r->ring_drops++; return; }
    q->q[(q->head + q->n) % CT_REVERSE_ROUNDS] = v;
    q->n++;
}
static uint16_t ring_shift(ct_u16ring_t *q)
{
    if (!q->n) return 0;
    uint16_t v = q->q[q->head];
    q->head = (uint8_t)((q->head + 1) % CT_REVERSE_ROUNDS);
    q->n--;
    return v;
}

/* ---- cable queue: a command for the parent; held while its link is closed */
static void cmd_put(ct_cmd_t *q, uint8_t head, uint8_t *n, unsigned cap, const uint16_t *w, uint32_t *drops)
{
    if (*n >= cap) { (*drops)++; return; }
    memcpy(q[(head + *n) % cap].w, w, sizeof q[0].w);
    (*n)++;
}
static void cable_push(ct_reverse_t *r, const uint16_t w[CT_CMD_WORDS])
{
    if (r->live) cmd_put(r->cq, r->cq_head, &r->cq_n, CT_REVERSE_CQ, w, &r->cq_drops);
    else cmd_put(r->held, r->held_head, &r->held_n, CT_REVERSE_HELD, w, &r->held_drops);
}
static void cable_cmd(ct_reverse_t *r, uint16_t c0, uint16_t c1, uint16_t c2)
{
    uint16_t w[CT_CMD_WORDS] = { c0, c1, c2 };
    cable_push(r, w);
}
/* A block to the parent: INIT_BLOCK, then CONT_BLOCK chunks of seven words. `avail` bytes of data are real, the rest
 * reads as zero (data[at] ?? 0). */
static void cable_push_block(ct_reverse_t *r, const uint8_t *data, size_t avail, size_t size)
{
    cable_cmd(r, CT_LINKCMD_INIT_BLOCK, (uint16_t)size, 1 + 128);
    for (size_t pos = 0; pos < size; pos += (CT_CMD_WORDS - 1) * 2) {
        uint16_t w[CT_CMD_WORDS] = { CT_LINKCMD_CONT_BLOCK };
        for (unsigned i = 0; i < CT_CMD_WORDS - 1; i++) {
            size_t at = pos + i * 2;
            unsigned lo = at < size && at < avail ? data[at] : 0;
            unsigned hi = at + 1 < size && at + 1 < avail ? data[at + 1] : 0;
            w[1 + i] = (uint16_t)(lo | (hi << 8));
        }
        cable_push(r, w);
    }
}

/* ---- the Switch's side helpers */

static void leader_send_block(ct_reverse_t *r, const uint8_t *data, size_t len) { ct_leader_send_block(r->leader, data, len, 1); }

static void run_deferred(ct_reverse_t *r, unsigned i)
{
    if (r->deferred[i].kind == DEF_REQUEST) {
        ct_leader_request(r->leader, r->deferred[i].type);
        r->request_at = r->leader->ticks;
    } else {
        leader_send_block(r, r->deferred_buf + r->deferred[i].off, r->deferred[i].len);
    }
}

/* Requests and blocks for the Switch, in order; held through the quiet after a trade. */
static void to_switch(ct_reverse_t *r, uint8_t kind, uint8_t type, const uint8_t *data, size_t len)
{
    if ((int64_t)r->leader->ticks < r->quiet_until || r->deferred_n) {
        if (r->deferred_n >= CT_REVERSE_DEFERRED || r->deferred_used + len > CT_REVERSE_DEFERRED_BYTES) { r->deferred_drops++; return; }
        unsigned i = r->deferred_n++;
        r->deferred[i].kind = kind;
        r->deferred[i].type = type;
        r->deferred[i].len = (uint16_t)len;
        r->deferred[i].off = r->deferred_used;
        if (len) memcpy(r->deferred_buf + r->deferred_used, data, len);
        r->deferred_used = (uint16_t)(r->deferred_used + len);
        return;
    }
    if (kind == DEF_REQUEST) {
        ct_leader_request(r->leader, type);
        r->request_at = r->leader->ticks;
    } else {
        leader_send_block(r, data, len);
    }
}

static void request_from_switch(ct_reverse_t *r, uint16_t type)
{
    r->pull = (int16_t)type;
    r->cable_request = (int16_t)type;
    r->rerequests = 0;
    int64_t gap = r->switch_block_at + REQUEST_GAP_FRAMES;
    if (gap > r->quiet_until) r->quiet_until = gap;
    to_switch(r, DEF_REQUEST, (uint8_t)type, NULL, 0);
}

/* A request the Switch has not started to answer is asked again. */
static void repeat_request(ct_reverse_t *r)
{
    if (r->pull < 0 || !r->request_at || r->leader->ticks - r->request_at < REREQUEST_FRAMES) return;
    if (ct_leader_receiving(r->leader)) return;
    if (r->rerequests >= REREQUESTS) { r->request_at = 0; return; }
    r->rerequests++;
    r->request_at = r->leader->ticks;
    ct_leader_request(r->leader, (uint16_t)r->pull);
}

static void end_extra(ct_reverse_t *r)
{
    r->next = NEXT_CLOSE;
    r->quiet_until = (int64_t)r->leader->ticks + AFTER_EXTRA_FRAMES;
}

static void release_deferred(ct_reverse_t *r)
{
    repeat_request(r);
    if (r->next == NEXT_EXTRA && (int64_t)r->leader->ticks >= r->extra_deadline) end_extra(r);
    if (!r->deferred_n || (int64_t)r->leader->ticks < r->quiet_until) return;
    unsigned n = r->deferred_n;
    r->deferred_n = 0;   /* splice(0): the sends do not defer again */
    for (unsigned i = 0; i < n; i++) run_deferred(r, i);
    r->deferred_used = 0;
}

/* ---- LinkPlayers */

/* The Switch's player as the other GBA sees it: as itself, with this side's link type. */
static void for_parent(const ct_reverse_t *r, uint8_t out[LP_SIZE])
{
    memcpy(out, r->switch_lp, LP_SIZE);
    ct_put16(out + LP_LINK_TYPE, r->link_type);
    out[LP_LINK_TYPE + 2] = 0;
    out[LP_LINK_TYPE + 3] = 0;
    /* In the Colosseum the id is the spot each player stands on. */
    if (r->link_type == LT_BATTLE && r->have_switch_battle_lp) out[LP_ID] = r->switch_battle_lp[0x18];
    else if (is_battle(r)) out[LP_ID] = r->have_parent_seen ? r->parent_seen[LP_ID] ^ 1 : 1;
}

/* The leader's LinkPlayer: the other GBA's, once it is known. */
static void maybe_leader_lp(ct_reverse_t *r)
{
    if (r->leader_lp_sent || !r->have_switch_lp || !r->have_parent_lp) return;
    r->leader_lp_sent = true;
    uint8_t block[200] = { 0 };
    memcpy(block, r->parent_lp, LP_SIZE);
    memset(block + LP_LINK_TYPE, 0, 4);
    leader_send_block(r, block, sizeof block);
}

static void send_leader_battle_lp(ct_reverse_t *r)
{
    if (r->leader_battle_lp_sent || !r->have_parent_lp) return;
    r->leader_battle_lp_sent = true;
    uint8_t out[BATTLE_LP_SIZE];
    memcpy(out, r->parent_lp + 16, BATTLE_LP_SIZE);
    ct_put16(out + 0x14, LT_BATTLE);
    out[0x16] = 0; out[0x17] = 0;
    out[0x18] = r->have_switch_battle_lp ? r->switch_battle_lp[0x18] ^ 1 : 0;
    leader_send_block(r, out, sizeof out);
}

static void maybe_start_exchange(ct_reverse_t *r)
{
    if (!r->exchange_due || !(r->have_parent_early || r->have_parent_lp)) return;
    r->exchange_due = false;
    ct_leader_player_ids(r->leader);
    r->pull = 0;
    ct_leader_request(r->leader, 0);
}

static void send_own_lp(ct_reverse_t *r)
{
    if (!r->have_switch_lp) return;
    r->lp_pending = false;
    r->live = true;
    uint8_t lp[LP_SIZE];
    for_parent(r, lp);
    cable_push_block(r, lp, LP_SIZE, LP_SIZE);
    while (r->held_n) {   /* cq.push(...held.splice(0)) */
        cmd_put(r->cq, r->cq_head, &r->cq_n, CT_REVERSE_CQ, r->held[r->held_head].w, &r->cq_drops);
        r->held_head = (uint8_t)((r->held_head + 1) % CT_REVERSE_HELD);
        r->held_n--;
    }
}

/* ---- pairing the Switch's rounds with the parent's */

static void closed(ct_reverse_t *r, uint16_t next_link_type)
{
    r->live = false;
    r->link_type = next_link_type;
    r->menu_outcome = 0;
    r->reopen_token = false;
}

static void answer_switch(ct_reverse_t *r, uint16_t count)
{
    ring_shift(&r->switch_rounds);
    ct_leader_standby(r->leader, count);
}

static void after_standby(ct_reverse_t *r)
{
    if (r->link_type == LT_TRADE_SCENE) {
        /* The save after a trade: three standbys, then the close back to the menu. */
        r->next = ++r->save_standbys < SAVE_STANDBYS ? NEXT_STANDBY : NEXT_CLOSE;
        return;
    }
    /* Into the room: keys until the next close; the other GBA's idle reports from before are dropped. */
    r->next = NEXT_CLOSE;
    ct_keyqueue_clear(&r->parent_keys);
    ct_steps_reset(&r->steps);
    r->keys_to_parent = true;
    r->keys_to_switch = true;
}

/* A close for the next part of the flow: the link type of the link the parent opens next. */
static void after_close(ct_reverse_t *r)
{
    switch (r->link_type) {
    case LT_TRADE_MENU:
        if (r->menu_outcome == MENU_START) {
            /* Into the trade scene: its link comes without a round of the Switch's. */
            closed(r, LT_TRADE_SCENE);
            r->save_standbys = 0;
            r->next = NEXT_STANDBY;
            return;
        }
        closed(r, r->menu_outcome == MENU_BOTH_CANCEL ? LT_ROOM_AGAIN : LT_TRADE_MENU);
        break;
    case LT_TRADE_SCENE:
        closed(r, LT_TRADE_MENU);
        r->after_trade = true;
        break;
    case LT_BATTLE:
        r->in_battle = false;
        closed(r, activity_link_type(r->activity));
        break;
    default:
        closed(r, LT_TRADE_MENU);
        break;
    }
    r->next = NEXT_REOPEN;
}

/* Sends this side's 0x2FFE or 0x5FFF; the step is done once the parent's has come. */
static bool send_step(ct_reverse_t *r, uint16_t kind, uint8_t action, uint16_t count)
{
    cable_cmd(r, kind, 0, 0);
    if (kind == CT_LINKCMD_READY_CLOSE_LINK) {
        /* Blocks from here on wait for the next link. */
        r->live = false;
        r->keys_to_parent = false;
        r->keys_to_switch = false;
        r->keys_await_switch = false;
    }
    r->step.active = true;
    r->step.kind = kind;
    r->step.action = action;
    r->step.count = count;
    return true;
}

static void step_done(ct_reverse_t *r, uint8_t action, uint16_t kind, uint16_t count)
{
    switch (action) {
    case ACT_EXIT:
        r->ended = true;
        ct_leader_close_link(r->leader, r->switch_close_count);
        event(r, CT_REVERSE_EV_ENDED, 0);
        break;
    case ACT_BATTLE:
        closed(r, LT_BATTLE);
        r->in_battle = true;
        r->next = NEXT_REOPEN;
        break;
    default:
        answer_switch(r, count);
        if (kind == CT_LINKCMD_READY_EXIT_STANDBY) after_standby(r);
        else after_close(r);
        break;
    }
}

/* Starts the next step from the Switch's side: false when there is none yet. */
static bool start_step(ct_reverse_t *r)
{
    if (r->ended) return false;
    if (r->exit_pending) {
        /* The Switch's exit key, still on its way to the cable, goes first. */
        if (ct_keyruns_has(&r->switch_keys, CT_KEY_EXIT_ROOM, CT_KEY_READY)) return false;
        r->exit_pending = false;
        return send_step(r, CT_LINKCMD_READY_CLOSE_LINK, ACT_EXIT, 0);
    }
    if (r->battle_pending) {
        r->battle_pending = false;
        return send_step(r, CT_LINKCMD_READY_CLOSE_LINK, ACT_BATTLE, 0);
    }
    if (!r->switch_rounds.n) return false;
    uint16_t count = r->switch_rounds.q[r->switch_rounds.head];
    switch (r->next) {
    case NEXT_LOCAL: {
        /* Its first round follows the player exchange; the cable has none there. */
        if (!r->leader_lp_sent) return false;
        r->s1_done = true;
        answer_switch(r, count);
        r->next = NEXT_STANDBY;
        uint16_t held[8];
        unsigned n = r->held_requests_n;
        memcpy(held, r->held_requests, sizeof held);
        r->held_requests_n = 0;
        for (unsigned i = 0; i < n; i++) request_from_switch(r, held[i]);
        return true;
    }
    case NEXT_REOPEN:
        if (!r->reopen_token) return false;
        r->reopen_token = false;
        answer_switch(r, count);
        r->next = NEXT_CLOSE;
        if (r->after_trade) {
            r->after_trade = false;
            r->next = NEXT_EXTRA;
            r->quiet_until = FOREVER;
            r->extra_deadline = (int64_t)r->leader->ticks + EXTRA_ROUND_DEADLINE;
        }
        return true;
    case NEXT_EXTRA:
        answer_switch(r, count);
        end_extra(r);
        return true;
    }
    /* A standby or a close: the parent's own, if it came first, decides which. */
    uint16_t guess = r->next == NEXT_STANDBY ? CT_LINKCMD_READY_EXIT_STANDBY : CT_LINKCMD_READY_CLOSE_LINK;
    uint16_t kind = r->parent_syncs.n ? r->parent_syncs.q[r->parent_syncs.head] : guess;
    return send_step(r, kind, ACT_SYNC, count);
}

static void pair(ct_reverse_t *r)
{
    for (;;) {
        if (!r->step.active && !start_step(r)) return;
        if (!r->step.active) continue;
        if (!r->parent_syncs.n) return;
        ring_shift(&r->parent_syncs);   /* a mismatch with step.kind is only logged in the JS */
        r->step.active = false;
        step_done(r, r->step.action, r->step.kind, r->step.count);
    }
}

/* ---- the Switch (wireless child), through the leader's hooks */

static void switch_joined(void *ctx, const uint8_t game_data[26])
{
    (void)game_data;
    ct_reverse_t *r = ctx;
    r->exchange_due = true;
    maybe_start_exchange(r);
}

static void switch_command(void *ctx, const uint16_t w[7])
{
    ct_reverse_t *r = ctx;
    unsigned op = w[0] & 0xff00;
    if (op == 0x6600) {
        if (w[1] == 0xffff || (int32_t)w[1] == r->last_round) return;   /* 0xFFFF: the board answered it */
        r->last_round = w[1];
        ring_push(r, &r->switch_rounds, w[1]);
        pair(r);
    } else if (op == 0x5f00) {
        if (r->exit_pending || r->ended) return;
        r->switch_close_count = w[1];
        r->exit_pending = true;
        pair(r);
    } else if (op == 0xbe00) {
        if (r->keys_await_switch) {
            r->keys_await_switch = false;
            ct_keyqueue_clear(&r->parent_keys);
            r->keys_to_switch = true;
        }
        ct_steps_report(&r->steps, (uint8_t)(w[1] & 0xff), (uint8_t)(w[1] >> 8));
        if (r->keys_to_parent) r->key_pushes++;
    }
}

static void parent_block(ct_reverse_t *r, const uint8_t *data, size_t len);

static void switch_block(void *ctx, unsigned count, const uint8_t *data)
{
    ct_reverse_t *r = ctx;
    size_t len = count * CT_FRAG_BYTES;
    r->switch_block_at = r->leader->ticks;
    r->request_at = 0;
    if (r->pull == 0 && is_link_player(data, len)) {
        r->pull = -1;
        if (!r->have_switch_lp) {
            memcpy(r->switch_lp, data, LP_SIZE);
            r->have_switch_lp = true;
            event(r, CT_REVERSE_EV_READY, 0);
        }
        if (r->have_parent_early) {
            uint8_t early[LP_SIZE];
            memcpy(early, r->parent_early, LP_SIZE);
            r->have_parent_early = false;
            parent_block(r, early, LP_SIZE);
        }
        maybe_leader_lp(r);
        return;
    }
    if (r->cfg.battle && !r->in_battle && count == (BATTLE_LP_SIZE + 11) / 12 && data[1] == 0x40 && data[0] >= 1 && data[0] <= 5) {
        memcpy(r->switch_battle_lp, data, BATTLE_LP_SIZE);
        r->have_switch_battle_lp = true;
        r->leader_battle_lp_sent = false;
        send_leader_battle_lp(r);
        r->battle_pending = true;
        pair(r);
        return;
    }
    if (r->pull >= 0 && r->cable_request == r->pull) {
        size_t size = ct_size_from_request((unsigned)r->pull);
        r->pull = -1;
        r->cable_request = -1;
        cable_push_block(r, data, len, size);
        return;
    }
    size_t size = r->in_battle ? len : ct_size_from_count(count);
    cable_push_block(r, data, len, size);
}

static bool key_source(void *ctx, uint8_t *code)
{
    ct_reverse_t *r = ctx;
    if (!r->keys_to_switch) {
        if (!r->keys_drain || !r->parent_keys.n) { r->keys_drain = false; return false; }
        *code = ct_keyqueue_pop(&r->parent_keys);
        return true;
    }
    *code = ct_keyqueue_pop(&r->parent_keys);
    return true;
}

static void on_tick(void *ctx) { release_deferred(ctx); }

static void push_switch_key(void *ctx, uint8_t code) { ct_keyruns_push(&((ct_reverse_t *)ctx)->switch_keys, code); }

/* ---- the parent (cable) */

static void link_up(ct_reverse_t *r)
{
    r->expect_parent_lp = true;
    /* The Switch's player comes once the other GBA's has started its exchange; on the first link the other GBA's
     * LinkPlayer decides how the Switch's is shown. A GBA parent sends its own with the 0x2222. */
    if (!r->have_switch_lp || !r->have_parent_lp) { r->lp_pending = true; r->lp_wait = 0; return; }
    send_own_lp(r);
}

static void parent_block(ct_reverse_t *r, const uint8_t *data, size_t len)
{
    if (r->expect_parent_lp && is_link_player(data, len)) {
        /* Its checks need the Switch's player, which its arrival lets the Switch send. */
        if (!r->have_switch_lp) {
            memcpy(r->parent_early, data, LP_SIZE);
            r->have_parent_early = true;
            maybe_start_exchange(r);
            return;
        }
        r->expect_parent_lp = false;
        uint16_t type = ct_le16(data + LP_LINK_TYPE);
        memcpy(r->parent_seen, data, LP_SIZE);
        r->have_parent_seen = true;
        if (r->lp_pending) send_own_lp(r);
        if (!r->have_parent_lp && type != r->link_type) {
            /* The other GBA says the link partners made different selections. */
            r->other_choice = type;
            event(r, CT_REVERSE_EV_OTHER_CHOICE, type);
            return;
        }
        r->other_choice = -1;
        if (!r->have_parent_lp) { memcpy(r->parent_lp, data, LP_SIZE); r->have_parent_lp = true; }
        if (!r->leader_lp_sent) maybe_leader_lp(r);
        else if (r->link_type != LT_TRADE_SCENE) {
            /* A reopened link: the Switch's round waiting for it can pass, and in a room the keys go on. */
            r->reopen_token = true;
            if (r->link_type != LT_BATTLE && r->link_type != LT_TRADE_MENU) {
                ct_steps_reset(&r->steps);
                r->keys_to_parent = true;
                /* Keys reach the Switch once it reports its own: after a battle it may still be saving. */
                r->keys_to_switch = false;
                r->keys_await_switch = true;
            }
        }
        pair(r);
        return;
    }
    /* The parent's own answer to its request, or its trade-menu and battle blocks. */
    if (len == 20 && r->link_type == LT_TRADE_MENU) {
        uint16_t command = ct_le16(data);
        if (command == MENU_START || command == MENU_BOTH_CANCEL) r->menu_outcome = command;
    }
    size_t size = r->cable_request >= 0 ? ct_size_from_request((unsigned)r->cable_request) : len;
    size_t out_len = size == MENU_BLOCK_SIZE && !r->in_battle ? MENU_BLOCK_SENT : size;
    uint8_t block[CT_MAX_BLOCK_BYTES] = { 0 };
    memcpy(block, data, size < len ? size : len);
    to_switch(r, DEF_BLOCK, 0, block, out_len);
}

void ct_reverse_cable_reset(ct_reverse_t *r)
{
    r->expect_parent_lp = false;
    r->rx_size = 0;
    r->cq_n = r->cq_head = 0;
    r->live = false;
    r->keys_to_parent = false;
}

void ct_reverse_game_command(ct_reverse_t *r, const uint16_t w[CT_CMD_WORDS])
{
    switch (w[0]) {
    case CT_LINKCMD_SEND_LINK_TYPE:
        link_up(r);
        break;
    case CT_LINKCMD_INIT_BLOCK:
        if (w[1] > CT_MAX_BLOCK_BYTES) r->parent_block_too_long++;
        r->rx_size = w[1] <= CT_MAX_BLOCK_BYTES ? w[1] : 0;
        r->rx_pos = 0;
        break;
    case CT_LINKCMD_CONT_BLOCK:
        if (!r->rx_size) break;
        for (unsigned i = 0; i < CT_CMD_WORDS - 1; i++) {
            unsigned at = r->rx_pos + i * 2;
            if (at < sizeof r->rx_buf) r->rx_buf[at] = (uint8_t)w[1 + i];
            if (at + 1 < sizeof r->rx_buf) r->rx_buf[at + 1] = (uint8_t)(w[1 + i] >> 8);
        }
        r->rx_pos = (uint16_t)(r->rx_pos + (CT_CMD_WORDS - 1) * 2);
        if (r->rx_pos >= r->rx_size) {
            uint16_t size = r->rx_size;
            r->rx_size = 0;
            parent_block(r, r->rx_buf, size);
        }
        break;
    case CT_LINKCMD_SEND_BLOCK_REQ:
        /* The other GBA asks for the Switch's trainer card right after the LinkPlayers; the Switch has it ready once it
         * has walked to its room, after its first round. */
        if (!r->s1_done) {
            if (r->held_requests_n < 8) r->held_requests[r->held_requests_n++] = w[1];
            else r->ring_drops++;
        } else {
            request_from_switch(r, w[1]);
        }
        break;
    case CT_LINKCMD_READY_EXIT_STANDBY:
    case CT_LINKCMD_READY_CLOSE_LINK:
        if (w[0] == CT_LINKCMD_READY_CLOSE_LINK) {
            /* drain_keys_on_close: the parent keys already queued still reach the Switch (see the cfg) */
            r->keys_drain = r->cfg.drain_keys_on_close && r->keys_to_switch && r->parent_keys.n > 0;
            r->keys_to_parent = false; r->keys_to_switch = false; r->keys_await_switch = false;
        }
        if (r->other_choice >= 0 && !r->have_parent_lp) {
            /* The other GBA ends a link-up it refused: answered here. */
            if (w[0] == CT_LINKCMD_READY_CLOSE_LINK) cable_cmd(r, CT_LINKCMD_READY_CLOSE_LINK, 0, 0);
            break;
        }
        ring_push(r, &r->parent_syncs, w[0]);
        pair(r);
        break;
    case CT_LINKCMD_SEND_HELD_KEYS:
        ct_keyqueue_push(&r->parent_keys, (uint8_t)(w[1] & 0xff));
        break;
    }
}

bool ct_reverse_next_command(ct_reverse_t *r, uint16_t w[CT_CMD_WORDS])
{
    if (r->lp_pending && ++r->lp_wait >= LP_WAIT_PACKETS) send_own_lp(r);
    if (r->cq_n) {
        memcpy(w, r->cq[r->cq_head].w, sizeof r->cq[0].w);
        r->cq_head = (uint8_t)((r->cq_head + 1) % CT_REVERSE_CQ);
        r->cq_n--;
        return true;
    }
    if (!r->keys_to_parent) return false;
    if (++r->key_pops >= CT_KEY_RATE_WINDOW) {
        double s = (double)r->key_pushes / (double)r->key_pops;
        r->key_step = s < 1 ? 1 : s > 3 ? 3 : s;
        r->key_pushes = r->key_pops = 0;
    }
    memset(w, 0, CT_CMD_WORDS * sizeof w[0]);
    w[0] = CT_LINKCMD_SEND_HELD_KEYS;
    w[1] = ct_keyruns_pop(&r->switch_keys, r->key_step);
    if (r->exit_pending && !r->step.active) pair(r);
    return true;
}

/* ---- setup */

void ct_reverse_open_group(ct_reverse_t *r)
{
    uint8_t beacon[24];
    const uint8_t *name = r->have_parent_lp ? r->parent_lp + 24 : r->cfg.group_name ? r->cfg.group_name : PLACEHOLDER_NAME;
    uint16_t tid = r->have_parent_lp ? ct_le16(r->parent_lp + 20) : 0;
    /* Shown as FireRed so the Switch joins without its checks for other versions. */
    uint16_t compatibility = 2 | (1 << 7) | (1 << 8) | (1 << 9) | (4 << 10);
    ct_leader_beacon(beacon, r->activity, name, tid, compatibility);
    ct_leader_open(r->leader, beacon);
}

static void game_command_cb(void *ctx, const uint16_t w[CT_CMD_WORDS]) { ct_reverse_game_command(ctx, w); }
static bool next_command_cb(void *ctx, uint16_t w[CT_CMD_WORDS]) { return ct_reverse_next_command(ctx, w); }
static void reset_cb(void *ctx) { ct_reverse_cable_reset(ctx); }
static bool ready_cb(void *ctx) { return ct_reverse_ready(ctx); }

ct_game_t ct_reverse_game(ct_reverse_t *r)
{
    ct_game_t g = { r, game_command_cb, next_command_cb, reset_cb, ready_cb };
    return g;
}

void ct_reverse_init(ct_reverse_t *r, ct_leader_t *leader, const ct_reverse_cfg_t *cfg)
{
    memset(r, 0, sizeof *r);
    r->leader = leader;
    r->cfg = *cfg;
    r->activity = cfg->battle ? cfg->activity : 4;
    r->pull = -1;
    r->cable_request = -1;
    r->other_choice = -1;
    r->switch_block_at = NEVER_BLOCK;
    r->link_type = activity_link_type(r->activity);
    r->next = NEXT_LOCAL;
    r->last_round = -1;
    r->key_step = 1.5;
    ct_keyruns_clear(&r->switch_keys);
    ct_keyqueue_clear(&r->parent_keys);
    ct_steps_init(&r->steps, push_switch_key, r);
    ct_leader_hooks_t h = { r, switch_joined, switch_command, switch_block, NULL, on_tick, key_source };
    ct_leader_set_hooks(leader, &h);
}
