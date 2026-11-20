/* SPDX-License-Identifier: AGPL-3.0-only
 * Derived from pokeldn (https://github.com/Decryptu/pokeldn), Copyright (c) the pokeldn authors.
 * Changes Copyright (c) 2026 Kizuna. See NOTICE.
 */
/* trade_host.c — portable C port of pokeldn HostTradeEngine (Direct Corner trade path). See trade_host.h.
 * Method names in comments are the Python's, so the two can be read side by side. */
#include "trade_host.h"
#include <string.h>

/* ---- RFU opcodes / words (pokeldn gba/rfu.py) ---- */
#define OP_MASK              0xFF00
#define OP_SEND_BLOCK_INIT   0x8800
#define OP_SEND_BLOCK        0x8900
#define OP_SEND_HELD_KEYS    0xBE00
#define OP_READY_EXIT_STANDBY 0x6600
#define OP_READY_CLOSE_LINK  0x5F00
#define OP_SEND_BLOCK_REQ    0xA100
#define OP_SEND_PLAYER_IDS   0x7700
#define FRAG_INDEX_MASK      0x1F
#define OWNER_FLAG           0x80
#define FRAG_BYTES           12

static uint16_t rd16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }

static void words_zero(uint16_t w[7]) { memset(w, 0, 7 * sizeof(uint16_t)); }
static void words_simple(uint16_t w[7], uint16_t op, uint16_t w1) { words_zero(w); w[0] = op; w[1] = w1; }
static void words_init(uint16_t w[7], int count, int owner) {
    words_zero(w);
    w[0] = OP_SEND_BLOCK_INIT; w[1] = (uint16_t)count; w[2] = (uint16_t)((owner | OWNER_FLAG) & 0xFFFF);
}
static void words_send_block(uint16_t w[7], int index, const uint8_t *chunk12) {
    w[0] = (uint16_t)(OP_SEND_BLOCK | (index & FRAG_INDEX_MASK));
    for (int i = 0; i < 6; i++) w[1 + i] = rd16(chunk12 + 2 * i);
}

/* ---- timing / config defaults ---- */
void th_timing_default(th_timing *t) {
    t->save_barrier_rounds = 6;
    t->save_final_standby_quiet_frames = 75;
    t->party_link_settle_frames = 30;
    t->startup_standby_echo_frames = 4;
    t->player_ids_repeat_frames = 8;
    t->link_player_idle_frames = 12;
    t->entry_final_standby_quiet_frames = 75;
    t->final_menu_ready_frames = 5 * 60;
    t->post_cancel_exit_wait_frames = 5 * 60;
    t->post_client_close_grace_frames = 15 * 60;
    t->close_retry_frames = 60;
}

void th_cfg_default(th_cfg *cfg) {
    memset(cfg, 0, sizeof *cfg);
    lp_default(&cfg->lp);
    memcpy(cfg->quote, LP_DEFAULT_QUOTE, sizeof cfg->quote);
    cfg->trades = 1;
    cfg->anim_delay = TRADE_DEFAULT_ANIM_FRAMES;
    cfg->trust_pia = 1;
    th_timing_default(&cfg->timing);
}

/* ---- BlockSender(data, owner, trust_pia).tick(None) ---- */
enum { SS_INIT, SS_STREAM, SS_HOLD, SS_DONE };
#define SENDER_WATCHDOG_INIT 4
#define SENDER_WATCHDOG_HOLD 6
#define SENDER_HOLD_RESEND_GAP 6

void th_sender_init(th_sender *s, const uint8_t *data, int len, int owner, int trust_pia) {
    memset(s, 0, sizeof *s);
    if (len > TH_BLOCK_MAX) len = TH_BLOCK_MAX;
    memcpy(s->data, data, (size_t)len);
    s->len = len;
    s->count = len > 0 ? (len + FRAG_BYTES - 1) / FRAG_BYTES : 1;
    s->owner = owner;
    s->trust_pia = trust_pia;
    s->state = SS_INIT;
}

int th_sender_done(const th_sender *s) { return s->state == SS_DONE; }

static void sender_chunk(const th_sender *s, int index, uint8_t out12[12]) {
    memset(out12, 0, 12);
    int start = index * FRAG_BYTES;
    int n = s->len - start;
    if (n < 0) n = 0;
    if (n > FRAG_BYTES) n = FRAG_BYTES;
    if (n) memcpy(out12, s->data + start, (size_t)n);
}

void th_sender_tick(th_sender *s, uint16_t words[7]) {
    uint8_t chunk[12];
    words_zero(words);
    if (s->state == SS_DONE) return;

    if (s->state == SS_INIT) {
        s->init_sends++;
        if (s->init_sends > SENDER_WATCHDOG_INIT) { s->state = SS_STREAM; s->index = 0; }
        else { words_init(words, s->count, s->owner); return; }
    }
    if (s->state == SS_STREAM) {
        int idx = s->index;
        sender_chunk(s, idx, chunk);
        words_send_block(words, idx, chunk);
        s->frag_sends++;
        if (s->frag_sends < 1) return;          /* stream_repeat 1 */
        s->frag_sends = 0;
        if (idx >= s->count - 1) { s->state = s->trust_pia ? SS_DONE : SS_HOLD; s->hold_sends = 0; }
        else s->index++;
        return;
    }
    if (s->state == SS_HOLD) {
        int last = s->count - 1;
        if (s->hold_gap < SENDER_HOLD_RESEND_GAP && s->hold_sends) {
            s->hold_gap++;
            words_zero(words);          /* no ack to excuse an early re-send */
            return;
        }
        s->hold_gap = 0;
        s->hold_sends++;
        if (s->hold_sends > SENDER_WATCHDOG_HOLD) { s->state = SS_DONE; words_zero(words); return; }
        sender_chunk(s, last, chunk);
        words_send_block(words, last, chunk);
    }
}

/* ---- engine internals ---- */
enum {
    EXP_NONE, EXP_LINK_PLAYER, EXP_CARD, EXP_MAIL, EXP_RIBBONS, EXP_WARP0, EXP_WARP1, EXP_WARP2, EXP_PARTY
};
enum { LC_NONE, LC_PARTY0, LC_PARTY1, LC_PARTY2, LC_MAIL, LC_RIBBONS };

#define LINK_KEY_EMPTY     0x11
#define LINK_KEY_UP        0x13
#define LINK_KEY_LEFT      0x14
#define LINK_KEY_READY     0x16
#define LINK_KEY_EXIT_ROOM 0x17

/* Native leader route from the cable-club entrance to the LEFT trade chair: (key, held frames). */
static const struct { uint8_t key; uint16_t count; } ENTRY_LEFT_CHAIR_ROUTE[] = {
    { LINK_KEY_EMPTY, 43 }, { LINK_KEY_UP, 9 }, { LINK_KEY_EMPTY, 4 }, { LINK_KEY_UP, 14 },
    { LINK_KEY_EMPTY, 31 }, { LINK_KEY_LEFT, 5 }, { LINK_KEY_EMPTY, 12 }, { LINK_KEY_UP, 17 },
    { LINK_KEY_EMPTY, 25 }, { LINK_KEY_READY, 1 }, { LINK_KEY_EMPTY, 7 },
};
#define ENTRY_ROUTE_N ((int)(sizeof ENTRY_LEFT_CHAIR_ROUTE / sizeof ENTRY_LEFT_CHAIR_ROUTE[0]))

static void set_state(trade_host *th, th_state s) { th->state = s; }

static void q_words(trade_host *th, const uint16_t w[7]) {
    if (th->wq_len >= TH_WORDS_MAX) { th->words_dropped++; return; }
    th_words *e = &th->words_q[(th->wq_head + th->wq_len) % TH_WORDS_MAX];
    memcpy(e->w, w, 7 * sizeof(uint16_t));
    th->wq_len++;
}

static void q_block(trade_host *th, const uint8_t *data, int len) {
    if (th->bq_len >= TH_BLOCKS_MAX || len > TH_BLOCK_MAX) { th->blocks_dropped++; return; }
    th_block *e = &th->blocks_q[(th->bq_head + th->bq_len) % TH_BLOCKS_MAX];
    memcpy(e->data, data, (size_t)len);
    e->len = (uint16_t)len;
    th->bq_len++;
}

static void q_block_req(trade_host *th, int reqtype) {
    uint16_t w[7];
    words_simple(w, OP_SEND_BLOCK_REQ, (uint16_t)reqtype);
    q_words(th, w);
}

static void q_close_link(trade_host *th, int n) {
    uint16_t w[7];
    for (int i = 0; i < n; i++) { words_simple(w, OP_READY_CLOSE_LINK, (uint16_t)th->exit_count); q_words(th, w); }
}

/* _request_and_send */
static void request_and_send(trade_host *th, int reqtype, const uint8_t *data, int len, int expected) {
    th->expected = expected;
    q_block_req(th, reqtype);
    q_block(th, data, len);
}

/* _send_linkcmd */
static void send_linkcmd(trade_host *th, uint16_t cmd, uint16_t cursor) {
    uint8_t b[TRADE_LINKCMD_BLOCK_SIZE];
    trade_linkcmd_block(cmd, cursor, b);
    q_block(th, b, TRADE_LINKCMD_BLOCK_SIZE);
}

/* ---- held keys ---- */
static void set_held_plan_entry(trade_host *th) {          /* _set_held_plan(ENTRY_LEFT_CHAIR_ROUTE, steady=None) */
    th->held_n = ENTRY_ROUTE_N;
    for (int i = 0; i < ENTRY_ROUTE_N; i++) {
        th->held_runs[i].key = ENTRY_LEFT_CHAIR_ROUTE[i].key;
        th->held_runs[i].count = ENTRY_LEFT_CHAIR_ROUTE[i].count;
    }
    th->held_i = 0;
    th->held_left = th->held_runs[0].count;
    th->held_steady = -1;
}

static void hold_key(trade_host *th, uint8_t key) {        /* _hold_key: one frame, then EMPTY steady */
    th->held_n = 1;
    th->held_runs[0].key = key;
    th->held_runs[0].count = 1;
    th->held_i = 0;
    th->held_left = 1;
    th->held_steady = LINK_KEY_EMPTY;
}

static void release_key(trade_host *th) {
    th->held_n = 0; th->held_i = 0; th->held_left = 0;
    th->held_steady = -1;
}

static void complete_room_exit(trade_host *th);

static int next_held_words(trade_host *th, uint16_t words[7]) {   /* _next_held_words */
    int keycode;
    while (th->held_i < th->held_n && th->held_left <= 0) {
        th->held_i++;
        if (th->held_i < th->held_n) th->held_left = th->held_runs[th->held_i].count;
    }
    if (th->held_i < th->held_n) {
        keycode = th->held_runs[th->held_i].key;
        th->held_left--;
    } else if (th->held_steady >= 0) {
        keycode = th->held_steady;
    } else {
        return 0;
    }
    th->held_count = (th->held_count + 1) & 0xFF;
    words_simple(words, OP_SEND_HELD_KEYS, (uint16_t)((th->held_count << 8) | keycode));
    if (keycode == LINK_KEY_EXIT_ROOM && th->state == TH_EXIT && th->child_exit_seen)
        complete_room_exit(th);
    return 1;
}

/* ---- state transitions ---- */
static void request_party_pair(trade_host *th) {           /* _request_party_pair */
    uint8_t buf[600];
    memset(buf, 0, sizeof buf);
    for (int i = 0; i < th->n_party; i++) memcpy(buf + i * MON_PARTY_SIZE, th->party[i], MON_PARTY_SIZE);
    th->party_expected_i = th->party_pair;      /* expected = f"party:{pair}" */
    request_and_send(th, TRADE_BLOCK_REQ_SIZE_200, buf + th->party_pair * 200, 200, EXP_PARTY);
}

static void begin_party_exchange(trade_host *th) {         /* _begin_party_exchange */
    set_state(th, TH_PARTY);
    memset(th->child_party, 0, sizeof th->child_party);
    th->party_pair = 0;
    request_party_pair(th);
}

static void begin_card_exchange(trade_host *th) {          /* _begin_card_exchange */
    set_state(th, TH_ENTRY_CARD);
    request_and_send(th, TRADE_BLOCK_REQ_SIZE_100, th->trainer_card, LP_TRAINER_CARD_SIZE, EXP_CARD);
}

static void enter_cancel_to_leave(trade_host *th) {        /* _enter_cancel_to_leave */
    set_state(th, TH_CANCEL);
    th->cancel_standby_count = -1;
    th->return_standby_count = -1;
    th->room_exit_wait = -1;
    th->child_exit_seen = 0;
    send_linkcmd(th, TRADE_BOTH_CANCEL_TRADE, 0);
}

static void leader_cancel_is_ready(trade_host *th) {       /* _leader_cancel_is_ready */
    th->host_cancel_ready = 1;
    if (th->child_cancel_requested) enter_cancel_to_leave(th);
}

static void finish_party_exchange(trade_host *th) {        /* _finish_party_exchange */
    th->expected = EXP_NONE;
    if (th->round >= th->cfg.trades) {
        set_state(th, TH_LEAVE_MENU);
        th->leave_menu_wait = th->cfg.timing.final_menu_ready_frames;
        th->host_cancel_ready = 0;
        th->child_cancel_requested = 0;
    } else {
        th->select_cancels = 0;
        set_state(th, TH_SELECT);
    }
}

static void begin_room_exit(trade_host *th, int child_already_exited) {   /* _begin_room_exit */
    set_state(th, TH_EXIT);
    th->room_exit_wait = -1;
    th->child_exit_seen = child_already_exited;
    hold_key(th, LINK_KEY_EXIT_ROOM);
}

static void complete_room_exit(trade_host *th) {           /* _complete_room_exit */
    if (th->state != TH_EXIT) return;
    release_key(th);
    set_state(th, TH_CLOSE);
    th->close_confirmed = 0;
    th->close_grace_wait = -1;
    q_close_link(th, th->cfg.timing.startup_standby_echo_frames);
    th->close_retry_wait = th->cfg.timing.close_retry_frames;
}

static void maybe_finish_entry(trade_host *th) {           /* _maybe_finish_entry */
    if (th->state == TH_ENTRY_SEAT && th->last_child_standby >= 3) {
        release_key(th);
        th->entry_final_standby_seen = 1;
        th->entry_standby_quiet = 0;
    }
}

static void commit(trade_host *th) {                       /* _commit */
    if (th->child_cursor < 0 || th->round >= th->cfg.trades) { th->errors++; return; }
    int host_slot = th->offered[th->round];
    int child_slot = th->child_cursor;
    const uint8_t *raw = th->child_party + child_slot * MON_PARTY_SIZE;
    if (th->received_count < TH_PARTY_MAX) memcpy(th->received[th->received_count++], raw, MON_PARTY_SIZE);
    memcpy(th->party[host_slot], raw, MON_PARTY_SIZE);
    th->round++;
    th->commits++;
    send_linkcmd(th, TRADE_CONFIRM_FINISH_TRADE, 0);
    set_state(th, TH_SAVE);
    th->save_rounds = 0;
    th->save_last_count = -1;
    th->save_final_standby_seen = 0;
    th->save_standby_quiet = 0;
    th->child_finish = 0;
    th->anim_wait = -1;
}

/* _on_child_linkcmd */
static void on_child_linkcmd(trade_host *th, uint16_t cmd, uint16_t cursor) {
    th_state st = th->state;
    if (cmd == TRADE_READY_TO_TRADE && st == TH_SELECT) {
        th->select_cancels = 0;
        th->child_cursor = cursor % 6;
        set_state(th, TH_CONFIRM);
        send_linkcmd(th, TRADE_SET_MONS_TO_TRADE, (uint16_t)th->offered[th->round < th->cfg.trades ? th->round : 0]);
    } else if (cmd == TRADE_INIT_BLOCK && st == TH_CONFIRM) {
        set_state(th, TH_ANIM);
        th->anim_wait = th->cfg.anim_delay;
        send_linkcmd(th, TRADE_START_TRADE, 0);
    } else if (cmd == TRADE_READY_FINISH_TRADE && st == TH_ANIM) {
        th->child_finish = 1;
    } else if (cmd == TRADE_REQUEST_CANCEL && st == TH_SELECT) {
        th->select_cancels++;
        if (th->select_cancels >= 2) {
            th->host_cancel_ready = 1;
            th->child_cancel_requested = 1;
            enter_cancel_to_leave(th);
        } else {
            send_linkcmd(th, TRADE_PARTNER_CANCEL_TRADE, 0);
        }
    } else if (cmd == TRADE_REQUEST_CANCEL && st == TH_LEAVE_MENU) {
        th->child_cancel_requested = 1;
        if (th->host_cancel_ready) enter_cancel_to_leave(th);
    } else if ((cmd == TRADE_READY_TO_TRADE || cmd == TRADE_READY_CANCEL_TRADE) && st == TH_LEAVE_MENU) {
        send_linkcmd(th, TRADE_PLAYER_CANCEL_TRADE, 0);
    }
}

/* _after_child_block */
static void after_child_block(trade_host *th, int count, const uint8_t *data) {
    int expected = th->expected;
    int expected_count = -1;
    switch (expected) {
        case EXP_LINK_PLAYER: expected_count = TRADE_COUNT_PARTY; break;
        case EXP_CARD:        expected_count = TRADE_COUNT_TRAINER_CARD; break;
        case EXP_MAIL:        expected_count = TRADE_COUNT_MAIL; break;
        case EXP_RIBBONS:     expected_count = TRADE_COUNT_RIBBON; break;
        case EXP_PARTY:       expected_count = TRADE_COUNT_PARTY; break;
        default: break;
    }
    if (expected_count < 0) return;                 /* unexpected_child_block */
    if (count != expected_count) { th->errors++; return; }   /* the Python raises ValueError */

    if (expected == EXP_LINK_PLAYER) {
        lp_link_player lp;
        if (!lp_parse_block(data, (size_t)count * FRAG_BYTES, &lp)) { th->rejected_link_players++; return; }
        th->child_link_player = lp;
        th->has_child_link_player = 1;
        th->expected = EXP_WARP0;
        q_block(th, th->link_player_block, 200);
        return;
    }
    if (expected == EXP_CARD) {
        memcpy(th->child_card, data, LP_TRAINER_CARD_SIZE);
        th->has_child_card = 1;
        th->expected = EXP_WARP1;
        return;
    }
    if (expected == EXP_PARTY) {
        int i = th->party_expected_i;
        memcpy(th->child_party + i * 200, data, 200);
        th->link_waiting_idle = 1;
        th->link_idle_frames = 0;
        th->link_completed = LC_PARTY0 + i;
        return;
    }
    if (expected == EXP_MAIL) {
        th->link_waiting_idle = 1; th->link_idle_frames = 0; th->link_completed = LC_MAIL;
        return;
    }
    if (expected == EXP_RIBBONS) {
        th->link_waiting_idle = 1; th->link_idle_frames = 0; th->link_completed = LC_RIBBONS;
    }
}

/* _on_child_block */
static void on_child_block(trade_host *th, int count, const uint8_t *data) {
    if (count == TRADE_COUNT_LINKCMD) on_child_linkcmd(th, rd16(data), rd16(data + 2));
    else after_child_block(th, count, data);
}

/* _on_child_standby */
static void on_child_standby(trade_host *th, int count) {
    if (th->state == TH_PARTY) {
        th->link_idle_frames = 0;
        return;
    }
    int repeats = 1;
    switch (th->state) {
        case TH_LINK_PLAYER: case TH_ENTRY_CARD: case TH_ENTRY_SEAT: case TH_CANCEL: case TH_RETURN_FIELD:
            repeats = th->cfg.timing.startup_standby_echo_frames; break;
        default: break;
    }
    for (int i = 0; i < repeats; i++) {
        uint16_t w[7];
        words_simple(w, OP_READY_EXIT_STANDBY, (uint16_t)count);
        q_words(th, w);
    }
    th->last_child_standby = count;
    if (count + 1 > th->exit_count) th->exit_count = count + 1;

    if (th->state == TH_LINK_PLAYER && th->expected == EXP_WARP0) {
        begin_card_exchange(th);
    } else if (th->state == TH_ENTRY_CARD && th->expected == EXP_WARP1) {
        set_state(th, TH_ENTRY_SEAT);
        th->expected = EXP_WARP2;
        set_held_plan_entry(th);
    } else if (th->state == TH_ENTRY_SEAT) {
        if (count >= 3) th->entry_standby_quiet = 0;
        maybe_finish_entry(th);
    } else if (th->state == TH_SAVE) {
        if (th->save_last_count < 0) {
            th->save_last_count = count;
            th->save_rounds = 1;
        } else if (count == th->save_last_count) {
            /* repeat of the same round */
        } else if (count == ((th->save_last_count + 1) & 0xFFFF)) {
            th->save_last_count = count;
            th->save_rounds++;
        }
        if (th->save_rounds >= th->cfg.timing.save_barrier_rounds) {
            th->save_standby_quiet = 0;
            if (!th->save_final_standby_seen) th->save_final_standby_seen = 1;
        }
    } else if (th->state == TH_CANCEL) {
        if (th->cancel_standby_count < 0) {
            th->cancel_standby_count = count;
        } else if (count == ((th->cancel_standby_count + 1) & 0xFFFF)) {
            set_state(th, TH_RETURN_FIELD);
            th->return_standby_count = count;
            th->room_exit_wait = th->cfg.timing.post_cancel_exit_wait_frames;
        }
    } else if (th->state == TH_RETURN_FIELD) {
        th->return_standby_count = count;
    }
}

/* ---- idle-slot handlers ---- */
static void idle_party_link_settle(trade_host *th) {        /* _idle_party_link_settle */
    th->link_idle_frames++;
    if (th->link_idle_frames < th->cfg.timing.party_link_settle_frames) return;
    int completed = th->link_completed;
    th->link_waiting_idle = 0;
    th->link_completed = LC_NONE;
    int i = (completed >= LC_PARTY0 && completed <= LC_PARTY2) ? completed - LC_PARTY0 : -1;
    if (i >= 0 && i < 2) {
        th->party_pair = i + 1;
        request_party_pair(th);
    } else if (i == 2) {
        uint8_t zeros[220];
        memset(zeros, 0, sizeof zeros);
        request_and_send(th, TRADE_BLOCK_REQ_SIZE_220, zeros, 220, EXP_MAIL);
    } else if (completed == LC_MAIL) {
        uint8_t zeros[40];
        memset(zeros, 0, sizeof zeros);
        request_and_send(th, TRADE_BLOCK_REQ_SIZE_40, zeros, 40, EXP_RIBBONS);
    } else if (completed == LC_RIBBONS) {
        finish_party_exchange(th);
    }
}

static void feed_idle_slot(trade_host *th) {                /* _feed_idle_slot */
    if (th->state == TH_SAVE && th->save_final_standby_seen) {
        th->save_standby_quiet++;                           /* _idle_save_final_standby */
        if (th->save_standby_quiet >= th->cfg.timing.save_final_standby_quiet_frames)
            begin_party_exchange(th);
        return;
    }
    if (th->state == TH_ENTRY_SEAT && th->entry_final_standby_seen) {
        th->entry_standby_quiet++;                          /* _idle_entry_final_standby */
        if (th->entry_standby_quiet >= th->cfg.timing.entry_final_standby_quiet_frames) {
            th->expected = EXP_NONE;
            begin_party_exchange(th);                       /* _begin_seated_activity */
        }
        return;
    }
    if (th->state == TH_LINK_PLAYER && th->expected == EXP_WARP0) {
        if (th->sender_active || th->bq_len) return;        /* _idle_link_player_wait */
        th->link_player_idle++;
        if (th->link_player_idle >= th->cfg.timing.link_player_idle_frames) begin_card_exchange(th);
        return;
    }
    if (th->state == TH_PARTY && th->link_waiting_idle) idle_party_link_settle(th);
}

/* ---- public ---- */
int trade_host_init(trade_host *th, const th_cfg *cfg) {
    memset(th, 0, sizeof *th);
    if (cfg->party_count < 1 || cfg->party_count > TH_PARTY_MAX) return -1;
    if (cfg->trades < 1 || cfg->trades > TH_PARTY_MAX) return -1;
    th->cfg = *cfg;
    th->n_party = cfg->party_count;
    if (trade_resolve_offered_slots(cfg->has_offered_slots ? cfg->offered_slots : NULL, cfg->trades,
                                    cfg->trade_slot, cfg->trades, cfg->party_count, th->offered) != 0) return -1;
    for (int i = 0; i < cfg->trades; i++) if (th->offered[i] >= cfg->party_count) return -1;
    memcpy(th->party, cfg->party, sizeof th->party);

    uint16_t species[TH_PARTY_MAX];
    for (int i = 0; i < th->n_party; i++) species[i] = mon_species(th->party[i]);
    lp_build_trainer_card(&cfg->lp, cfg->card_flag_id, species, th->n_party, LP_HOST_NAME_PAD,
                          cfg->quote, th->trainer_card);
    memset(th->link_player_block, 0, sizeof th->link_player_block);
    lp_build_block(&cfg->lp, LP_HOST_NAME_PAD, th->link_player_block);

    th->state = TH_LINK_PLAYER;
    th->child_cursor = -1;
    th->anim_wait = -1;
    th->save_last_count = -1;
    th->last_child_standby = -1;
    th->cancel_standby_count = -1;
    th->return_standby_count = -1;
    th->room_exit_wait = -1;
    th->leave_menu_wait = -1;
    th->close_grace_wait = -1;
    th->held_steady = -1;
    th->close_retry_wait = cfg->timing.close_retry_frames;
    recv_block_init(&th->rx);

    /* SEND_PLAYER_IDS is idempotent in RfuHandleReceiveCommand, so repeating it is safe. */
    for (int i = 0; i < cfg->timing.player_ids_repeat_frames; i++) {
        uint16_t w[7];
        words_zero(w);
        w[0] = OP_SEND_PLAYER_IDS; w[1] = 2; w[2] = 1;      /* playerCount 2, linkPlayerIdx (1,0,0,0) */
        q_words(th, w);
    }
    th->expected = EXP_LINK_PLAYER;
    q_block_req(th, TRADE_BLOCK_REQ_SIZE_NONE);
    return 0;
}

void trade_host_feed_child_slot(trade_host *th, const uint8_t slot[14]) {
    int idle = 1;
    for (int i = 0; i < 14; i++) if (slot[i]) { idle = 0; break; }
    if (idle) { feed_idle_slot(th); return; }
    if (th->state == TH_PARTY && th->link_waiting_idle) th->link_idle_frames = 0;
    th->link_player_idle = 0;

    uint16_t word0 = rd16(slot);
    switch (word0 & OP_MASK) {
    case OP_SEND_BLOCK_INIT:                                /* _child_send_block_init */
        recv_block_on_init(&th->rx, rd16(slot + 2), slot[4]);
        break;
    case OP_SEND_BLOCK: {                                   /* _child_send_block */
        int was_done = th->rx.done;
        if (th->rx.count <= BLOCK_MAX_FRAGS)                /* a larger count can never complete */
            recv_block_on_block(&th->rx, word0 & FRAG_INDEX_MASK, slot + 2);
        if (th->rx.done && !was_done) {
            uint8_t data[BLOCK_MAX_BYTES];
            int count = th->rx.count;
            recv_block_consume(&th->rx, data);
            on_child_block(th, count, data);
        }
        break;
    }
    case OP_SEND_HELD_KEYS: {                               /* _child_send_held_keys */
        int key = rd16(slot + 2) & 0xFF;
        if (key == LINK_KEY_READY && th->state == TH_ENTRY_SEAT) {
            maybe_finish_entry(th);
        } else if (key == LINK_KEY_EXIT_ROOM && th->state == TH_RETURN_FIELD) {
            begin_room_exit(th, 1);
        } else if (key == LINK_KEY_EXIT_ROOM && th->state == TH_EXIT) {
            th->child_exit_seen = 1;
            complete_room_exit(th);
        }
        break;
    }
    case OP_READY_EXIT_STANDBY:                             /* _child_ready_exit_standby */
        on_child_standby(th, rd16(slot + 2));
        break;
    case OP_READY_CLOSE_LINK:                               /* _child_ready_close_link */
        if (th->state != TH_CLOSE) break;
        if (!th->close_confirmed) {
            th->close_confirmed = 1;
            th->close_grace_wait = th->cfg.timing.post_client_close_grace_frames;
        }
        break;
    default:
        break;                                              /* SEND_PACKET etc.: Union Room only */
    }
}

void trade_host_tick(trade_host *th, uint16_t words[7]) {
    if (th->state == TH_LEAVE_MENU) {                       /* _tick_leave_menu */
        if (th->state == TH_LEAVE_MENU && th->leave_menu_wait >= 0) {
            th->leave_menu_wait--;
            if (th->leave_menu_wait <= 0) { th->leave_menu_wait = -1; leader_cancel_is_ready(th); }
        }
    }
    if (th->state == TH_RETURN_FIELD && th->room_exit_wait >= 0) {   /* _tick_room_exit_wait */
        th->room_exit_wait--;
        if (th->room_exit_wait <= 0) begin_room_exit(th, 0);
    }
    if (th->state == TH_CLOSE && !th->disconnect_requested) {        /* _tick_close_link */
        if (th->close_grace_wait >= 0) {
            th->close_grace_wait--;
            if (th->close_grace_wait <= 0) { th->close_grace_wait = -1; th->disconnect_requested = 1; }
        }
        th->close_retry_wait--;
        if (th->close_retry_wait <= 0) {
            q_close_link(th, th->cfg.timing.startup_standby_echo_frames);
            th->close_retry_wait = th->cfg.timing.close_retry_frames;
        }
    }
    if (th->state == TH_ANIM && th->anim_wait >= 0) {                /* _tick_anim */
        if (th->anim_wait > 0) th->anim_wait--;
        else if (th->child_finish) commit(th);
    }

    /* _next_parent_words */
    if (th->wq_len) {
        memcpy(words, th->words_q[th->wq_head].w, 7 * sizeof(uint16_t));
        th->wq_head = (th->wq_head + 1) % TH_WORDS_MAX;
        th->wq_len--;
        return;
    }
    if (next_held_words(th, words)) return;
    if (!th->sender_active && th->bq_len) {
        th_block *b = &th->blocks_q[th->bq_head];
        th_sender_init(&th->sender, b->data, b->len, 0, th->cfg.trust_pia);
        th->sender_active = 1;
        th->bq_head = (th->bq_head + 1) % TH_BLOCKS_MAX;
        th->bq_len--;
    }
    if (th->sender_active) {
        th_sender_tick(&th->sender, words);
        if (th_sender_done(&th->sender)) th->sender_active = 0;
        return;
    }
    words_zero(words);
}

int trade_host_established(const trade_host *th) { return th->has_child_link_player; }
int trade_host_disconnect_requested(const trade_host *th) { return th->disconnect_requested; }
void trade_host_mark_disconnect_sent(trade_host *th) {
    if (!th->disconnect_requested) return;
    th->done = 1;
    set_state(th, TH_DONE);
}
th_state trade_host_state(const trade_host *th) { return th->state; }
int trade_host_done(const trade_host *th) { return th->done; }
int trade_host_close_confirmed(const trade_host *th) { return th->close_confirmed; }
int trade_host_commits(const trade_host *th) { return th->commits; }
int trade_host_received_count(const trade_host *th) { return th->received_count; }
const uint8_t *trade_host_received_mon(const trade_host *th, int i) {
    return (i >= 0 && i < th->received_count) ? th->received[i] : NULL;
}

const char *trade_host_state_name(th_state s) {
    static const char *const N[TH_STATE_COUNT] = {
        "H_LINK_PLAYER", "H_ENTRY_CARD", "H_ENTRY_SEAT", "H_PARTY", "H_SELECT", "H_CONFIRM", "H_ANIM",
        "H_SAVE", "H_LEAVE_MENU", "H_CANCEL", "H_RETURN_FIELD", "H_EXIT", "H_CLOSE", "H_DONE",
    };
    return (unsigned)s < TH_STATE_COUNT ? N[s] : "H_?";
}

/* ---- pia_activity vtable ---- */
static void act_feed(void *ctx, const uint8_t slot[14]) { trade_host_feed_child_slot((trade_host *)ctx, slot); }
static void act_tick(void *ctx, uint16_t words[7]) { trade_host_tick((trade_host *)ctx, words); }
static int  act_disc(void *ctx) { return trade_host_disconnect_requested((const trade_host *)ctx); }
static void act_sent(void *ctx) { trade_host_mark_disconnect_sent((trade_host *)ctx); }
static int  act_est(void *ctx) { return trade_host_established((const trade_host *)ctx); }

pia_activity trade_host_activity(trade_host *th) {
    pia_activity a;
    a.ctx = th;
    a.feed_child_slot = act_feed;
    a.tick = act_tick;
    a.disconnect_requested = act_disc;
    a.mark_disconnect_sent = act_sent;
    a.established = act_est;
    return a;
}
