/* SPDX-License-Identifier: AGPL-3.0-only
 * Derived from GB-Link-Switch-LDN (https://github.com/GB-Link/GB-Link-Switch-LDN), Copyright (c) the GB-Link-Switch-LDN authors.
 * Changes Copyright (c) 2026 Kizuna. See NOTICE.
 */
/* ct_translator.h -- wireless FRLG child <-> cable player 0. C port of CableTranslator, GB-Link-Switch-LDN
 * web/js/cable/translator.js (commit c972db4, AGPL-3.0; see NOTICE).
 *
 * Runs on this adapter when the SWITCH leads the wireless group (Celio slave role, GB-Link's own pairing: slave -> translator,
 * master -> reverse): it joins the Switch's group as an FRLG child and plays cable player 0 (the parent) to the
 * remote GBA's cable stream. With holdJoin (always, over a relay) the group is only marked seen until the cable section
 * opens (ct_translator_cable_reset), then joined.
 *
 * In:  ct_translator_board_frame()  RFU1 frames from the wireless side: BROADCAST, CONNECT_ACK, DISCONNECT, HOST_SEND.
 *      ct_translator_frame()        once per GBA frame (59.727 Hz).
 *      the ct_game_t from ct_translator_game() for ct_link: game commands in, next command out, cable reset, ready.
 * Out: the send callback            RFU1 frames: CLIENT_SEND (104 B; payload length in header byte 0), CONNECT_REQ and
 *                                   DISCONNECT (16 B) -- what celio.js's glue (clientFrame/command) builds.
 *      cfg.event                     linked / group-seen changes (JS onLinked).
 *
 * Dropped: Ruby/Sapphire "dressing" (LinkPlayer, card, game data shown as Emerald), refuse() /
 * hostNotReady / the national-dex compatibility check, the Emerald bypass, roomsSeen/severalRooms (a group shows one
 * room), log strings. bar.timedOut (never set in the JS) is gone. Battle paths are ported and gated by cfg.battle: off,
 * only trade groups are joined and the cable's link type always means trade.
 * Fixed storage: cq 96 commands (the JS limit, same drop rule), child-block and held-block queues of at most 8 blocks
 * (the JS limits) in 640-byte arenas (drops past that are counted). */
#ifndef CT_TRANSLATOR_H
#define CT_TRANSLATOR_H

#include "ct_common.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CT_TR_CABLE_QUEUE 96
#define CT_TR_BLOCKS 8
#ifndef CT_TR_ARENA
#define CT_TR_ARENA 640
#endif

enum { CT_TR_EV_LINKED = 1, CT_TR_EV_GROUP_SEEN = 2 };   /* arg: linked (0/1) */

typedef struct {
    bool hold_join;     /* true over the relay */
    bool battle;        /* stage 2 gate */
    void (*event)(void *ctx, int event, uint32_t arg);
    void *event_ctx;
} ct_translator_cfg_t;

typedef struct {
    uint8_t count, need, repairs;
    bool receiving, done;
    int8_t last_index;
    uint32_t flags, heard_at;
} ct_recv_t;

/* A FIFO of whole blocks in one byte arena. */
typedef struct {
    uint16_t len[CT_TR_BLOCKS];
    uint8_t flags[CT_TR_BLOCKS];
    uint8_t n;
    uint16_t used;
    uint8_t buf[CT_TR_ARENA];
} ct_blockq_t;

/* The child's NI (name) transfer state. */
typedef struct { uint8_t state, phase, n[4], now[4]; int16_t remain; uint8_t src[26], header[7]; } ct_ni_t;

typedef struct {
    ct_frame_fn send;
    void *send_ctx;
    ct_translator_cfg_t cfg;

    bool join_allowed, group_seen, lp_pull_waiting;
    int8_t link;                 /* -1, SEARCH 0, CONNECTING 1, NI 2, UNI 3 */
    uint16_t host_id;
    uint32_t link_frames, silent_frames, connect_frames;

    struct { uint8_t mode; bool initiated; int32_t host_count, local_count, burst_for; uint32_t since_host, since_initiate, burst_n, rounds; } bar;
    ct_recv_t rx0, rx1;
    uint8_t rx0_buf[CT_MAX_BLOCK_BYTES];
    int8_t repair_type;          /* -1: none */
    struct { bool valid; uint8_t count; uint16_t len; int64_t until; uint8_t data[CT_MAX_BLOCK_BYTES]; } repeat_due;
    int64_t pull_at;
    struct { bool active, is_lp, is_card, is_battle_lp; uint8_t state, count, index, rr; uint8_t data[CT_MAX_BLOCK_BYTES]; } snd;
    ct_ni_t ni;
    ct_blockq_t child_queue;
    uint32_t host_frames, child_frames;
    uint8_t tag, key_count;

    bool have_ruby_lp, have_host_lp, lp_sent_to_host, lp_send_done, round0_started, keys_active;
    uint8_t ruby_lp[60], host_lp[60];
    ct_keyqueue_t ruby_keys;     /* game -> leader */
    ct_keyruns_t host_keys;      /* leader -> game */
    double key_step;
    uint32_t key_pushes, key_pops;
    int32_t last_host_key_count;
    uint8_t purpose;
    uint32_t close_rounds_left, close_count;
    bool cancel_pending, cancel_close, cancel_return;
    bool card_armed, card_requested, have_ruby_card, card_queued, card_pull_waiting, card_send_done, pending_game_standby;
    uint32_t card_at;
    uint8_t ruby_card[100];
    bool exit_key_seen, host_closed_seen, room_closed;

    ct_blockq_t held;
    int32_t pending_pull;

    uint32_t cable_packets, ready_at, lp_early_wait;
    bool announced, session_ruby_lp, link_closing, p0_lp_delivered;
    uint8_t expect, pull_type;
    uint16_t rx_size, rx_pos;
    uint8_t rx_buf[CT_MAX_BLOCK_BYTES];
    uint8_t cq_head, cq_n;
    uint16_t cq[CT_TR_CABLE_QUEUE][CT_CMD_WORDS];
    uint32_t sessions;

    uint8_t activity, battle_state;
    bool battle_round;

    /* counters */
    uint32_t cq_drops, child_drops, held_drops;   /* the JS's own limits (it drops these too) */
    uint32_t arena_drops;                         /* a block the JS would have kept: an arena was full */
} ct_translator_t;

void ct_translator_init(ct_translator_t *t, ct_frame_fn send, void *send_ctx, const ct_translator_cfg_t *cfg);
void ct_translator_board_frame(ct_translator_t *t, const uint8_t *frame, size_t len);
void ct_translator_frame(ct_translator_t *t);
ct_game_t ct_translator_game(ct_translator_t *t);
void ct_translator_game_command(ct_translator_t *t, const uint16_t words[CT_CMD_WORDS]);
bool ct_translator_next_command(ct_translator_t *t, uint16_t words[CT_CMD_WORDS]);
void ct_translator_cable_reset(ct_translator_t *t);
static inline bool ct_translator_ready(const ct_translator_t *t) { return t->group_seen; }     /* readyToLink */
static inline bool ct_translator_linked(const ct_translator_t *t) { return t->link == 3; }
static inline bool ct_translator_joined(const ct_translator_t *t) { return t->link >= 1; }

#ifdef __cplusplus
}
#endif
#endif
