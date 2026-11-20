/* SPDX-License-Identifier: AGPL-3.0-only
 * Derived from GB-Link-Switch-LDN (https://github.com/GB-Link/GB-Link-Switch-LDN), Copyright (c) the GB-Link-Switch-LDN authors.
 * Changes Copyright (c) 2026 Kizuna. See NOTICE.
 */
/* ct_reverse.h -- cable player 1 <-> wireless leader: the Switch joins a group this adapter leads (ct_leader), and the remote
 * GBA is the cable parent (player 0). C port of ReverseTranslator, GB-Link-Switch-LDN web/js/cable/reverse.js (commit
 * c972db4, AGPL-3.0; see NOTICE).
 *
 * The Switch's game sets the pace: each of its standby rounds (0x6600 count) is a point in the Cable Club's flow where both
 * GBAs send 0x2FFE (standby) or 0x5FFF (close), or where the parent opened a new link. This side sends its own as the
 * Switch gets there and answers the Switch once the parent's has come.
 *
 * Wiring: ct_reverse_init() installs itself as the leader's hooks; ct_reverse_game() is the ct_game_t a ct_link drives.
 *
 * Dropped from the JS: forSwitch's Ruby/Sapphire dressing (a GBA's LinkPlayer goes to the Switch as is),
 * refusal()/onRefused (Emerald national-dex checks), the Emerald bypass, log strings. otherChoice is kept (a status +
 * event). Battle paths are ported and gated by cfg.battle (stage 2): off, the group is always a trade room and the
 * Switch's battle LinkPlayer is not looked for.
 * Fixed storage instead of JS arrays (overflow drops and counts, tests check zero): cq 64 and held 32 cable commands,
 * deferred sends 8 entries / 640 bytes, 16 rounds and syncs, 8 held requests; parent blocks are capped at 288 bytes
 * (JS 512; no Cable Club block is longer than 220). */
#ifndef CT_REVERSE_H
#define CT_REVERSE_H

#include "ct_common.h"
#include "ct_leader.h"
#include "ct_steps.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef CT_REVERSE_CQ
#define CT_REVERSE_CQ 64
#endif
#ifndef CT_REVERSE_HELD
#define CT_REVERSE_HELD 32
#endif
#define CT_REVERSE_DEFERRED 8
#define CT_REVERSE_DEFERRED_BYTES 640
#define CT_REVERSE_ROUNDS 16
#define CT_REVERSE_LP 60

enum {
    CT_REVERSE_EV_READY = 1,          /* the Switch's player is known (its LinkPlayer is in) */
    CT_REVERSE_EV_OTHER_CHOICE = 2,   /* arg: the parent's link type; it chose another room */
    CT_REVERSE_EV_ENDED = 3,          /* both left the room: the session is over */
};

typedef struct {
    uint8_t activity;     /* 4 trade, 1 single battle, 2 double battle (battle only with .battle) */
    bool battle;          /* stage 2 gate */
    /* Kizuna deviation, off = reverse.js exactly: on the parent's READY_CLOSE_LINK, keys of the parent
     * already queued for the Switch still go before the keys stop. The relay delivers 4 cable commands at a time, so the
     * other GBA's "sat down" key (KEY_READY) and its close arrived together; reverse.js stops the keys at once, the key
     * never reached the Switch, and the GBA went into the trade menu while the Switch sat waiting for its partner. */
    bool drain_keys_on_close;
    /* The group's name until the other GBA's LinkPlayer is in (then its trainer's): 8 bytes, FRLG charset, 0xFF-terminated.
     * NULL = "CELIO" (reverse.js). Kizuna shows its own name. */
    const uint8_t *group_name;
    void (*event)(void *ctx, int event, uint32_t arg);
    void *event_ctx;
} ct_reverse_cfg_t;

typedef struct { uint16_t q[CT_REVERSE_ROUNDS]; uint8_t head, n; } ct_u16ring_t;
typedef struct { uint16_t w[CT_CMD_WORDS]; } ct_cmd_t;

typedef struct {
    ct_leader_t *leader;
    ct_reverse_cfg_t cfg;
    uint8_t activity;

    bool have_switch_lp, have_parent_lp, have_switch_battle_lp, have_parent_seen, have_parent_early;
    uint8_t switch_lp[CT_REVERSE_LP], parent_lp[CT_REVERSE_LP], parent_seen[CT_REVERSE_LP], parent_early[CT_REVERSE_LP];
    uint8_t switch_battle_lp[28];
    bool leader_lp_sent, leader_battle_lp_sent;
    int16_t pull, cable_request;       /* -1: none */
    uint16_t held_requests[8];
    uint8_t held_requests_n;
    bool s1_done;
    int32_t other_choice;              /* -1: none */
    bool lp_pending, exchange_due;
    uint32_t lp_wait;
    int64_t switch_block_at;
    uint32_t request_at, rerequests;

    uint16_t link_type;
    uint8_t next;
    uint32_t save_standbys;
    uint16_t menu_outcome;             /* 0: none */
    bool after_trade, in_battle, ended;
    int64_t quiet_until, extra_deadline;

    /* deferred requests and blocks for the Switch, in order; all released together */
    struct { uint8_t kind; uint8_t type; uint16_t len, off; } deferred[CT_REVERSE_DEFERRED];
    uint8_t deferred_n;
    uint16_t deferred_used;
    uint8_t deferred_buf[CT_REVERSE_DEFERRED_BYTES];

    /* pairing */
    ct_u16ring_t switch_rounds, parent_syncs;
    int32_t last_round;
    struct { bool active; uint8_t action; uint16_t kind, count; } step;
    bool reopen_token, battle_pending, exit_pending;
    uint16_t switch_close_count;

    /* cable session */
    bool expect_parent_lp, live, keys_to_parent, keys_to_switch, keys_await_switch, keys_drain;
    uint16_t rx_size, rx_pos;
    uint8_t rx_buf[CT_MAX_BLOCK_BYTES];
    uint8_t cq_head, cq_n, held_head, held_n;
    ct_cmd_t cq[CT_REVERSE_CQ];
    ct_cmd_t held[CT_REVERSE_HELD];
    ct_keyruns_t switch_keys;
    ct_steps_t steps;
    ct_keyqueue_t parent_keys;
    double key_step;
    uint32_t key_pushes, key_pops;

    /* counters */
    uint32_t cq_drops, held_drops, deferred_drops, ring_drops, parent_block_too_long;
} ct_reverse_t;

/* Installs the leader hooks. The board then calls ct_reverse_open_group() to open the group the Switch joins. */
void ct_reverse_init(ct_reverse_t *r, ct_leader_t *leader, const ct_reverse_cfg_t *cfg);
void ct_reverse_open_group(ct_reverse_t *r);
/* The game side for ct_link (gameCommand / nextCommand / cableReset / ready). */
ct_game_t ct_reverse_game(ct_reverse_t *r);
void ct_reverse_game_command(ct_reverse_t *r, const uint16_t words[CT_CMD_WORDS]);
bool ct_reverse_next_command(ct_reverse_t *r, uint16_t words[CT_CMD_WORDS]);
void ct_reverse_cable_reset(ct_reverse_t *r);
static inline bool ct_reverse_ready(const ct_reverse_t *r) { return ct_leader_linked(r->leader); }

#ifdef __cplusplus
}
#endif
#endif
