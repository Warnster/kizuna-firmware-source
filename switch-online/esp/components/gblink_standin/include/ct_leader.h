/* SPDX-License-Identifier: AGPL-3.0-only
 * Derived from GB-Link-Switch-LDN (https://github.com/GB-Link/GB-Link-Switch-LDN), Copyright (c) the GB-Link-Switch-LDN authors.
 * Changes Copyright (c) 2026 Kizuna. See NOTICE.
 */
/* ct_leader.h -- the wireless leader's link layer (pokefirered librfu as a parent with one child in slot 0, as the
 * board's glp_host side sees an RFU adapter). C port of RfuLeader, GB-Link-Switch-LDN web/js/cable/leader.js
 * (commit c972db4, AGPL-3.0; see NOTICE).
 *
 * In:  ct_leader_board_frame()  RFU1 frames from the glp_host side: CONNECT_REQ, DISCONNECT, CLIENT_SEND (the Switch).
 *      ct_leader_tick()         once per GBA frame (59.727 Hz).
 * Out: the send callback        RFU1 frames to the glp_host side: BROADCAST (36 B) every 30 ticks while open,
 *                               CONNECT_ACK / CONNECT_NACK / DISCONNECT (16 B), one HOST_SEND (104 B) per tick once joined.
 *      hooks                    on_joined / on_command / on_block / on_closed / on_tick and key_source (ct_reverse sets them).
 *
 * Fixed storage instead of the JS arrays: CT_LEADER_OWN queued own commands (a 200-byte block is 4 INIT + 17 fragments),
 * CT_LEADER_ECHOES echoes, a CT_LEADER_CHILD_BUF-byte FIFO of the Switch's frames. Overflow drops the newest and counts it
 * (the JS arrays are unbounded); the tests check the counters stay zero. */
#ifndef CT_LEADER_H
#define CT_LEADER_H

#include "ct_common.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef CT_LEADER_OWN
#define CT_LEADER_OWN 64
#endif
#ifndef CT_LEADER_ECHOES
#define CT_LEADER_ECHOES 16
#endif
#ifndef CT_LEADER_CHILD_BUF
#define CT_LEADER_CHILD_BUF 384
#endif

enum { CT_LEADER_IDLE, CT_LEADER_OPEN, CT_LEADER_NAMING, CT_LEADER_ANSWERING, CT_LEADER_LINKED, CT_LEADER_CLOSED };

typedef struct {
    void *ctx;
    void (*on_joined)(void *ctx, const uint8_t game_data[26]);      /* linked: the Switch's 26-byte RfuGameData */
    void (*on_command)(void *ctx, const uint16_t words[7]);         /* each command of the Switch, blocks excepted */
    void (*on_block)(void *ctx, unsigned count, const uint8_t *data);   /* a whole block: count fragments, count*12 B */
    void (*on_closed)(void *ctx);                                   /* the Switch left */
    void (*on_tick)(void *ctx);                                     /* once per tick, first */
    bool (*key_source)(void *ctx, uint8_t *code);                   /* held key for this frame; false: none */
} ct_leader_hooks_t;

typedef struct {
    bool valid, done, init_echo_set, last_echo_set, whole_echoed;
    uint8_t count;
    uint32_t flags, init_echo, last_echo;
    uint8_t data[CT_MAX_BLOCK_BYTES];
} ct_leader_recv_t;

typedef struct {
    ct_frame_fn send;
    void *send_ctx;
    ct_leader_hooks_t hooks;
    ct_rand_fn rand32;
    void *rand_ctx;

    uint16_t devid, child_devid;
    uint8_t state;
    bool have_beacon, name_done, have_ni_ack;
    uint8_t beacon[24];
    uint32_t ticks;

    uint16_t cf_head, cf_used;           /* the Switch's CLIENT_SEND payloads: [len][bytes]... */
    uint8_t cf_buf[CT_LEADER_CHILD_BUF];

    uint8_t child_name[26];
    uint16_t child_name_at;
    uint8_t ni_ack[3];
    uint8_t recv_n[4];
    uint8_t join_step, idle_null;
    uint8_t child_seq;
    uint8_t key_count;

    uint8_t own_head, own_n, echo_head, echo_n;
    uint16_t own[CT_LEADER_OWN][7];
    uint16_t echoes[CT_LEADER_ECHOES][7];

    ct_leader_recv_t recv;
    uint32_t child_quiet_at;
    int64_t child_busy_at;

    /* counters */
    uint32_t own_drops, echo_drops, child_drops, out_of_order;
} ct_leader_t;

/* rand draws the leader's devid now and the child's devid at each join (JS: 1 + floor(Math.random() * 0xfffe)). */
void ct_leader_init(ct_leader_t *l, ct_frame_fn send, void *send_ctx, ct_rand_fn rand32, void *rand_ctx);
void ct_leader_set_hooks(ct_leader_t *l, const ct_leader_hooks_t *hooks);

/* 24-byte librfu game data broadcast (leaderBeacon): serial, compatibility, trainer id, activity, name, checksum. */
void ct_leader_beacon(uint8_t out[24], uint8_t activity, const uint8_t name[8], uint16_t trainer_id, uint16_t compatibility);

void ct_leader_open(ct_leader_t *l, const uint8_t beacon[24]);   /* opens (or updates) the group the Switch sees */
void ct_leader_close(ct_leader_t *l);
void ct_leader_board_frame(ct_leader_t *l, const uint8_t *frame, size_t len);
void ct_leader_tick(ct_leader_t *l);

/* Game level (link_rfu_2.c as the leader); each queues commands, one goes out per tick. */
void ct_leader_command(ct_leader_t *l, const uint16_t words[7]);
void ct_leader_player_ids(ct_leader_t *l);
void ct_leader_request(ct_leader_t *l, uint16_t type);
void ct_leader_standby(ct_leader_t *l, uint16_t count);          /* twice */
void ct_leader_close_link(ct_leader_t *l, uint16_t count);
void ct_leader_echo_close_link(ct_leader_t *l, uint16_t count);
unsigned ct_leader_send_block(ct_leader_t *l, const uint8_t *data, size_t len, unsigned repeat);   /* INIT x4, frags */

void ct_leader_child_game_data(const ct_leader_t *l, uint8_t out[26]);
static inline bool ct_leader_linked(const ct_leader_t *l) { return l->state == CT_LEADER_LINKED; }
static inline bool ct_leader_joined(const ct_leader_t *l)
{ return l->state == CT_LEADER_NAMING || l->state == CT_LEADER_ANSWERING || l->state == CT_LEADER_LINKED; }
static inline bool ct_leader_child_quiet(const ct_leader_t *l) { return (int64_t)l->child_quiet_at > l->child_busy_at; }
/* A block from the Switch is being received (reverse.js: leader.recv && !leader.recv.done). */
static inline bool ct_leader_receiving(const ct_leader_t *l) { return l->recv.valid && !l->recv.done; }

#ifdef __cplusplus
}
#endif
#endif
