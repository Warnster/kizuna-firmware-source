/* SPDX-License-Identifier: AGPL-3.0-only
 * Derived from GB-Link-Switch-LDN (https://github.com/GB-Link/GB-Link-Switch-LDN), Copyright (c) the GB-Link-Switch-LDN authors.
 * Changes Copyright (c) 2026 Kizuna. See NOTICE.
 */
/* ct_link.h -- the cable side over the relay's LINK stratum: a C port of CelioLink (GB-Link-Switch-LDN
 * web/js/cable/celio.js lines 47-266, commit c972db4, AGPL-3.0; see NOTICE).
 *
 * One "section" is one cable link the game opens. The link reports HANDSHAKE_RECEIVED (0xFF03) only when its game side is
 * ready and the partner is present; the relay then sends START_HANDSHAKE to both (game->reset()). The slave is up at once;
 * the master waits for CONNECT_LINK. As slave, commands wait for the peer's CONNECT_LINK or 4 s. A section ends when a
 * command whose first word is 0x5FFF has been both sent and received; after an EXIT_ROOM key (0xCAFE 0x17) that is
 * LINK_CLOSED (0xFF07), otherwise LINK_RECONNECTING (0xFF06) and ready again 400 ms later.
 * Pacing: one nextCommand() per 26 ms slot, up to 4 slots ahead, batched in fours (one 64-byte LINK_DATA); an all-zero
 * batch is not sent; a run of 0xCAFE 0x11 (no key) goes once, then zero words. Receive: delivered as soon as it arrives.
 *
 * Dropped from the JS: Socket.IO, uuids and command de-duplication, requestWithRetry/in-flight window, the reorder Map
 * (the relay client delivers LINK_DATA in order, once: kzn_link_session). Statuses go out through io.status in order.
 * Added: the AWAIT_MODE value is configurable (the far side reports 0xFF01 so the relay always makes it the slave).
 *
 * No RTOS, no malloc, no printf. Time is the caller's now_ms (monotonic, ms). */
#ifndef CT_LINK_H
#define CT_LINK_H

#include "ct_common.h"

#ifdef __cplusplus
extern "C" {
#endif

enum {
    CT_LINK_AWAIT_MODE_EMULATOR = 0xff01,   /* always the slave */
    CT_LINK_AWAIT_MODE = 0xff02,
    CT_LINK_HANDSHAKE_RECEIVED = 0xff03,
    CT_LINK_LINK_CONNECTED = 0xff05,
    CT_LINK_LINK_RECONNECTING = 0xff06,
    CT_LINK_LINK_CLOSED = 0xff07,

    CT_LINK_CMD_SET_MODE_MASTER = 0x10,
    CT_LINK_CMD_SET_MODE_SLAVE = 0x11,
    CT_LINK_CMD_START_HANDSHAKE = 0x12,
    CT_LINK_CMD_CONNECT_LINK = 0x13,

    CT_LINK_PACKET_MS = 26,
    CT_LINK_AHEAD = 4,
    CT_LINK_REOPEN_MS = 400,
    CT_LINK_PEER_WAIT_MS = 4000,
};

typedef struct {
    void *ctx;
    void (*status)(void *ctx, uint16_t status);                              /* -> LINK_STATUS */
    void (*data)(void *ctx, uint32_t seq, const uint8_t frame[CT_DATA_BYTES]);   /* -> LINK_DATA (4 cmds x 8 LE u16) */
    void (*state)(void *ctx, bool connected);                                /* optional: a section opened/closed */
} ct_link_io_t;

typedef struct {
    ct_game_t game;
    ct_link_io_t io;
    uint16_t await_status;
    int8_t master;            /* -1 until the relay chose */
    bool running, partner, connected, peer_up, waiting, handshake, closed;
    bool sent_close, got_close, exit_seen;
    uint8_t batch_n;
    uint32_t cafe_streak;
    uint32_t sequence, expected;
    uint32_t slots;
    int64_t ready_after, since;
    uint16_t batch[CT_BATCH][CT_CMD_WORDS];
    /* counters */
    uint32_t sections, data_dup, data_gap;
} ct_link_t;

void ct_link_init(ct_link_t *l, const ct_game_t *game, const ct_link_io_t *io, uint16_t await_status);
void ct_link_start(ct_link_t *l, int64_t now_ms);     /* reports await_status */
void ct_link_stop(ct_link_t *l);
void ct_link_set_partner(ct_link_t *l, bool present, int64_t now_ms);   /* the relay says the peer is in the room */
void ct_link_on_command(ct_link_t *l, uint8_t command, int64_t now_ms); /* LINK_CMD from the relay */
void ct_link_on_data(ct_link_t *l, uint32_t seq, const uint8_t frame[CT_DATA_BYTES], int64_t now_ms);  /* LINK_DATA */
void ct_link_pump(ct_link_t *l, int64_t now_ms);      /* every <= 8 ms */

#ifdef __cplusplus
}
#endif
#endif
