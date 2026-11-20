/* SPDX-License-Identifier: AGPL-3.0-only
 * Derived from pokeldn (https://github.com/Decryptu/pokeldn), Copyright (c) the pokeldn authors.
 * Changes Copyright (c) 2026 Kizuna. See NOTICE.
 */
/* mg_host.h — leader-side FRLG Mystery Gift engine: the RFU conversation that wraps an mg_server
 * session. Portable C port of pokeldn frlg/gift/host_mystery_gift.py (HostMysteryGiftEngine).
 *
 * Contract (pokeldn's feed_child_slot()/tick()): feed every new child UNI command row (idle rows
 * too: they count the console's quiet frames), and call tick() once per VBlank for the parent's
 * gSendCmd row. Phases: SEND_PLAYER_IDS + one SEND_BLOCK_REQ -> swap LinkPlayer blocks -> one
 * standby barrier -> wait for the console to go quiet -> the gift messages, each block paced
 * (nothing acknowledges a block; one sent too early is dropped and the console hangs) -> the
 * close-link handshake, holding the link while the console saves. Pure/deterministic. */
#ifndef MG_HOST_H
#define MG_HOST_H
#include <stdint.h>
#include <stddef.h>
#include "block.h"
#include "mg_link.h"
#include "mg_server.h"
#include "pia_activity.h"

typedef struct {
    int startup_standby_echo_frames;    /* 4: a one-VBlank echo can be missed */
    int client_ready_idle_frames;       /* 20: the console reaches MysteryGiftClient_Create late */
    int inter_block_gap_frames;         /* 36: the console takes up to 13 frames per block, dies at 16 */
    int player_ids_repeat_frames;       /* 8: a single-VBlank SEND_PLAYER_IDS is missed */
    int block_repeat;                   /* 2 */
    int ram_script_block_repeat;        /* 3: a lost RAM-script fragment cannot be resent */
    int close_retry_frames;             /* 60 */
    int post_client_close_grace_frames; /* 300: the console's link close and save must finish */
} mg_timing;
extern const mg_timing MG_DEFAULT_TIMING;

/* Our LinkPlayer [include/link.h:158]; the console checks only the magics and the version. */
typedef struct {
    const char *name;        /* ASCII, up to 7 characters */
    uint32_t    trainer_id;
    uint16_t    version;     /* 0x4004 FireRed, 0x4005 LeafGreen */
} mg_link_player;

typedef enum { MGH_LINK_PLAYER, MGH_START, MGH_GIFT, MGH_CLOSE, MGH_DONE } mgh_state;

#define MGH_WORDS_MAX   128
#define MGH_BLOCKS_MAX  8
#define MGH_RECV_MAX    8
#define MGH_LP_BLOCK    200   /* the wireless path pulls a fixed 200-byte buffer (count 17) */

typedef struct {
    mg_timing   timing;
    mgh_state   state;
    mg_server   server;

    uint16_t    words_q[MGH_WORDS_MAX][7];
    int         wq_head, wq_len, words_dropped;
    struct { const uint8_t *data; size_t len; uint16_t ident; } blocks[MGH_BLOCKS_MAX];
    int         bq_head, bq_len;
    block_sender sender;
    int         sender_active;
    int         gap;

    recv_block  rx;
    mg_rx       receiver;
    uint8_t     recv_q[MGH_RECV_MAX][BLOCK_MAX_BYTES];
    size_t      recv_len[MGH_RECV_MAX];
    int         rq_head, rq_len;
    mg_msg      msg;

    uint8_t     lp_block[MGH_LP_BLOCK];
    uint8_t     child_link_player[28];
    int         child_link_player_valid;
    int         expect_link_player, link_player_seen, host_lp_queued, host_lp_complete;
    int         standby_seen, pending_standby_count;   /* -1 = none latched */
    int         idle_run, exit_count;
    int         close_confirmed, close_grace_wait, close_retry_wait;   /* grace -1 = not started */
    int         disconnect_requested, done, gift_sent, result;

    int         child_mp_id, rejected_link_players, our_blocks_sent;
    int         child_frames, child_idles, parent_polls;
} mg_host;

/* gift must outlive the host; timing NULL = MG_DEFAULT_TIMING. */
void mg_host_init(mg_host *h, const mg_gift *gift, const mg_link_player *lp, const mg_timing *timing);
void mg_host_feed_child_slot(mg_host *h, const uint8_t slot[14]);
void mg_host_tick(mg_host *h, uint16_t words[7]);
/* Call once the 'D' frame went out (after disconnect_requested). */
void mg_host_mark_disconnect_sent(mg_host *h);
/* The vtable pia_hsess/pia_host_init take; ctx = h, which must outlive the session. */
pia_activity mg_host_activity(mg_host *h);

#endif
