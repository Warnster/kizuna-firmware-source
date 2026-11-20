/* SPDX-License-Identifier: AGPL-3.0-only
 * Derived from pokeldn (https://github.com/Decryptu/pokeldn), Copyright (c) the pokeldn authors.
 * Changes Copyright (c) 2026 Kizuna. See NOTICE.
 */
/* rfu_leader.h — the LEADER (parent) side of the emulator RFU protocol, the inner payload of Pia
 * Reliable protocol 10. Portable C port of pokeldn gba/rfu_leader.py (RFULeader + ChildEcho) and
 * the parent halves of gba/ni.py (ParentNIReceiver, ParentNISender).
 *
 * receive() once per unique in-order Reliable payload (the Reliable layer de-duplicates), tick()
 * once per VBlank. Flow: child 'C' -> our 'A' + 'G'0; the child's NI game data (acked per window)
 * -> 'G'1 + our NI join status (stop-and-wait) -> UNI, where every tick emits the 5-row gRecvCmds
 * table: row 0 our gSendCmd, row 1 the child's own commands mirrored back (it waits on them).
 * Pure/deterministic. */
#ifndef RFU_LEADER_H
#define RFU_LEADER_H
#include <stdint.h>
#include <stddef.h>
#include "rfu.h"

#define RFUL_FRAME_MAX     96   /* parent UNI 'T' = 4 + 8 + roundup4(3 + 70) = 88 */
#define RFUL_PENDING_MAX   8
#define RFUL_ECHO_MAX      64   /* one console block's distinct commands; dropping one is never correct */
#define RFUL_SEEN_NI_MAX   16
#define RFUL_PNI_MAX       8
#define RFUL_GAME_DATA_MAX 64
#define RFU_STATUS_JOIN_GROUP_OK 5

typedef enum { RFUL_WAIT_CONNECT, RFUL_CHILD_NI, RFUL_PARENT_NI, RFUL_UNI, RFUL_DISCONNECTED } rful_state;

typedef enum {
    RFUL_EV_NONE, RFUL_EV_CONNECT, RFUL_EV_CONNECT_DUPLICATE, RFUL_EV_CONNECT_REJECTED,
    RFUL_EV_DISCONNECT, RFUL_EV_K_ACK, RFUL_EV_PARENT_NI_ACK, RFUL_EV_NI_ACK_IGNORED,
    RFUL_EV_NI_OUT_OF_PHASE, RFUL_EV_CHILD_NI_DUPLICATE, RFUL_EV_CHILD_NI,
    RFUL_EV_CHILD_NI_COMPLETE, RFUL_EV_UNI_EARLY, RFUL_EV_UNI,
} rful_event;

typedef struct { uint8_t len; uint8_t b[RFUL_FRAME_MAX]; } rful_frame;
typedef struct { uint8_t state, n, phase, len; uint8_t payload[RFU_SLOT_LEN]; } rful_ni_key;
typedef struct { uint8_t state, n, phase, size; uint8_t payload[8]; } rful_ni_step;

typedef struct {
    rful_state state;
    uint8_t    host_session_id[2];
    uint8_t    connect_id[2];
    int        have_connect;
    uint8_t    bm_slot, join_status;
    uint32_t   ts;
    uint8_t    child_cmd[RFU_SLOT_LEN];  /* last child UNI command, tag bits cleared */

    /* queued frames (A, G, NI acks) go out before anything else, one per tick */
    rful_frame pending[RFUL_PENDING_MAX];
    int        p_head, p_len;

    /* row one: the console's commands mirrored back, repeats coalesced */
    uint8_t    echo_q[RFUL_ECHO_MAX][RFU_SLOT_LEN];
    int        echo_head, echo_len;
    uint8_t    echo_row[RFU_SLOT_LEN];
    int        echo_dropped, echo_coalesced, echo_backlog_peak;

    /* the child's NI game data (26-byte RfuGameData) */
    uint8_t    ni_hdr[8];
    int        ni_hdr_len, ni_payload_size, ni_data_size;
    uint8_t    ni_buf[RFUL_GAME_DATA_MAX];
    int        ni_buf_len, ni_phase_count[4], ni_complete;
    uint8_t    child_game_data[RFUL_GAME_DATA_MAX];
    int        child_game_data_len;
    rful_ni_key seen_ni[RFUL_SEEN_NI_MAX];
    int        seen_ni_len;

    /* our NI: the 1-byte join status */
    rful_ni_step pni[RFUL_PNI_MAX];
    int        pni_len, pni_i;
    int        parent_waiting;
    uint8_t    pw_state, pw_n, pw_phase;
    uint8_t    pcur_slot[16];
    uint8_t    pcur_len;

    int        k_acks, uni_in, uni_out;
} rfu_leader;

/* host_session_id: native leaders use a random low byte and 0xf1 high byte. bm_slot 1 seats the
 * child in RFU slot 0 / multiplayer id 1. */
void       rfu_leader_init(rfu_leader *l, const uint8_t host_session_id[2], uint32_t start_ts);
rful_event rfu_leader_receive(rfu_leader *l, const uint8_t *inner, size_t len);
/* parent_cmd = our 14-byte gSendCmd row, NULL for idle. Returns the frame length (0 = nothing). */
size_t     rfu_leader_tick(rfu_leader *l, const uint8_t *parent_cmd, uint8_t out[RFUL_FRAME_MAX]);
/* Graceful close: the 'D' frame (0 if never connected). */
size_t     rfu_leader_disconnect_frame(rfu_leader *l, uint8_t out[8]);
void       rfu_leader_on_ldn_leave(rfu_leader *l);
int        rfu_leader_echo_backlog(const rfu_leader *l);

#endif
