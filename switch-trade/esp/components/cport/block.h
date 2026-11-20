/* SPDX-License-Identifier: AGPL-3.0-only
 * Derived from frlg-ldn-trade (https://github.com/tornadus/frlg-ldn-trade), Copyright (c) the frlg-ldn-trade authors.
 * Changes Copyright (c) 2026 Kizuna. See NOTICE.
 */
/* block.h — RFU block transfer, portable C port of frlgsim/block.py.
 * Receive-side reassembler (idempotent, reorder/retransmit tolerant) + the CHILD's
 * ACK-gated send FSM (INIT -> STREAM -> HOLD -> DONE). Pure/deterministic. */
#ifndef BLOCK_H
#define BLOCK_H
#include <stdint.h>
#include <stddef.h>
#include "rfu.h"

#define BLOCK_FRAG_BYTES 12
#define BLOCK_MAX_FRAGS  32                              /* SEND_BLOCK index is 5 bits */
#define BLOCK_MAX_BYTES  (BLOCK_MAX_FRAGS * BLOCK_FRAG_BYTES)

int      block_frag_count(size_t nbytes);
uint32_t block_all_received_mask(int count);

typedef struct {
    int      count;
    int      owner;          /* -1 if unset */
    uint32_t flags;
    uint8_t  buf[BLOCK_MAX_BYTES];
    size_t   buf_len;
    int      receiving;
    int      done;
    int      last_index;
    int      epochs;
} recv_block;
void   recv_block_init(recv_block *rb);
void   recv_block_on_init(recv_block *rb, int count, int owner);
void   recv_block_on_block(recv_block *rb, int index, const uint8_t frag12[12]);
size_t recv_block_data(const recv_block *rb, uint8_t *out);
size_t recv_block_consume(recv_block *rb, uint8_t *out);

typedef enum { BS_INIT, BS_STREAM, BS_HOLD, BS_DONE } bs_state;
typedef struct {
    const uint8_t *data;
    size_t   data_len;
    int      count, owner;
    bs_state state;
    int      index;
    int      init_sends, hold_sends, rr;
    int      watchdog_init, watchdog_hold;
    int      trust_pia;
    int      stream_repeat;  /* sends per fragment in STREAM; 0/1 = once */
    int      frag_sends;
} block_sender;
void block_sender_init(block_sender *bs, const uint8_t *data, size_t len,
                       int owner, int trust_pia);
/* Emit this VBlank's 7-int gSendCmd (pre-tag). ack = peer-1 recv_block reflection, or NULL. */
void block_sender_tick(block_sender *bs, const recv_block *ack, uint16_t out_words[7]);
int  block_sender_done(const block_sender *bs);
/* Leader-side redundancy: the console never re-requests a lost fragment, so the Mystery Gift
 * leader streams each one `repeat` times (pokeldn BlockSender stream_repeat). */
void block_sender_set_repeat(block_sender *bs, int repeat);

#endif
