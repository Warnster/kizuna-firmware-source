/* SPDX-License-Identifier: AGPL-3.0-only
 * Derived from frlg-ldn-trade (https://github.com/tornadus/frlg-ldn-trade), Copyright (c) the frlg-ldn-trade authors.
 * Changes Copyright (c) 2026 Kizuna. See NOTICE.
 */
/* block.c — portable C port of frlgsim/block.py. See block.h. */
#include "block.h"
#include <string.h>

int block_frag_count(size_t nbytes) {
    if (nbytes == 0) return 1;
    size_t c = (nbytes + BLOCK_FRAG_BYTES - 1) / BLOCK_FRAG_BYTES;
    return (int)(c < 1 ? 1 : c);
}

uint32_t block_all_received_mask(int count) {
    if (count >= 32) return 0xFFFFFFFFu;
    return (1u << count) - 1u;
}

/* ---------------- receive ---------------- */
void recv_block_init(recv_block *rb) {
    memset(rb, 0, sizeof(*rb));
    rb->owner = -1;
    rb->last_index = -1;
}

void recv_block_on_init(recv_block *rb, int count, int owner) {
    /* New epoch only when size changes or not mid-block; same-size resend is idempotent. */
    if (!rb->receiving || rb->done || count != rb->count) {
        rb->count = count;
        rb->owner = owner;
        rb->flags = 0;
        rb->buf_len = (size_t)count * BLOCK_FRAG_BYTES;
        if (rb->buf_len > BLOCK_MAX_BYTES) rb->buf_len = BLOCK_MAX_BYTES;
        memset(rb->buf, 0, rb->buf_len);
        rb->receiving = 1;
        rb->done = 0;
        rb->last_index = -1;
        rb->epochs++;
    }
}

void recv_block_on_block(recv_block *rb, int index, const uint8_t frag12[12]) {
    if (!rb->receiving || !(index >= 0 && index < rb->count)) return;
    rb->last_index = index;
    rb->flags |= (1u << index);
    memcpy(rb->buf + (size_t)index * BLOCK_FRAG_BYTES, frag12, BLOCK_FRAG_BYTES);
    if (rb->flags == block_all_received_mask(rb->count)) rb->done = 1;
}

size_t recv_block_data(const recv_block *rb, uint8_t *out) {
    memcpy(out, rb->buf, rb->buf_len);
    return rb->buf_len;
}

size_t recv_block_consume(recv_block *rb, uint8_t *out) {
    size_t n = recv_block_data(rb, out);
    rb->receiving = 0;
    rb->done = 0;
    rb->flags = 0;
    return n;
}

/* ---------------- send ---------------- */
void block_sender_init(block_sender *bs, const uint8_t *data, size_t len,
                       int owner, int trust_pia) {
    memset(bs, 0, sizeof(*bs));
    bs->data = data;
    bs->data_len = len;
    bs->count = block_frag_count(len);
    bs->owner = owner;
    bs->state = BS_INIT;
    bs->watchdog_init = 4;
    bs->watchdog_hold = 6;
    bs->trust_pia = trust_pia;
}

int block_sender_done(const block_sender *bs) { return bs->state == BS_DONE; }

void block_sender_set_repeat(block_sender *bs, int repeat) { bs->stream_repeat = repeat; }

static void bs_chunk(const block_sender *bs, int index, uint8_t out12[12]) {
    memset(out12, 0, 12);
    size_t start = (size_t)index * BLOCK_FRAG_BYTES;
    size_t n = (start < bs->data_len) ? (bs->data_len - start) : 0;
    if (n > BLOCK_FRAG_BYTES) n = BLOCK_FRAG_BYTES;
    if (n) memcpy(out12, bs->data + start, n);
}

void block_sender_tick(block_sender *bs, const recv_block *ack, uint16_t out[7]) {
    memset(out, 0, 7 * sizeof(uint16_t));
    if (bs->state == BS_DONE) return;
    uint8_t chunk[12];

    if (bs->state == BS_INIT) {
        bs->init_sends++;
        int init_acked = ack && ack->receiving && ack->count == bs->count;
        int armed = bs->trust_pia
            ? (init_acked || bs->init_sends > bs->watchdog_init)
            : (init_acked || (ack == NULL && bs->init_sends > bs->watchdog_init));
        if (armed) { bs->state = BS_STREAM; bs->index = 0; /* fall through */ }
        else { rfu_init_words((uint16_t)bs->count, (uint16_t)bs->owner, out); return; }
    }

    if (bs->state == BS_STREAM) {
        int idx = bs->index;
        bs_chunk(bs, idx, chunk);
        rfu_send_block_words((uint16_t)idx, chunk, out);
        if (++bs->frag_sends < bs->stream_repeat) return;
        bs->frag_sends = 0;
        if (idx >= bs->count - 1) { bs->state = bs->trust_pia ? BS_DONE : BS_HOLD; bs->hold_sends = 0; }
        else bs->index++;
        return;
    }

    if (bs->state == BS_HOLD) {
        bs->hold_sends++;
        int last = bs->count - 1;
        uint32_t full = block_all_received_mask(bs->count);
        if (ack != NULL) {
            if (ack->last_index == last) {
                if (ack->flags == full) { bs->state = BS_DONE; return; }
                if (ack->count == bs->count) {
                    int missing[BLOCK_MAX_FRAGS], m = 0;
                    for (int i = 0; i < bs->count; i++)
                        if (!((ack->flags >> i) & 1u)) missing[m++] = i;
                    if (m > 0) {
                        bs->rr = (bs->rr + 1) % m;
                        int idx = missing[bs->rr];
                        bs_chunk(bs, idx, chunk);
                        rfu_send_block_words((uint16_t)idx, chunk, out);
                        return;
                    }
                }
            }
        } else if (bs->hold_sends > bs->watchdog_hold) {
            bs->state = BS_DONE; return;
        }
        bs_chunk(bs, last, chunk);
        rfu_send_block_words((uint16_t)last, chunk, out);
        return;
    }
}
