/* SPDX-License-Identifier: AGPL-3.0-only
 * Derived from pokeldn (https://github.com/Decryptu/pokeldn), Copyright (c) the pokeldn authors.
 * Changes Copyright (c) 2026 Kizuna. See NOTICE.
 */
/* mg_host.c — portable C port of pokeldn frlg/gift/host_mystery_gift.py. See mg_host.h. */
#include "mg_host.h"
#include "charmap.h"
#include "rfu.h"
#include <string.h>

const mg_timing MG_DEFAULT_TIMING = {
    .startup_standby_echo_frames = 4, .client_ready_idle_frames = 20, .inter_block_gap_frames = 36,
    .player_ids_repeat_frames = 8, .block_repeat = 2, .ram_script_block_repeat = 3,
    .close_retry_frames = 60, .post_client_close_grace_frames = 5 * 60,
};

#define COUNT_PARTY 17                 /* the LinkPlayer block arrives as 17 x 12-byte fragments */
#define LP_FIELD2   0x8000
#define LANGUAGE_ENGLISH 2
#define HOST_NAME_PAD 0xFF             /* robust host-role fixed-field terminators */
static const uint8_t GAMEFREAK_MAGIC[16] = "GameFreak inc.";

static void put16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static void put32(uint8_t *p, uint32_t v) { put16(p, (uint16_t)v); put16(p + 2, (uint16_t)(v >> 16)); }

/* LinkPlayerBlock [link.c:343]: magic, struct LinkPlayer (28 B), magic; zero-padded to 200. */
static void build_link_player_block(const mg_link_player *lp, uint8_t out[MGH_LP_BLOCK]) {
    memset(out, 0, MGH_LP_BLOCK);
    memcpy(out, GAMEFREAK_MAGIC, 16);
    uint8_t *p = out + 16;
    put16(p, lp->version);
    put16(p + 2, LP_FIELD2);
    put32(p + 4, lp->trainer_id);
    charmap_encode(lp->name, 8, HOST_NAME_PAD, p + 8);
    /* progress flags, never_read, progress copy, gender, link type, player id: zero */
    put16(p + 26, LANGUAGE_ENGLISH);
    memcpy(out + 44, GAMEFREAK_MAGIC, 16);
}

static void queue_words(mg_host *h, const uint16_t w[7]) {
    if (h->wq_len == MGH_WORDS_MAX) { h->words_dropped++; return; }
    memcpy(h->words_q[(h->wq_head + h->wq_len++) % MGH_WORDS_MAX], w, 7 * sizeof(uint16_t));
}

static void queue_block(mg_host *h, const uint8_t *data, size_t len, uint16_t ident) {
    if (h->bq_len == MGH_BLOCKS_MAX) return;
    int i = (h->bq_head + h->bq_len++) % MGH_BLOCKS_MAX;
    h->blocks[i].data = data; h->blocks[i].len = len; h->blocks[i].ident = ident;
}

/* SEND_PLAYER_IDS is idempotent on the console; SEND_BLOCK_REQ is not. */
static void queue_player_ids(mg_host *h) {
    /* w1 = playerCount, w2 = linkPlayerIdx[0..1]: one child in RFU slot 0 reads mpId 1 */
    const uint16_t w[7] = { RFU_SEND_PLAYER_IDS, 2, 0x0001, 0, 0, 0, 0 };
    for (int i = 0; i < h->timing.player_ids_repeat_frames; i++) queue_words(h, w);
}

static void queue_close_words(mg_host *h) {
    uint16_t w[7];
    rfu_close_link_words((uint16_t)h->exit_count, w);
    for (int i = 0; i < h->timing.startup_standby_echo_frames; i++) queue_words(h, w);
}

/* Both sides must expose READY_CLOSE_LINK before the console is released into its save. */
static void begin_close(mg_host *h) {
    h->state = MGH_CLOSE;
    queue_close_words(h);
    h->close_retry_wait = h->timing.close_retry_frames;
}

static void finish_gift(mg_host *h, int result) {
    h->result = result;
    h->gift_sent = result == MG_SVR_CARD_SENT;
    begin_close(h);
}

static void queue_message(mg_host *h, const mg_action *a) {
    if (mg_msg_build(&h->msg, a->ident, a->data, a->len, a->size) != 0) return;
    for (int i = 0; i < h->msg.nblocks; i++) {
        const uint8_t *p;
        size_t n = mg_msg_block(&h->msg, i, &p);
        queue_block(h, p, n, a->ident);
    }
}

/* Returns 1 when a whole message reached the server. */
static int drain_recv_blocks(mg_host *h) {
    while (h->rq_len && mg_rx_active(&h->receiver)) {
        int i = h->rq_head;
        h->rq_head = (h->rq_head + 1) % MGH_RECV_MAX;
        h->rq_len--;
        mg_rx_result r = mg_rx_feed(&h->receiver, h->recv_q[i], h->recv_len[i]);
        if (r < 0) {
            /* A console retransmit of the previous message surfaces as a stale header: drop it and
             * re-arm the same ident. */
            mg_rx_expect(&h->receiver, (uint16_t)h->receiver.expected_ident);
            continue;
        }
        if (r == MG_RX_PENDING) continue;
        mg_server_on_received(&h->server, h->receiver.ident, h->receiver.buf, h->receiver.size);
        return 1;
    }
    return 0;
}

static void pump_server(mg_host *h) {
    while (h->state == MGH_GIFT) {
        const mg_action *a = mg_server_next(&h->server);
        if (a->kind == MG_ACT_SEND) {
            if (h->bq_len || h->sender_active) return;
            queue_message(h, a);
            return;
        }
        if (a->kind == MG_ACT_RECV) {
            if (!mg_rx_active(&h->receiver)) mg_rx_expect(&h->receiver, a->ident);
            if (!drain_recv_blocks(h)) return;
            continue;
        }
        finish_gift(h, a->kind == MG_ACT_DONE ? a->result : MG_SVR_NOTHING_SENT);
        return;
    }
}

static void begin_gift(mg_host *h) {
    h->state = MGH_GIFT;
    pump_server(h);
}

void mg_host_init(mg_host *h, const mg_gift *gift, const mg_link_player *lp, const mg_timing *timing) {
    memset(h, 0, sizeof(*h));
    h->timing = timing ? *timing : MG_DEFAULT_TIMING;
    h->state = MGH_LINK_PLAYER;
    mg_server_init(&h->server, gift);
    recv_block_init(&h->rx);
    mg_rx_init(&h->receiver);
    build_link_player_block(lp, h->lp_block);
    h->pending_standby_count = -1;
    h->close_grace_wait = -1;
    h->child_mp_id = -1;
    /* One SEND_BLOCK_REQ only [link_rfu_2.c:1813]; a repeat starts a second send. */
    queue_player_ids(h);
    h->expect_link_player = 1;
    const uint16_t req[7] = { RFU_SEND_BLOCK_REQ, 0, 0, 0, 0, 0, 0 };
    queue_words(h, req);
}

static void complete_link_player_barrier(mg_host *h) {
    if (h->state != MGH_LINK_PLAYER) return;
    /* Exactly one SetLinkStandbyCallback round precedes the client [union_room.c:2391]. */
    h->pending_standby_count = -1;
    h->state = MGH_START;
    h->standby_seen = 1;
    h->idle_run = 0;
}

static void on_child_block(mg_host *h, int count, const uint8_t *data, size_t len) {
    if (h->state == MGH_LINK_PLAYER && h->expect_link_player) {
        /* The child ships gBlockSendBuffer verbatim; asked too early it sends stale bytes. Never
         * re-request: a duplicate BLOCK_REQ can restart a completed send. */
        if (count != COUNT_PARTY || len < 60 || memcmp(data, GAMEFREAK_MAGIC, 14)
            || memcmp(data + 44, GAMEFREAK_MAGIC, 14)) {
            h->rejected_link_players++;
            h->expect_link_player = 0;
            recv_block_init(&h->rx);
            return;
        }
        memcpy(h->child_link_player, data + 16, 28);
        h->child_link_player_valid = 1;
        h->expect_link_player = 0;
        h->link_player_seen = 1;
        h->host_lp_queued = 1;
        queue_block(h, h->lp_block, MGH_LP_BLOCK, 0);
        return;
    }
    if (h->state == MGH_GIFT) {
        if (h->rq_len == MGH_RECV_MAX) return;
        int i = (h->rq_head + h->rq_len++) % MGH_RECV_MAX;
        if (len > BLOCK_MAX_BYTES) len = BLOCK_MAX_BYTES;
        memcpy(h->recv_q[i], data, len);
        h->recv_len[i] = len;
        if (drain_recv_blocks(h)) pump_server(h);
    }
}

static void on_child_standby(mg_host *h, int count) {
    uint16_t w[7];
    rfu_exit_standby_words((uint16_t)count, w);
    for (int i = 0; i < h->timing.startup_standby_echo_frames; i++) queue_words(h, w);
    if (count + 1 > h->exit_count) h->exit_count = count + 1;
    if (h->state == MGH_LINK_PLAYER && h->link_player_seen) {
        if (h->host_lp_complete) complete_link_player_barrier(h);
        else h->pending_standby_count = count;   /* latch: the child may stop once it sees our echo */
    }
}

static void on_child_close(mg_host *h, int count) {
    if (count > h->exit_count) h->exit_count = count;
    if (h->state == MGH_LINK_PLAYER || h->state == MGH_START || h->state == MGH_GIFT) begin_close(h);
    if (h->state != MGH_CLOSE) return;
    if (!h->close_confirmed) {
        h->close_confirmed = 1;
        h->close_grace_wait = h->timing.post_client_close_grace_frames;
    }
}

void mg_host_feed_child_slot(mg_host *h, const uint8_t slot[14]) {
    h->child_frames++;
    rfu_parsed rec;
    rfu_parse_slot(slot, &rec);
    if (!rec.valid) {
        h->child_idles++;
        h->idle_run++;
        if (h->state == MGH_START && h->standby_seen && h->idle_run >= h->timing.client_ready_idle_frames)
            begin_gift(h);
        return;
    }
    h->idle_run = 0;
    switch (rec.op) {
    case RFU_SEND_BLOCK_INIT: {
        /* owner byte = mpId | 0x80; mpId 0 means the console missed SEND_PLAYER_IDS. */
        int mp_id = rec.owner_raw & 0x7F;
        h->child_mp_id = mp_id;
        if (h->state == MGH_LINK_PLAYER && mp_id != 1) queue_player_ids(h);
        recv_block_on_init(&h->rx, rec.count, rec.owner_raw);
        break;
    }
    case RFU_SEND_BLOCK: {
        int was_done = h->rx.done;
        recv_block_on_block(&h->rx, rec.index, rec.frag);
        if (h->rx.done && !was_done) {
            uint8_t data[BLOCK_MAX_BYTES];
            int count = h->rx.count;
            size_t n = recv_block_consume(&h->rx, data);
            on_child_block(h, count, data, n);
        }
        break;
    }
    case RFU_READY_EXIT_STANDBY: on_child_standby(h, rec.count); break;
    case RFU_READY_CLOSE_LINK:   on_child_close(h, rec.count); break;
    default: break;
    }
}

static void tick_close(mg_host *h) {
    if (h->close_grace_wait >= 0) {
        if (--h->close_grace_wait <= 0) {
            h->close_grace_wait = -1;
            h->disconnect_requested = 1;
            return;
        }
    }
    if (--h->close_retry_wait <= 0) {
        queue_close_words(h);
        h->close_retry_wait = h->timing.close_retry_frames;
    }
}

void mg_host_tick(mg_host *h, uint16_t words[7]) {
    memset(words, 0, 7 * sizeof(uint16_t));
    h->parent_polls++;
    if (h->state == MGH_CLOSE && !h->disconnect_requested) tick_close(h);

    if (h->wq_len) {
        memcpy(words, h->words_q[h->wq_head], 7 * sizeof(uint16_t));
        h->wq_head = (h->wq_head + 1) % MGH_WORDS_MAX;
        h->wq_len--;
        return;
    }
    if (h->gap > 0) { h->gap--; return; }

    if (!h->sender_active && h->bq_len) {
        int i = h->bq_head;
        h->bq_head = (h->bq_head + 1) % MGH_BLOCKS_MAX;
        h->bq_len--;
        int repeat = h->timing.block_repeat;
        if (h->blocks[i].ident == MG_LINKID_RAM_SCRIPT && h->timing.ram_script_block_repeat > repeat)
            repeat = h->timing.ram_script_block_repeat;
        block_sender_init(&h->sender, h->blocks[i].data, h->blocks[i].len, 0, 1);
        block_sender_set_repeat(&h->sender, repeat);
        h->sender_active = 1;
    }
    if (!h->sender_active) return;

    block_sender_tick(&h->sender, NULL, words);
    if (!block_sender_done(&h->sender)) return;
    if (h->state == MGH_LINK_PLAYER) {
        h->our_blocks_sent++;
        h->host_lp_complete = 1;
        if (h->pending_standby_count >= 0) complete_link_player_barrier(h);
    }
    h->sender_active = 0;
    h->gap = h->timing.inter_block_gap_frames;
    if (!h->bq_len && h->state == MGH_GIFT) {
        mg_server_on_sent(&h->server);
        pump_server(h);
    }
}

void mg_host_mark_disconnect_sent(mg_host *h) {
    if (!h->disconnect_requested) return;
    h->done = 1;
    h->state = MGH_DONE;
}

static void mgh_a_feed(void *c, const uint8_t slot[14]) { mg_host_feed_child_slot((mg_host *)c, slot); }
static void mgh_a_tick(void *c, uint16_t words[7]) { mg_host_tick((mg_host *)c, words); }
static int  mgh_a_disc(void *c) { return ((mg_host *)c)->disconnect_requested; }
static void mgh_a_sent(void *c) { mg_host_mark_disconnect_sent((mg_host *)c); }
static int  mgh_a_est(void *c) { return ((mg_host *)c)->child_link_player_valid; }

pia_activity mg_host_activity(mg_host *h) {
    pia_activity a = { h, mgh_a_feed, mgh_a_tick, mgh_a_disc, mgh_a_sent, mgh_a_est };
    return a;
}
