/* SPDX-License-Identifier: AGPL-3.0-only */
/* ct_link.c -- CelioLink, ported from GB-Link-Switch-LDN web/js/cable/celio.js lines 47-266 (commit c972db4,
 * https://github.com/GB-Link/GB-Link-Switch-LDN). Copyright (c) the GB-Link-Switch-LDN authors. Modified for kizuna
 * 2026-11-02: C port, transport parts dropped (see ct_link.h). AGPL-3.0, see NOTICE. */
#include "ct_link.h"

#include <string.h>

static void status(ct_link_t *l, uint16_t v) { if (l->io.status) l->io.status(l->io.ctx, v); }

static void set_connected(ct_link_t *l, bool v)
{
    if (l->connected == v) return;
    l->connected = v;
    if (l->io.state) l->io.state(l->io.ctx, v);
}

void ct_link_init(ct_link_t *l, const ct_game_t *game, const ct_link_io_t *io, uint16_t await_status)
{
    memset(l, 0, sizeof *l);
    l->game = *game;
    l->io = *io;
    l->await_status = await_status;
    l->master = -1;
}

void ct_link_start(ct_link_t *l, int64_t now_ms)
{
    (void)now_ms;
    if (l->running) return;
    l->running = true;
    status(l, l->await_status);
}

void ct_link_stop(ct_link_t *l)
{
    l->running = false;
    set_connected(l, false);
}

static void maybe_ready(ct_link_t *l, int64_t now)
{
    if (!l->running || l->master < 0 || !l->partner || l->connected || l->waiting || l->handshake || l->closed) return;
    if (now < l->ready_after || !l->game.ready(l->game.ctx)) return;
    l->waiting = true;
    status(l, CT_LINK_HANDSHAKE_RECEIVED);
}

void ct_link_set_partner(ct_link_t *l, bool present, int64_t now_ms)
{
    l->partner = present;
    maybe_ready(l, now_ms);
}

static void up(ct_link_t *l, int64_t now)
{
    l->since = now;
    l->slots = 0;
    /* The master's ConnectLink came after its partner (the slave) linked: it sends at once. */
    l->peer_up = l->master == 1;
    l->sections++;
    set_connected(l, true);
    status(l, CT_LINK_LINK_CONNECTED);
}

static void start_sending(ct_link_t *l, int64_t now)
{
    l->peer_up = true;
    l->since = now;
    l->slots = 0;
}

void ct_link_on_command(ct_link_t *l, uint8_t command, int64_t now_ms)
{
    switch (command) {
    case CT_LINK_CMD_SET_MODE_SLAVE:
    case CT_LINK_CMD_SET_MODE_MASTER:
        if (l->master >= 0) break;
        l->master = command == CT_LINK_CMD_SET_MODE_MASTER;
        maybe_ready(l, now_ms);
        break;
    case CT_LINK_CMD_START_HANDSHAKE:
        if (!l->waiting) break;
        l->waiting = false;
        l->game.reset(l->game.ctx);
        l->sent_close = l->got_close = l->exit_seen = false;
        l->batch_n = 0;
        l->cafe_streak = 0;
        if (l->master == 1) l->handshake = true;
        else up(l, now_ms);
        break;
    case CT_LINK_CMD_CONNECT_LINK:
        if (l->master != 1) {
            if (l->connected && !l->peer_up) start_sending(l, now_ms);
            break;
        }
        if (!l->handshake) break;
        l->handshake = false;
        up(l, now_ms);
        break;
    }
}

static void flush(ct_link_t *l)
{
    uint8_t out[CT_DATA_BYTES];
    bool any = false;
    for (unsigned c = 0; c < CT_BATCH; c++)
        for (unsigned i = 0; i < CT_CMD_WORDS; i++) {
            uint16_t w = c < l->batch_n ? l->batch[c][i] : 0;
            if (w) any = true;
            ct_put16(out + (c * CT_CMD_WORDS + i) * 2, w);
        }
    l->batch_n = 0;
    if (!any) return;
    if (l->io.data) l->io.data(l->io.ctx, l->sequence, out);
    l->sequence++;
}

/* A command whose first word is 0x5FFF, sent and received, ends the section; EXIT_ROOM before it ends the link. */
static bool check_close(ct_link_t *l, int64_t now)
{
    if (!l->connected || !l->sent_close || !l->got_close) return false;
    if (l->batch_n) {
        while (l->batch_n < CT_BATCH) memset(l->batch[l->batch_n++], 0, sizeof l->batch[0]);
        flush(l);
    }
    set_connected(l, false);
    if (l->exit_seen) {
        l->closed = true;
        status(l, CT_LINK_LINK_CLOSED);
    } else {
        status(l, CT_LINK_LINK_RECONNECTING);
        l->ready_after = now + CT_LINK_REOPEN_MS;
    }
    return true;
}

void ct_link_on_data(ct_link_t *l, uint32_t seq, const uint8_t frame[CT_DATA_BYTES], int64_t now_ms)
{
    /* The relay client delivers in order, once; a repeat is dropped, a gap is counted and passed (no reorder buffer). */
    if (seq < l->expected) { l->data_dup++; return; }
    if (seq > l->expected) l->data_gap++;
    l->expected = seq + 1;
    if (!l->connected && !l->handshake) return;
    for (unsigned c = 0; c < CT_BATCH; c++) {
        uint16_t w[CT_CMD_WORDS];
        bool any = false;
        for (unsigned i = 0; i < CT_CMD_WORDS; i++) { w[i] = ct_le16(frame + (c * CT_CMD_WORDS + i) * 2); any |= w[i] != 0; }
        if (!any) continue;
        if (w[0] == 0x5fff) l->got_close = true;
        if (w[0] == 0xcafe && (w[1] & 0xff) == CT_KEY_EXIT_ROOM) l->exit_seen = true;
        l->game.game_command(l->game.ctx, w);
    }
    check_close(l, now_ms);
}

void ct_link_pump(ct_link_t *l, int64_t now_ms)
{
    if (!l->connected) { maybe_ready(l, now_ms); return; }
    if (!l->peer_up) {
        if (now_ms - l->since < CT_LINK_PEER_WAIT_MS) return;
        start_sending(l, now_ms);
    }
    int64_t played = (now_ms - l->since) / CT_LINK_PACKET_MS;
    while ((int64_t)l->slots < played + CT_LINK_AHEAD) {
        uint16_t w[CT_CMD_WORDS];
        if (!l->game.next_command(l->game.ctx, w)) memset(w, 0, sizeof w);
        if (w[0] == 0x5fff) l->sent_close = true;
        if (w[0] == 0xcafe && (w[1] & 0xff) == CT_KEY_EXIT_ROOM) l->exit_seen = true;
        /* Like the adapter: a run of "no key" reports travels as its first one. */
        if (w[0] == 0xcafe && w[1] == CT_KEY_EMPTY) {
            if (l->cafe_streak++ > 0) memset(w, 0, sizeof w);
        } else {
            bool any = false;
            for (unsigned i = 0; i < CT_CMD_WORDS; i++) any |= w[i] != 0;
            if (any) l->cafe_streak = 0;
        }
        memcpy(l->batch[l->batch_n++], w, sizeof w);
        l->slots++;
        if (l->batch_n == CT_BATCH) flush(l);
        if (check_close(l, now_ms)) return;
    }
}
