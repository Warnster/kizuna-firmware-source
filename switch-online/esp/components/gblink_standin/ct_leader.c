/* SPDX-License-Identifier: AGPL-3.0-only */
/* ct_leader.c -- RfuLeader, ported from GB-Link-Switch-LDN web/js/cable/leader.js (commit c972db4,
 * https://github.com/GB-Link/GB-Link-Switch-LDN). Copyright (c) the GB-Link-Switch-LDN authors. Modified for kizuna
 * 2026-11-02: C port with fixed-size queues (see ct_leader.h), randomness injected. AGPL-3.0, see NOTICE. */
#include "ct_leader.h"

#include <string.h>

enum { ST_NULL = 0, ST_NI_START = 1, ST_NI = 2, ST_NI_END = 3, ST_UNI = 4 };
enum { BEACON_TICKS = 30, ECHO_RETRY = 16, ECHO_RETRY_MISSING = 48, UNI_PAYLOAD = 70 };
#define CHILD_SLOT_BIT (1u << 18)

static uint16_t random_devid(ct_rand_fn r, void *ctx)
{
    /* 1 + floor(Math.random() * 0xfffe) with Math.random() = r / 2^32, exact */
    return (uint16_t)(1 + (((uint64_t)r(ctx) * 0xfffe) >> 32));
}

static void send_cmd(ct_leader_t *l, uint32_t type, uint32_t header)
{
    uint8_t f[16];
    ct_rfu1_frame(f, type, header, sizeof f);
    l->send(l->send_ctx, f, sizeof f);
}

static size_t parent_frame(uint8_t *out, unsigned state, unsigned n, unsigned phase, const uint8_t *payload, size_t len, bool ack)
{
    uint32_t h = CHILD_SLOT_BIT | (state << 14) | (ack ? 1u << 13 : 0) | (n << 11) | (phase << 9) | (uint32_t)len;
    out[0] = (uint8_t)h; out[1] = (uint8_t)(h >> 8); out[2] = (uint8_t)(h >> 16);
    if (len) memcpy(out + 3, payload, len);
    return 3 + len;
}

/* The join answer, sent as the parent's NI data: control block, then the status byte (5 JOIN_GROUP_OK), then END and
 * NULL. Each step repeats until the child acks it. */
static const struct { uint8_t state, n; uint8_t len; uint8_t payload[5]; } JOIN_OK[4] = {
    { ST_NI_START, 1, 5, { 0x00, 0x05, 0x00, 0x01, 0x00 } },
    { ST_NI_START, 2, 2, { 0x00, 0x00 } },
    { ST_NI, 1, 1, { 0x05 } },
    { ST_NI_END, 0, 0, { 0 } },
};

void ct_leader_beacon(uint8_t b[24], uint8_t activity, const uint8_t name[8], uint16_t trainer_id, uint16_t compatibility)
{
    memset(b, 0, 24);
    ct_put16(b + 0, 0x0002);
    ct_put16(b + 2, compatibility);
    ct_put16(b + 4, trainer_id);
    b[12] = activity & 0x7f;
    memcpy(b + 16, name, 8);
    unsigned sum = 0;
    for (unsigned i = 0; i < 8; i++) sum += b[16 + i] + b[2 + i];
    b[15] = (uint8_t)~sum;
}

static void reset_child(ct_leader_t *l)
{
    l->cf_head = l->cf_used = 0;
    memset(l->child_name, 0, sizeof l->child_name);
    l->child_name_at = 0;
    l->name_done = false;
    memset(l->recv_n, 0, sizeof l->recv_n);
    l->have_ni_ack = false;
    l->join_step = 0;
    l->idle_null = 0;
    l->child_seq = 0xff;
    l->recv.valid = false;
    l->own_n = l->own_head = 0;
    l->echo_n = l->echo_head = 0;
    l->child_quiet_at = 0;
    l->child_busy_at = -1;
}

void ct_leader_init(ct_leader_t *l, ct_frame_fn send, void *send_ctx, ct_rand_fn rand32, void *rand_ctx)
{
    memset(l, 0, sizeof *l);
    l->send = send;
    l->send_ctx = send_ctx;
    l->rand32 = rand32;
    l->rand_ctx = rand_ctx;
    l->devid = random_devid(rand32, rand_ctx);
    l->state = CT_LEADER_IDLE;
    reset_child(l);
}

void ct_leader_set_hooks(ct_leader_t *l, const ct_leader_hooks_t *hooks) { l->hooks = *hooks; }

/* ---- game level */

void ct_leader_command(ct_leader_t *l, const uint16_t words[7])
{
    if (l->own_n >= CT_LEADER_OWN) { l->own_drops++; return; }
    memcpy(l->own[(l->own_head + l->own_n) % CT_LEADER_OWN], words, 14);
    l->own_n++;
}

static void cmd7(ct_leader_t *l, uint16_t w0, uint16_t w1, uint16_t w2)
{
    uint16_t w[7] = { w0, w1, w2, 0, 0, 0, 0 };
    ct_leader_command(l, w);
}

void ct_leader_player_ids(ct_leader_t *l) { cmd7(l, 0x7700, 2, 0x0001); }
void ct_leader_request(ct_leader_t *l, uint16_t type) { cmd7(l, 0xa100, type, 0); }
void ct_leader_standby(ct_leader_t *l, uint16_t count) { cmd7(l, 0x6600, count, 0); cmd7(l, 0x6600, count, 0); }
void ct_leader_close_link(ct_leader_t *l, uint16_t count) { cmd7(l, 0x5f00, count, 0); }

static void echo_push(ct_leader_t *l, const uint16_t w[7])
{
    if (l->echo_n >= CT_LEADER_ECHOES) { l->echo_drops++; return; }
    memcpy(l->echoes[(l->echo_head + l->echo_n) % CT_LEADER_ECHOES], w, 14);
    l->echo_n++;
}

/* The child closes once its own READY_CLOSE_LINK comes back, which it sends only once: echoed for it. */
void ct_leader_echo_close_link(ct_leader_t *l, uint16_t count)
{
    uint16_t w[7] = { 0x5f00, count, 0, 0, 0, 0, 0 };
    echo_push(l, w);
}

/* The leader's block: its INIT on four frames, then each fragment on `repeat` frames in a row, never resent. */
unsigned ct_leader_send_block(ct_leader_t *l, const uint8_t *data, size_t len, unsigned repeat)
{
    unsigned count = len ? (unsigned)((len + CT_FRAG_BYTES - 1) / CT_FRAG_BYTES) : 1;
    for (unsigned i = 0; i < 4; i++) cmd7(l, 0x8800, (uint16_t)count, 0x80);
    for (unsigned i = 0; i < count; i++) {
        uint16_t w[7] = { (uint16_t)(0x8900 | i) };
        for (unsigned k = 0; k < 6; k++) {
            size_t at = i * CT_FRAG_BYTES + k * 2;
            w[1 + k] = (uint16_t)((at < len ? data[at] : 0) | ((at + 1 < len ? data[at + 1] : 0) << 8));
        }
        for (unsigned r = 0; r < repeat; r++) ct_leader_command(l, w);
    }
    return count;
}

/* ---- the Switch's blocks: INIT, then fragments (repeated or resent until our echoes show them all) */

static void child_block(ct_leader_t *l, const uint16_t *w)
{
    ct_leader_recv_t *r = &l->recv;
    if ((w[0] & 0xff00) == 0x8800) {
        unsigned count = w[1];
        if (count < 1 || count > CT_MAX_FRAGS) return;
        if (!r->valid || r->done || r->count != count) {
            memset(r, 0, sizeof *r);
            r->valid = true;
            r->count = (uint8_t)count;
        }
        return;
    }
    if (!r->valid || r->done) return;
    unsigned index = w[0] & 0x1f;
    if (index >= r->count) return;
    for (unsigned k = 0; k < 6; k++) {
        r->data[index * CT_FRAG_BYTES + k * 2] = (uint8_t)w[1 + k];
        r->data[index * CT_FRAG_BYTES + k * 2 + 1] = (uint8_t)(w[1 + k] >> 8);
    }
    r->flags |= 1u << index;
    if (r->flags == ((1u << r->count) - 1)) {
        r->done = true;
        if (l->hooks.on_block) l->hooks.on_block(l->hooks.ctx, r->count, r->data);
    }
}

/* ---- frames */

static void broadcast(ct_leader_t *l)
{
    uint8_t f[36];
    ct_rfu1_frame(f, CT_RFU1_BROADCAST, l->devid | (l->state == CT_LEADER_OPEN ? 0 : 0xffu << 16), sizeof f);
    for (unsigned i = 0; i < 6; i++)
        for (unsigned k = 0; k < 4; k++) f[12 + i * 4 + k] = l->beacon[i * 4 + 3 - k];
    l->send(l->send_ctx, f, sizeof f);
}

void ct_leader_open(ct_leader_t *l, const uint8_t beacon[24])
{
    memcpy(l->beacon, beacon, 24);
    l->have_beacon = true;
    if (l->state == CT_LEADER_IDLE || l->state == CT_LEADER_CLOSED) l->state = CT_LEADER_OPEN;
    broadcast(l);
}

void ct_leader_close(ct_leader_t *l)
{
    if (l->child_devid) send_cmd(l, CT_RFU1_DISCONNECT, l->child_devid);
    l->state = CT_LEADER_CLOSED;
    l->child_devid = 0;
}

void ct_leader_child_game_data(const ct_leader_t *l, uint8_t out[26]) { memcpy(out, l->child_name, 26); }

static void cf_push(ct_leader_t *l, const uint8_t *p, size_t len)
{
    if (l->cf_used + 1 + len > CT_LEADER_CHILD_BUF) { l->child_drops++; return; }
    unsigned at = (l->cf_head + l->cf_used) % CT_LEADER_CHILD_BUF;
    l->cf_buf[at] = (uint8_t)len;
    for (size_t i = 0; i < len; i++) l->cf_buf[(at + 1 + i) % CT_LEADER_CHILD_BUF] = p[i];
    l->cf_used = (uint16_t)(l->cf_used + 1 + len);
}

static size_t cf_pop(ct_leader_t *l, uint8_t *out)
{
    if (!l->cf_used) return 0;
    size_t len = l->cf_buf[l->cf_head];
    for (size_t i = 0; i < len; i++) out[i] = l->cf_buf[(l->cf_head + 1 + i) % CT_LEADER_CHILD_BUF];
    l->cf_head = (uint16_t)((l->cf_head + 1 + len) % CT_LEADER_CHILD_BUF);
    l->cf_used = (uint16_t)(l->cf_used - 1 - len);
    return len;
}

void ct_leader_board_frame(ct_leader_t *l, const uint8_t *frame, size_t len)
{
    if (len < 12) return;
    uint32_t type = ct_be32(frame + 4), header = ct_be32(frame + 8);
    if (type == CT_RFU1_CONNECT_REQ) {
        if (l->state != CT_LEADER_OPEN || (header & 0xffff) != l->devid) { send_cmd(l, CT_RFU1_CONNECT_NACK, 0); return; }
        l->child_devid = random_devid(l->rand32, l->rand_ctx);
        l->state = CT_LEADER_NAMING;
        reset_child(l);
        send_cmd(l, CT_RFU1_CONNECT_ACK, l->child_devid);
        broadcast(l);
        return;
    }
    if (type == CT_RFU1_DISCONNECT) {
        if (!l->child_devid) return;
        l->child_devid = 0;
        l->state = CT_LEADER_CLOSED;
        if (l->hooks.on_closed) l->hooks.on_closed(l->hooks.ctx);
        return;
    }
    if (type == CT_RFU1_CLIENT_SEND && l->child_devid) {
        size_t length = frame[8] < CT_RFU1_SEND_DATA ? frame[8] : CT_RFU1_SEND_DATA;
        if (length > len - 12) length = len - 12;
        if (length) cf_push(l, frame + 12, length);
    }
}

static void child_ni(ct_leader_t *l, unsigned h, const uint8_t *payload, size_t size)
{
    unsigned state = (h >> 10) & 15, ack = (h >> 9) & 1, n = (h >> 7) & 3, phase = (h >> 5) & 3;
    if (ack) {
        /* The child acking a step of the join answer. */
        if (l->state == CT_LEADER_ANSWERING && l->join_step < 4 && JOIN_OK[l->join_step].state == state && JOIN_OK[l->join_step].n == n) {
            if (++l->join_step == 4) l->idle_null = 2;
        }
        return;
    }
    if (l->state != CT_LEADER_NAMING) return;
    if (state == ST_NI_START || state == ST_NI || state == ST_NI_END) {
        if (state == ST_NI && n == ((l->recv_n[phase] + 1u) & 3)) {
            l->recv_n[phase] = (uint8_t)n;
            /* childName.push, assembled in send order as childGameData() would */
            if (l->child_name_at < 26) {
                size_t take = size < (size_t)(26 - l->child_name_at) ? size : (size_t)(26 - l->child_name_at);
                memcpy(l->child_name + l->child_name_at, payload, take);
            }
            if (l->child_name_at < 26) l->child_name_at = (uint16_t)(l->child_name_at + size);
        }
        parent_frame(l->ni_ack, state, state == ST_NI_END ? 0 : n, phase, NULL, 0, true);
        l->have_ni_ack = true;
        if (state == ST_NI_END) l->name_done = true;
    } else if (state == ST_NULL && l->name_done) {
        /* The child finished its name send: answer the join. */
        l->state = CT_LEADER_ANSWERING;
        l->have_ni_ack = false;
    }
}

/* Each echo of a block's last fragment makes the child resend every fragment still missing, so the last fragment goes
 * back when it first arrives and when the block is whole, then only after a pause; INIT once, then after a pause. */
static bool echo_wanted(ct_leader_t *l, const uint16_t *w)
{
    ct_leader_recv_t *r = &l->recv;
    unsigned op = w[0] & 0xff00;
    if (!r->valid || (op == 0x8800 && w[1] != r->count)) return true;
    if (op == 0x8800) {
        if (r->init_echo_set && l->ticks - r->init_echo < ECHO_RETRY) return false;
        r->init_echo = l->ticks;
        r->init_echo_set = true;
        return true;
    }
    if (op != 0x8900 || (w[0] & 0x1f) != (unsigned)(r->count - 1)) return true;
    uint32_t wait = r->done ? (r->whole_echoed ? ECHO_RETRY : 0) : ECHO_RETRY_MISSING;
    if (r->last_echo_set && l->ticks - r->last_echo < wait) return false;
    r->last_echo = l->ticks;
    r->last_echo_set = true;
    if (r->done) r->whole_echoed = true;
    return true;
}

static void child_uni(ct_leader_t *l, const uint8_t *slot, size_t len)
{
    if (l->state == CT_LEADER_ANSWERING || l->state == CT_LEADER_NAMING) {
        l->state = CT_LEADER_LINKED;
        if (l->hooks.on_joined) l->hooks.on_joined(l->hooks.ctx, l->child_name);
    }
    if (len < CT_SLOT_BYTES || slot[1] == 0) { l->child_quiet_at = l->ticks; return; }
    l->child_busy_at = l->ticks;
    unsigned seq = slot[0] >> 5;
    uint16_t w[7];
    for (unsigned i = 0; i < 7; i++) w[i] = ct_le16(slot + i * 2);
    /* The child resends block fragments untagged; they sit outside the sequence. */
    bool untagged = seq == 0 && (w[0] & 0xff00) == 0x8900 && l->child_seq != 0xff && l->child_seq != 7;
    if (!untagged) {
        if (l->child_seq != 0xff && seq != ((l->child_seq + 1u) & 7)) l->out_of_order++;
        l->child_seq = (uint8_t)seq;
    }
    w[0] &= 0xff1f;
    unsigned op = w[0] & 0xff00;
    if (op == 0x8800 || op == 0x8900) child_block(l, w);
    if (echo_wanted(l, w)) {
        const uint16_t *waiting = l->echo_n ? l->echoes[(l->echo_head + l->echo_n - 1) % CT_LEADER_ECHOES] : NULL;
        if (!waiting || memcmp(waiting, w, 14) != 0) echo_push(l, w);
    }
    if (op != 0x8800 && op != 0x8900 && l->hooks.on_command) l->hooks.on_command(l->hooks.ctx, w);
}

/* A child packet may hold several subframes back to back. */
static void from_child(ct_leader_t *l, const uint8_t *data, size_t len)
{
    for (size_t o = 0; o + 2 <= len;) {
        unsigned h = ct_le16(data + o), size = h & 31;
        if (o + 2 + size > len) break;
        if (((h >> 10) & 15) == ST_UNI) child_uni(l, data + o + 2, size);
        else child_ni(l, h, data + o + 2, size);
        o += 2 + size;
        if (h == 0) break;
    }
}

/* Queued commands first; otherwise held keys every frame while the game sends them. */
static bool next_own(ct_leader_t *l, uint16_t w[7])
{
    if (l->own_n) {
        memcpy(w, l->own[l->own_head], 14);
        l->own_head = (uint8_t)((l->own_head + 1) % CT_LEADER_OWN);
        l->own_n--;
        return true;
    }
    uint8_t code;
    if (!l->hooks.key_source || !l->hooks.key_source(l->hooks.ctx, &code)) return false;
    memset(w, 0, 14);
    w[0] = 0xbe00;
    w[1] = (uint16_t)(code | (l->key_count << 8));
    l->key_count++;
    return true;
}

/* An empty slot 1 reads as index 0, the echo a child waiting on a one-fragment block resends on: it holds a word with
 * no command or index until the block is through. */
static bool take_echo(ct_leader_t *l, uint16_t w[7])
{
    if (l->echo_n) {
        memcpy(w, l->echoes[l->echo_head], 14);
        l->echo_head = (uint8_t)((l->echo_head + 1) % CT_LEADER_ECHOES);
        l->echo_n--;
        return true;
    }
    const ct_leader_recv_t *r = &l->recv;
    if (r->valid && r->count == 1 && !(r->whole_echoed && l->ticks - r->last_echo >= ECHO_RETRY)) {
        memset(w, 0, 14);
        w[0] = 0x00ff;
        return true;
    }
    return false;
}

static size_t next_frame(ct_leader_t *l, uint8_t *out)
{
    if (l->state == CT_LEADER_NAMING) {
        if (l->have_ni_ack) { memcpy(out, l->ni_ack, 3); return 3; }
        memset(out, 0, 3);
        return 3;
    }
    if (l->state == CT_LEADER_ANSWERING) {
        if (l->join_step < 4) return parent_frame(out, JOIN_OK[l->join_step].state, JOIN_OK[l->join_step].n, 0,
                                                  JOIN_OK[l->join_step].payload, JOIN_OK[l->join_step].len, false);
        if (l->idle_null > 0) { l->idle_null--; return parent_frame(out, ST_NULL, 1, 0, NULL, 0, false); }
    }
    uint16_t own[7], echo[7];
    bool have_own = next_own(l, own);
    bool have_echo = take_echo(l, echo);
    uint8_t zero[UNI_PAYLOAD] = { 0 };
    parent_frame(out, ST_UNI, 0, 0, zero, UNI_PAYLOAD, false);
    if (have_own) for (unsigned i = 0; i < 7; i++) ct_put16(out + 3 + i * 2, own[i]);
    if (have_echo) for (unsigned i = 0; i < 7; i++) ct_put16(out + 3 + CT_SLOT_BYTES + i * 2, echo[i]);
    return 3 + UNI_PAYLOAD;
}

void ct_leader_tick(ct_leader_t *l)
{
    l->ticks++;
    if (l->hooks.on_tick) l->hooks.on_tick(l->hooks.ctx);
    if (l->state == CT_LEADER_OPEN && l->ticks % BEACON_TICKS == 0) broadcast(l);
    if (!l->child_devid) return;
    uint8_t child[CT_RFU1_SEND_DATA];
    size_t n = cf_pop(l, child);
    if (n) from_child(l, child, n);
    uint8_t payload[3 + UNI_PAYLOAD];
    size_t plen = next_frame(l, payload);
    uint8_t f[CT_RFU1_FRAME_MAX];
    ct_rfu1_frame(f, CT_RFU1_HOST_SEND, plen & 0x7f, sizeof f);
    memcpy(f + 12, payload, plen);
    l->send(l->send_ctx, f, sizeof f);
}
