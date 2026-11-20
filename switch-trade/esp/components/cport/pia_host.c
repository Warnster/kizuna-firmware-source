/* SPDX-License-Identifier: AGPL-3.0-only
 * Derived from pokeldn (https://github.com/Decryptu/pokeldn), Copyright (c) the pokeldn authors.
 * Changes Copyright (c) 2026 Kizuna. See NOTICE.
 */
/* pia_host.c — portable C port of pokeldn frlg/link/host_session.py (HostSession) and
 * ldn/host_pia.py (HostPeerProtocol, build_messages, decode_datagram, reliable_output_batches).
 * See pia_host.h. */
#include "pia_host.h"
#include "rfu.h"
#include "reliable.h"
#include "pia_zstd.h"
#include <string.h>

/* host_pia.py timers, in seconds exactly as the Python holds them */
#define NET_RETRY_S     0.5
#define ACCEPT_RETRY_S  0.25
#define RTT_PERIOD_S    0.315
#define QUIET_GATE_S    0.25

static uint16_t rd16(const uint8_t *p) { return (uint16_t)((p[0] << 8) | p[1]); }

/* ================= HostSession ================= */

void pia_hsess_init(pia_hsess *s, const pia_activity *activity, const uint8_t host_session_id[2], const relh_cfg *rel_cfg) {
    relh_cfg c = rel_cfg ? *rel_cfg : RELH_DEFAULT_CFG;
    memset(s, 0, sizeof(*s));
    c.retransmit_limit = PIA_HOST_RTX_LIMIT;
    relh_init(&s->rel, &c);
    rfu_leader_init(&s->rfu, host_session_id, 1);
    s->act = *activity;
}

int pia_hsess_established(const pia_hsess *s) { return s->act.established(s->act.ctx); }

typedef struct { pia_hsess *s; int events; } hs_rx;

static void hs_deliver(void *ctx, uint16_t seq, uint8_t flagsA, const uint8_t *p, int len) {
    hs_rx *d = (hs_rx *)ctx;
    pia_hsess *s = d->s;
    (void)flagsA;
    rful_event ev = rfu_leader_receive(&s->rfu, p, (size_t)len);
    if (ev == RFUL_EV_NONE) return;
    d->events++;
    s->events[ev]++;
    if (ev == RFUL_EV_CONNECT && !s->has_connect_seq) { s->has_connect_seq = 1; s->connect_seq = seq; }
    if (ev == RFUL_EV_UNI) s->act.feed_child_slot(s->act.ctx, s->rfu.child_cmd);
}

int pia_hsess_receive(pia_hsess *s, const uint8_t *wire, int len, double now_ms) {
    hs_rx d = { s, 0 };
    if (s->stopped) return 0;
    relh_receive(&s->rel, wire, len, now_ms, hs_deliver, &d);
    return d.events;
}

void pia_hsess_note_rtt(pia_hsess *s, double rtt_ms) { relh_note_rtt(&s->rel, rtt_ms); }

void pia_hsess_on_ldn_leave(pia_hsess *s) {
    s->stopped = 1;
    rfu_leader_on_ldn_leave(&s->rfu);
}

int pia_hsess_tick(pia_hsess *s, double now_ms, relh_emission *out, int max_out) {
    if (s->stopped) return 0;
    int n = relh_poll(&s->rel, now_ms, out, max_out - 1);   /* keep room for this tick's frame */
    for (int i = 0; i < n; i++) {
        uint16_t ack_id; uint8_t mask[16];
        if (out[i].flagsA != RELH_FLAGSA_CTRL) continue;
        s->peer_open_ack_sent = 1;
        /* Native opens A only once the cumulative ack (wrapping) covers C itself. */
        if (s->has_connect_seq && rel_parse_bulk_ack(out[i].payload, out[i].len, &ack_id, mask) == 0
            && (uint16_t)(ack_id - (uint16_t)(s->connect_seq + 1)) < 0x8000)
            s->connect_ack_sent = 1;
    }

    if (relh_inflight(&s->rel) >= s->rel.cfg.max_inflight) {
        s->send_window_full = 1;              /* pause the activity until the window drains */
        s->window_full_ticks++;
        return n;
    }
    if (s->send_window_full) { s->send_window_full = 0; s->window_full_ticks = 0; }

    /* Hole guard: a closed hole releases every held frame into the console's 8-deep RFU queue.
     * Never gate the close/disconnect path. */
    if (!(s->close_poll_sent && s->act.disconnect_requested(s->act.ctx))
        && relh_outstanding(&s->rel) >= PIA_HOST_OUTSTANDING_MAX) {
        s->console_backlogged = 1;
        s->backlog_ticks++;
        return n;
    }
    if (s->console_backlogged) { s->console_backlogged = 0; s->backlog_ticks = 0; }

    if (s->close_poll_sent && s->act.disconnect_requested(s->act.ctx) && !s->disconnect_sent) {
        uint8_t d[8];
        size_t dl = rfu_leader_disconnect_frame(&s->rfu, d);
        if (dl && relh_send(&s->rel, d, (int)dl, now_ms, RELH_FLAGSA_GBA, &out[n]) == 1) n++;
        s->disconnect_sent = 1;
        s->act.mark_disconnect_sent(s->act.ctx);
        return n;
    }

    uint16_t words[7]; uint8_t row[RFU_SLOT_LEN];
    int have_words = s->rfu.state == RFUL_UNI;
    if (have_words) { s->act.tick(s->act.ctx, words); rfu_words_to_slot(words, row); }
    /* Native leader ordering: cumulatively ack C, not merely the INIT before it, before opening A. */
    if (!s->rel.local_opened && !s->connect_ack_sent) return n;
    uint8_t inner[RFUL_FRAME_MAX];
    size_t il = rfu_leader_tick(&s->rfu, have_words ? row : NULL, inner);
    if (!il) return n;
    int ok = s->rel.local_opened ? relh_send(&s->rel, inner, (int)il, now_ms, RELH_FLAGSA_GBA, &out[n]) == 1
                                 : relh_open(&s->rel, inner, (int)il, now_ms, RELH_FLAGSA_GBA, &out[n]) == 1;
    if (ok) n++;
    if (have_words && (words[0] & RFU_RFUCMD_MASK) == RFU_READY_CLOSE_LINK) s->close_poll_sent = 1;
    return n;
}

/* ================= HostPeerProtocol ================= */

void pia_host_cfg_trade_defaults(pia_host_cfg *cfg) {
    cfg->scene_id = 22287;
    cfg->max_participants = 6;
    cfg->response_first = 1;
    cfg->protocol_tick_ms = 0;
}

int pia_host_zstd_stored(const uint8_t *in, int len, uint8_t *out, int out_cap) {
    /* magic, FHD 00 (no dictionary, size or checksum), window descriptor 0x18 (8 KiB, the console's
     * canonical form), then one last raw block: header (size << 3) | last. */
    static const uint8_t prefix[6] = { 0x28, 0xb5, 0x2f, 0xfd, 0x00, 0x18 };
    if (len < 0 || len > 8192 || out_cap < len + 9) return -1;
    uint32_t b = ((uint32_t)len << 3) | 1u;
    memmove(out + 9, in, (size_t)len);           /* in may alias out + 9 */
    memcpy(out, prefix, 6);
    out[6] = (uint8_t)b; out[7] = (uint8_t)(b >> 8); out[8] = (uint8_t)(b >> 16);
    return len + 9;
}

static void emit_event(pia_host_t *h, pia_host_event ev) { if (h->cfg.event) h->cfg.event(h->cfg.ctx, ev); }

/* uint32 ms -> pokeldn's float seconds, extended past the 49-day wrap, never backwards. */
static double clock_s(pia_host_t *h, uint32_t now_ms) {
    if (!h->time_init) { h->time_init = 1; h->now_ms64 = now_ms; h->last_ms = now_ms; }
    else {
        int32_t d = (int32_t)(now_ms - h->last_ms);
        if (d > 0) { h->now_ms64 += (uint32_t)d; h->last_ms = now_ms; }
    }
    return (double)h->now_ms64 / 1000.0;
}

typedef struct { uint8_t proto; const uint8_t *payload; int len; int msgflags; } ph_msg;

/* build_messages: tile, zstd (stored form), footer, 0xff pad, encrypt with our IP. Leaves the
 * datagram in scratch_b and returns its length (0 if it would not fit). */
static int build_messages(pia_host_t *h, const ph_msg *m, int n, uint16_t dst, uint16_t src, uint16_t pktid,
                          int compress, int auto_compress, int establishing, int footer, uint16_t footer_var,
                          uint64_t nonce) {
    uint8_t *buf = h->scratch_a, *msgs = h->scratch_a + 9;
    int blen = 0, room = PIA_HOST_TX_MAX - PIA_CT_OFF - 9 - 2 - 15;   /* datagram fits scratch_b */
    for (int i = 0; i < n; i++) {
        if (blen + 5 + m[i].len > room) return 0;
        blen += rel_build_message(m[i].proto, m[i].payload, m[i].len, m[i].msgflags, msgs + blen);
    }
    int z = compress || (auto_compress && blen >= PIA_HOST_COMPRESS_MIN);
    uint8_t *pt = msgs;
    if (z) { blen = pia_host_zstd_stored(msgs, blen, buf, PIA_HOST_TX_MAX); pt = buf; }
    if (footer) { pt[blen++] = (uint8_t)(footer_var >> 8); pt[blen++] = (uint8_t)footer_var; }
    int pad = (16 - (blen % 16)) % 16;
    memset(pt + blen, 0xFF, (size_t)pad); blen += pad;
    pia_header hd;
    hd.dst = dst; hd.src = src; hd.pktid = pktid; hd.enc = 0x90;
    hd.flags = (uint8_t)((pad << 4) | (z ? 1 : 0) | (establishing ? 2 : 0));
    hd.footer = (uint8_t)(footer ? 2 : 0);
    for (int i = 0; i < 8; i++) hd.nonce8[i] = (uint8_t)(nonce >> (56 - 8 * i));
    return (int)pia_encrypt(&h->crypto, pt, (size_t)blen, h->cfg.our_ip, &hd, h->scratch_b);
}

static uint64_t take_nonce(pia_host_t *h) { return h->nonce++; }   /* native: big-endian counter */

static void send_dg(pia_host_t *h, int len, const uint8_t ip[4]) {
    if (len <= 0) return;
    h->cfg.send(h->cfg.ctx, h->scratch_b, (size_t)len, ip);
    h->tx_datagrams++;
}

/* build_message: one message, no message flags, on the native nonce sequence. */
static int build_one(pia_host_t *h, uint8_t proto, const uint8_t *p, int len, uint16_t dst, uint16_t src,
                     uint16_t pktid, int compress, int establishing, int footer, uint16_t footer_var) {
    ph_msg m = { proto, p, len, -1 };
    return build_messages(h, &m, 1, dst, src, pktid, compress, 0, establishing, footer, footer_var, take_nonce(h));
}

static uint16_t bump(uint16_t id) { return id < 0xFFFF ? (uint16_t)(id + 1) : 1; }

void pia_host_init(pia_host_t *h, const pia_host_cfg *cfg, const pia_activity *activity) {
    uint8_t seed[8];
    memset(h, 0, sizeof(*h));
    h->cfg = *cfg;
    if (cfg->session_name) {
        size_t n = strlen(cfg->session_name);
        if (n > PC_PLAYER_NAME_MAX) n = PC_PLAYER_NAME_MAX;
        memcpy(h->session_name, cfg->session_name, n);
    }
    h->cfg.session_name = h->session_name;
    if (h->cfg.max_participants < 2 || h->cfg.max_participants > PIA_HOST_MAX_PARTICIPANTS + 1)
        h->cfg.max_participants = h->cfg.max_participants < 2 ? 6 : PIA_HOST_MAX_PARTICIPANTS + 1;  /* LDN: 2..8 */
    if (h->cfg.active_app_data_len > PIA_HOST_APP_DATA_MAX) h->cfg.active_app_data_len = PIA_HOST_APP_DATA_MAX;
    if (h->cfg.active_app_data_len > 0) memcpy(h->app_data, cfg->active_app_data, (size_t)h->cfg.active_app_data_len);
    h->cfg.active_app_data = h->app_data;
    pia_hsess_init(&h->sess, activity, cfg->host_session_id, NULL);
    pia_crypto_init(&h->crypto, cfg->ssid, FRLG_GAME_KEY);
    cfg->rng(cfg->ctx, seed, 8);                     /* PiaNonceSequence(native=True): urandom(8) BE */
    for (int i = 0; i < 8; i++) h->nonce = (h->nonce << 8) | seed[i];
    h->net_sequence = 2;
    h->session_packet_id = 2;
    h->reliable_packet_id = 2;
    h->rtt_systime = 0x10000;
}

void pia_host_set_participants(pia_host_t *h, const uint8_t (*ips)[4], int n) {
    if (n > PIA_HOST_MAX_PARTICIPANTS) n = PIA_HOST_MAX_PARTICIPANTS;
    if (n < 0) n = 0;
    for (int i = 0; i < n; i++) memcpy(h->participants[i], ips[i], 4);
    h->nparticipants = n;
}

void pia_host_on_participant_joined(pia_host_t *h) {
    if (h->joined) return;
    h->joined = 1;
    h->next_net_send = 0.0;
}

void pia_host_on_ldn_leave(pia_host_t *h) { pia_hsess_on_ldn_leave(&h->sess); }

/* ---- Session ---- */

static void unicast_session_update(pia_host_t *h, int len) {
    /* The console receives ~1 in 5 broadcast data frames and needs the type 5 before its type 6. */
    uint8_t seen[PIA_HOST_MAX_PARTICIPANTS + 1][4]; int ns = 0;
    for (int i = 0; i <= h->nparticipants; i++) {
        const uint8_t *ip = i < h->nparticipants ? h->participants[i] : (h->has_join ? h->join.ip : NULL);
        int dup = 0;
        if (!ip || !memcmp(ip, h->cfg.broadcast_ip, 4)) continue;
        for (int k = 0; k < ns; k++) if (!memcmp(seen[k], ip, 4)) dup = 1;
        if (dup) continue;
        memcpy(seen[ns++], ip, 4);
        send_dg(h, len, ip);
    }
}

/* build_session_acceptance + _send_session_acceptance. */
static void send_session_acceptance(pia_host_t *h) {
    uint8_t constant8[8], rnd[4], resp[37];
    uint16_t host_var = h->join.destination_var, guest_var = h->join.source_var;
    uint8_t *upd = h->scratch_c;
    pc_ldn_constant_id(h->cfg.our_mac, constant8);
    int ul = pc_build_session_update(&h->join, constant8, host_var, h->cfg.our_ip,
                                     (const uint8_t *)h->session_name, (int)strlen(h->session_name),
                                     NULL, 1, NULL, 0, upd);
    if (ul < 0) return;
    h->cfg.rng(h->cfg.ctx, rnd, 4);
    pc_build_session_join_response(&h->join, constant8, host_var, rnd, resp);
    /* Native allocates the response nonce first but transmits the update first. */
    uint64_t n_resp = take_nonce(h), n_upd = take_nonce(h);
    ph_msg mr = { PC_PROTO_SESSION, resp, 37, -1 }, mu = { PC_PROTO_SESSION, upd, ul, -1 };
    int len;
    if (h->cfg.response_first) {
        len = build_messages(h, &mr, 1, guest_var, host_var, 1, 0, 0, 0, 1, guest_var, n_resp);
        send_dg(h, len, h->join.ip);
    }
    len = build_messages(h, &mu, 1, PC_SESSION_VAR, host_var, 1, 1, 0, 0, 1, guest_var, n_upd);
    send_dg(h, len, h->cfg.broadcast_ip);
    unicast_session_update(h, len);
    if (!h->cfg.response_first) {
        len = build_messages(h, &mr, 1, guest_var, host_var, 1, 0, 0, 0, 1, guest_var, n_resp);
        send_dg(h, len, h->join.ip);
    }
}

static void answer_leave_request(pia_host_t *h, const pia_header *hd, const uint8_t *p, int n,
                                 const uint8_t src_ip[4]) {
    uint8_t rnd[4], resp[15];
    /* Unanswered, the leaver resends every 500 ms and leaves LDN after the fourth. */
    if (!h->has_guest || hd->src != h->guest_var || n < 15 || rd16(p + 13) != h->guest_var) return;
    h->cfg.rng(h->cfg.ctx, rnd, 4);
    if (pc_build_session_leave_response(p, n, rnd, resp) < 0) return;
    h->leave_requests_in++;
    /* On the guest's unicast counter: a retail FireRed ignored type 4s numbered below it. */
    int len = build_one(h, PC_PROTO_SESSION, resp, 15, h->guest_var, PIA_HOST_VAR, h->reliable_packet_id,
                        0, 0, 1, h->guest_var);
    h->reliable_packet_id = bump(h->reliable_packet_id);
    send_dg(h, len, src_ip);
    if (h->leave_requests_in == 1) emit_event(h, PIA_HOST_EV_LEAVE);
}

static void receive_session(pia_host_t *h, const pia_header *hd, const uint8_t *p, int n,
                            const uint8_t src_ip[4], double now) {
    if (n < 1) return;
    if (p[0] == PC_SESSION_JOIN_REQUEST) {
        uint8_t expected[8];
        pc_session_join_t *j = &h->join, *parsed = &h->join_rx;
        if (pc_parse_session_join(p, n, parsed) != 0) return;       /* malformed: identity untouched */
        pc_ldn_constant_id(h->cfg.our_mac, expected);
        if (memcmp(parsed->destination_constant_id, expected, 8) || parsed->destination_var != PIA_HOST_VAR
            || parsed->source_var != hd->src || memcmp(parsed->ip, src_ip, 4))
            return;                                                  /* identity mismatch */
        if (!h->session_join_seen) { h->session_join_seen = 1; emit_event(h, PIA_HOST_EV_JOIN); }
        *j = *parsed;
        h->has_join = 1;
        h->has_guest = 1; h->guest_var = j->source_var; memcpy(h->guest_ip, src_ip, 4);
        if (!h->session_finalized) {
            send_session_acceptance(h);
            h->session_accepts++;
            h->has_next_accept = 1; h->next_accept = now + ACCEPT_RETRY_S;
        }
    } else if (p[0] == PC_SESSION_UPDATE_ACK && !h->session_finalized) {
        h->session_finalized = 1;
        h->has_next_accept = 0;
        h->has_next_tick = 1; h->next_protocol_tick = now;
        emit_event(h, PIA_HOST_EV_FINALIZED);
    } else if (p[0] == PC_SESSION_LEAVE_REQUEST) {
        answer_leave_request(h, hd, p, n, src_ip);
    }
}

/* ---- RTT ---- */

static void receive_rtt(pia_host_t *h, const uint8_t *p, int n, const uint8_t src_ip[4], double now) {
    pc_rtt_t r;
    if (pc_parse_rtt(p, n, &r) != 0 || !h->session_finalized || !h->has_guest) return;
    if (r.type == 0) {
        uint8_t resp[21];
        h->rtt_requests_in++;
        h->rtt_template_len = n < 21 ? n : 21;
        memcpy(h->rtt_template, p, (size_t)h->rtt_template_len);
        h->has_rtt_template = 1;
        pc_build_rtt_response(p, n, resp);
        int len = build_one(h, PC_PROTO_RTT, resp, 21, PC_SESSION_VAR, PIA_HOST_VAR, h->session_packet_id,
                            0, 0, 1, h->guest_var);
        h->session_packet_id = (uint16_t)(h->session_packet_id + 1) ? (uint16_t)(h->session_packet_id + 1) : 1;
        send_dg(h, len, src_ip);
        h->rtt_responses_out++;
        if (!h->has_next_rtt) { h->has_next_rtt = 1; h->next_rtt_send = 0.0; }
    } else if (r.type == 1) {
        uint64_t systime = 0;
        h->rtt_responses_in++;
        for (int i = 0; i < 8; i++) systime |= (uint64_t)r.systime[i] << (8 * i);   /* little-endian */
        for (int i = 0; i < PIA_HOST_RTT_PENDING; i++)
            if (h->rtt_pending[i].used && h->rtt_pending[i].systime == systime) {
                double ms = (now - h->rtt_pending[i].sent) * 1000.0;
                h->rtt_pending[i].used = 0;
                pia_hsess_note_rtt(&h->sess, ms > 0.0 ? ms : 0.0);
                break;
            }
    }
}

static void originate_rtt(pia_host_t *h, double now) {
    uint8_t req[21];
    int slot = 0;
    h->rtt_systime++;
    pc_build_rtt_request(h->rtt_template, h->rtt_template_len, h->rtt_systime, req);
    int len = build_one(h, PC_PROTO_RTT, req, 21, PC_SESSION_VAR, PIA_HOST_VAR, h->session_packet_id,
                        0, 0, 1, h->guest_var);
    h->session_packet_id = (uint16_t)(h->session_packet_id + 1) ? (uint16_t)(h->session_packet_id + 1) : 1;
    send_dg(h, len, h->guest_ip);
    for (int i = 0; i < PIA_HOST_RTT_PENDING; i++) {         /* a free slot, else the oldest */
        if (!h->rtt_pending[i].used) { slot = i; break; }
        if (h->rtt_pending[i].sent < h->rtt_pending[slot].sent) slot = i;
    }
    h->rtt_pending[slot].systime = h->rtt_systime;
    h->rtt_pending[slot].sent = now;
    h->rtt_pending[slot].used = 1;
    h->rtt_requests_out++;
}

/* ---- Reliable ---- */

/* _apply_carry_forward: unacked data frames of the last CARRY_DEPTH datagrams ride again, before
 * this tick's frames; pure acks never ride. Writes the final list to h->emit and returns its size. */
static int apply_carry_forward(pia_host_t *h, const relh_emission *outs, int n) {
    relh_emission *carried = h->emit;                /* carried first, this tick's frames after */
    uint16_t have[PIA_HOST_EMIT_MAX];
    int nh = 0, nc = 0;
    for (int i = 0; i < n; i++) have[nh++] = outs[i].seq;
    for (int d = 0; d < h->carry_n; d++) {
        int idx = (h->carry_head - 1 - d + 2 * PIA_HOST_CARRY_DEPTH) % PIA_HOST_CARRY_DEPTH;  /* newest first */
        for (int j = 0; j < h->carry_len[idx]; j++) {
            const pia_host_carry *it = &h->carry[idx][j];
            const relh_tx *e = relh_find(&h->sess.rel, it->seq);
            int in_have = 0;
            for (int k = 0; k < nh; k++) if (have[k] == it->seq) { in_have = 1; break; }
            if (in_have || !e || e->acked) continue;
            have[nh++] = it->seq;
            relh_emission *c = &carried[nc++];
            c->seq = it->seq; c->flagsA = it->flagsA; c->ack = it->ack; c->retransmitted = it->retransmitted;
            c->payload = e->inner; c->len = e->len;
        }
    }
    {   int idx = h->carry_head, w = 0;
        for (int i = 0; i < n && w < PIA_HSESS_EMIT_MAX; i++) if (outs[i].flagsA != RELH_FLAGSA_CTRL) {
            pia_host_carry *c = &h->carry[idx][w++];
            c->seq = outs[i].seq; c->flagsA = outs[i].flagsA; c->ack = outs[i].ack;
            c->retransmitted = outs[i].retransmitted;
        }
        h->carry_len[idx] = w;
        h->carry_head = (h->carry_head + 1) % PIA_HOST_CARRY_DEPTH;
        if (h->carry_n < PIA_HOST_CARRY_DEPTH) h->carry_n++;
    }
    if (nc) {
        uint16_t lo = have[0];
        for (int k = 1; k < nh; k++) if (have[k] < lo) lo = have[k];     /* min(have): numeric */
        for (int i = 1; i < nc; i++) {                                    /* stable */
            relh_emission v = carried[i]; int j = i - 1;
            while (j >= 0 && (uint16_t)(carried[j].seq - lo) > (uint16_t)(v.seq - lo)) { carried[j + 1] = carried[j]; j--; }
            carried[j + 1] = v;
        }
        h->carried_frames += nc;
    }
    memcpy(h->emit + nc, outs, (size_t)n * sizeof(relh_emission));
    return nc + n;
}

static void send_reliable_batch(pia_host_t *h, const relh_emission *items, int n) {
    ph_msg m[PIA_HOST_BATCH_MAX];
    int off = 0;
    for (int i = 0; i < n; i++) {
        int l = relh_serialize(&items[i], h->scratch_c + off);
        m[i].proto = PC_PROTO_RELIABLE; m[i].payload = h->scratch_c + off; m[i].len = l;
        m[i].msgflags = relh_message_flags(&items[i]);
        off += l;
    }
    int len = build_messages(h, m, n, h->guest_var, PIA_HOST_VAR, h->reliable_packet_id, 0, 1, 0, 1,
                             h->guest_var, take_nonce(h));
    send_dg(h, len, h->guest_ip);
    h->reliable_messages_out += n;
    h->reliable_packet_id = bump(h->reliable_packet_id);
}

static void send_reliable(pia_host_t *h, const relh_emission *outs, int n, double now) {
    int total = 0;
    if (n <= 0 || !h->has_guest) return;
    if (h->has_last_rx && (now - h->last_rx) > QUIET_GATE_S) {
        /* The console goes silent ~0.5 s into its save: no resends, no carry-forward into it. */
        for (int i = 0; i < n; i++) if (!outs[i].retransmitted) h->emit[total++] = outs[i];
        if (!total) return;
    } else {
        total = apply_carry_forward(h, outs, n);
    }
    /* reliable_output_batches: at most BATCH_MAX, and an ack or resend ends its batch so the
     * message flags cannot inherit. */
    int start = 0;
    for (int i = 0; i < total; i++) {
        if (i - start >= PIA_HOST_BATCH_MAX) { send_reliable_batch(h, h->emit + start, i - start); start = i; }
        if (relh_message_flags(&h->emit[i]) >= 0) { send_reliable_batch(h, h->emit + start, i + 1 - start); start = i + 1; }
    }
    if (start < total) send_reliable_batch(h, h->emit + start, total - start);
}

/* ---- inbound ---- */

/* decode_datagram: the padding must be 0xff, the zstd flag must decompress, the messages must
 * tile the whole plaintext. Returns the message count (msgs point into scratch), -1 on error. */
static int decode_datagram(pia_host_t *h, const uint8_t *dg, size_t len, const uint8_t src_ip[4],
                           pia_header *hd, rel_message_t *msgs, int max_msgs) {
    if (!pia_is_pia(dg, len) || len - PIA_CT_OFF > PIA_HOST_DG_MAX) return -1;
    pia_header_unpack(dg, hd);
    long n = pia_decrypt(&h->crypto, dg, len, src_ip, h->rx_pt);
    if (n < 0) return -1;
    int pad = hd->flags >> 4;
    if (pad) {
        if (n < pad) return -1;
        for (int i = 0; i < pad; i++) if (h->rx_pt[n - 1 - i] != 0xFF) return -1;
        n -= pad;
    }
    if (hd->footer) {
        if (n < hd->footer) return -1;
        n -= hd->footer;
    }
    const uint8_t *app = h->rx_pt;
    if (hd->flags & 1) {
        if (n < 4 || h->rx_pt[0] != 0x28 || h->rx_pt[1] != 0xb5 || h->rx_pt[2] != 0x2f || h->rx_pt[3] != 0xfd)
            return -1;
        n = pia_decompress(h->rx_pt, (size_t)n, h->rx_app, sizeof h->rx_app);
        if (n < 0) return -1;
        app = h->rx_app;
    }
    int consumed = 0;
    int count = rel_parse_messages(app, (int)n, msgs, max_msgs, &consumed);
    if (count < 1 || consumed != n) return -1;
    return count;
}

int pia_host_receive(pia_host_t *h, const uint8_t *dg, size_t len, const uint8_t src_ip[4], uint32_t now_ms) {
    double now = clock_s(h, now_ms);
    pia_header hd;
    const rel_message_t *msgs = h->rx_msgs;
    h->has_last_rx = 1; h->last_rx = now;
    if (!h->joined) return -1;                       /* before the Pia probe was initialised */
    int nm = decode_datagram(h, dg, len, src_ip, &hd, h->rx_msgs, PIA_HOST_RX_MSGS);
    if (nm < 0) { h->rx_rejected++; return -1; }
    h->rx_datagrams++;
    for (int i = 0; i < nm; i++) {
        const rel_message_t *m = &msgs[i];
        if (m->proto == PC_PROTO_NET) {
            uint8_t t0, t1; const uint8_t *body; int blen;
            if (pc_parse_net(m->payload, m->size, &t0, &t1, &body, &blen) != 0) continue;
            if (t1 == PC_NET_CONN_RESPONSE) {
                if (!h->net_acked) emit_event(h, PIA_HOST_EV_NET_ACKED);
                h->net_acked = 1;
            } else if (t1 == PC_NET_UPDATE_PROPERTY_ACK) {
                h->property_acks++;
            }
        } else if (m->proto == PC_PROTO_SESSION) {
            receive_session(h, &hd, m->payload, m->size, src_ip, now);
        } else if (m->proto == PC_PROTO_RTT) {
            receive_rtt(h, m->payload, m->size, src_ip, now);
        } else if (m->proto == PC_PROTO_RELIABLE) {
            if (!h->session_finalized) continue;     /* Reliable before Session finalization */
            h->reliable_messages_in++;
            pia_hsess_receive(&h->sess, m->payload, m->size, now * 1000.0);
        }
    }
    return nm;
}

/* ---- the timers ---- */

void pia_host_tick(pia_host_t *h, uint32_t now_ms) {
    double now = clock_s(h, now_ms);
    if (h->joined && !h->net_acked && now >= h->next_net_send) {
        uint8_t stations[PIA_HOST_MAX_PARTICIPANTS + 1][4];
        int ns = 1, max = h->cfg.max_participants;          /* <= 8: 30 + 22 * 8 bytes */
        memcpy(stations[0], h->cfg.our_ip, 4);
        for (int i = 0; i < h->nparticipants && ns < max; i++) memcpy(stations[ns++], h->participants[i], 4);
        int nl = pc_build_net_conn_request(h->net_sequence, PIA_HOST_VAR, h->cfg.our_mac, h->crypto.net_id,
                                           (const uint8_t (*)[4])stations, ns, max, h->scratch_c);
        if (nl > 0) {
            int len = build_one(h, PC_PROTO_NET, h->scratch_c, nl, 0, PIA_HOST_VAR, 0, 1, 1, 0, 0);
            send_dg(h, len, h->cfg.broadcast_ip);
            /* Unicast copies too: the console receives ~1 in 5 broadcast frames. */
            for (int i = 0; i < h->nparticipants; i++) send_dg(h, len, h->participants[i]);
        }
        h->next_net_send = now + NET_RETRY_S;
    }
    if (h->has_join && !h->session_finalized && h->has_next_accept && now >= h->next_accept) {
        send_session_acceptance(h);
        h->session_accepts++;
        h->next_accept = now + ACCEPT_RETRY_S;
    }
    if (h->session_finalized && h->has_rtt_template && h->has_guest && h->has_next_rtt && now >= h->next_rtt_send) {
        originate_rtt(h, now);
        h->next_rtt_send = now + RTT_PERIOD_S;
    }
    if (h->session_finalized && h->has_next_tick && now >= h->next_protocol_tick) {
        relh_emission out[PIA_HSESS_EMIT_MAX];
        int n = pia_hsess_tick(&h->sess, now * 1000.0, out, PIA_HSESS_EMIT_MAX);
        send_reliable(h, out, n, now);
        h->next_protocol_tick = now + (h->cfg.protocol_tick_ms > 0 ? h->cfg.protocol_tick_ms / 1000.0
                                                                   : 1.0 / 59.727);
    }
    if (pia_hsess_established(&h->sess) && !h->property_started) {
        h->property_started = 1;
        h->has_next_property = 1; h->next_property_send = 0.0;
        emit_event(h, PIA_HOST_EV_PROPERTY_STARTED);   /* network.set_application_data(active) */
    }
    if (h->property_started && h->has_next_property && now >= h->next_property_send) {
        int pl = pc_build_net_property_update(1, h->crypto.net_id, (uint16_t)(1 + h->nparticipants),
                                              h->cfg.max_participants, h->cfg.scene_id, 1,
                                              h->app_data, h->cfg.active_app_data_len, h->scratch_c);
        int len = build_one(h, PC_PROTO_NET, h->scratch_c, pl, 0, PIA_HOST_VAR, 0, 1, 1, 0, 0);
        if (h->has_guest) send_dg(h, len, h->guest_ip);
        h->next_property_send = now + NET_RETRY_S;
    }
}

uint32_t pia_host_next_deadline_ms(pia_host_t *h, uint32_t now_ms, uint32_t default_ms) {
    double now = clock_s(h, now_ms), best = default_ms / 1000.0, d[5];
    int n = 0;
    if (h->joined && !h->net_acked) d[n++] = h->next_net_send;
    if (h->has_join && !h->session_finalized && h->has_next_accept) d[n++] = h->next_accept;
    if (h->session_finalized && h->has_next_rtt) d[n++] = h->next_rtt_send;
    if (h->session_finalized && h->has_next_tick) d[n++] = h->next_protocol_tick;
    if (h->property_started && h->has_next_property) d[n++] = h->next_property_send;
    for (int i = 0; i < n; i++) {
        double wait = d[i] - now > 0 ? d[i] - now : 0.0;
        if (wait < best) best = wait;
    }
    uint32_t ms = (uint32_t)(best * 1000.0);
    return (double)ms < best * 1000.0 ? ms + 1 : ms;              /* round up: never wake early */
}
