/* SPDX-License-Identifier: AGPL-3.0-only
 * Derived from pokeldn (https://github.com/Decryptu/pokeldn), Copyright (c) the pokeldn authors.
 * Changes Copyright (c) 2026 Kizuna. See NOTICE.
 */
/* sim.c — C port of pokeldn/frlg/link/sim.py (joiner path, conn != NULL, pace_ms = 0).
 * See sim.h. Faithful to the Python; console-tuned constants live in sim.h. */
#include "sim.h"
#include "pia_zstd.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* Per-frame and heartbeat bench lines. Builds that define KZ_QUIET_TRACE (switch-online, the production online image)
 * compile them out; the arguments stay referenced so nothing becomes unused. */
#ifdef KZ_QUIET_TRACE
#define KZ_TRACE(...) do { if (0) printf(__VA_ARGS__); } while (0)
#else
#define KZ_TRACE(...) printf(__VA_ARGS__)
#endif

/* METADATA_FRAME from reliable.py: "4a002a005801004c656166477265656e5f65" + 00*28 (46 bytes). */
static const uint8_t METADATA_FRAME[46] = {
    0x4a,0x00,0x2a,0x00,0x58,0x01,0x00,0x4c,0x65,0x61,0x66,0x47,0x72,0x65,0x65,0x6e,0x5f,0x65,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };

/* ---- small sets ---- */
static int ts_seen(sim_t *s, uint32_t ts) {
    for (int i = 0; i < s->acked_ts_n; i++) if (s->acked_ts[i] == ts) return 1;
    return 0;
}
static void ts_add(sim_t *s, uint32_t ts) {
    if (s->acked_ts_n < SIM_TSRING) { s->acked_ts[s->acked_ts_n++] = ts; return; }
    memmove(s->acked_ts, s->acked_ts + 1, (SIM_TSRING - 1) * sizeof(uint32_t));
    s->acked_ts[SIM_TSRING - 1] = ts;
}
static int seen_in_has(sim_t *s, uint16_t seq) {
    for (int i = 0; i < s->seen_in_n; i++) if (s->seen_in[i] == seq) return 1;
    return 0;
}
static void note_in_seq(sim_t *s, uint16_t seq) {
    if (seen_in_has(s, seq)) return;
    if (s->seen_in_n < SIM_SEENRING) s->seen_in[s->seen_in_n++] = seq;
    else { memmove(s->seen_in, s->seen_in + 1, (SIM_SEENRING - 1) * 2); s->seen_in[SIM_SEENRING - 1] = seq; }
    if ((uint16_t)(seq - s->last_in_seq) < 0x8000) s->last_in_seq = seq;
}

static void next_nonce(sim_t *s, uint8_t out8[8]) {
    for (int i = 0; i < 8; i++) out8[i] = (uint8_t)(s->nonce >> (56 - 8 * i));
    s->nonce++;
    if (s->nonce == 0) s->nonce = 1;
}
static uint16_t next_pktid(sim_t *s, uint16_t dv) {
    for (int i = 0; i < SIM_PKTIDMAP; i++) {
        if (s->pktid_map[i].used && s->pktid_map[i].dst == dv) {
            uint16_t p = s->pktid_map[i].ctr;
            s->pktid_map[i].ctr = (p < 0xFFFF) ? (uint16_t)(p + 1) : 1;
            return p;
        }
    }
    for (int i = 0; i < SIM_PKTIDMAP; i++) {
        if (!s->pktid_map[i].used) {
            s->pktid_map[i].used = 1; s->pktid_map[i].dst = dv; s->pktid_map[i].ctr = 2;
            return 1;
        }
    }
    return 1;   /* map full (never happens: dsts are {0,1,host_var}) */
}

/* ---- framing: _send_messages (pace_ms = 0 path) ---- */
typedef struct { uint8_t proto; const uint8_t *payload; int len; int msgflags; } sim_msg_t;

static void send_messages(sim_t *s, const sim_msg_t *msgs, int nmsg,
                          int dst_var, int src_var, int compress, int footer,
                          int establishing, int unicast, int pktid, int footer_var) {
    if (nmsg <= 0) return;
    uint16_t dv = (dst_var >= 0) ? (uint16_t)dst_var : s->host_var;
    uint16_t sv = (src_var >= 0) ? (uint16_t)src_var : s->our_var;

    uint8_t body[SIM_MAXAPP + 64];
    int blen = 0;
    for (int i = 0; i < nmsg; i++) {
        blen += rel_build_message(msgs[i].proto, msgs[i].payload, msgs[i].len,
                                  msgs[i].msgflags, body + blen);
    }
    int do_zstd = compress || (blen >= SIM_COMPRESS_MIN);   /* HAVE_ZSTD always true here */
    if (do_zstd) {
        uint8_t z[SIM_MAXAPP + 64];
        long zl = pia_compress(body, (size_t)blen, z, sizeof(z));
        if (zl >= 0) { memcpy(body, z, (size_t)zl); blen = (int)zl; }
        else do_zstd = 0;   /* compression failed — fall back to raw (still valid) */
    }
    int fsize = 0;
    if (footer) {
        uint16_t fv = (footer_var >= 0) ? (uint16_t)footer_var : dv;
        body[blen++] = (uint8_t)(fv >> 8); body[blen++] = (uint8_t)fv; fsize = 2;
    }
    int pad = (16 - (blen % 16)) % 16;
    memset(body + blen, 0xFF, (size_t)pad); blen += pad;

    int flags = (do_zstd ? 1 : 0) | (establishing ? 2 : 0);
    if (pktid < 0) pktid = next_pktid(s, dv);

    pia_header h;
    h.dst = dv; h.src = sv; h.pktid = (uint16_t)pktid;
    next_nonce(s, h.nonce8);
    h.enc = 0x90; h.flags = (uint8_t)((pad << 4) | flags); h.footer = (uint8_t)fsize;

    uint8_t dg[SIM_MAXDG];
    size_t dglen = pia_encrypt(&s->crypto, body, (size_t)blen, s->our_ip, &h, dg);
    s->send(s->send_ctx, dg, dglen, unicast ? s->host_ip : s->broadcast);
    s->tx_count++;
}

static void send_one(sim_t *s, uint8_t proto, const uint8_t *payload, int len,
                     int dst_var, int src_var, int compress, int footer,
                     int establishing, int unicast, int pktid, int footer_var) {
    sim_msg_t m = { proto, payload, len, -1 };
    send_messages(s, &m, 1, dst_var, src_var, compress, footer, establishing,
                  unicast, pktid, footer_var);
}

static void tx_reliable(sim_t *s, int seq, uint8_t flagsA, const uint8_t *inner, int inner_len) {
    uint16_t s16 = (seq < 0) ? SIM_RELIABLE_SEQ_START : (uint16_t)seq;
    uint8_t rel[8 + REL_MAXINNER];
    int rl = rel_build_reliable(s16, rel_send_low(&s->rel), inner, inner_len, flagsA, rel);
    send_one(s, REL_PROTO_RELIABLE, rel, rl, s->host_var, s->our_var, 0, 1, 0, 1, -1, -1);
}

/* one frame queued for a datagram (with its inner bytes copied in) */
typedef struct { int seq; uint8_t flagsA; uint8_t inner[REL_MAXINNER]; int inner_len; } sim_bf_t;

static void bf_add(sim_bf_t *batch, int *bn, int seq, uint8_t flagsA,
                   const uint8_t *inner, int inner_len) {
    if (*bn >= 40) return;
    sim_bf_t *e = &batch[(*bn)++];
    e->seq = seq; e->flagsA = flagsA;
    if (inner_len > REL_MAXINNER) inner_len = REL_MAXINNER;
    memcpy(e->inner, inner, (size_t)inner_len); e->inner_len = inner_len;
}

/* The batch builder's two scratch areas (~13.6 KB) were function-local statics: dead RAM in any image that never runs the sim (the host
 * role). They are heap objects now, allocated on first use (or up front with sim_prealloc(), before Wi-Fi/Bluetooth fragment the heap)
 * and then kept for the life of the session. */
static sim_bf_t *s_final;
static uint8_t (*s_relbuf)[8 + REL_MAXINNER];

void sim_release(void) {
    free(s_final);  s_final = NULL;
    free(s_relbuf); s_relbuf = NULL;
}

int sim_prealloc(void) {
    if (!s_final)  s_final  = malloc(64 * sizeof *s_final);
    if (!s_relbuf) s_relbuf = malloc(64 * sizeof *s_relbuf);
    return s_final != NULL && s_relbuf != NULL;
}

static void tx_reliable_batch(sim_t *s, sim_bf_t *batch, int bn) {
    if (bn <= 0) return;

    /* carry-forward: prepend still-unacked data frames of the previous datagrams (newest first). */
    /* Only the SIM_RELIABLE_BATCH_MAX - bn lowest-seq carried frames ever go out (cn is clipped to `room` below), so keep just that many,
     * sorted ascending as they arrive, instead of all SIM_CARRY_DEPTH*16 (-5.9 KB of this task's stack). Same set, same order. */
    sim_bf_t carried[SIM_RELIABLE_BATCH_MAX]; int cn = 0;
    int ccap = SIM_RELIABLE_BATCH_MAX - bn; if (ccap < 0) ccap = 0;
    uint16_t have[64]; int have_n = 0;
    for (int i = 0; i < bn; i++) if (batch[i].seq >= 0) have[have_n++] = (uint16_t)batch[i].seq;

    for (int d = 0; d < s->carry_count; d++) {
        int idx = (s->carry_head - 1 - d + SIM_CARRY_DEPTH * 2) % SIM_CARRY_DEPTH;  /* newest..oldest */
        for (int j = 0; j < s->carry_len[idx]; j++) {
            uint16_t seq = s->carry[idx][j].seq; uint8_t fa = s->carry[idx][j].flagsA;
            int in_have = 0; for (int h = 0; h < have_n; h++) if (have[h] == seq) { in_have = 1; break; }
            if (in_have) continue;
            const rel_entry_t *e = rel_find(&s->rel, seq);
            if (!e || e->acked) continue;
            if (have_n < 64) have[have_n++] = seq;
            /* insertion into the sorted keep-the-lowest-ccap list */
            int pos = cn; while (pos > 0 && carried[pos - 1].seq > seq) pos--;
            if (pos >= ccap) continue;                       /* would sit beyond the kept window */
            int last = (cn < ccap) ? cn : ccap - 1;          /* when full, the highest entry falls off the end */
            for (int k = last; k > pos; k--) carried[k] = carried[k - 1];
            carried[pos].seq = seq; carried[pos].flagsA = fa;
            memcpy(carried[pos].inner, e->inner, (size_t)e->inner_len);
            carried[pos].inner_len = e->inner_len;
            if (cn < ccap) cn++;
        }
    }
    int room = SIM_RELIABLE_BATCH_MAX - bn; if (room < 0) room = 0;
    if (cn > room) cn = room;

    /* record this batch's DATA frames (seq != None, flagsA != CTRL) into the carry ring */
    {
        int idx = s->carry_head; int w = 0;
        for (int i = 0; i < bn && w < 16; i++) {
            if (batch[i].seq >= 0 && batch[i].flagsA != REL_FLAGSA_CTRL) {
                s->carry[idx][w].seq = (uint16_t)batch[i].seq;
                s->carry[idx][w].flagsA = batch[i].flagsA; w++;
            }
        }
        s->carry_len[idx] = w;
        s->carry_head = (s->carry_head + 1) % SIM_CARRY_DEPTH;
        if (s->carry_count < SIM_CARRY_DEPTH) s->carry_count++;
    }

    /* final = carried[0..cn) + batch */
    if (!sim_prealloc()) return;   /* out of memory: skip this batch; the reliable layer resends */
    sim_bf_t *final_ = s_final;     /* heap, not the stack */
    int fn = 0;
    for (int i = 0; i < cn; i++) final_[fn++] = carried[i];
    for (int i = 0; i < bn && fn < 64; i++) final_[fn++] = batch[i];

    /* build reliable messages, send in chunks of RELIABLE_BATCH_MAX */
    uint8_t (*relbuf)[8 + REL_MAXINNER] = s_relbuf;
    sim_msg_t msgs[64];
    for (int i = 0; i < fn; i++) {
        uint16_t s16 = (final_[i].seq < 0) ? SIM_RELIABLE_SEQ_START : (uint16_t)final_[i].seq;
        int inl = final_[i].inner_len; if (inl > REL_MAXINNER) inl = REL_MAXINNER;
        int rl = rel_build_reliable(s16, rel_send_low(&s->rel), final_[i].inner, inl,
                                    final_[i].flagsA, relbuf[i]);
        msgs[i].proto = REL_PROTO_RELIABLE; msgs[i].payload = relbuf[i]; msgs[i].len = rl;
        msgs[i].msgflags = (final_[i].flagsA == REL_FLAGSA_CTRL) ? 0x40 : -1;
    }
    for (int i = 0; i < fn; i += SIM_RELIABLE_BATCH_MAX) {
        int chunk = fn - i; if (chunk > SIM_RELIABLE_BATCH_MAX) chunk = SIM_RELIABLE_BATCH_MAX;
        send_messages(s, msgs + i, chunk, s->host_var, s->our_var, 0, 1, 0, 1, -1, -1);
    }
}

/* ---- inbound ---- */
static void on_gba_in(sim_t *s, const uint8_t *payload, int len);

int sim_process_datagram(sim_t *s, const uint8_t *dg, int dg_len, const uint8_t src_ip[4]) {
    if (!pia_is_pia(dg, (size_t)dg_len)) return 0;
    pia_header hdr; pia_header_unpack(dg, &hdr);
    if (!s->learned && hdr.src != 0) {
        s->host_var = hdr.src; s->learned = 1;
        if (s->conn) {
            int ov = s->our_var, hv = s->host_var;
            pc_conn_learn_ids(s->conn, &ov, &hv);
        }
    }
    uint8_t pt[SIM_MAXAPP];
    long ptlen = pia_decrypt(&s->crypto, dg, (size_t)dg_len, src_ip, pt);
    if (ptlen < 0) { s->rx_fail++; return 0; }
    uint8_t app[SIM_MAXAPP];
    long applen = pia_decompress(pt, (size_t)ptlen, app, sizeof(app));
    if (applen < 0) return 0;

    rel_message_t msgs[SIM_MAXMSGS]; int station, tail;
    int nm = rel_parse_app(app, (int)applen, msgs, SIM_MAXMSGS, &station, &tail);
    if (nm > SIM_MAXMSGS) nm = SIM_MAXMSGS;   /* defensive: never index past msgs[] */
    for (int i = 0; i < nm; i++) {
        rel_message_t *m = &msgs[i];
        if (m->proto == REL_PROTO_RELIABLE) {
            rel_reliable_t rl;
            if (rel_parse_reliable(m->payload, m->size, &rl) != 0) continue;
            if (rl.flagsA & 0x01) {
                s->ack_owed = 1;
                if (!seen_in_has(s, rl.seq)) {
                    note_in_seq(s, rl.seq);
                    if (rl.payload_len >= 1 && rl.payload[0] == 0x57)
                        on_gba_in(s, rl.payload, rl.payload_len);
                }
                rel_note_received(&s->rel, rl.seq);
            } else {
                uint16_t ackid; uint8_t mask[16];
                int pr = rel_parse_bulk_ack(rl.payload, rl.payload_len, &ackid, mask);
                /* Kizuna: passive diagnostic only, no behavior change. up_infl climbed
                 * monotonically to the window cap with since_ack never resetting during a live session
                 * -- the Switch's own downstream traffic never stopped, so either no
                 * bulk-ack ever arrives on this branch, it fails to parse, or window_lo never advances
                 * for a reason not visible from the outside. Log every attempt so the next run shows which. */
                uint16_t wlo_before = s->rel.window_lo;
                if (pr == 0) rel_on_ack(&s->rel, ackid, mask, 1, (uint32_t)(s->tick_count * SIM_MS_PER_VBLANK));
                /* Kizuna: the original 60-line cap exhausted during the early pre-join
                 * handshake, well before the critical post-accept window where since_ack actually stalls --
                 * leaving a total blind spot across exactly the moment this needs to explain. Uncapped but
                 * throttled instead: always log a FAILED parse or a no-op ack (window_lo unchanged) since
                 * those are rare and exactly what we're hunting for; log a normal advancing ack only once
                 * per ~30 ticks (~500ms) so a multi-minute session stays printable. tick_count, not a wall
                 * clock -- this file is also built host-side for unit tests (sim.h's own header comment). */
                static int32_t s_ack_dbg_last_tick;
                int stuck = (pr == 0 && s->rel.window_lo == wlo_before);
                int failed = (pr != 0);
                int due = (s->tick_count - s_ack_dbg_last_tick) >= 30;
                if (failed || stuck || due) { s_ack_dbg_last_tick = s->tick_count;
                    KZ_TRACE("  [ack-dbg] tick=%d bulk-ack plen=%d parse=%s ackid=0x%04x mask0=%02x window_lo %u->%u%s\n",
                           (int)s->tick_count, rl.payload_len, pr == 0 ? "ok" : "FAIL", pr == 0 ? ackid : 0,
                           pr == 0 ? mask[0] : 0, wlo_before, s->rel.window_lo,
                           failed ? " <-- PARSE FAILED" : (stuck ? " <-- NO-OP (window_lo unchanged)" : "")); }
            }
        } else if (s->conn) {
            pc_conn_on_message(s->conn, (uint8_t)m->proto, m->payload, m->size, s->tick_count);
        }
    }
    s->rx_count++;
    return 1;
}

static void on_gba_in(sim_t *s, const uint8_t *payload, int len) {
    gba_in rec;
    if (!gba_parse_in(payload, (size_t)len, &rec) || !rec.valid) return;

    if (rec.type == GBA_TYPE_A) {
        if (!s->gba_accepted) s->gba_accepted = 1;
        return;
    }
    if (rec.type == GBA_TYPE_D) {
        if (!s->host_disconnected) s->host_disconnected = 1;
        return;
    }
    if (rec.type != GBA_TYPE_T) return;

    if (rec.ts_valid) {
        s->host_t_in++;
        { int d = s->tick_count - s->last_host_t_tick; if (d < 0) d = 0; if (d > 5) d = 5; s->td[d]++; }
        s->last_host_t_tick = s->tick_count;
        if (!ts_seen(s, rec.ts)) {
            ts_add(s, rec.ts);
            s->k_seq++;
            s->pending_k_ts = rec.ts; s->has_pending_k_ts = 1;
        }
    }
    if (rec.has_ni) {
        int state = rec.llsf_state;
        uint8_t ack_out[2];
        size_t alen = ni_receiver_on_host_ni(&s->ni_recv, state, rec.ni_ack, rec.ni_n, rec.ni_phase,
                                             rec.ni_payload, (size_t)rec.ni_payload_len, ack_out);
        if (alen > 0) {
            int same = s->has_cur_ni_ack && s->cur_ni_ack_len == (int)alen &&
                       memcmp(s->cur_ni_ack, ack_out, alen) == 0;
            if (same) {
                s->host_ni_repeat++;
                if (s->host_ni_repeat % SIM_RECV_NI_REACK_EVERY == 0) s->has_ni_ack_bytes = 0;
            } else {
                s->host_ni_repeat = 0;
            }
            memcpy(s->cur_ni_ack, ack_out, alen); s->cur_ni_ack_len = (int)alen; s->has_cur_ni_ack = 1;
        }
        if (rec.ni_ack == 1) {
            s->host_ni_ack_state = state; s->has_host_ni_ack_state = 1;
            int same_key = s->has_host_ni_ack_key && s->hnak_state == state &&
                           s->hnak_n == rec.ni_n && s->hnak_phase == rec.ni_phase;
            if (same_key) {
                s->host_ni_ack_repeat++;
                if (s->host_ni_ack_repeat >= SIM_HOST_ACK_REPEAT_BEFORE_RESEND &&
                    s->ni_built && !s->has_host_ni_resend) {
                    uint8_t rs[NI_SLOT_MAX]; size_t rl;
                    if (ni_sender_resend_after(&s->ni, state, rec.ni_n, rec.ni_phase, rs, &rl)) {
                        memcpy(s->host_ni_resend, rs, rl); s->host_ni_resend_len = (int)rl;
                        s->has_host_ni_resend = 1;
                    }
                }
            } else {
                s->hnak_state = state; s->hnak_n = rec.ni_n; s->hnak_phase = rec.ni_phase;
                s->has_host_ni_ack_key = 1;
                s->host_ni_ack_repeat = 0; s->has_host_ni_resend = 0;
                s->null_reemits = 0; s->null_ticks = 0;
            }
        }
        if (state == RFU_LCOM_NULL && rec.ni_ack == 0) s->host_ni_null_seen = 1;
        if (s->ni_recv.has_status && !s->ni_status_logged) {
            s->ni_status_logged = 1;
            if (s->ni_recv.status != NI_STATUS_JOIN_GROUP_OK) s->ni_rejected = 1;
        }
    }
    if (rec.llsf_state == 4) s->host_uni_seen = 1;
    if (!s->host_close_seen && rec.nslots > 0) {
        rfu_parsed hp; rfu_parse_slot(rec.slots[0], &hp);
        if (hp.valid && hp.op == RFU_READY_CLOSE_LINK) { s->host_close_seen = 1; s->host_close_tick = s->tick_count; }
    }

    s->slot_credit = s->slot_credit + 1;
    if (s->slot_credit > SIM_SLOT_CREDIT_MAX) s->slot_credit = SIM_SLOT_CREDIT_MAX;

    if (s->engine && s->engine->feed_in_frame) s->engine->feed_in_frame(s->engine->ctx, &rec);
}

/* ---- NI + slot building ---- */
static void ensure_ni(sim_t *s) {
    if (s->ni_built) return;
    s->ni_built = 1;
    int vlow = s->engine->lp_version_low ? s->engine->lp_version_low : 5;   /* LeafGreen */
    uint16_t tid = s->engine->lp_trainer_id ? s->engine->lp_trainer_id : 0x8822;
    const char *name = s->engine->lp_name ? s->engine->lp_name : "EMU";
    int activity = s->engine->ni_activity ? s->engine->ni_activity : NI_ACTIVITY_TRADE;
    int started = s->engine->ni_started;
    uint8_t pinfo[4] = {0,0,0,0};
    uint8_t src[26];
    int lang = s->engine->lp_language ? s->engine->lp_language : NI_LANGUAGE_ENGLISH;
    if (s->engine->lp_name_raw_valid) {
        /* Japanese (or any non-ASCII) cartridge: pass its own FRLG-charmap name bytes through
         * verbatim — charmap_encode would drop kana. */
        ni_build_game_data_raw(vlow, tid, s->engine->lp_name_raw, lang, activity, started, pinfo, src);
    } else {
        ni_build_game_data(vlow, tid, name, lang, activity, started, pinfo, src);
    }
    if (s->engine->ni_compat_or) {   /* compat is the first word after the 2-byte serial */
        uint16_t compat = (uint16_t)(src[2] | (src[3] << 8)) | s->engine->ni_compat_or;
        src[2] = (uint8_t)compat; src[3] = (uint8_t)(compat >> 8);
    }
    ni_sender_init(&s->ni, src, 26, 14, 1);
}

static int wrap_t(sim_t *s, const uint8_t *slot, int slot_len, uint8_t *out) {
    int n = (int)gba_wrap_t(slot, (size_t)slot_len, s->ts, out);
    s->ts = (s->ts + 1) & 0xFFFFFFFF;
    return n;
}

/* Returns frame length into out, or -1 for "no slot this VBlank" (Python None). */
static int gba_frame(sim_t *s, uint8_t *out) {
    s->has_emitted_ni_ack = 0;

    if (s->conn && s->gba_accepted && !s->ni_done) {
        ensure_ni(s);
        if (!ni_sender_done(&s->ni)) {
            int emit_next = 0;
            if (!s->has_ni_awaiting) emit_next = 1;
            else if (s->has_host_ni_ack_key && s->hnak_state == s->ni_aw_state &&
                     s->hnak_n == s->ni_aw_n && s->hnak_phase == s->ni_aw_phase) emit_next = 1;
            if (emit_next) {
                uint8_t slot[NI_SLOT_MAX]; size_t slen;
                if (ni_sender_next_slot(&s->ni, slot, &slen)) {
                    int st, n, ph; uint8_t last[NI_SLOT_MAX]; size_t ll;
                    ni_sender_last_emitted(&s->ni, &st, &n, &ph, last, &ll);
                    s->ni_aw_state = st; s->ni_aw_n = n; s->ni_aw_phase = ph; s->has_ni_awaiting = 1;
                    s->ni_wait_ticks = 0;
                    return wrap_t(s, slot, (int)slen, out);
                }
            } else {
                s->ni_wait_ticks++;
                if (s->ni_wait_ticks % SIM_NI_ACK_WAIT_RESEND == 0) {
                    int st, n, ph; uint8_t last[NI_SLOT_MAX]; size_t ll;
                    if (ni_sender_last_emitted(&s->ni, &st, &n, &ph, last, &ll))
                        return wrap_t(s, last, (int)ll, out);
                }
            }
        }
        /* host blocked on its sub-frame ack: outranks our own re-sends */
        if (s->has_cur_ni_ack &&
            !(s->has_ni_ack_bytes && s->ni_ack_bytes_len == s->cur_ni_ack_len &&
              memcmp(s->ni_ack_bytes, s->cur_ni_ack, s->cur_ni_ack_len) == 0)) {
            memcpy(s->emitted_ni_ack, s->cur_ni_ack, s->cur_ni_ack_len);
            s->emitted_ni_ack_len = s->cur_ni_ack_len; s->has_emitted_ni_ack = 1;
            return wrap_t(s, s->cur_ni_ack, s->cur_ni_ack_len, out);
        }
        /* host re-acking one of our sub-frames -> resend the next, paced */
        if (ni_sender_done(&s->ni) && !s->host_uni_seen && s->has_host_ni_resend &&
            s->null_reemits < SIM_NULL_REEMIT_MAX) {
            s->null_ticks++;
            if (s->null_ticks % SIM_NULL_REEMIT_EVERY == 0) {
                s->null_reemits++;
                return wrap_t(s, s->host_ni_resend, s->host_ni_resend_len, out);
            }
        }
        if (!s->host_uni_seen) {
            if (s->has_cur_ni_ack && !s->host_ni_null_seen &&
                (s->tick_count - s->last_t_tick >= SIM_CHILD_SILENCE_LIMIT)) {
                s->silence_forced++;
                memcpy(s->emitted_ni_ack, s->cur_ni_ack, s->cur_ni_ack_len);
                s->emitted_ni_ack_len = s->cur_ni_ack_len; s->has_emitted_ni_ack = 1;
                return wrap_t(s, s->cur_ni_ack, s->cur_ni_ack_len, out);
            }
            return -1;
        }
        s->ni_done = 1;
    }

    if (s->d_sent) return -1;     /* our RFU link is closed: no more UNI rows */
    uint16_t words[7];
    s->engine->tick(s->engine->ctx, words);
    if (!s->close_out_seen && (words[0] & RFU_RFUCMD_MASK) == RFU_READY_CLOSE_LINK) {
        s->close_out_seen = 1; s->close_out_tick = s->tick_count; s->close_by_host = s->host_close_seen;
        printf("  [sim] child READY_CLOSE_LINK emitted (host-led=%d)%s\n", s->close_by_host, s->close_by_host ? " -> 'D' follows" : "");
    }
    /* linkstate held-keys branch requires engine.established (relay=0) -> skipped */
    uint8_t cmd14[RFU_SLOT_LEN];
    rfu_slot_builder_build(&s->slot, words, cmd14);
    uint8_t uni[RFU_UNI_SLOT_LEN];
    rfu_uni_slot(cmd14, uni);
    return wrap_t(s, uni, RFU_UNI_SLOT_LEN, out);
}

/* ---- the per-VBlank send state machine ---- */
static void drive_reliable(sim_t *s) {
    int32_t tick = s->tick_count;
    uint32_t now_ms = (uint32_t)(s->tick_count * SIM_MS_PER_VBLANK);

    if (!s->rel_opened) {
        uint16_t seq = rel_queue(&s->rel, METADATA_FRAME, 46, REL_FLAGSA_INIT, now_ms);
        tx_reliable(s, seq, REL_FLAGSA_INIT, METADATA_FRAME, 46);
        s->rel_opened = 1;
        return;
    }
    if (s->has_connect_id && !s->gba_conn_sent) {
        uint8_t frame[32];
        int flen = (int)gba_build_connect(s->connect_id, (size_t)s->connect_id_len, frame);
        uint16_t seq = rel_queue(&s->rel, frame, flen, REL_FLAGSA_GBA, now_ms);
        tx_reliable(s, seq, REL_FLAGSA_GBA, frame, flen);
        s->gba_conn_sent = 1;
        return;
    }

    sim_bf_t batch[40]; int bn = 0;
    int in_block_phase = s->gba_accepted && !s->engine->in_seat_phase;
    int rtx_limit = in_block_phase ? SIM_RTX_GAP_LIMIT : SIM_RTX_GAP_LIMIT_NI;
    rel_due_t due[REL_MAX_INFLIGHT];
    int nd = rel_due_retransmits(&s->rel, now_ms, rtx_limit, due, REL_MAX_INFLIGHT);
    for (int i = 0; i < nd && i < SIM_RELIABLE_BATCH_MAX; i++)
        bf_add(batch, &bn, due[i].seq, due[i].flagsA, due[i].inner, due[i].inner_len);

    /* _k_seqs.intersection_update(unacked): drop seqs no longer in the window */
    { int w = 0; for (int i = 0; i < s->k_seqs_n; i++)
        if (rel_find(&s->rel, s->k_seqs[i])) s->k_seqs[w++] = s->k_seqs[i];
      s->k_seqs_n = w; }

    if (s->has_pending_k_ts && s->k_seqs_n < SIM_K_INFLIGHT_MAX) {
        uint8_t kf[16]; gba_build_k((uint32_t)s->k_seq, 1, s->pending_k_ts, kf);
        uint16_t seq = rel_queue(&s->rel, kf, 16, REL_FLAGSA_GBA, now_ms);
        if (s->k_seqs_n < REL_MAX_INFLIGHT) s->k_seqs[s->k_seqs_n++] = seq;
        bf_add(batch, &bn, seq, REL_FLAGSA_GBA, kf, 16);
        s->has_pending_k_ts = 0; s->k_out++;
    }

    /* Opt-in (leave_d): a REAL leader that started the close blocks until we disconnect [union_room_chat.c:665 style];
     * a native child answers its READY_CLOSE_LINK and sends 'D' right after. The mystery-gift flow stays host-D-driven. */
    if (s->leave_d && s->close_out_seen && s->close_by_host && !s->d_sent && s->has_connect_id && s->connect_id_len >= 2 &&
        (s->tick_count - s->close_out_tick) >= SIM_CLOSE_D_DELAY_TICKS) {
        uint8_t dframe[6];
        int dlen = (int)gba_build_disconnect(s->connect_id, dframe);
        uint16_t seq = rel_queue(&s->rel, dframe, dlen, REL_FLAGSA_GBA, now_ms);
        bf_add(batch, &bn, seq, REL_FLAGSA_GBA, dframe, dlen);
        s->d_sent = 1; s->d_tick = s->tick_count;
        printf("  [sim] child 'D' (disconnect) queued after READY_CLOSE_LINK\n");
    }

    if (s->gba_accepted && !s->d_sent) {
        /* Burst loop (SIM_UP_BURST_MAX): normally ONE game frame per tick (classic behavior). A second
         * iteration runs ONLY when the engine reports a real relay backlog (has_backlog, optional) AND the
         * credit/inflight gates still allow — so a C6 whose brain tick runs at the GBA poll rate (~26/s) can
         * drain a queued burst instead of permanently trailing production by design margin ~0. Offline
         * engines (has_backlog==NULL) never loop. */
        for (int _burst = 0; _burst < SIM_UP_BURST_MAX; _burst++) {
            int _out = rel_outstanding(&s->rel);
            int _quiet = (s->tick_count - s->last_t_tick) >= SIM_CHILD_SILENCE_LIMIT;
            int _starved = _quiet && _out < s->max_inflight * s->silence_ceil;
            /* tr-1.56: bypass the slot-credit throttle when the GBA has a real UP backlog (a block burst).
             * slot_credit(=2) keeps the steady UNI phase ~1:1 with the Switch (childf/trade stability), but it
             * DEADLOCKS a menu-re-entry block transfer: the host pauses its own block mid-way (~8/17, waiting for
             * the child's block in MP lockstep), so credit stops replenishing (granted only per host DATA frame,
             * sim.c:315) and the GBA's remaining block chunks FREEZE in the UP FIFO (observed: t_out froze at
             * 3816) -> they never reach the host -> the host waits forever. Draining the backlog (still bounded by
             * the reliable window SIM_MAX_INFLIGHT) delivers the child's whole block, so the host resumes its own.
             * Active only when depth>0, so the proven no-backlog steady state is byte-for-byte unchanged. */
            int _host_quiet = (s->tick_count - s->last_host_t_tick) >= SIM_BYPASS_HOST_QUIET;
            int _has_backlog = _host_quiet && s->engine->has_backlog && s->engine->has_backlog(s->engine->ctx);
            int _gated = ((_out >= s->max_inflight && !_starved) ||
                          (s->slot_credit <= 0 && !_quiet && !_has_backlog));
            if (_starved && _out >= s->max_inflight) s->silence_forced++;
            if (_gated) {
                if (_out >= s->max_inflight && !_starved) s->gate_win++; else s->gate_credit++;
                if (_burst == 0 && s->engine->poll_send_done) s->engine->poll_send_done(s->engine->ctx);
                break;
            }
            uint8_t inner[64];
            int ilen = gba_frame(s, inner);
            if (ilen < 0) s->no_frame++;
            if (ilen >= 0) {
                if (s->slot_credit > 0) s->slot_credit--;
                s->last_seat_emit = tick;
                uint16_t seq = rel_queue(&s->rel, inner, ilen, REL_FLAGSA_GBA, now_ms);
                bf_add(batch, &bn, seq, REL_FLAGSA_GBA, inner, ilen);
                s->t_out++;
                if (s->tick_count - s->last_t_tick > s->max_silence)
                    s->max_silence = s->tick_count - s->last_t_tick;
                s->last_t_tick = s->tick_count;
                if (s->has_emitted_ni_ack) {
                    s->ni_ack_seq = seq; s->has_ni_ack_seq = 1;
                    memcpy(s->ni_ack_bytes, s->emitted_ni_ack, s->emitted_ni_ack_len);
                    s->ni_ack_bytes_len = s->emitted_ni_ack_len; s->has_ni_ack_bytes = 1;
                }
            }
            if (!(s->engine->has_backlog && s->engine->has_backlog(s->engine->ctx)))
                break;
        }
    }

    int due_ack = (tick - s->last_ack_tick) >= SIM_ACK_PERIOD;
    if (due_ack && (s->ack_owed || s->rel.recv_ooo_n > 0)) {
        uint8_t ap[20]; rel_ack_payload(&s->rel, ap);
        bf_add(batch, &bn, -1, REL_FLAGSA_CTRL, ap, 20);
        s->ack_owed = 0; s->last_ack_tick = tick;
    }

    /* tr-1.55 DIAG: reliable-state trace during the trade phase (~2/s at the C6 tick rate). Menu-re-entry
     * bottleneck: which direction stalls? up_inflight near SIM_MAX_INFLIGHT(24) = our UP (GBA block) window
     * full waiting for Switch acks; ooo>0 / since_ack high = we're under-acking the Switch's DOWN block. */
    if (s->gba_accepted && (s->tick_count % 8) == 0) {
        /* Kizuna: + tin (host 'T' frames received = polls, the slot-credit source), k (K-acks sent),
         * sf (silence-override emissions). tin flat while t_out climbs = we are emitting unsolicited slots. */
        KZ_TRACE("  [rel] t=%d up_infl=%d/%d ooo=%d ack_owed=%d bn=%d t_out=%d rx=%d since_ack=%d tin=%d k=%d sf=%d cr=%d gw=%d gc=%d nf=%d td=%d,%d,%d,%d,%d,%d\n",
               (int)s->tick_count, rel_outstanding(&s->rel), s->max_inflight,
               s->rel.recv_ooo_n, (int)s->ack_owed, bn, s->t_out, s->rx_count,
               (int)(tick - s->last_ack_tick), s->host_t_in, s->k_out, s->silence_forced, s->slot_credit,
               s->gate_win, s->gate_credit, s->no_frame, s->td[0], s->td[1], s->td[2], s->td[3], s->td[4], s->td[5]);
    }

    tx_reliable_batch(s, batch, bn);
}

/* ---- public ---- */
void sim_init(sim_t *s, const pia_crypto *crypto, sim_engine *engine,
              sim_send_fn send, void *send_ctx,
              const uint8_t our_ip[4], const uint8_t host_ip[4], pc_conn_t *conn,
              const uint8_t *connect_id, int connect_id_len, uint64_t nonce_seed) {
    memset(s, 0, sizeof(*s));
    s->max_inflight = SIM_MAX_INFLIGHT; s->silence_ceil = SIM_SILENCE_OVERRIDE_CEIL;
    s->crypto = *crypto;
    s->engine = engine;
    s->leave_d = engine->leave_d;
    s->send = send; s->send_ctx = send_ctx;
    memcpy(s->our_ip, our_ip, 4); memcpy(s->host_ip, host_ip, 4);
    memcpy(s->broadcast, host_ip, 3); s->broadcast[3] = 255;
    s->conn = conn;

    rfu_slot_builder_init(&s->slot);
    s->ts = SIM_TS_SEED;
    s->out_seq = SIM_RELIABLE_SEQ_START;
    s->nonce = nonce_seed ? nonce_seed : 1;
    s->last_ack_tick = -100;
    s->last_seat_emit = -100;

    rel_link_init(&s->rel, SIM_RELIABLE_SEQ_START);
    s->rel.rtt_jitter_k = SIM_RTT_JITTER_K;
    s->rel.dup_nack_threshold = SIM_DUP_NACK_THRESHOLD;
    s->rel.has_rto_ceil = 1; s->rel.rto_ceil_ms = SIM_RTO_CEIL_MS;
    s->rel.rto_backoff = SIM_RTO_BACKOFF;
    s->rel.has_rto_bootstrap = 1; s->rel.rto_bootstrap_ms = SIM_RTO_BOOTSTRAP_MS;

    s->our_var = conn ? conn->our_var : REL_STATION_JOINER;
    s->host_var = REL_STATION_HOST;
    if (conn) conn->our_var = s->our_var;

    if (connect_id && connect_id_len > 0) {
        if (connect_id_len > 16) connect_id_len = 16;
        memcpy(s->connect_id, connect_id, (size_t)connect_id_len);
        s->connect_id_len = connect_id_len; s->has_connect_id = 1;
    }
    ni_receiver_init(&s->ni_recv);
    if (engine && !engine->ni_started) engine->ni_started = 1;   /* default True */
}

int sim_connected(const sim_t *s) {
    return s->conn == NULL || pc_conn_connected(s->conn);
}

void sim_feed_datagram(sim_t *s, const uint8_t *dg, int dg_len, const uint8_t src_ip[4]) {
    if (s->inq_n >= SIM_INQ) return;   /* drop under overflow */
    if (dg_len > SIM_MAXDG) return;
    int idx = (s->inq_head + s->inq_n) % SIM_INQ;
    memcpy(s->inq[idx].dg, dg, (size_t)dg_len); s->inq[idx].len = dg_len;
    memcpy(s->inq[idx].src, src_ip, 4);
    s->inq_n++;
}

void sim_tick(sim_t *s) {
    s->tick_count++;
    /* recv is drained here (post-increment) so process_datagram sees the same tick as Python. */
    while (s->inq_n > 0) {
        int idx = s->inq_head;
        sim_process_datagram(s, s->inq[idx].dg, s->inq[idx].len, s->inq[idx].src);
        s->inq_head = (s->inq_head + 1) % SIM_INQ; s->inq_n--;
    }
    if (s->conn) {
        for (int i = 0; i < s->conn->rtt_samples_n; i++)
            rel_add_rtt_sample(&s->rel, s->conn->rtt_samples[i] * SIM_MS_PER_VBLANK);
        s->conn->rtt_samples_n = 0;
        pc_conn_maybe_originate_rtt(s->conn, s->tick_count);
        pc_conn_maybe_repeat_join(s->conn, s->tick_count);
        const pc_out_t *outs; int no = pc_conn_drain(s->conn, &outs);
        for (int i = 0; i < no; i++) {
            const pc_out_t *e = &outs[i];
            send_one(s, e->proto, e->payload, e->payload_len, e->dst, e->src,
                     e->compress, e->footer, e->establishing, e->unicast, e->pktid, e->footer_var);
        }
    }
    if (sim_connected(s)) drive_reliable(s);
}

void sim_reply_now(sim_t *s) {
    /* steady state only (join and NI phases stay tick-driven). Something arrived since the last look: process it
     * and answer at once -- the K ack and, if a poll gave us credit, one game frame -- without advancing the VBlank clock.
     * Waiting for the next 60 Hz tick put up to 17 ms on both legs of every poll/ack round trip (~30 polls/s, not 60). */
    if (s->inq_n <= 0 || !s->gba_accepted || s->d_sent || !sim_connected(s)) return;
    while (s->inq_n > 0) {
        int idx = s->inq_head;
        sim_process_datagram(s, s->inq[idx].dg, s->inq[idx].len, s->inq[idx].src);
        s->inq_head = (s->inq_head + 1) % SIM_INQ; s->inq_n--;
    }
    drive_reliable(s);
}

void sim_skip_ticks(sim_t *s, int n) {
    if (n > 0) s->tick_count += n;
}

void sim_set_window(sim_t *s, int max_inflight, int silence_ceil) {
    if (max_inflight < 1) max_inflight = 1;
    if (max_inflight > REL_MAX_INFLIGHT) max_inflight = REL_MAX_INFLIGHT;
    if (silence_ceil < 1) silence_ceil = 1;
    s->max_inflight = max_inflight; s->silence_ceil = silence_ceil;
}
