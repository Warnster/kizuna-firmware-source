/* SPDX-License-Identifier: AGPL-3.0-only
 * Derived from pokeldn (https://github.com/Decryptu/pokeldn), Copyright (c) the pokeldn authors.
 * Changes Copyright (c) 2026 Kizuna. See NOTICE.
 */
/* sim.h — the per-VBlank trade orchestrator. C port of pokeldn/frlg/link/sim.py (joiner path,
 * conn != NULL, pace_ms = 0 — the config the reference standalone run used).
 *
 * Drives transport <-> crypto <-> Pia <-> the trade engine. On the ESP the engine is the RelayEngine
 * (real GBA slots over the Pico UART); on the host test it's a fake slot queue. The Sim is
 * PUSH-driven: feed each inbound datagram to sim_process_datagram(), then call sim_tick() once per
 * VBlank. Outbound datagrams leave via the send callback. No pacing, no linkstate (unused when the
 * engine reports established=0 / in_seat_phase=0, i.e. the relay).
 */
#ifndef SIM_H
#define SIM_H
#include <stdint.h>
#include <stddef.h>
#include "pia_connect.h"
#include "reliable.h"
#include "ni.h"
#include "gbaframe.h"
#include "rfu.h"
#include "crypto.h"

/* ---- console-faithful constants (sim.py top). DO NOT change without re-reading sim.py. ---- */
#define SIM_RELIABLE_SEQ_START 0xFFF0
#define SIM_MS_PER_VBLANK      (1000.0 / 59.727)
#define SIM_RELIABLE_BATCH_MAX 9
#define SIM_UP_BURST_MAX       1   /* max game T-frames per tick. Was 2 (tr-1.27, a drain aid for the single-chip
                                    * C6 whose tick ran at the GBA poll rate, ~26/s, i.e. slower than the host's
                                    * 60 Hz polls, so two per tick still meant at most one per host poll).
                                    * Kizuna: on the two-chip board the tick runs at 60 Hz,
                                    * so two per tick = two child slots inside ONE console poll, which the FRLG
                                    * parent cannot take: RfuMain2_Parent keeps exactly one child slot per poll
                                    * and treats any rolling tag that is not +1 mod 8 as a receive error, fatal
                                    * on the fifth (link_rfu_2.c:876-892; pokeldn docs/frlg_link.md "One child
                                    * slot per parent poll"). Every M2 bench death (runs 5/11/12/13)
                                    * followed the only phase that emits dense non-idle slots, the LinkPlayer
                                    * exchange. The has_backlog credit bypass (tr-1.56) is kept, at one per tick. */
#define SIM_CARRY_DEPTH        4
#define SIM_RTX_GAP_LIMIT      1
#define SIM_RTX_GAP_LIMIT_NI   2
#define SIM_MAX_INFLIGHT       24
#define SIM_RTT_JITTER_K       1.0
#define SIM_DUP_NACK_THRESHOLD 1
#define SIM_RTO_CEIL_MS        120.0
#define SIM_RTO_BACKOFF        1.0
#define SIM_RTO_BOOTSTRAP_MS   200.0
#define SIM_RECV_NI_REACK_EVERY 20
#define SIM_HOST_ACK_REPEAT_BEFORE_RESEND 10
#define SIM_NULL_REEMIT_EVERY  12
#define SIM_NI_ACK_WAIT_RESEND 12
#define SIM_NULL_REEMIT_MAX    30
#define SIM_K_INFLIGHT_MAX     4
#define SIM_CHILD_SILENCE_LIMIT 20
/* Kizuna bench: the tr-1.56 has_backlog credit bypass may only fire once the host has
 * stopped polling for this many ticks (the mid-block deadlock it was written for). While the host polls (the
 * seat walk: one held-keys slot per VBlank both ways) it must never add a slot beyond the credit, because our
 * 60.0 Hz tick runs slightly faster than the console's 59.727 Hz polls and every surplus slot is a second
 * slot inside one poll. pokeldn sim.py: "NOTHING is exempt from the credit pacer, the seat walk included". */
#define SIM_BYPASS_HOST_QUIET  8
#define SIM_SILENCE_OVERRIDE_CEIL 2
#define SIM_SLOT_CREDIT_MAX    2
#define SIM_ACK_PERIOD         2
#define SIM_COMPRESS_MIN       62
#define SIM_TS_SEED            0x0000362E
#define SIM_BARRIER_EMITS      6
#define SIM_CLOSE_D_DELAY_TICKS 12   /* host-led close: our 'D' this long after our READY_CLOSE_LINK answer (a native leaver: ~0.1-0.2 s) */
#define SIM_LEAVE_GRACE_TICKS  120   /* after our 'D', keep the link up this long so it is delivered, then leave */

/* buffers */
#define SIM_MAXDG   1600   /* max Pia datagram */
#define SIM_MAXAPP  1400   /* decompressed app blob */
#define SIM_MAXMSGS 64   /* a real trade datagram tiles into more Pia messages than the handshake's
                          * handful; 16 was too small and (with the old rel_parse_messages count bug)
                          * overran msgs[]. 64 covers real traffic; the parser also caps at this now. */

/* ---- engine interface (RelayEngine on ESP, fake queue in test) ---- */
typedef struct sim_engine {
    void  (*tick)(void *ctx, uint16_t words[7]);        /* fill next GBA slot (idle=all-zero) */
    void  (*feed_in_frame)(void *ctx, const gba_in *rec); /* deliver host slots to the GBA */
    void  (*poll_send_done)(void *ctx);                 /* optional; may be NULL */
    int   (*has_backlog)(void *ctx);                    /* optional; may be NULL. Nonzero = the engine has MORE
                                                         * real slots queued beyond the one just ticked out —
                                                         * lets drive_reliable emit a small burst (up to
                                                         * SIM_UP_BURST_MAX per tick) to drain a relay backlog
                                                         * instead of falling behind at 1/tick. NULL = classic
                                                         * one-per-tick (offline tests unchanged). */
    void  *ctx;
    int   in_seat_phase;   /* relay: 0 */
    int   established;     /* relay: 0 */
    int   leave_d;         /* proxy engines: answer a host-led READY_CLOSE_LINK with the child 'D' (see sim.c) */
    uint16_t ni_compat_or; /* extra RfuGameCompatibilityData bits OR'd into the NI game data (0 = none; the cable
                            * engine sets canLinkNationally/hasNationalDex/gameClear) */
    /* NI game-data identity (LinkPlayer defaults: version_low=5, tid=0x8822, "EMU") */
    int      lp_version_low;
    uint16_t lp_trainer_id;
    const char *lp_name;          /* ASCII name (English carts); ignored if lp_name_raw is set */
    /* RAW 8-byte FRLG-charmap trainer name straight from the cartridge's own Broadcast(0x16).
     * Set this (lp_name_raw_valid=1) for a JAPANESE cart — its name is kana, which the ASCII
     * charmap can't encode. Takes precedence over lp_name. */
    uint8_t lp_name_raw[8];
    int     lp_name_raw_valid;
    int     lp_language;          /* NI_LANGUAGE_* ; 0 => default (English) */
    int   ni_activity;     /* NI_ACTIVITY_TRADE */
    int   ni_started;      /* 1 */
} sim_engine;

typedef void (*sim_send_fn)(void *ctx, const uint8_t *dg, size_t len, const uint8_t dst_ip[4]);

/* ---- internal small sets ---- */
#define SIM_TSRING   1024
#define SIM_SEENRING 1024
#define SIM_PKTIDMAP 8
typedef struct { uint16_t seq; uint8_t flagsA; } sim_cf_t;

typedef struct {
    /* wiring */
    pia_crypto  crypto;
    sim_engine *engine;
    sim_send_fn send;
    void       *send_ctx;
    uint8_t     our_ip[4], host_ip[4], broadcast[4];
    pc_conn_t  *conn;

    /* rfu slot builder */
    rfu_slot_builder slot;

    /* trade / K-ack */
    uint32_t ts;
    int      k_seq;
    uint32_t acked_ts[SIM_TSRING]; int acked_ts_n;   /* ring; dedup host ts */
    int      has_pending_k_ts; uint32_t pending_k_ts;
    uint16_t k_seqs[REL_MAX_INFLIGHT]; int k_seqs_n; /* seqs carrying K frames */
    int      host_t_in;
    int      gate_win, gate_credit, no_frame;   /* why a tick did not emit (window full / no slot credit / nothing to send) */
    int      td[6];                           /* ticks between host polls: 0,1,2,3,4,5+ */
    int32_t  last_host_t_tick;   /* tick of the last host 'T' frame (poll) received */
    int32_t  last_t_tick;
    int      silence_forced, max_silence;
    int      t_out, k_out;

    /* NI send */
    ni_sender ni; int ni_built; int ni_done;
    int      has_ni_awaiting; int ni_aw_state, ni_aw_n, ni_aw_phase;
    int      ni_wait_ticks;
    ni_receiver ni_recv;
    int      has_cur_ni_ack; uint8_t cur_ni_ack[2]; int cur_ni_ack_len;
    int      has_ni_ack_seq; uint16_t ni_ack_seq;
    int      has_ni_ack_bytes; uint8_t ni_ack_bytes[2]; int ni_ack_bytes_len;
    int      has_emitted_ni_ack; uint8_t emitted_ni_ack[2]; int emitted_ni_ack_len;
    int      host_uni_seen;
    int      host_ni_null_seen;
    int      has_host_ni_ack_state; int host_ni_ack_state;
    int      host_ni_repeat;
    int      has_host_ni_ack_key; int hnak_state, hnak_n, hnak_phase;
    int      host_ni_ack_repeat;
    int      has_host_ni_resend; uint8_t host_ni_resend[NI_SLOT_MAX]; int host_ni_resend_len;
    int      null_reemits, null_ticks;
    int      ni_status_logged;
    int      ni_rejected;
    int      host_disconnected;
    int      close_out_seen; int32_t close_out_tick;   /* our engine emitted READY_CLOSE_LINK */
    int      host_close_seen; int32_t host_close_tick;  /* the host's own READY_CLOSE_LINK row arrived */
    int      leave_d;                                    /* opt-in: send the child 'D' after a host-led close */
    int      close_by_host;                              /* ... before ours: the host started the close */
    int      d_sent; int32_t d_tick;                    /* our child 'D' was queued */

    /* reliable */
    uint16_t out_seq;
    struct { uint16_t dst; uint16_t ctr; int used; } pktid_map[SIM_PKTIDMAP];
    uint64_t nonce;
    uint16_t last_in_seq;
    rel_link_t rel;
    int      rel_opened;
    int      ack_owed;
    int32_t  last_ack_tick;
    int32_t  tick_count;
    int      has_connect_id; uint8_t connect_id[16]; int connect_id_len;
    int      gba_conn_sent, gba_accepted;
    int      slot_credit;
    int32_t  last_seat_emit;
    uint16_t seen_in[SIM_SEENRING]; int seen_in_n;

    /* carry-forward: ring of last CARRY_DEPTH data batches */
    sim_cf_t carry[SIM_CARRY_DEPTH][SIM_RELIABLE_BATCH_MAX];
    int      carry_len[SIM_CARRY_DEPTH];
    int      carry_count;   /* how many slots of the ring are populated (<=CARRY_DEPTH) */
    int      carry_head;    /* next write index (ring) */

    int      learned;
    uint16_t our_var, host_var;

    /* inbound queue (drained by sim_tick, matching Python's recv-inside-tick order) */
    struct { uint8_t dg[SIM_MAXDG]; int len; uint8_t src[4]; } inq[4];
    int inq_head, inq_n;

    /* runtime window: local trade keeps the console-faithful 24/2; the online path (RP2040 across a
     * relay, Switch side on a PC host link) runs 6/1 because the Switch stops acking past a single lost packet
     * once more than ~6 are unacked (GB-Link pia_link.c sends new data only while < 6 are outstanding). Set
     * with sim_set_window() any time; defaults from SIM_MAX_INFLIGHT / SIM_SILENCE_OVERRIDE_CEIL in sim_init(). */
    int max_inflight;
    int silence_ceil;

    /* diagnostics */
    int rx_count, tx_count, rx_fail;
} sim_t;
#define SIM_INQ 4

/* our_ip/host_ip 4 bytes; conn must be initialised by the caller (pc_conn_init) and its our_var set.
 * connect_id may be NULL. */
void sim_init(sim_t *s, const pia_crypto *crypto, sim_engine *engine,
              sim_send_fn send, void *send_ctx,
              const uint8_t our_ip[4], const uint8_t host_ip[4], pc_conn_t *conn,
              const uint8_t *connect_id, int connect_id_len, uint64_t nonce_seed);

/* Enqueue an inbound datagram; sim_tick() drains it (with the post-increment tick, like Python). */
void sim_feed_datagram(sim_t *s, const uint8_t *dg, int dg_len, const uint8_t src_ip[4]);
/* Handle one inbound datagram immediately. Returns 1 if it was a valid Pia packet. */
int  sim_process_datagram(sim_t *s, const uint8_t *dg, int dg_len, const uint8_t src_ip[4]);
/* Advance one VBlank: increment tick, drain the inbound queue, then send. */
void sim_tick(sim_t *s);
/* answer what just arrived (K ack + at most credit-many game frames) without advancing the clock. Steady state only. */
void sim_reply_now(sim_t *s);
/* Kizuna: advance the sim clock by n VBlanks WITHOUT sending. For a caller whose real-time
 * tick loop fell behind (a stall): the RTO/ack timers (all read tick_count) catch up with wall time, but no
 * burst of back-to-back UP slots is produced for the missed periods -- those would land inside one console
 * poll (see SIM_UP_BURST_MAX). Inbound datagrams queued meanwhile are left for the next real sim_tick. */
void sim_skip_ticks(sim_t *s, int n);
/* reliable UP window + starvation-override ceiling at runtime. max_inflight 1..REL_MAX_INFLIGHT,
 * silence_ceil >= 1 (1 = the override never sends past the window). Local default 24/2, online 6/1. */
void sim_set_window(sim_t *s, int max_inflight, int silence_ceil);
int  sim_connected(const sim_t *s);
/* Allocate the sim's ~13.6 KB of scratch up front (it is otherwise allocated on the first send). 1 = ok. */
int  sim_prealloc(void);
void sim_release(void);   /* free the batch scratch sim_prealloc took (stopped sims only) */

#endif
