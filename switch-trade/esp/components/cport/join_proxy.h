/* join_proxy.h — the JOIN-role frame proxy. The Switch HOSTS the trade room and the adapter is the RFU child (cport sim.c); this
 * sim_engine does NOT play the GBA: it ships every host UNI frame the sim receives to a REMOTE follower engine (pokeldn TradeEngine behind
 * the relay, over BLE -> phone -> relay) and plays the child rows that come back. The mirror image of proxy_activity.h (host role).
 * Pure/portable C; the BLE transport is a callback. Wire contract, shared with the phone app (opaque pipe):
 *
 *   JUP   adapter->remote  [0x10][seq u16 LE][count u8][count x record]   count 1..8, seq = index of the first record (host T frames fed since
 *                           session start, wrapping u16). record = [nslots u8 0..5][mask u8: bit i = slot i is non-zero][14 B x popcount(mask)]
 *                           One record per host T frame fed to the engine after the first UNI frame (an all-idle frame or a host keepalive is
 *                           [nslots][0], 2 bytes), so the remote engine counts frames exactly like a local one.
 *   JTICK adapter->remote  [0x12][idx u16 LE][ticks u8][psd u8]   exactly 5 bytes. The remote runs engine.tick() `ticks` times, THEN
 *                           engine.poll_send_done() `psd` times; idx = index of the first tick (sim ticks since session start, wrapping u16).
 *                           The remote answers every tick with exactly one JDOWN row. A tick run followed by a psd run share one message;
 *                           a tick after a psd starts a new one, so the order of feeds / ticks / psds on the wire is the local order.
 *   JDOWN remote->adapter  [0x11][seq u16 LE][count u8][count x 14-byte child rows]   count 1..16; row i answers tick idx seq+i; an idle row is 14 zeros.
 *   CTRL  both ways        [0x03][code u8][0][0][0]   (shared with the host-role proxy; codes 1..7 as there)
 *         2 CLOSE (either)  3 PEER_DOWN (phone->adapter)  7 DONE   plus, join role only:
 *         8 JOIN_HELLO (adapter->remote, first message: a fresh follower, session start)
 *         9 JOIN_ACCEPTED (adapter->remote: the human tapped yes on the Switch)
 *         10 JOIN_UNI_UP (adapter->remote: the first UNI frame arrived)
 *         11 REMOTE_DONE (remote->adapter: its engine has finished the trades; informational)
 *   A message is never exactly 4 bytes (the 4-byte data write is the relay_ping echo).
 *
 * BUNDLES: as in proxy_activity.h — every BLE notification, write and relay payload carries one or several complete messages back to back,
 * each self-delimiting, <= 240 bytes; this module sends one bundle per flush, paced to max(flush_ms, link interval) + congestion backoff,
 * retried (not dropped) when the transport refuses. The three outbound kinds ride in the ORDER the sim produced them (feed, tick, feed...).
 *
 * Threading: jp_on_data() runs on the BLE host task; everything else on the brain task (one task: feeds, ticks and flushes never race).
 * The DOWN ring and the received-CTRL flags are the only shared state (SPSC, GCC __atomic acquire/release). */
#ifndef JOIN_PROXY_H
#define JOIN_PROXY_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "sim.h"

#define JP_ROW_LEN        14
#define JP_MAX_SLOTS      5
#define JP_MAX_REC        8                 /* records per JUP message */
#define JP_MAX_DOWN       16                /* rows per JDOWN message */
#define JP_BUNDLE_MAX     240
#define JP_BUNDLE_FLUSH_AT 200
#define JP_FLUSH_MS_DEFAULT 15
#define JP_EXTRA_STEP_MS  8
#define JP_EXTRA_MAX_MS   60
#define JP_OPS_BYTES      1536              /* outbound op log (records + ticks + psds), ~0.5 s of a stalled link */
#define JP_DOWN_RING      64
#define JP_CREDIT_WINDOW  56
#define JP_CREDIT_STALL_TICKS 600
#define JP_DOWN_SHED_AT   (JP_DOWN_RING - 8)
#define JP_PREFILL        3
#define JP_PREFILL_MAX_WAIT 6
#define JP_BACKLOG_AT     12                /* DOWN depth from which has_backlog asks the sim for a 2-row catch-up burst */

enum { JP_MSG_CTRL = 3, JP_MSG_JUP = 0x10, JP_MSG_JDOWN = 0x11, JP_MSG_JTICK = 0x12 };
enum {
    JP_CTRL_CLOSE = 2, JP_CTRL_PEER_DOWN = 3, JP_CTRL_DONE = 7,
    JP_CTRL_JOIN_HELLO = 8, JP_CTRL_JOIN_ACCEPTED = 9, JP_CTRL_JOIN_UNI_UP = 10, JP_CTRL_REMOTE_DONE = 11,
};
#define JP_CTRL_MAX 11

typedef bool (*jp_send_fn)(void *ctx, const uint8_t *bundle, size_t len);
/* Diagnostics: kind 'h' = a non-idle host record (idx = its JUP index, bytes = the record encoding), 'c' = a non-zero child row played to the Switch
 * (idx = its number in play order, bytes = the 14-byte row). Called on the brain task. */
typedef void (*jp_trace_fn)(void *ctx, char kind, uint16_t idx, const uint8_t *bytes, size_t len);

typedef struct jp_activity {
    jp_send_fn send;
    void      *send_ctx;
    jp_trace_fn trace;
    void      *trace_ctx;
    uint32_t   trace_host_left, trace_child_left;   /* how many more of each kind to report (0 = off) */
    uint16_t   played_idx;                          /* tick index of the next row to be played (counts every DOWN row consumed) */

    /* brain task: outbound op log, a byte ring of FEED [0xF0][nslots][mask][slots] / TICK [0xF1] / PSD [0xF2] items */
    uint8_t    ops[JP_OPS_BYTES];
    uint16_t   ops_head, ops_tail;           /* byte indexes; empty when equal */
    uint16_t   feed_seq, tick_next_idx;      /* index the next enqueued record / tick gets */
    uint16_t   sent_feed_seq, sent_tick_idx; /* first index not yet committed to a bundle */
    uint32_t   ctrl_pending;
    bool       hello_sent, retry, flushed_once, primed, uni_seen;
    uint32_t   flush_ms, link_itvl_ms, flush_extra, ok_streak, last_flush_ms;
    bool       match_itvl;
    uint8_t    prefill_wait;
    uint32_t   credits_sent;                 /* ticks enqueued (each is owed one DOWN row) */

    /* BLE task writes, brain task reads */
    uint8_t    down[JP_DOWN_RING][JP_ROW_LEN];
    uint32_t   down_head, down_tail, flags, down_started;
    uint16_t   down_expected;
    uint32_t   down_lost;

    /* counters, one writer each */
    uint32_t   rec_fed, ticks, psds, tick_skipped, ops_overflow;               /* brain */
    uint32_t   bundles_tx, bundle_bytes_tx, ble_retries, ctrl_tx;              /* brain */
    uint32_t   underruns, prefill_idle, stall_ticks, credit_resets, rows_played;   /* brain */
    uint32_t   host_frames;                  /* brain: every host T frame the sim handed us (UNI, keepalive or not) */
    uint32_t   nonuni_drop;                  /* brain: host T frames that were neither UNI nor keepalive (dropped, not proxied) */
    int        last_state, last_slot_len, last_drop_sig;   /* brain: the host's latest LLSF state / slot_len; signature of the last dropped frame */
    uint32_t   down_rows, gaps, dups, down_overflow, bad_msgs, ctrl_rx;        /* BLE */
    uint32_t   idle_shed, bundles_rx, oversize_rx;                             /* BLE */
} jp_activity;

void jp_init(jp_activity *jp, jp_send_fn send, void *send_ctx);
/* New session on the same object: clears the op log, the DOWN ring, indexes and flags (counters and pacing kept), and queues JOIN_HELLO. */
void jp_session_begin(jp_activity *jp);
void jp_set_flush_ms(jp_activity *jp, uint32_t ms);
void jp_set_link_itvl_ms(jp_activity *jp, uint32_t ms);
void jp_set_match_itvl(jp_activity *jp, bool on);
uint32_t jp_effective_flush_ms(const jp_activity *jp);
void jp_set_trace(jp_activity *jp, jp_trace_fn fn, void *ctx, uint32_t host_n, uint32_t child_n);

/* Fill a sim_engine whose tick / feed_in_frame / poll_send_done / has_backlog are this proxy (identity left at the sim defaults). */
void jp_as_engine(jp_activity *jp, sim_engine *e);

bool jp_on_data(jp_activity *jp, const uint8_t *data, size_t len);   /* false: not a join-proxy buffer (ping echo, relay slot bytes, ...) */
void jp_ctrl(jp_activity *jp, uint8_t code);
void jp_flush(jp_activity *jp, uint32_t now_ms);
void jp_flush_now(jp_activity *jp);
bool jp_flag(const jp_activity *jp, uint8_t code);
int  jp_down_depth(const jp_activity *jp);

/* Encode / decode one record, exposed for the tests and the e2e model. out needs 2 + 14*5 bytes; returns the length. */
size_t jp_encode_record(const gba_in *rec, uint8_t *out);

#endif
