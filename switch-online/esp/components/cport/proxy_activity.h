/* proxy_activity.h — a pia_activity that does NOT play the trade partner itself: it ships every child UNI row it is fed
 * to a REMOTE engine (over the BLE data characteristic -> phone -> relay) and plays back the parent rows that come back.
 * Pure/portable C (no ESP-IDF): the BLE transport is a callback. The wire contract is shared with the phone app and the
 * remote engine:
 *
 *   UP    adapter->remote  [0x01][seq u16 LE][count u8][count x 14-byte child rows]   (feeds the remote engine; no answer per row)
 *   TICK  adapter->remote  [0x04][idx u16 LE][count u8][0x00]   exactly 5 bytes: `count` (1..8) local activity->tick() calls, idx = index
 *                           of the first of them (every tick call since session start, wrapping u16). The remote engine is ticked
 *                           `count` times and answers each tick with exactly one DOWN row.
 *   DOWN  remote->adapter  [0x02][seq u16 LE][count u8][count x 14-byte parent rows]   (row i answers TICK index seq+i; seq = tick index)
 *   CTRL  both ways        [0x03][code u8][0][0][0]
 *         1 HELLO (adapter->remote: fresh engine) 2 CLOSE (either) 3 PEER_DOWN (phone->adapter) 4 ESTABLISHED
 *         (remote->adapter) 5 DISCONNECT_REQUESTED (remote->adapter) 6 DISCONNECT_SENT (adapter->remote) 7 DONE
 *   seq/idx = index of the first row (UP) or tick (TICK, DOWN) in the message, counting since session start, wrapping u16; at most 8 per
 *   message (<= 116 bytes). A proxy message is never exactly 4 bytes (4-byte data writes are the relay_ping echo).
 *
 * BUNDLES (contract amendment): every BLE notification, BLE write and relay payload carries one or SEVERAL complete messages back to
 * back. Each message is self-delimiting (UP/DOWN: 4 + 14*count with count in byte 3; TICK/CTRL: exactly 5 bytes), receivers loop until
 * the buffer is consumed, a trailing partial or unknown kind is malformed (the rest is dropped and counted). A bundle never exceeds
 * 240 bytes; the ping stays an exact 4-byte message and never rides in a bundle. This module sends one bundle per flush:
 * [HELLO][UP...][TICK...][other CTRL], paced to at most one per flush_ms (default 15) or at once when it reaches 200 bytes. A failed
 * send keeps everything pending and is retried on the next flush call (counted in ble_retries, never silently dropped).
 *
 * Threading: px_on_data() is called on the BLE host task, everything else on the session task. The DOWN ring and the
 * received-CTRL flags are the only shared state (single producer / single consumer, GCC __atomic acquire/release);
 * every counter has exactly one writer. */
#ifndef PROXY_ACTIVITY_H
#define PROXY_ACTIVITY_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "pia_activity.h"

#define PX_ROW_LEN   14
#define PX_MAX_ROWS  8
#define PX_MAX_MSG   (4 + PX_MAX_ROWS * PX_ROW_LEN)
#define PX_BUNDLE_MAX      240          /* hard cap on any bundle */
#define PX_BUNDLE_FLUSH_AT 200          /* a bundle this full is sent at once, without waiting for flush_ms */
#define PX_FLUSH_MS_DEFAULT 15
#define PX_EXTRA_STEP_MS    8           /* pacing added per congestion episode (a notify refused after the previous one went through) */
#define PX_EXTRA_MAX_MS     60
#define PX_UP_RING   64
#define PX_DOWN_RING 64

enum { PX_MSG_UP = 1, PX_MSG_DOWN = 2, PX_MSG_CTRL = 3, PX_MSG_TICK = 4 };
enum {
    PX_CTRL_HELLO = 1, PX_CTRL_CLOSE = 2, PX_CTRL_PEER_DOWN = 3, PX_CTRL_ESTABLISHED = 4,
    PX_CTRL_DISCONNECT_REQUESTED = 5, PX_CTRL_DISCONNECT_SENT = 6, PX_CTRL_DONE = 7,
};
#define PX_TICK_PENDING_MAX 64
/* Credit window: TICK credits sent whose DOWN row has not yet been played (or declared lost). Below the DOWN ring size, so a burst of
 * answers after a Bluetooth/relay stall can never overflow it; at 60 ticks/s it still covers an 0.9 s round trip at full speed. */
#define PX_CREDIT_WINDOW    56
#define PX_CREDIT_STALL_TICKS 600   /* ticks (~10 s) with the window full and nothing played before the outstanding credits are written off:
                                     * long enough that a Bluetooth/relay stall (late but intact answers) is not mistaken for loss */
#define PX_DOWN_SHED_AT       (PX_DOWN_RING - 8)   /* ring depth from which all-zero (idle) DOWN rows are shed instead of queued */
#define PX_PREFILL            3     /* DOWN rows to hold back before (re)starting playback, so one late bundle does not inject idle rows */
#define PX_PREFILL_MAX_WAIT   6     /* ticks to wait for the prefill when some rows are queued but fewer than PX_PREFILL */

/* One attempt, non-blocking; false when the transport could not take it now (the activity keeps the data and retries on the next flush). */
typedef bool (*px_send_fn)(void *ctx, const uint8_t *bundle, size_t len);

typedef struct px_activity {
    px_send_fn send;
    void      *send_ctx;

    /* session task */
    uint8_t    up[PX_UP_RING][PX_ROW_LEN];
    uint16_t   up_head;          /* ring index of the oldest unsent row */
    uint16_t   up_count;         /* rows waiting */
    uint16_t   up_first_seq;     /* seq of the oldest unsent row */
    uint16_t   up_next_seq;      /* seq the next fed row gets */
    uint16_t   tick_first_idx;   /* tick index of the oldest unsent credit */
    uint16_t   tick_pending;     /* tick credits waiting (saturates at PX_TICK_PENDING_MAX) */
    uint16_t   tick_next_idx;    /* index the next credit gets */
    uint32_t   ctrl_pending;     /* bit n = CTRL code n still to send */
    bool       hello_sent;
    bool       retry;            /* the last send failed: try again on the next flush regardless of pacing */
    bool       flushed_once;
    bool       primed;           /* playback is running (prefill satisfied) */
    uint32_t   flush_ms;         /* minimum spacing between bundles (the floor) */
    uint32_t   link_itvl_ms;     /* negotiated BLE connection interval, 0 = unknown */
    bool       match_itvl;       /* pace to max(flush_ms, link interval): one bundle per connection event is all a 1-packet link carries */
    uint32_t   flush_extra;      /* congestion backoff added to the spacing: +PX_EXTRA_STEP_MS per refused-notify episode, decays on success */
    uint32_t   ok_streak;
    uint32_t   last_flush_ms;
    uint8_t    prefill_wait;

    /* BLE task writes, session task reads */
    uint8_t    down[PX_DOWN_RING][PX_ROW_LEN];
    uint32_t   down_head;        /* free-running producer count */
    uint32_t   down_tail;        /* free-running consumer count */
    uint32_t   flags;            /* bit n = CTRL code n received */
    uint32_t   down_started;     /* 1 once the first DOWN row arrived */
    uint16_t   down_expected;    /* producer only: seq the next DOWN row should carry */
    uint32_t   down_lost;        /* producer only: DOWN rows that never came (gaps), written off against the credit window */

    /* counters, one writer each */
    uint32_t   up_rows, up_overflow, ctrl_tx;                                 /* session task */
    uint32_t   bundles_tx, bundle_bytes_tx;                                   /* session task: bundles actually sent */
    uint32_t   ble_retries;                                                   /* session task: failed send attempts (data kept, retried) */
    uint32_t   ble_drops;                                                     /* session task: data abandoned (UP ring overflow) */
    uint32_t   underruns, prefill_idle;                                       /* session task: idle rows injected mid-stream (ring empty / refilling) */
    uint32_t   ticks, tick_msgs, tick_overflow, credits_sent;                 /* session task */
    uint32_t   stall_ticks, credit_resets;                                    /* session task */
    uint32_t   down_rows, gaps, dups, down_overflow, bad_msgs, ctrl_rx;       /* BLE task */
    uint32_t   idle_shed, bundles_rx, oversize_rx;                            /* BLE task */
} px_activity;

void px_init(px_activity *px, px_send_fn send, void *send_ctx);
void px_set_flush_ms(px_activity *px, uint32_t ms);
/* Tell the activity the negotiated connection interval (ms, 0 = unknown). With match_itvl on (the default) bundles are paced to at least
 * one per interval: flushing faster than the link drains only builds a standing queue in the BLE stack, which adds delay and no throughput. */
void px_set_link_itvl_ms(px_activity *px, uint32_t ms);
void px_set_match_itvl(px_activity *px, bool on);
uint32_t px_effective_flush_ms(const px_activity *px);   /* what pacing is actually in force right now */
pia_activity px_as_activity(px_activity *px);

/* BLE data write. Returns false when it is not a proxy buffer (exactly 4 bytes = the ping echo, or a first byte that is not DOWN/CTRL),
 * so the caller can offer it to its other handlers. A malformed proxy buffer is consumed (true), what precedes the bad message is
 * applied, and bad_msgs counts it. */
bool px_on_data(px_activity *px, const uint8_t *data, size_t len);

/* Queue a CTRL message; the next flush sends it. */
void px_ctrl(px_activity *px, uint8_t code);
/* Send at most a few bundles: HELLO first, then UP rows, TICK credits, other CTRL. Call at the end of every session-loop pass with a
 * millisecond clock; a bundle goes out when flush_ms has elapsed since the last one, when it is full, or when the last send failed. */
void px_flush(px_activity *px, uint32_t now_ms);
/* Same, ignoring the pacing (teardown, tests). */
void px_flush_now(px_activity *px);

bool px_flag(const px_activity *px, uint8_t code);     /* was CTRL `code` received */
int  px_down_depth(const px_activity *px);             /* DOWN rows waiting to be played */

#endif
