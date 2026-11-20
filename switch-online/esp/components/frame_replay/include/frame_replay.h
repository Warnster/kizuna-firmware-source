/* frame_replay -- replay relayed held-keys frames on the receiver's own frame clock.
 *
 * The walking phase of a FRLG wireless room relays SEND_HELD_KEYS (0xBE00) slots: one per VBlank, the key byte in
 * word 1's low byte and a rolling frame counter in its high byte. The information is in the *run lengths*: a press is
 * N identical frames. Relayed over the internet (adapter -> relay -> phone -> adapter and back) the frames arrive
 * bunched, and the old receivers emitted them as they arrived (and folded a backlog), so the press lengths the far
 * console saw were the arrival rhythm, not the recorded one: characters turned instead of stepping and drifted.
 *
 * This module holds frames for a fixed delay and releases them by their counter against the receiver's own clock
 * (Switch side: wall time at 60 Hz, one slot per Switch poll; GBA side: one call per GBA frame). A 12-frame press on the
 * GBA is a 12-frame press on the Switch, `delay` frames later. Rules:
 *   - The 8-bit wire counter is unwrapped on arrival into a monotonic sequence (consecutive frames are never more than a
 *     few counts apart), so a frame's due time is base_due + (ext - base_ext) however deep the queue gets. (Judging depth
 *     with the raw counter broke down past ~127 queued frames: bench, 191 frames "not yet due".)
 *   - A new timeline starts (anchor = now + delay) on the first keys frame after the queue has been empty for
 *     FR_GAP_REBASE frames, after a non-keys frame, or when a frame sits further than 2*delay in the future.
 *   - A frame later than `delay` behind schedule (a network stall longer than the buffer) is emitted at once and
 *     becomes the new anchor: the burst behind it then plays at the right rhythm, `stall` later. Nothing is folded.
 *   - When several frames fell due at once (a missed or deferred poll, a late burst) the newest state wins and timing
 *     stays exact: a due frame is dropped (`collapsed`) if its key is the one already shown (a continuation) or the same as
 *     the frame behind it (an identical state). Only a one-frame event, a key unlike both neighbours, is emitted itself,
 *     and then the frame behind it goes out on the next pop, one frame late (`skipped`). The game steps in 16-frame units,
 *     so a key change one frame off flips a step when the press ended on a boundary (bench, "3 steps become 4").
 *   - Adaptive depth: `delay` starts at the configured value (the floor) and grows by FR_DELAY_GROW on every late arrival,
 *     up to FR_DELAY_MAX; after FR_DELAY_DECAY frames without one it shrinks by one frame. Lead control then moves the
 *     timeline to the new target on idle frames, so the buffer sizes itself to the link's jitter instead of guessing.
 *   - Backlog drain: when more than delay + FR_DRAIN_MARGIN frames are queued (a stall just ended, or the consumer polled
 *     slowly for a while) and the frame being emitted is a no-key idle, the idle run ahead of it is cut: its idle frames
 *     are dropped (the hold repeat shows the same keys anyway) and the timeline moves earlier by as many frames, so the
 *     latency comes back within one pause instead of one frame per idle. Pressed runs are never shortened.
 *   - Lead control: every queued frame remembers when it arrived; `lead` = due - arrived is how long it waited, and its
 *     running average is kept near `delay`. The two clocks never agree exactly (GBA 59.73 Hz, Switch 60 Hz, the GBA side's
 *     frame counter ran at 61.5/s on the bench), and a stall leaves its length behind as extra lead. So when a no-key
 *     idle frame (FR_IDLE_KEY) is emitted the timeline is slewed by ONE frame: later if the average lead fell under
 *     delay/2 (`slewed_late`), earlier if it rose over 3*delay/2 (`catchup`). Only an idle run ever changes length,
 *     by one frame at a time, so positions stay exact. Pressed frames are never dropped or stretched.
 *   - Any other opcode (blocks, standby, close) is emitted in arrival order, at once when it reaches the head.
 * Pure C, no allocation, no RTOS: the caller serialises calls. Host-tested in host_tests/. */
#pragma once
#include <stdint.h>
#include <stdbool.h>

#define FR_N             192   /* frames held (18 B each): ~3 s at 60 Hz. The Switch stops polling in menus and at the trade PC while the
                                  * GBA keeps walking; 1 s overflowed (bench: drops=26 pressed frames in one session). */
#define FR_DELAY_DEFAULT 20    /* frames: ~333 ms, covers the bench's bursts; {"op":"jitter","frames":N} changes it */
#define FR_GAP_REBASE    120   /* queue empty this many frames (2 s) -> the next keys frame starts a fresh timeline */
#define FR_DELAY_MAX     90    /* adaptive ceiling: 1.5 s */
#define FR_DELAY_GROW    6     /* frames added to the buffer per late arrival (100 ms) */
#define FR_DELAY_DECAY   600   /* frames between one-frame shrinks when nothing is late (10 s) */
#define FR_DRAIN_MARGIN  12    /* backlog above delay+this is drained through idle runs (see catch-up) */
#define FR_DISCONT       600   /* an unwrapped counter this far from the anchor (10 s) is a new stream */
#define FR_IDLE_KEY      0x11  /* key byte of a held-keys frame with nothing pressed (ldn_brain.c "plain no-key slot") */

typedef struct {
    uint16_t q[FR_N][7];
    int32_t  arr[FR_N];                  /* receiver time each frame arrived */
    int32_t  ext[FR_N];                  /* the frame's counter unwrapped to a monotonic sequence (8-bit on the wire) */
    bool     have_ext; uint8_t last_ctr; int32_t last_ext;
    int head, tail;                      /* empty when head == tail */
    int32_t lead_avg16;                  /* running average of (due - arrived), in 1/16 frame */
    int delay;                           /* current buffer depth, frames */
    int delay_floor;                     /* configured minimum (fr_init / fr_set_delay) */
    int32_t last_late;                   /* receiver time of the last late arrival (for the decay) */
    bool based;
    int32_t base_ext;                    /* unwrapped counter of the anchor frame */
    int32_t base_due;                    /* receiver time at which the anchor frame is (was) due */
    int32_t empty_since;                 /* receiver time the queue last became empty */
    uint32_t enq, deq, drops, skipped, rebased, late, catchup, slewed_late, shed_idle, collapsed;
    int last_key;                        /* key byte of the last held-keys frame emitted (-1 = none) */
} frame_replay_t;

void fr_init(frame_replay_t *r, int delay_frames);
void fr_set_delay(frame_replay_t *r, int delay_frames);
bool fr_is_keys(const uint16_t w[7]);                       /* a SEND_HELD_KEYS slot (word 0 high byte 0xBE) */
/* Queue one relayed frame; `now` = the receiver's frame clock when it arrived. When full, an idle frame that sits between
 * two idle frames is removed first (`shed_idle`: shortens an idle run by one, moves nobody); only if there is none does the
 * oldest frame go (`drops`). The consumer stops popping while the far console stops polling (menus, the trade PC), so a
 * long pause must not cost pressed frames. */
void fr_push(frame_replay_t *r, const uint16_t w[7], int32_t now);
/* The frame to emit at receiver time `now`, if one is due. false = emit nothing (hold / idle). */
bool fr_pop(frame_replay_t *r, uint16_t out[7], int32_t now);
int  fr_depth(const frame_replay_t *r);
/* Frames at the head that are already due (the receiver is behind by this many). */
int  fr_due_depth(const frame_replay_t *r, int32_t now);
