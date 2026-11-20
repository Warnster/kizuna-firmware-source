/* SPDX-License-Identifier: AGPL-3.0-only
 * Derived from frlg-ldn-trade (https://github.com/tornadus/frlg-ldn-trade), Copyright (c) the frlg-ldn-trade authors.
 * Derived from frlg-ldn-trade-esp32 (https://github.com/easyworld/frlg-ldn-trade-esp32), Copyright (c) Easy World and the frlg-ldn-trade-esp32 authors.
 * Changes Copyright (c) 2026 Kizuna. See NOTICE.
 */
/* gba_bridge.c — the GBA-adapter <-> LDN-sim trade seam. Stage 2b: the parent-side RFU data-phase relay
 * toward the real GBA, ported from the S3's proven gba_relay.c.
 *
 * The C6 is the PARENT toward the GBA. The GBA (child) sits at its join screen ("Le") polling
 * ReceiveData(0x26); librfu's RFUSTATE_CHILD_TRY_JOIN has NO timeout of its own, but its NI-recv slot
 * faults to RECV_FAILED (~139 idle frames) -> JOIN_GROUP_NO -> disconnect if we ever go silent/incomplete.
 * So every 0x26 we serve the COMPLETE 5-subframe join-status NI (re-looping as keepalive), with the status
 * byte held at 0x00 ("still waiting", ignored by TRY_JOIN) until the real Switch approves (host_uni_seen ||
 * ni_recv.status==JOIN_GROUP_OK), then 0x05 (JOIN_GROUP_OK) -> the child advances into UNI trade, where we
 * stream the parent UNI frame (row0 = Switch slot, row1 = echo of the GBA's own last slot) every frame.
 *
 * Threading: on_gba_cmd + build_recv both run on the GBA task (command phase) -> the NI machine state is
 * single-threaded, no lock. The Switch slot (eng_feed, brain task) + GBA slot (eng_tick, brain task) cross
 * tasks -> portMUX. join_released is a volatile flag (brain writes, GBA task reads). */
#include "gba_bridge.h"
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "frame_replay.h"   /* replay of relayed held-keys frames on the Switch's clock */

/* Per-frame and heartbeat bench lines. Builds that define KZ_QUIET_TRACE (switch-online, the production online image)
 * compile them out; the arguments stay referenced so nothing becomes unused. */
#ifdef KZ_QUIET_TRACE
#define KZ_TRACE(...) do { if (0) printf(__VA_ARGS__); } while (0)
#else
#define KZ_TRACE(...) printf(__VA_ARGS__)
#endif

/* ---- parent NI join-status sequence (gba_relay.c s_ni_seq) ---- */
#define NI_LCOM_NULL  0
#define NI_LCOM_START 1
#define NI_LCOM_NI    2
#define NI_LCOM_END   3
typedef struct { uint8_t state, n, phase, size; uint8_t pay[5]; } ni_subframe_t;
static const ni_subframe_t s_ni_seq[5] = {
    { NI_LCOM_START, 1, 0, 5, {0x00, 0x05, 0x00, 0x01, 0x00} },
    { NI_LCOM_START, 2, 0, 2, {0x00, 0x00} },
    { NI_LCOM_NI,    1, 0, 1, {0x05} },                 /* payload[0] overwritten with the live status */
    { NI_LCOM_END,   0, 0, 0, {0} },
    { NI_LCOM_NULL,  1, 0, 0, {0} },
};

/* GBA-task-only NI machine state (no lock needed) */
static int     s_ni_stage, s_ni_null_served;
/* childf fix (tr-1.01) — the PARENT join-status NI (JOIN_GROUP_OK) send. s_join_ni_stage walks s_ni_seq[0..3]
 * = START(5B ctrl) / START(2B ctrl) / NI[0x05] / END, one subframe per CONFIRMED-served 0x26 poll. */
static int     s_join_ni_stage;     /* 0..3 index into s_ni_seq */
static bool    s_join_ni_done;      /* END subframe delivered -> child now advances to CHILD_JOINED (UNI) */
static bool    s_join_ni_staged;    /* build_recv staged a join-NI subframe this pull (recv_advance gate) */
static bool    s_ni_join_ok_sent;   /* have we emitted the 0x05 status subframe since release? */
static bool    s_player_ids_sent;   /* tr-1.06: has SEND_PLAYER_IDS been delivered yet this attempt? one-shot -
                                     * see build_recv's UNI branch. Resending it is NOT idempotent (decomp:
                                     * every LCOM_UNI frame is unconditionally treated as new, newDataFlag=1
                                     * every time, librfu_rfu.c:2067-2105) -> repeats re-trigger game handling. */
static bool    s_ni_held;           /* NI cycle delivered once, not yet approved -> HOLD with empty 0x26 replies
                                     * (keep the child polling, per librfu count==0) instead of re-looping the
                                     * complete NI (which the child completes repeatedly -> expects the next phase
                                     * -> resets + re-logs = the tr-0.74 join-hold re-login). Cleared on approval. */
static uint8_t s_child_ack_state, s_child_ack_n, s_child_ack_phase;
static bool    s_child_ack_pending;
static unsigned s_child_acks_sent, s_ni_child_frames;

static volatile bool s_join_released;   /* brain sets (Switch approved); GBA task reads in ni_emit */
/* tr-1.15 (supersedes tr-1.14's debounce, REJECTED by user -- "we should wait for the switch to have sent the
 * approval back", not a fake timer): user directly observed on real hardware that our join-approved signal
 * fires before a human could possibly have clicked the Switch's real accept/decline prompt. Root cause found
 * in ldn_brain.c: `approved` was `host_uni_seen || ni_recv.status==JOIN_GROUP_OK` -- but this project's OWN
 * build log (tr-0.25) documents host_uni_seen's ORIGINAL observed sequence as "NI status ->
 * host_uni_seen -> Switch shows the trade join Yes/No prompt" -- i.e. host_uni_seen was ALWAYS known to fire
 * BEFORE the prompt even appears, let alone gets answered. It was never a valid accept signal; nobody caught
 * it because earlier testers clicked fast enough for it not to matter. FIX is in ldn_brain.c: `approved` now
 * depends ONLY on ni_recv.status==NI_STATUS_JOIN_GROUP_OK, the field the Switch actually uses to carry its
 * real yes/no decision (ni.c:169 refreshes it from each received NI subframe; a non-OK value already trips
 * ni_rejected downstream, sim.c:309 -- so "no" was already handled, we just weren't waiting for the real "yes").
 * s_join_select_us kept for DIAGNOSTIC LOGGING ONLY (how long the real approval actually took) -- it is NOT a
 * gate on anything below. */
static int64_t s_join_select_us;
static volatile bool s_gba_ni_done;     /* diag: GBA sent NI_END (its NI send done) — NOT safe-for-UNI yet */
static volatile bool s_gba_uni_started; /* tr-0.94: GBA sent a UNI slot (state 4) -> CHILD_JOINED, recv buffer up
                                         * -> NOW safe to send PARENT UNI. This is the real gate (NI_END was too early). */

/* cross-task slots */
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static uint16_t s_switch_words[7]; static int s_switch_len;   /* eng_feed -> row0 (Switch's slot) */
static uint16_t s_gba_words[7];     static bool s_gba_valid;   /* on_gba_cmd -> row1 echo (latest, stable) */
/* tr-1.26 (echo-once latch, now a FIFO): the real parent echoes each child slot in exactly ONE downstream
 * frame: the parent-recv path copies childRecvBuffer into gRecvCmds then ZEROES the buffer (link_rfu_2.c:
 * 900-906), and MoveSendCmdToRecv zeroes gSendCmd after copying (:759-767) -- so every command occupies one
 * frame, rows are zero otherwise. Once-only echo also prevents a stale barrier/held-keys slot leaking into a
 * later round. (Replaces tr-1.09's s_uni_new_pending "count=0 unless changed" gate -- see build_recv's tr-1.26
 * note for why that gate froze the child's whole game engine.)
 *
 * Kizuna ECHO FIFO: the latch held only the LATEST slot. On the single-chip C6 that was
 * exact (one GBA slot per 0x26 poll per build_recv). On the two-chip board GBA slots arrive over the RP2040
 * UART at the GBA's own rhythm while build_recv runs on the brain loop, so two slots between two build_recv
 * calls lost the first slot's echo. The GBA then saw a hole in its own block bitmask and ran a blind repair
 * round (HandleSendFailure, link_rfu_2.c:1015: every M2 bench run shows 2-3 re-sent fragments plus 6-9
 * last-fragment holds vs 3 holds on the C6) -- extra non-idle slots toward the Switch for nothing. Rules
 * from pokeldn's working host (docs/frlg_link.md, rfu_leader.ChildEcho): never drop a distinct command;
 * coalesce a repeat that is still waiting (SendLastBlock re-sends the same fragment every frame while it
 * waits -- one queued copy is enough; a repeat arriving after its echo went out is a new question and is
 * answered). Tag-stripped content (the parent strips childSendCmdId before mirroring). */
#define ECHO_Q_N 8
static uint16_t s_eq[ECHO_Q_N][7];
static int      s_eq_head, s_eq_tail;                 /* empty when head==tail */
static unsigned s_eq_enq, s_eq_deq, s_eq_drops, s_eq_coalesced;
static inline void echo_clear(void) { s_eq_head = s_eq_tail = 0; }

/* BATTLE MODE: set when the Switch's room is a Colosseum (activity 1 single / 2 double). Trade rooms (4)
 * and unknown rooms (0) keep every rule in this file exactly as it was. In battle mode both consoles must move
 * every avatar from the same frames (hw the Switch dropped the room, or the GBA froze mid-battle,
 * when they disagreed), so nothing is merged in either direction:
 *   - up: the GBA's echo (row1) is queued on arrival, as in trade, so every echoed slot must also reach the
 *     Switch -- no held-keys merging in the up FIFO (128 slots);
 *   - down: every Switch slot reaches the GBA (no held-keys merging; the one-shot READY can no longer be eaten),
 *     with a 128-slot down FIFO instead of 32.
 * Echoing on forward instead was tried (hw run 7): the echo lagged, the GBA re-sent every block INIT and last
 * fragment 15-30 times (3 in trade), each repeat went to the Switch, and the GBA gave up before the battle. */
static bool s_battle;
void gba_bridge_set_battle(bool on) {
    if (on != s_battle) printf("  [bridge] relay mode: %s\n", on ? "BATTLE (docs/100)" : "trade");
    s_battle = on;
}
bool gba_bridge_battle(void) { return s_battle; }

/* UP-direction FIFO (GBA -> Switch). The Switch consumes ONE UNI slot per VBlank; the GBA emits one slot
 * per RFU frame. A plain snapshot (s_gba_words) forwarded EACH VBlank repeats stale slots and DROPS any GBA
 * slot produced between two eng_tick calls -> block fragments get duplicated/lost -> the Switch reassembles a
 * corrupt party -> crash on "ok". This ring forwards every captured GBA slot EXACTLY ONCE, in order; eng_tick
 * idle-fills when empty. (DOWN stays a snapshot: the GBA re-polls the SAME parent frame many times per frame,
 * so a stable latest Switch slot is the correct parent behaviour there.) */
#define GBA_SHED_AT 3   /* UP FIFO depth from which held-keys frames may fold (GB-Link PIA_OUT_SHED_AT) */
#define GBA_Q_N 128   /* was 32: overflowed (drop-oldest) in the held-keys room; drops=84 */
/* the UP path replays the GBA's held-keys frames on the Switch's 60 Hz clock (components/frame_replay) instead of
 * a FIFO that emitted them as they arrived and folded a backlog -- the arrival rhythm became the press rhythm the Switch
 * saw (characters turned instead of stepping, drifted). Clock = wall time in VBlanks; one pop per Switch poll. */
#ifdef KZ_ONLINE_PACING
static frame_replay_t s_up_fr;                         /* 4.3 KB: online only -- in the local image it moved the heap enough that
                                                        * the trade session's sim no longer fit after Bluetooth stopped */
#endif
static bool     s_up_fr_init;
static unsigned s_gq_coalesced;                        /* [upq] coal= : frames the replay skipped (consumer slower than 60 Hz) */
static int32_t  up_now(void) { return (int32_t)(esp_timer_get_time() / 16667); }
/* a real GBA child sends a SEND_HELD_KEYS slot EVERY frame, EMPTY (0x11) included, and the parent's whole
 * room simulation runs only on frames with a non-zero child command (pokefirered overworld.c CB1_UpdateLinkState ->
 * IsRfuRecvQueueEmpty; RfuRecvQueue_Enqueue drops all-zero frames). Sending an all-zero slot whenever no relayed frame was
 * due froze the Switch's simulation for that frame (~25% of polls on the bench): the Switch's world ran at ~45 Hz against
 * the GBA's 60 Hz and the two drifted. So when nothing is due, repeat the last held-keys state, as the GBA would. */
#define UP_HOLD_FRAMES 120                       /* stop repeating 2 s after the last real held-keys frame (menus, stalls) */
static uint16_t s_up_hold[7]; static bool s_up_hold_valid; static int32_t s_up_hold_t; static unsigned s_up_hold_repeats;
#ifdef KZ_ONLINE_PACING
static void     up_fr_reset(void) { int d = s_up_fr_init ? s_up_fr.delay : FR_DELAY_DEFAULT; fr_init(&s_up_fr, d); s_up_fr_init = true; }
#else
static void     up_fr_reset(void) { }
#endif
/* (battle): the once-only, in-order UP FIFO used instead of the replay while s_battle is set */
static uint16_t s_gq[GBA_Q_N][7];
static int      s_gq_head, s_gq_tail;                 /* empty when head==tail */
static unsigned s_gq_enq, s_gq_deq, s_gq_drops;       /* stats */
static uint16_t s_mir_w0, s_mir_w1;                     /* the host's latest non-zero row1 (its accepted copy of our slot) */
static unsigned s_mir_n; static int64_t s_mir_us;
/* Online asked for held-keys merging; main's merge (same key byte only, always on) does it and
 * keeps key changes. The setter stays for the rfu_host op; the merge is unconditional. */
void gba_bridge_set_coalesce_up_keys(bool on) { (void)on; }
static volatile bool s_gq_drop_flag; static volatile uint16_t s_gq_drop_w0, s_gq_drop_w1;   /* last dropped frame, for the log */

/* DOWN-direction FIFO (Switch -> GBA), tr-1.21 -- mirrors the UP FIFO above for exactly the same reason.
 * The old code kept a single shared SNAPSHOT (s_switch_words) and justified it with "the GBA re-polls the SAME
 * parent frame many times per frame, so a stable latest Switch slot is the correct parent behaviour". That
 * holds for IDLE/steady state, but NOT for a SEQUENCED BLOCK TRANSFER, where every fragment must reach the
 * child exactly once and in order. Hardware proof (tr-1.20 run, 03:44:45.663-45.981): once the UP direction
 * was fixed the Switch finally sent its COMPLETE block -- 8903,8904,...,8910 -- 15+ distinct SEND_BLOCK
 * fragments inside ~320ms, several only 2-4ms apart, while the GBA polls 0x26 only every ~37ms. A snapshot
 * silently discards roughly half of them, so the child can never reassemble the block, waits, and dies.
 * (This is the same failure the UP FIFO comment already warns about: dropped/duplicated fragments -> the peer
 * "reassembles a corrupt party".) Queue every Switch slot; drain exactly one per delivered 0x26. */
#define SW_Q_N 128      /* battle mode uses all of it */
/* KZ_ONLINE_PACING (switch-online only): the online relay's pacing -- the UP replay on the Switch's clock with the held-keys
 * repeat, DN_KEEP thinning and the 64-slot down ring (production switch-online). Local images (switch-trade) keep the local trade path of switch-trade-v2.4: the UP FIFO with the same-key held-keys merge and the 32-slot

 * down ring that replaces the newest queued held-keys slot. The online pacing on the local path stopped a Switch-led trade
 * at the room entry (bench: JOIN_GROUP_OK delivered, the GBA never walked in). Battle rooms (s_battle)
 * use their own once-only rules in both. */
#ifdef KZ_ONLINE_PACING
#define SW_Q_TRADE 64   /* online: room for a bunched drain */
#else
#define SW_Q_TRADE 32   /* local: switch-trade-v2.4 */
#endif
#define DN_KEEP 20  /* keepalive spacing (Switch frames) for an unchanged key byte toward the GBA */
static uint16_t s_dq[SW_Q_N][7];
static int      s_dq_head, s_dq_tail;                 /* empty when head==tail */
static unsigned s_dq_enq, s_dq_deq, s_dq_drops;       /* stats */
/* where each Switch slot (mpid 0) goes, so a stall shows whether the Switch sent nothing new or the bridge lost it */
static unsigned s_dn_in, s_dn_same, s_dn_zero, s_dn_fold, s_dn_be;
/* tr-1.22: the queue must be drained at the CHILD'S FRAME RATE, not per 0x26 poll. The child re-polls 0x26
 * MANY times per frame (the old snapshot comment was right about that part), and librfu treats EVERY state=4
 * delivery as unconditionally new (newDataFlag=1, librfu_rfu.c:2067-2105) -- so popping a fragment per poll
 * delivers several "new" frames inside one child frame and trips ERR_RECV_DATA_OVERWRITED (the tr-1.06/1.09
 * spam bug class, re-created by tr-1.21: the GBA died 0.5s after CHILD_JOINED instead of 6.2s).
 * Correct pacing = one parent frame per child frame: deliver the queue head ONCE, then serve count=0 until the
 * child emits its next UNI slot (proof it consumed ours and advanced a frame), then pop and arm the next. */
static bool s_dn_pending;      /* head not yet delivered this child-frame */

/* identity + join gate */
static uint16_t s_id_tid; static uint8_t s_id_name[8], s_id_lang, s_id_activity; static bool s_id_valid;
static volatile bool s_join_selected;

/* tr-1.51 POST-SAVE SAVE-CHAIN CONTINUATION (frlg-ldn-trade ground-truth proven). After the trade
 * commits (CONFIRM_FINISH 0xDCBA) the Switch is REACTIVE in the save-chain: it only ECHOES the child's
 * READY_EXIT_STANDBY (0x6600, word1=count) and RE-EXCHANGES (BLOCK_REQ 0xA100 / party re-pull) once it has
 * seen enough child rounds to cover its own save. Our real GBA emits a FIXED ~5 (CB2_SaveAndEndTrade) then
 * PARKS at its gated menu re-entry -> the Switch never gets enough rounds -> NETWORK_DESTROYED. frlgsim (the
 * correct guest) instead LOOPS child-initiated 0x6600 rounds in lockstep until the host re-exchanges. So once
 * the GBA parks, the C6 CONTINUES that loop toward the Switch: emit 0x6600 count=N, advance to N+1 only when the
 * Switch echoes N (LOCKSTEP, never a colliding one-shot like tr-1.50), until the Switch sends its BLOCK_REQ
 * re-exchange, then hand back by relaying the re-pull to the GBA (unblocks its menu re-entry). The C6-driven
 * echoes are SUPPRESSED to the parked GBA (its own view stays idle). Gate takeover on >=2 GBA barriers (past
 * the ~9s LinkFullSave gap that sits between the GBA's 1st and 2nd barrier) + quiet + last round echoed, so we
 * never take over mid-save-gap (which would desync the GBA from the Switch). */
#define SC_PARK_US 3500000                /* GBA-idle time after its last barrier before the C6 takes over */
static volatile bool s_sc_active;         /* CONFIRM_FINISH seen -> save chain running */
static volatile bool s_sc_reexchanged;    /* Switch re-pulled the party -> chain over, hand back to the GBA */
static volatile uint16_t s_sc_gba_count;  /* highest 0x6600 count the GBA sent up */
static volatile int      s_sc_gba_rounds; /* # of GBA 0x6600 barriers seen post-CONFIRM (>=2 = past save gap) */
static volatile int64_t  s_sc_gba_us;     /* time of the GBA's last 0x6600 */
static volatile uint16_t s_sc_sw_count;   /* highest 0x6600 count the Switch echoed down */
static volatile bool s_sc_takeover;       /* C6 is driving the barrier loop */
static uint16_t s_sc_take_count;          /* the count the C6 is currently driving (brain-task only, lockstep) */
/* tr-1.53: throttle the takeover emit. tr-1.51 emitted the 0x6600 EVERY tick while parked -> the sim queued a
 * new RELIABLE frame per tick (~9 over the ~340ms takeover), and 9 barrier frames + the 17-frag party block
 * exceeded SIM_MAX_INFLIGHT(24) -> the up-send gated MID-BLOCK -> the host never got the full block -> the
 * menu-re-entry party exchange stalled. The real child (SendReadyExitStandbyUntilAllReady) only re-sends every
 * ~60 frames. So emit ONCE per count (the reliable layer retransmits it) + a ~1s safety re-emit, keeping the
 * reliable inflight free for the block exchange. */
static uint16_t s_sc_take_emitted = 0xFFFF;/* the count value last emitted (0xFFFF = none) */
static int64_t  s_sc_take_emit_us;         /* time of the last takeover emit */

/* tr-1.64 STANDBY-ROUND COUNTER TRANSLATION — 2nd-trade freeze (hw 09-18 + 09-23, two runs, same signature:
 * host echoes count N once then goes silent while the real GBA re-sends count N-1 forever, ~1.2s cadence,
 * librfu :1529). READY_EXIT_STANDBY (0x6600) only registers if w1 EQUALS the receiver's OWN
 * resendExitStandbyCount (link_rfu_2.c:1178-1180), which is PERSISTENT across trades within a session
 * (increments per completed round, :1545, never reset per trade). A C6 takeover round is inherently
 * ASYMMETRIC: the host completes it (ctr++) while the parked GBA never runs it, so after trade #1's save the
 * host's counter is permanently N ahead of the GBA's (N = phantom rounds driven). The mismatch then trips
 * the VERY NEXT 0x6600 checkpoint either side hits post-trade-1 — proven twice now: once at trade #2's own
 * save-standby end (09-18), once at an EARLIER post-trade-1 sync point before the save prompt even showed
 * (09-23) — same bug, different trigger point, because 0x6600 is used at more than one sync checkpoint.
 * tr-1.61 tried to fix this AND replace the takeover's proven eager "drive next round immediately on echo"
 * with a "HOLD for the re-exchange" — hw 09-23 proved the HOLD half is wrong on its own (it broke trade #1's
 * own post-save menu return, reverted as tr-1.62). tr-1.64 keeps the takeover eager-advance loop COMPLETELY
 * UNCHANGED from tr-1.60 (proven, both trades) and ONLY adds: (1) increment s_sc_ctr_offset by one for every
 * phantom round the eager loop drives (still inline, no hold), and (2) translate every REAL (non-takeover)
 * 0x6600 word by that offset -- UP (GBA->host) +offset, DOWN (host->GBA) -offset -- so every future 0x6600
 * checkpoint (savechain or otherwise) lines up on both sides again. Translation is gated on `!s_sc_takeover`:
 * while a takeover's own synthesized frames are in flight both sides already compare in the HOST's native
 * domain (s_sc_take_count is never translated), so translating those too would break the very loop offset
 * comes from. Deliberately NOT gated on s_sc_active/s_sc_reexchanged -- the 09-23 freeze proved 0x6600 needs
 * translation even outside the save-chain's own active-window bookkeeping. Offset persists per SESSION
 * (reset at join only, matching the host's own persistent register). */
static volatile int s_sc_ctr_offset;       /* host resendExitStandbyCount minus the GBA's (real, non-takeover) */

/* tr-1.60 UP-PATH ROLLING-TAG OWNERSHIP — root cause of the post-save re-entry stall (hw log 09-18 05:54 +
 * ground truth). The child stamps childSendCmdId (bits 5-7 of w0's low byte) on EVERY non-idle send and the
 * parent processes a child slot only when that tag ADVANCES; a NON-advancing (same-tag) slot is a librfu
 * re-send and is DISCARDED (link_rfu_2.c:884-888). Two relay violations of this contract:
 *   (1) the C6's synthesized save-chain takeover 0x6600 frames carried tag=0 (unstamped) — hw: GBA count=9
 *       tag4, C6 count=10/11 tag0,tag0: the host processed count=10 (tag changed 4->0) but silently DROPPED
 *       count=11 (0->0, never echoed — every run);
 *   (2) on the A100 wake the GBA's menu re-entry link re-sync REPLAYS its backup queue VERBATIM — the stale
 *       0x6600 count=9 with its ORIGINAL tag 4 (UpdateBackupQueue, link_rfu_2.c:769-785). A real parent
 *       discards it (tag 4 == last accepted 4), but our tag-0 takeover frames sat in between, so the host saw
 *       0->4 = "new" and PROCESSED a stale READY_EXIT_STANDBY mid-block-re-exchange -> its FSM derailed ->
 *       host stopped its own block at 6-9/17 -> NETWORK_DESTROYED ~18s later.
 * Fix = make the relay a correct child sender (frlgsim SlotBuilder, rfu.py:118-132 — ONE continuous tag for
 * every non-idle slot incl. its takeover barriers; ChildBuildSendCmd, link_rfu_2.c:944-963):
 *   - DEDUP: drop a captured GBA slot whose tag equals the last forwarded one (librfu re-send; the real
 *     parent's :884 check would reject it — our reliable transport already delivered the original).
 *   - RE-STAMP: present a relay-owned sequential tag on every slot sent UP (real + synthesized), so the
 *     chain the host sees is unbroken regardless of takeover frames or FIFO drops. */
static int      s_up_gba_tag = -1;   /* last GBA-stamped tag forwarded (dedup key); -1 = none yet */
static uint8_t  s_up_tag;            /* relay-owned childSendCmdId presented to the host */
static unsigned s_up_dups;           /* GBA same-tag re-sends discarded */

static inline void put_u16le(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static inline uint16_t get_u16le(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }

/* ---- reset / gates ---- */
static void hni_clear(void);   /* defined with the host-NI relay queue below */
void gba_bridge_reset(void) {
    s_ni_stage = 0; s_ni_null_served = 0; s_ni_join_ok_sent = false; s_ni_held = false; s_child_ack_pending = false;
    s_join_released = false; s_gba_ni_done = false; s_gba_uni_started = false;
    s_join_ni_stage = 0; s_join_ni_done = false; s_join_ni_staged = false;   /* childf fix (tr-1.01) */
    s_player_ids_sent = false;                                              /* tr-1.06 */
    s_sc_active = s_sc_reexchanged = s_sc_takeover = false; s_sc_gba_count = s_sc_gba_rounds = 0; s_sc_sw_count = 0; s_sc_take_count = 0;  /* tr-1.51 */ s_sc_ctr_offset = 0;  /* tr-1.64: per-SESSION (offset persists across trades, resets per join) */
    hni_clear();
    portENTER_CRITICAL(&s_mux);
    s_switch_len = 0; s_gba_valid = false; echo_clear();                 /* tr-1.26 / */
    up_fr_reset(); s_gq_coalesced = 0; s_gq_drop_flag = false; s_up_hold_valid = false; s_up_hold_repeats = 0;
    s_gq_head = s_gq_tail = 0; s_gq_enq = s_gq_deq = s_gq_drops = 0;   /* battle FIFO */
    s_dq_head = s_dq_tail = 0; s_dq_enq = s_dq_deq = s_dq_drops = 0; s_dn_pending = true;  /* tr-1.21/1.22 */
    s_up_gba_tag = -1; s_up_tag = 0; s_up_dups = 0;                                        /* tr-1.60 */
    portEXIT_CRITICAL(&s_mux);
}

void gba_bridge_note_gba_join_selected(void) {
    if (!s_join_selected) { s_join_selected = true; printf("  [bridge] GBA selected the room (0x1f) -> join gate OPEN\n"); }
    s_join_select_us = esp_timer_get_time();   /* tr-1.15: diagnostic only -- timestamps "waiting for reply" start */
    /* Fresh join attempt: clear any relay state from a previous attempt. */
    s_ni_stage = 0; s_ni_null_served = 0; s_ni_join_ok_sent = false; s_ni_held = false; s_child_ack_pending = false; s_join_released = false;
    s_gba_ni_done = false; s_gba_uni_started = false;
    s_join_ni_stage = 0; s_join_ni_done = false; s_join_ni_staged = false;   /* childf fix (tr-1.01) */
    s_player_ids_sent = false;                                              /* tr-1.06 */
    s_sc_active = s_sc_reexchanged = s_sc_takeover = false; s_sc_gba_count = s_sc_gba_rounds = 0; s_sc_sw_count = 0; s_sc_take_count = 0;  /* tr-1.51 */ s_sc_ctr_offset = 0;  /* tr-1.64: per-SESSION (offset persists across trades, resets per join) */
    hni_clear();
    /* tr-1.32 CRITICAL: flush the cross-task relay QUEUES too. This function cleared every per-attempt FLAG
     * but left the mux-protected FIFOs (s_dq down, s_gq up) + latest-slot snapshots untouched -- and
     * gba_bridge_reset() (the only code that flushed them) is NEVER called. On a FRESH boot the FIFOs are
     * zero-init so the first join is clean, hiding the bug; but on a tr-1.31 RE-ARMED session the down-FIFO
     * still holds STALE Switch block fragments from the previous (dead) session, and the new session's very
     * first 0x26 polls stream those stale fragments to the GBA -> corrupt block -> the GBA dies ~2s after
     * CHILD_JOINED (hw 05:44/05:52: re-armed joins died at 5325/3816 words vs 34673 on the fresh boot -- the
     * user's "accepting on the Switch crashes the GBA after a re-arm"). Flush under the mux so the brain task
     * can't be mid-enqueue. */
    portENTER_CRITICAL(&s_mux);
    s_switch_len = 0; s_gba_valid = false; echo_clear();
    up_fr_reset(); s_gq_coalesced = 0; s_gq_drop_flag = false; s_up_hold_valid = false; s_up_hold_repeats = 0;
    s_gq_head = s_gq_tail = 0; s_gq_enq = s_gq_deq = s_gq_drops = 0;   /* battle FIFO */
    s_dq_head = s_dq_tail = 0; s_dq_enq = s_dq_deq = s_dq_drops = 0; s_dn_pending = true;
    s_up_gba_tag = -1; s_up_tag = 0; s_up_dups = 0;                                        /* tr-1.60 */
    portEXIT_CRITICAL(&s_mux);
}
bool gba_bridge_gba_join_selected(void) { return s_join_selected; }
/* tr-1.31: clear the room-selected latch when a SESSION ends (scan_task re-arm). Without this, the latch —
 * set once at 0x1f
 and never cleared — made a re-armed STAGE B start the NEXT brain the instant membership
 * returned, firing the Switch's "wants to join" prompt before the player had re-selected the room (the exact
 * premature-join bug STAGE B's gating originally fixed). */
void gba_bridge_clear_join_selected(void) { s_join_selected = false; }

void gba_bridge_set_join_released(bool released) {
    static int seen = 0;
    if (released && !s_join_released) {   /* transition to approved */
        if (seen < 1) { seen++;
            int64_t elapsed_ms = (esp_timer_get_time() - s_join_select_us) / 1000;   /* tr-1.15: diagnostic only */
            printf("  [bridge] JOIN RELEASED (Switch sent JOIN_GROUP_OK, %lldms after join-select) -> feeding GBA\n",
                   (long long)elapsed_ms); }
        /* leave the HOLD and re-run the NI from the top so the child now receives the 0x05 status subframe. */
        s_ni_held = false; s_ni_stage = 0; s_ni_null_served = 0; s_ni_join_ok_sent = false;
    }
    s_join_released = released;
}
bool gba_bridge_join_released(void) { return s_join_released; }

void gba_bridge_set_gba_identity(uint16_t tid, const uint8_t name_raw[8], uint8_t language, uint8_t activity) {
    portENTER_CRITICAL(&s_mux);
    s_id_tid = tid; memcpy(s_id_name, name_raw, 8); s_id_lang = language; s_id_activity = activity;
    bool first = !s_id_valid; s_id_valid = true;
    portEXIT_CRITICAL(&s_mux);
    if (first)
        printf("  [bridge] GBA identity: tid=0x%04x name=%02x%02x%02x%02x%02x%02x%02x%02x lang=%u act=0x%02x\n",
               tid, name_raw[0],name_raw[1],name_raw[2],name_raw[3],
               name_raw[4],name_raw[5],name_raw[6],name_raw[7], language, activity);
}
bool gba_bridge_get_gba_identity(uint16_t *tid, uint8_t name_raw[8], uint8_t *language, uint8_t *activity) {
    portENTER_CRITICAL(&s_mux);
    bool v = s_id_valid;
    if (v) { *tid = s_id_tid; memcpy(name_raw, s_id_name, 8); *language = s_id_lang; *activity = s_id_activity; }
    portEXIT_CRITICAL(&s_mux);
    return v;
}

/* ---- RETIRED (tr-0.85): the synthetic parent-NI parade + child-ack builders. The join phase is now pure
 * relay (real host NI teed by the brain; the GBA's subframes acked ISR-side in gba_word_sm). Kept for
 * reference only — the flagged-wrong S3 reconstruction (CLAUDE.md data-layer rule). ---- */
__attribute__((unused))
static int ni_emit_current(uint8_t *b) {
    const ni_subframe_t *f = &s_ni_seq[s_ni_stage];
    uint32_t llsf = ((uint32_t)(f->state & 0xF) << 14) | ((uint32_t)(f->n & 3) << 11)
                  | ((uint32_t)(f->phase & 3) << 9) | ((uint32_t)1 << 18) | (f->size & 0x7Fu);
    b[0] = (uint8_t)llsf; b[1] = (uint8_t)(llsf >> 8); b[2] = (uint8_t)(llsf >> 16);
    for (int i = 0; i < f->size; i++) b[3 + i] = f->pay[i];

    bool released = s_join_released;
    if (f->state == NI_LCOM_NI && f->size >= 1) {
        b[3] = released ? 0x05u : 0x00u;   /* JOIN_GROUP_OK once approved, else benign "still waiting" */
        if (released) s_ni_join_ok_sent = true;   /* the child has now been handed JOIN_GROUP_OK */
    }

    if (f->state == NI_LCOM_NULL) {
        if (++s_ni_null_served >= 3) {
            /* Only advance to UNI once we've actually EMITTED the 0x05 status subframe this cycle. If release
             * happened after stage 2 (status already sent as 0x00), re-loop so the child receives 0x05 before
             * we leave the NI phase — else it never sees JOIN_GROUP_OK and stays stuck at "Le". (host-sim bug) */
            if (released && s_ni_join_ok_sent) { s_ni_stage = 5; printf("  [bridge] NI complete -> UNI trade phase\n"); }
            else { s_ni_held = true; s_ni_null_served = 0; }   /* one full NI cycle delivered but NOT approved ->
                                     * HOLD: build_recv now serves EMPTY 0x26 keepalives (keep the child polling)
                                     * instead of re-looping the complete NI, which the child completes each loop
                                     * -> expects the next phase -> resets/re-logs (the tr-0.74 join-hold re-login).
                                     * Cleared to the 0x05 delivery when the Switch approves (set_join_released). */
        }
    } else if (s_ni_stage < 4) {
        s_ni_stage++;   /* always advance through the full NI — never stall the child's recv slot */
    }
    return 3 + f->size;
}

__attribute__((unused))
static int ni_emit_child_ack(uint8_t *b, uint8_t state, uint8_t n, uint8_t phase) {
    uint32_t llsf = ((uint32_t)(state & 0xF) << 14) | ((uint32_t)1 << 13) /* ack=1 */
                  | ((uint32_t)(n & 3) << 11) | ((uint32_t)(phase & 3) << 9) | ((uint32_t)1 << 18);
    b[0] = (uint8_t)llsf; b[1] = (uint8_t)(llsf >> 8); b[2] = (uint8_t)(llsf >> 16);
    return 3;
}

/* Kizuna: passive diagnostic only, no behavior change. The child's own trainer-card bytes
 * ride inside these NI_LCOM_NI subframe payloads and we have never captured them -- gba_bridge_set_gba_identity()
 * is only ever called from the GBA's own 0x16 broadcast (rfu1_bridge.c), which never fires when the GBA is a
 * pure joiner (confirmed on the bench: bc_in stayed 0 through a full session). Symptom: the Switch
 * shows the sim's hardcoded "EMU" stub identity instead of the real cartridge trainer. Concatenate the raw
 * NI_LCOM_NI payload bytes here and dump them once NI_LCOM_END latches, so the real byte layout (pret
 * librfu_rfu.c: rfu_NI_CHILD_setSendGameName sends 26 bytes from RfuLinkStatus.my.serialNo -- serial(2) +
 * game name(8) + user name(8) + ~10 more bytes not yet confirmed from the GitHub source alone) can be nailed
 * down from a REAL capture before gba_bridge_set_gba_identity() is wired to this path, instead of guessed. */
static uint8_t s_ni_child_capture[64]; static int s_ni_child_capture_len; static bool s_ni_child_dumped;

/* Scan the GBA's 0x24/0x25 send words for its own NI subframes -> queue an ack (gba_relay.c ni_scan_child).
 * Child LLSF halfword (LE): size:5 | phase:2 | n:2 | ack:1 | state:4. Zero halfword = separator (skip). */
static void ni_scan_child(const uint32_t *data, int len) {
    if (len <= 0) return;
    const uint8_t *p = (const uint8_t *)data;
    int nbytes = len * 4, off = 0;
    while (off + 2 <= nbytes) {
        uint16_t h = (uint16_t)(p[off] | (p[off + 1] << 8));
        if (h == 0) { off += 2; continue; }
        uint8_t size = h & 0x1F, phase = (h >> 5) & 3, n = (h >> 7) & 3, ack = (h >> 9) & 1, state = (h >> 10) & 0xF;
        s_ni_child_frames++;
        if (!ack && (state == NI_LCOM_START || state == NI_LCOM_NI || state == NI_LCOM_END)) {
            s_child_ack_state = state; s_child_ack_n = n; s_child_ack_phase = phase;
            s_child_ack_pending = true;
            if (state == NI_LCOM_NI && !s_ni_child_dumped) {
                int room = (int)sizeof(s_ni_child_capture) - s_ni_child_capture_len;
                int take = size < room ? size : room;
                if (take > 0) { memcpy(s_ni_child_capture + s_ni_child_capture_len, p + off + 2, take);
                                 s_ni_child_capture_len += take; }
            }
            if (state == NI_LCOM_END) {
                s_gba_ni_done = true;   /* RFU1 path: the dp-task latch is gone; mirror gsm_scan_child_ni */
                if (!s_ni_child_dumped) {
                    s_ni_child_dumped = true;
                    printf("  [ni-child] raw trainer-card bytes (%d): ", s_ni_child_capture_len);
                    for (int i = 0; i < s_ni_child_capture_len; i++) printf("%02x", s_ni_child_capture[i]);
                    printf("\n");
                    /* Kizuna: decoded from a real bench capture (this exact function's hex dump),
                     * not guessed -- same 24-byte RfuGameData layout the host's own 0x16 broadcast uses
                     * (serial@0, compat word@2-3, trainerId@4-5 LE, partnerInfo@6-9, tradeSpecies@10-11,
                     * activity@12 with bit7="started", name_raw@16-23), plus 2 trailing bytes this child
                     * block carries that the host broadcast doesn't. Capture: 02 00 81 13 8e 79 00 00 00 00
                     * 00 00 84 00 00 00 00 14 0c 01 07 ff 00 00 00 00 -> compat 0x1381 decodes to language=1
                     * (Japanese) version=4 (FireRed), trainerId=0x798e, activity byte 0x84 = started|TRADE
                     * (0x04), name_raw terminates with 0xff at index 5 like frlg_char's own terminator. This
                     * replaces the sim's hardcoded "EMU"/0x8822 stub (gba_bridge.h's documented default) with
                     * the real cartridge identity, matching what the GBA's own 0x16 broadcast path would have
                     * fed had the GBA ever hosted (it doesn't, in a Switch-hosts/GBA-joins trade). */
                    if (s_ni_child_capture_len >= 24) {
                        uint16_t tid = (uint16_t)(s_ni_child_capture[4] | (s_ni_child_capture[5] << 8));
                        uint8_t language = s_ni_child_capture[2] & 0x3;
                        uint8_t activity = s_ni_child_capture[12] & 0x7F;
                        gba_bridge_set_gba_identity(tid, s_ni_child_capture + 16, language, activity);
                        printf("  [ni-child] decoded identity: tid=0x%04x lang=%u activity=0x%02x\n",
                               tid, language, activity);
                    }
                }
            }
        }
        if (state == 4) s_gba_uni_started = true;             /* child sent a UNI slot (CHILD_JOINED) */
        off += 2 + size;
    }
}

bool gba_bridge_take_child_ack(uint8_t llsf[3]) {
    if (!s_child_ack_pending) return false;
    s_child_ack_pending = false; s_child_acks_sent++;
    uint32_t v = ((uint32_t)(s_child_ack_state & 0xF) << 14) | ((uint32_t)1 << 13)
               | ((uint32_t)(s_child_ack_n & 3) << 11) | ((uint32_t)(s_child_ack_phase & 3) << 9) | ((uint32_t)1 << 18);
    llsf[0] = (uint8_t)v; llsf[1] = (uint8_t)(v >> 8); llsf[2] = (uint8_t)(v >> 16);
    return true;
}

/* ---- tr-0.85 host-NI relay queue: the Switch's REAL parent-NI subframes (teed by the brain's eng_feed),
 * served to the GBA verbatim over 0x26. Replaces the synthetic s_ni_seq parade (the flagged-wrong S3
 * reconstruction — CLAUDE.md data-layer rule) with pure relay. Retransmission duplicates are deduped on
 * push (consecutive same state/n/phase); the queue drops-oldest on overflow (retransmission recovers). ---- */
#define HNI_N   8
#define HNI_MAX 32
typedef struct { uint8_t state, n, phase, len; uint8_t pay[HNI_MAX]; } hni_frame_t;
static hni_frame_t s_hni[HNI_N];
static volatile int s_hni_head, s_hni_tail;     /* tail=oldest (serve), head=next write */
static bool s_hni_staged;                        /* build_recv staged the tail frame this pull */
static unsigned s_hni_pushed, s_hni_served, s_hni_dropped;

static void hni_clear(void) {
    portENTER_CRITICAL(&s_mux);
    s_hni_head = s_hni_tail = 0; s_hni_staged = false;
    portEXIT_CRITICAL(&s_mux);
}

void gba_bridge_push_host_ni(int state, int n, int phase, const uint8_t *payload, int len) {
    if (len < 0) len = 0;
    if (len > HNI_MAX) len = HNI_MAX;
    portENTER_CRITICAL(&s_mux);
    int prev = (s_hni_head + HNI_N - 1) % HNI_N;
    bool dup = (s_hni_head != s_hni_tail)                    /* non-empty */
            && s_hni[prev].state == state && s_hni[prev].n == n && s_hni[prev].phase == phase
            && s_hni[prev].len == len && memcmp(s_hni[prev].pay, payload, len) == 0;
    if (!dup) {
        int nxt = (s_hni_head + 1) % HNI_N;
        if (nxt == s_hni_tail) { s_hni_tail = (s_hni_tail + 1) % HNI_N; s_hni_dropped++; }
        s_hni[s_hni_head].state = (uint8_t)state; s_hni[s_hni_head].n = (uint8_t)n;
        s_hni[s_hni_head].phase = (uint8_t)phase; s_hni[s_hni_head].len = (uint8_t)len;
        memcpy(s_hni[s_hni_head].pay, payload, len);
        s_hni_head = nxt; s_hni_pushed++;
    }
    portEXIT_CRITICAL(&s_mux);
}

void gba_bridge_recv_advance(void) {
    /* childf fix (tr-1.01): advance the join-status NI by ONE subframe, but ONLY when the ISR confirmed the
     * previously staged join-NI subframe was actually served (this fn is called by the dp task on staged_served,
     * and s_join_ni_staged marks that the served frame WAS a join-NI subframe). If an inline child-ack preempted
     * it, staged_served is false, this isn't called, and build_recv re-serves the SAME subframe next poll (no
     * loss). At END, latch done -> the child proceeds to CHILD_JOINED and starts sending UNI slots. */
    if (s_join_ni_staged && !s_join_ni_done) {
        s_join_ni_staged = false;
        if (s_ni_seq[s_join_ni_stage].state == NI_LCOM_END) {
            s_join_ni_done = true;
            printf("  [bridge] JOIN_GROUP_OK NI fully delivered (END served) -> await child UNI slot\n");
        } else if (s_join_ni_stage < 3) {
            s_join_ni_stage++;
        }
    }
    (void)s_hni_served; (void)s_hni_dropped; (void)s_hni_pushed; (void)s_child_acks_sent;
}

/* M1: the dp task latches this from sm->gba_ni_done (GBA sent NI_END — diagnostic only now). */
void gba_bridge_set_gba_ni_done(bool v) { s_gba_ni_done = v; }
/* tr-0.94: the REAL safe-for-UNI gate — dp task latches from sm->gba_uni_started (GBA sent a state-4 UNI slot). */
void gba_bridge_set_gba_uni_started(bool v) { s_gba_uni_started = v; }

/* ---- GBA command-phase side (GBA task) ---- */
void gba_bridge_on_gba_cmd(uint8_t cmd, const uint32_t *data, int ndata) {
    if (ndata <= 0) return;
    (void)cmd;
    /* data[0] is the SEND length-header word, NOT a subframe — scan the child's LLSF subframes from data[1]. */
    if (ndata > 1) ni_scan_child(data + 1, ndata - 1);  /* ack the child's own NI subframes (else it faults) */

    /* Extract the GBA's outgoing 14-byte slot for eng_tick (-> Switch) + row1 echo.
     * tr-1.20 FIX (hw-evidenced): the old code assumed the send was "[len header word][slot bytes]" and read
     * the 7 u16 straight from data[1]. That is WRONG, and the line above proves it -- ni_scan_child(data+1)
     * parses data[1..] as LLSF SUBFRAMES, each "[2-byte header][size bytes]". So the slot payload starts 2
     * BYTES LATER than assumed. Consequence: s_gba_words[0] was the LLSF HEADER, every real slot word was
     * shifted down one index, and the slot's LAST u16 was truncated -- i.e. EVERY slot we forwarded to the
     * Switch was corrupt. Hardware proof: [up] logged a constant w0=0x100e, which decodes exactly as an LLSF
     * header (state=(h>>10)&0xF = 4 = UNI, size=h&0x1F = 14 = one gRecvCmd slot), while the real RFU command
     * sat in w1 (0x89xx = RFUCMD_SEND_BLOCK, pokefirered include/link_rfu.h:18). By contrast the DOWN
     * direction logs a proper command in w0 (0x7700 SEND_PLAYER_IDS / 0x8800 SEND_BLOCK_INIT / 0x8900
     * SEND_BLOCK / 0xA100 SEND_BLOCK_REQ) -- the asymmetry was the tell.
     * Correct parse: walk the subframes, find the UNI one (state==4), and take ITS payload as the slot. */
    if (ndata >= 2) {
        const uint8_t *p = (const uint8_t *)(data + 1);
        int nbytes = (ndata - 1) * 4, off = 0;
        const uint8_t *slot = NULL;
        while (off + 2 <= nbytes) {
            uint16_t h = (uint16_t)(p[off] | (p[off + 1] << 8));
            if (h == 0) { off += 2; continue; }                  /* zero halfword = subframe separator */
            uint8_t size = h & 0x1F, state = (h >> 10) & 0xF;
            if (state == 4 && size >= 14 && off + 2 + 14 <= nbytes) { slot = p + off + 2; break; }
            off += 2 + size;
        }
        if (slot) {                                              /* only a real UNI subframe yields a slot */
            /* tr-1.27: IDLE (all-zero) slots are NOT queued and NOT echoed. The real parent ignores them
             * entirely — its per-child copy runs only `if (childRecvBuffer[i][1])` (link_rfu_2.c:877), so an
             * idle child slot is never forwarded into gRecvCmds nor echoed down, and it does not advance the
             * child's rolling tag either (ChildBuildSendCmd leaves tag untouched on idle). Queueing them was
             * what saturated the UP FIFO (tr-1.26 hw run: depth pegged 30/32, drops=5, ~1.1s of standing UP
             * latency at the ~38ms poll cadence); eng_tick idle-fills identically when the queue is empty, so
             * the wire content is unchanged — only the latency/backlog goes away. Tag safety: the Switch's
             * tag check also skips idles (same :877 gate), and every NON-idle slot still queues once-only in
             * order. */
            bool nonzero = false;
            for (int i = 0; i < 14; i++) if (slot[i]) { nonzero = true; break; }
            if (nonzero) {
                portENTER_CRITICAL(&s_mux);
                for (int i = 0; i < 7; i++) {
                    uint16_t v = get_u16le(slot + i * 2);
                    s_gba_words[i] = v;
                }
                s_gba_valid = true;
                /* ECHO FIFO (see s_eq decl): queue this slot's tag-stripped echo exactly once, unless an
                 * identical echo is still waiting at the queue tail (a SendLastBlock repeat) -> coalesce. */
                {
                    uint16_t e[7]; e[0] = (uint16_t)((s_gba_words[0] & 0xFF00u) | (s_gba_words[0] & 0x001Fu));
                    for (int i = 1; i < 7; i++) e[i] = s_gba_words[i];
                    bool same = false;
                    if (s_eq_head != s_eq_tail) {
                        int newest = (s_eq_head + ECHO_Q_N - 1) % ECHO_Q_N;
                        same = true; for (int i = 0; i < 7; i++) if (s_eq[newest][i] != e[i]) { same = false; break; }
                    }
                    if (same) s_eq_coalesced++;
                    else {
                        int nxt = (s_eq_head + 1) % ECHO_Q_N;
                        if (nxt == s_eq_tail) { s_eq_tail = (s_eq_tail + 1) % ECHO_Q_N; s_eq_drops++; }  /* full -> drop-oldest */
                        for (int i = 0; i < 7; i++) s_eq[s_eq_head][i] = e[i];
                        s_eq_head = nxt; s_eq_enq++;
                    }
                }
                /* tr-1.23: the down queue is drained in build_recv (streaming), NOT here -- see the s_dn_pending
                 * note; gating the drain on this event deadlocked (the child can't initiate UNI). */
                /* tr-1.60 TAG DEDUP (see s_up_gba_tag decl): a non-idle slot whose rolling tag EQUALS the last
                 * forwarded one is a librfu VERBATIM RE-SEND (backup-queue replay on link re-sync,
                 * link_rfu_2.c:769-785, or an unrebuilt UNI buffer re-read) — the real parent discards it
                 * (:884, tag must advance). Forwarding it double-processes a one-shot on the host (the stale
                 * post-A100 0x6600 count=9 that derailed the re-entry). Drop it; every FRESH frame has tag+1.
                 * The echo/save-chain tracking above stays ungated (tr-1.56 semantics preserved). */
                int gtag = (s_gba_words[0] >> 5) & 7;
                bool dup = (gtag == s_up_gba_tag);
                if (!dup) {
                    s_up_gba_tag = gtag;
                    /* hw: in the room the GBA's SEND_HELD_KEYS stream (one per
                     * frame) outran the brain's slot rate (15 ms floor deferrals on the single-core C3) and this
                     * FIFO sat full, dropping 16% of frames oldest-first — including, most likely, the single
                     * frame that carried the EXIT_ROOM key when the player left, so the Switch never ran its own
                     * exit and both sides waited forever. Held keys are state, not a sequence: if the newest
                     * queued slot is a 0xBE00 with the SAME key byte, replace it (keep the newest count) instead
                     * of appending. Any other frame (a key change, 0x5F00 READY_CLOSE_LINK, blocks) still
                     * appends, so it can no longer be pushed out by a held-keys backlog. */
                    if (s_battle) {
                        /* (battle): bypass the replay -- it skips a frame when the consumer runs slower than 60 Hz,
                         * and a battle stays in lockstep only if every GBA slot reaches the Switch once, in order. Plain FIFO,
                         * no merging, drop-oldest only when full. */
                        int nxt = (s_gq_head + 1) % GBA_Q_N;
                        if (nxt == s_gq_tail) {
                            s_gq_drop_w0 = s_gq[s_gq_tail][0]; s_gq_drop_w1 = s_gq[s_gq_tail][1]; s_gq_drop_flag = true;
                            s_gq_tail = (s_gq_tail + 1) % GBA_Q_N; s_gq_drops++;
                        }
                        for (int i = 0; i < 7; i++) s_gq[s_gq_head][i] = s_gba_words[i];
                        s_gq_head = nxt; s_gq_enq++;
                    } else {
#ifdef KZ_ONLINE_PACING
                        /* queue for replay on the Switch's clock. Nothing folds any more: a press is its run length,
                         * and the Switch must see the GBA's run lengths, not the relay's arrival rhythm. */
                        if (!s_up_fr_init) up_fr_reset();
                        { uint32_t before = s_up_fr.drops;
                          fr_push(&s_up_fr, s_gba_words, up_now());
                          if (s_up_fr.drops != before) { s_gq_drop_w0 = s_gba_words[0]; s_gq_drop_w1 = s_gba_words[1]; s_gq_drop_flag = true; } }
#else
                        /* local (switch-trade-v2.4): a held-keys slot with the SAME key byte as the newest queued one replaces
                         * it; anything else appends, drop-oldest only when full. */
                        bool coalesced = false;
                        if (s_gq_head != s_gq_tail && (s_gba_words[0] & 0xFF00u) == 0xBE00u) {
                            int newest = (s_gq_head + GBA_Q_N - 1) % GBA_Q_N;
                            if ((s_gq[newest][0] & 0xFF00u) == 0xBE00u
                                && (s_gq[newest][1] & 0x00FFu) == (s_gba_words[1] & 0x00FFu)) {
                                for (int i = 0; i < 7; i++) s_gq[newest][i] = s_gba_words[i];
                                s_gq_coalesced++; coalesced = true;
                            }
                        }
                        if (!coalesced) {
                            int nxt = (s_gq_head + 1) % GBA_Q_N;
                            if (nxt == s_gq_tail) {
                                s_gq_drop_w0 = s_gq[s_gq_tail][0]; s_gq_drop_w1 = s_gq[s_gq_tail][1]; s_gq_drop_flag = true;
                                s_gq_tail = (s_gq_tail + 1) % GBA_Q_N; s_gq_drops++;
                            }
                            for (int i = 0; i < 7; i++) s_gq[s_gq_head][i] = s_gba_words[i];
                            s_gq_head = nxt; s_gq_enq++;
                        }
#endif
                    }
                } else {
                    s_up_dups++;
                }
                /* tr-1.51: track the GBA's own save-chain barriers (see s_sc_* decl). Only while the save chain
                 * is active and the C6 hasn't taken over yet -- a GBA barrier here means it's still driving. */
                if (s_sc_active && !s_sc_takeover && (s_gba_words[0] & 0xFF00u) == 0x6600u) {
                    s_sc_gba_count = s_gba_words[1]; s_sc_gba_rounds++; s_sc_gba_us = esp_timer_get_time();
                }
                portEXIT_CRITICAL(&s_mux);
                /* tr-1.60 diagnostic: show every discarded re-send (throttled). */
#ifdef KZ_ONLINE_PACING
#define DROPS_UP (s_battle ? s_gq_drops : (unsigned)s_up_fr.drops)
#else
#define DROPS_UP s_gq_drops
#endif
                if (s_gq_drop_flag) { s_gq_drop_flag = false;
                    static int dropdbg;
                    if (dropdbg < 60) { dropdbg++;
                        printf("  [upq] DROPPED oldest GBA slot w0=%04x w1=%04x (drops=%u)\n", s_gq_drop_w0, s_gq_drop_w1, DROPS_UP); }
                }
                static int dupdbg;
                if (dup && dupdbg < 40) { dupdbg++;
                    printf("  [tagdup] dropped GBA re-send %04x %04x %04x (tag %d, dups=%u)\n",
                           s_gba_words[0], s_gba_words[1], s_gba_words[2], gtag, s_up_dups);
                }
            }
        }
    }
    static int dbg;
    if (dbg < 40) { dbg++;
        KZ_TRACE("  [bridge] GBA cmd=%02x ndata=%d w0=%08lx slot=%04x %04x %04x childf=%u ackq=%d\n",
               cmd, ndata, (unsigned long)data[0], s_gba_words[0], s_gba_words[1], s_gba_words[2],
               s_ni_child_frames, s_child_ack_pending);
    }
    /* tr-1.24 DIAGNOSTIC (alignment ground truth): dump the RAW bytes of the child's send, exactly as they
     * arrived, so the slot offset can be determined from evidence instead of inferred. Why this is needed:
     * pokefirered link_rfu_2.c:1351/1387/1390 PROVE the child's block command is "RFUCMD_SEND_BLOCK |
     * gRfu.sendBlock.next" with next starting at 0 and incrementing by exactly 1 -> the wire must carry
     * 0x8900, 0x8901, 0x8902, ... But after tr-1.20 we parse 0x89c3, 0x89e4, 0x8905, 0x8926 ... i.e. the low
     * byte steps by 0x21 instead of 1, while the high byte reads a correct 0x89. A pure offset shift would
     * corrupt BOTH bytes, so the real layout is something we have not modelled yet. Print the bytes and read
     * the answer off the wire. */
    /* tr-1.25: the tr-1.24 cap was consumed by the early NI phase, so every captured UNI sample was an all-zero
     * idle slot -- useless for the alignment question. Trigger ONLY on a non-empty parsed slot, i.e. exactly
     * the block-transfer frames where the 0x89xx index mismatch shows up. */
    static int rawdbg;
    if (rawdbg < 12 && ndata >= 2 && s_gba_words[0] != 0) { rawdbg++;
        KZ_TRACE("  [bridge] RAW cmd=%02x ndata=%d :", cmd, ndata);
        for (int i = 0; i < ndata; i++) {
            uint32_t d = data[i];
            KZ_TRACE(" %02x %02x %02x %02x", (unsigned)(d & 0xFF), (unsigned)((d >> 8) & 0xFF),
                   (unsigned)((d >> 16) & 0xFF), (unsigned)((d >> 24) & 0xFF));
        }
        KZ_TRACE("\n");
    }
    /* tr-1.52 DIAGNOSTIC: menu-re-entry up-side trace (GBA's block response), gated on s_sc_reexchanged. */
    if (s_sc_reexchanged) { static uint16_t u0, u1, u2; static int udbg;
        if (udbg < 400 && (s_gba_words[0] != u0 || s_gba_words[1] != u1 || s_gba_words[2] != u2)) {
            udbg++; u0 = s_gba_words[0]; u1 = s_gba_words[1]; u2 = s_gba_words[2];
            printf("  [re-up] GBA->SW %04x %04x %04x\n", u0, u1, u2); } }
}

/* ReceiveData(0x26) reply (gba_relay.c gba_relay_take_parent_frame): child-ack -> NI phase -> UNI phase.
 * out[0] = count header (bits0-6 = #bytes), then the frame bytes packed LE into words. NEVER count=0
 * once we're relaying — a zero table means "alive, no cmds yet", not "no partner". */
int gba_bridge_build_recv(uint32_t *out, int max_out) {
    if (max_out < 1) return 0;

    /* M1 (tr-0.86) — the two-phase reply, gated on the GBA's OWN NI completion (s_gba_ni_done, latched in
     * gba_word_sm when the GBA sends NI_END). Retires the tr-0.85 phantom host-NI relay (the parent does NOT
     * stream NI data to a child — librfu establishment is child->parent only; Agent-3 map link_rfu_2.c).
     *   NI phase (!s_gba_ni_done): child subframes are acked ISR-side (gba_word_sm); 0x26 otherwise = count=0
     *     "still waiting" (librfu_rfu.c:1917). A UNI frame here would race ahead and fault the child.
     *   UNI phase (s_gba_ni_done): the child flipped to TYPE_UNI recv (link_rfu_2.c:463) and expects a
     *     CONTINUOUS UNI stream. Serve the Switch's REAL slots when present, else a VALID all-zero UNI idle
     *     frame that HOLDS the child alive through the Switch's slower establishment + the human accept
     *     (memory: all-zero idle until established). NEVER count=0 in UNI — a zero table is a stall, not a hold. */
    static int s_br_dbg;
    s_join_ni_staged = false;

    /* childf fix (tr-1.01, decomp childf-needs-join-group-ok-ni): once the GBA has finished its OWN name-send
     * (NI_END -> s_gba_ni_done) it is in TRY_JOIN, polling to RECEIVE a parent NI carrying JOIN_GROUP_OK(=5)
     * (link_rfu_2.c:445,2364). Serve the join-status NI (s_ni_seq: START(5B ctrl)/START(2B ctrl)/NI[0x05]/END),
     * ONE subframe per poll, IDEMPOTENT — advanced only on staged_served (gba_bridge_recv_advance) so an inline
     * child-ack can't drop a subframe. Gated STRICTLY on gba_ni_done so it NEVER fires in the pre-name join-wait
     * (the tr-0.73..76 "Le" regression was firing early). After END the child reaches CHILD_JOINED and starts
     * sending UNI slots (gba_uni_started) -> the UNI code below takes over. */
    /* tr-1.16 CRITICAL FIX (user-observed, real hardware): this block was gated ONLY on s_gba_ni_done -- the
     * GBA finishing its OWN name-send -- and NEVER checked s_join_released. s_ni_seq[2]'s payload is a
     * hardcoded {0x05} = JOIN_GROUP_OK, and line ~356 copies fr->pay verbatim, so the moment the child finished
     * its name-send we told it "the other trainer said yes" -- ~500ms after join-select, 35 SECONDS before the
     * Switch actually approved (hw log 02:49:08.5 deliver vs 02:49:43.8 real JOIN_GROUP_OK). The s_ni_seq
     * comment "payload[0] overwritten with the live status" is STALE: that overwrite only ever existed in
     * ni_emit_current(), which is the RETIRED path (__attribute__((unused))) -- this live tr-1.01 path never
     * had it. This also violated CLAUDE.md's explicit mandatory rule ("NEVER fake JOIN_GROUP_OK" / "the RP2040
     * has NO JOIN_GROUP_OK over 0x26"). Gate on s_join_released so the join-status NI is delivered ONLY after
     * the Switch really sent its approval; until then the keepalive path below holds the child. */
    if (s_join_released && s_gba_ni_done && !s_gba_uni_started && !s_join_ni_done) {
        const ni_subframe_t *fr = &s_ni_seq[s_join_ni_stage];
        uint32_t llsf = ((uint32_t)(fr->state & 0xF) << 14) | ((uint32_t)(fr->n & 3) << 11)
                      | ((uint32_t)(fr->phase & 3) << 9) | ((uint32_t)1 << 18) | (fr->size & 0x7Fu);
        uint8_t b[3 + 5]; b[0] = (uint8_t)llsf; b[1] = (uint8_t)(llsf >> 8); b[2] = (uint8_t)(llsf >> 16);
        for (int i = 0; i < fr->size; i++) b[3 + i] = fr->pay[i];
        int nb = 3 + fr->size;
        s_join_ni_staged = true;
        if (s_br_dbg < 80) { s_br_dbg++;
            printf("  [recv26] JOIN-NI stage=%d state=%d n=%d size=%d -> deliver JOIN_GROUP_OK(5) to GBA\n",
                   s_join_ni_stage, fr->state, fr->n, fr->size); }
        out[0] = (uint32_t)nb & 0x7Fu;
        int nw = (nb + 3) / 4;
        for (int i = 0; i < nw && 1 + i < max_out; i++) { int o = i * 4;
            out[1 + i] = (uint32_t)b[o]
                       | ((o + 1 < nb) ? ((uint32_t)b[o + 1] << 8)  : 0)
                       | ((o + 2 < nb) ? ((uint32_t)b[o + 2] << 16) : 0)
                       | ((o + 3 < nb) ? ((uint32_t)b[o + 3] << 24) : 0);
        }
        return 1 + nw;
    }

    /* tr-1.02 FIX (childf, replaces the tr-0.94 gate): stream UNI as soon as OUR OWN join-status NI delivery
     * is done (s_join_ni_done, END served), NOT once the child reciprocates its own UNI slot (s_gba_uni_started).
     * Waiting for the child's reciprocal signal was a MUTUAL-WAIT DEADLOCK: per decomp, the child never sends
     * its own UNI slot until it has ALREADY received a UNI frame (state=4) from us — it can't initiate UNI, only
     * respond into it (RFUSTATE_CHILD_TRY_JOIN -> CHILD_JOINED registers the 70B recv buffer in the SAME state
     * transition as receiving JOIN_GROUP_OK, link_rfu_2.c:445-463 -> librfu_rfu.c:2074 ERR_RECV_BUFF_OVER's
     * precondition is already satisfied by the time our join-NI END is served). This also restores our OWN
     * rule stated 15 lines above ("NEVER count=0 once we're relaying") that tr-0.94 had silently violated.
     * Root-caused (hardware logic-analyzer capture, tr-1.01 test): after JOIN_GROUP_OK was fully delivered, the
     * GBA sent one NULL/keepalive frame then went 140ms silent (zero wire activity, not a timeout we could have
     * answered) before resetting — consistent with it sitting parked waiting for OUR UNI stream that this gate
     * was withholding. gba_ni_done alone (NI_END, pre-JOIN_GROUP_OK) is still too early -> keep that as the
     * ONLY pre-UNI keepalive gate. */
    if (!s_gba_ni_done) {
        if (s_br_dbg < 60) { s_br_dbg++; KZ_TRACE("  [recv26] keepalive count=0 (pre-NI, waiting for child name-send)\n"); }
        out[0] = 0; return 1;
    }
    /* tr-1.16: the child HAS finished its name-send but the Switch has NOT approved yet -> this is the real
     * "waiting for the other trainer to answer" hold. Serve count=0 keepalives, NOT UNI (falling through to the
     * UNI stream below would be the same class of bug as the faked JOIN_GROUP_OK above -- advancing the child
     * on our own authority). Decomp-proven safe to hold here indefinitely: a count==0 reply makes the child
     * skip all NI analyze, open no recv slot, run no failCounter, and keep polling 0x26 forever
     * (librfu_rfu.c:1917) -- and the user confirmed on real hardware that the GBA sits happily at "Waiting for
     * reply from trainer X" for 20+ seconds. */
    if (!s_join_released) {
        if (s_br_dbg < 60) { s_br_dbg++;
            KZ_TRACE("  [recv26] keepalive count=0 (child name-send done; WAITING for the Switch user to accept)\n"); }
        out[0] = 0; return 1;
    }

    /* tr-1.26 ROOT-CAUSE FIX (decomp: MscCallback_Child link_rfu_2.c:589-608): STREAM a state=4 parent frame
     * on EVERY delivered 0x26 poll once the child is in UNI. The child's ENTIRE game-side engine is clocked
     * by non-empty deliveries: childSendCount++ — which drives CallRfuFunc() (SendLastBlock retries, standby
     * 0x6600 emission, everything) and ChildBuildSendCmd (its next slot) — happens ONLY when the poll carried
     * a non-empty UNI payload (recv.newDataFlag). A count=0 reply ticks NOTHING. tr-1.09's "count=0 unless
     * content changed" gate therefore froze the GBA dead the moment the block exchange stopped producing new
     * content (the tr-1.25 wall: parked at Task_PlayerExchange case 4, silent 18s, 0x..494E relogin). tr-1.09's
     * REAL bug was repeated CONTENT (SEND_PLAYER_IDS re-processing), not delivery itself — the faithful parent
     * model is: deliver every poll; each command appears in exactly ONE frame; rows zero otherwise
     * (MoveSendCmdToRecv zeroes gSendCmd link_rfu_2.c:759-767; the parent zeroes childRecvBuffer after the echo
     * copy :900-906). Rate is safe: one frame per GBA-initiated poll (>=16.7ms apart) = the real parent's
     * cadence; the tr-1.21 ERR_RECV_DATA_OVERWRITED incident was multiple deliveries inside ONE child frame,
     * which per-poll pacing cannot produce (tr-1.23 already delivered per-poll whenever the FIFO was non-empty
     * and the hw block exchange ran clean). */
    uint8_t f[3 + 70];
    memset(f, 0, sizeof f);
    uint32_t llsf = ((uint32_t)(4 & 0xF) << 14) | ((uint32_t)1 << 18) | ((uint32_t)70 & 0x7Fu);
    f[0] = (uint8_t)llsf; f[1] = (uint8_t)(llsf >> 8); f[2] = (uint8_t)(llsf >> 16);
    bool row0_filled = false, row1_filled = false;
    portENTER_CRITICAL(&s_mux);
    if (s_dq_head != s_dq_tail) {   /* row0: POP one queued Switch slot per delivered poll (tr-1.23) */
        for (int i = 0; i < 7; i++) put_u16le(f + 3 + i * 2, s_dq[s_dq_tail][i]);
        s_dq_tail = (s_dq_tail + 1) % SW_Q_N; s_dq_deq++;
        row0_filled = true;
    }
    if (s_eq_head != s_eq_tail) {   /* row1 = the oldest un-echoed GBA slot, ONE per delivered frame */
        for (int i = 0; i < 7; i++) put_u16le(f + 3 + 14 + i * 2, s_eq[s_eq_tail][i]);
        s_eq_tail = (s_eq_tail + 1) % ECHO_Q_N; s_eq_deq++;
        row1_filled = true;
    }
    if (0) {   /* tr-1.26 echo-once body kept for reference (the FIFO above replaces it; see s_eq decl) */
        /* tr-1.26 row1 = echo of the GBA's own slot, ONCE, with the rolling tag STRIPPED from word0's low
         * byte — exactly what a real parent broadcasts: it validates+strips childSendCmdId (bits 5-7 of
         * byte0, `childRecvBuffer[i][0] &= 0x1f` link_rfu_2.c:896) and its downstream IS gRecvCmds
         * (rfu_UNI_setSendData(gRfu.recvCmds) :496/:928). tr-1.25 echoed VERBATIM: the child parses the FULL
         * low byte as the fragment index (`next = gRecvCmds[i][0] & ~RFUCMD_MASK` :1157) and `1 << next` is 0
         * on ARM for next>=32, so its own-block receivedFlags never filled -> recvBlock[1] never FINISHED ->
         * AreAllPlayersFinishedReceiving() (:1245-1255) never true -> parked at Task_PlayerExchange case 4
         * (:1866) -> gReceivedRemoteLinkPlayers never set -> it never initiated the 0x6600 standby that the
         * Switch-leader silently waits for (Rfu_LinkStandby leader branch :1577-1591 waits for the CHILD's
         * 0x6600 first — frlg-ldn-trade barrier.py). Words 1..6 are payload, tag-free, copied verbatim. */
        uint16_t w0 = (uint16_t)((s_gba_words[0] & 0xFF00u) | (s_gba_words[0] & 0x001Fu));
        put_u16le(f + 3 + 14, w0);
        for (int i = 1; i < 7; i++) put_u16le(f + 3 + 14 + i * 2, s_gba_words[i]);
        row1_filled = true;
    }
    portEXIT_CRITICAL(&s_mux);
    if (!row0_filled && !s_player_ids_sent) {
        /* M2a (tr-0.89) one-shot SEND_PLAYER_IDS — tr-1.26 ids FIX: LoadLinkPlayerIds (link_rfu_2.c:1058-1067)
         * does multiplayerId = ids[gRfu.childSlot]; ids[] is indexed by RFU CHILD SLOT, NOT by player row.
         * The GBA sits on OUR child slot 0 (our LLSF bmSlot above is 1<<18 = slot 0 and the child accepts the
         * frames), so it reads ids[0]. The old ids={0,1,0,0} handed it multiplayerId=0 = it believed it was
         * the HOST: its block-send FSM then watched row0 for its "echo" (the Switch's own block — clean
         * indices, full flags) and completed with ZERO retries while its own row1 block never registered —
         * the tr-1.25 clean silence, and it would run Leader paths in trade.c later. The real Switch sends
         * ids[0]=1 for a slot-0 joiner (frlg-ldn-trade frlgsim/trade.py:602-609 validates this live). */
        f[3] = 0x00; f[4] = 0x77;          /* word0 = 0x7700 = RFUCMD_SEND_PLAYER_IDS */
        f[5] = 0x02; f[6] = 0x00;          /* word1 = playerCount = 2 */
        f[7] = 0x01; f[8] = 0x00;          /* ids[0]=1 (the GBA, on OUR child slot 0), ids[1]=0 */
        f[9] = 0x00; f[10] = 0x00;         /* ids[2]=ids[3]=0 ; rows 1-4 left zero */
        row0_filled = true;
        s_player_ids_sent = true;
        printf("  [bridge] SEND_PLAYER_IDS delivered (one-shot, ids[0]=1 -> GBA multiplayerId=1)\n");
    }
    if (s_br_dbg < 60 && (row0_filled || row1_filled)) { s_br_dbg++;
        printf("  [recv26] UNI stream row0=%d row1=%d\n", (int)row0_filled, (int)row1_filled); }
    out[0] = (uint32_t)(sizeof f) & 0x7Fu;   /* 73 bytes */
    int nw = (int)((sizeof f + 3) / 4);
    for (int i = 0; i < nw && 1 + i < max_out; i++) {
        int o = i * 4;
        out[1 + i] = (uint32_t)f[o]
                   | ((o + 1 < (int)sizeof f) ? ((uint32_t)f[o + 1] << 8)  : 0)
                   | ((o + 2 < (int)sizeof f) ? ((uint32_t)f[o + 2] << 16) : 0)
                   | ((o + 3 < (int)sizeof f) ? ((uint32_t)f[o + 3] << 24) : 0);
    }
    return 1 + nw;
}

/* ---- brain side (brain task) ---- */
/* Dequeue the next queued GBA slot (once-only, in order). Returns false when the FIFO is empty so the caller
 * idle-fills (send all-zero, never a stale repeat). */
bool gba_bridge_take_gba_slot(uint16_t words[7]) {
    bool v = false;
    portENTER_CRITICAL(&s_mux);
    int32_t now_f = up_now();
#ifdef KZ_ONLINE_PACING
    if (s_battle) {                                          /* the battle FIFO, once-only, in order */
#else
    if (true) {                                              /* local: the UP FIFO (battle and trade); never a repeat */
#endif
        if (s_gq_head != s_gq_tail) {
            for (int i = 0; i < 7; i++) words[i] = s_gq[s_gq_tail][i];
            s_gq_tail = (s_gq_tail + 1) % GBA_Q_N; s_gq_deq++; v = true;
        }
    }
#ifdef KZ_ONLINE_PACING
    else if (s_up_fr_init && fr_pop(&s_up_fr, words, now_f)) {   /* the frame due now on the Switch's clock, or none */
        s_gq_coalesced = s_up_fr.skipped; v = true;
        /* repeat only what a GBA repeats: EMPTY (0x11) and the four directions (0x12-0x15). Event codes (READY, EXIT_ROOM,
         * START, A, IDLE, HANDLE_*_QUEUE, EXIT_SEAT: 0x16-0x1d) appear once and move the Switch into link standby; repeating
         * one for 2 s filled the reliable window and deadlocked the standby handshake. */
        if ((words[0] & 0xFF00u) == 0xBE00u) {
            uint8_t key = (uint8_t)words[1];
            if (key >= 0x11 && key <= 0x15) { memcpy(s_up_hold, words, sizeof s_up_hold); s_up_hold_valid = true; s_up_hold_t = now_f; }
            else s_up_hold_valid = false;
        } else s_up_hold_valid = false;
    } else if (s_up_hold_valid && now_f - s_up_hold_t <= UP_HOLD_FRAMES) {
        memcpy(words, s_up_hold, sizeof s_up_hold); s_up_hold_repeats++; v = true;   /* held keys: what the GBA would send */
    }
#else
    (void)now_f; (void)s_up_hold; (void)s_up_hold_t; (void)s_up_hold_valid; (void)s_up_fr_init;
#endif
    if (v) {
        /* tr-1.60 RE-STAMP (see s_up_gba_tag decl): replace the GBA's rolling tag (bits 5-7 of w0's low
         * byte — ChildBuildSendCmd never uses those bits for payload, link_rfu_2.c:950) with the
         * relay-owned sequential counter, exactly like frlgsim's SlotBuilder (rfu.py:118-132). While
         * nothing is synthesized/dropped this reproduces the GBA's own sequence verbatim; when the C6
         * synthesizes takeover frames (below) or the FIFO drops, the chain the host sees stays unbroken. */
        words[0] = (uint16_t)((words[0] & 0xFF1Fu) | ((uint16_t)s_up_tag << 5));
        s_up_tag = (s_up_tag + 1) & 7;
        /* tr-1.64 UP COUNTER TRANSLATION (see s_sc_ctr_offset decl): present the GBA's REAL standby count in
         * the HOST's domain so link_rfu_2.c:1179 matches on the host side. Gated on !s_sc_takeover: during an
         * active takeover the real GBA is parked (no real slots flow), so this never fires concurrently with
         * the takeover's own host-domain-native synthesized frames below. */
        if ((words[0] & 0xFF00u) == 0x6600u && s_sc_ctr_offset != 0 && !s_sc_takeover)
            words[1] = (uint16_t)(words[1] + (uint16_t)s_sc_ctr_offset);
    }
    portEXIT_CRITICAL(&s_mux);
    if (v) return true;                                 /* a real GBA slot -> relay it (priority) */
    /* tr-1.51: GBA FIFO empty. If the save chain is running and the GBA has PARKED (>=2 barriers = past the
     * ~9s save gap, then quiet > SC_PARK_US, last round echoed) and the Switch hasn't re-exchanged, CONTINUE
     * the child-initiated 0x6600 loop toward the Switch in LOCKSTEP (emit count; advance only on the Switch's
     * matching echo, in put_host_slot). This provides the extra rounds the Switch needs to re-exchange. */
    if (s_sc_active && !s_sc_reexchanged && s_sc_gba_rounds >= 2 && s_sc_gba_count > 0
        && s_sc_sw_count >= s_sc_gba_count
        && (esp_timer_get_time() - s_sc_gba_us) > SC_PARK_US) {
        if (!s_sc_takeover) { s_sc_takeover = true;
            /* tr-1.64 FIX: s_sc_take_count is what we SEND to (and expect echoed BY) the host, so it must be
             * in the HOST's domain from the start -- gba_count+1 is the GBA-domain "next round" number, which
             * only happens to equal the host's expected value when offset==0 (trade #1's takeover). On any
             * later takeover (offset already nonzero from a prior trade) this under-counts by `offset`: the
             * emitted frame looks like a STALE repeat of a round the host already completed, so it never
             * echoes anything back -- hw 09-23, 2nd session, froze silent forever right after "driving count".
             * (DOWN translation is gated off during s_sc_takeover, so the echo comparison below also sees the
             * host's raw/untranslated value -- computing take_count in host-domain here keeps both sides of
             * that comparison consistent.) */
            s_sc_take_count = (uint16_t)(s_sc_gba_count + 1 + (uint16_t)s_sc_ctr_offset);
            s_sc_take_emitted = 0xFFFF; s_sc_take_emit_us = 0;
            printf("  [sc] GBA parked after count=%u -> C6 takeover, driving count=%u (host-domain, offset %d)\n",
                   (unsigned)s_sc_gba_count, (unsigned)s_sc_take_count, s_sc_ctr_offset); }
        /* tr-1.53: emit ONCE per count (+ ~1s safety re-emit); the reliable layer retransmits it. Emitting
         * every tick floods the reliable inflight and starves the block exchange (see s_sc_take_emitted). */
        int64_t now = esp_timer_get_time();
        if (s_sc_take_count != s_sc_take_emitted || (now - s_sc_take_emit_us) > 1000000) {
            s_sc_take_emitted = s_sc_take_count; s_sc_take_emit_us = now;
            /* tr-1.60: synthesized frames MUST carry the relay-owned rolling tag too (they were sent with
             * tag=0, which un-deduped the GBA's post-A100 backup replay — see s_up_gba_tag decl). */
            portENTER_CRITICAL(&s_mux);
            words[0] = (uint16_t)(0x6600u | ((uint16_t)s_up_tag << 5));
            s_up_tag = (s_up_tag + 1) & 7;
            portEXIT_CRITICAL(&s_mux);
            words[1] = s_sc_take_count;
            for (int i = 2; i < 7; i++) words[i] = 0;
            return true;
        }
        return false;                                  /* already emitted this count -> idle, let it deliver/echo */
    }
    return false;
}

/* diagnostics: FIFO stats for the brain to log. */
void gba_bridge_dn_stats(unsigned *in, unsigned *same, unsigned *zero, unsigned *be, unsigned *fold, unsigned *enq, unsigned *drops) {
    portENTER_CRITICAL(&s_mux);
    *in = s_dn_in; *same = s_dn_same; *zero = s_dn_zero; *be = s_dn_be; *fold = s_dn_fold; *enq = s_dq_enq; *drops = s_dq_drops;
    portEXIT_CRITICAL(&s_mux);
}

void gba_bridge_echo_stats(unsigned *enq, unsigned *deq, unsigned *drops, unsigned *coalesced) {
    portENTER_CRITICAL(&s_mux);
    if (enq)       *enq = s_eq_enq;
    if (deq)       *deq = s_eq_deq;
    if (drops)     *drops = s_eq_drops;
    if (coalesced) *coalesced = s_eq_coalesced;
    portEXIT_CRITICAL(&s_mux);
}

unsigned gba_bridge_upq_coalesced(void) { return s_gq_coalesced; }
#ifdef KZ_ONLINE_PACING
void gba_bridge_set_replay_delay(int frames) {
    portENTER_CRITICAL(&s_mux);
    if (!s_up_fr_init) up_fr_reset();
    fr_set_delay(&s_up_fr, frames);
    portEXIT_CRITICAL(&s_mux);
}
void gba_bridge_replay_info(int *delay, unsigned *late, unsigned *rebased, unsigned *catchup) {
    portENTER_CRITICAL(&s_mux);
    if (delay)   *delay   = s_up_fr_init ? s_up_fr.delay : FR_DELAY_DEFAULT;
    if (late)    *late    = s_up_fr.late;
    if (rebased) *rebased = s_up_fr.rebased;
    if (catchup) *catchup = s_up_fr.catchup + (s_up_fr.slewed_late << 16);   /* low 16: catch-up, high 16: slewed later */
    portEXIT_CRITICAL(&s_mux);
}
unsigned gba_bridge_replay_shed(void) { return s_up_fr.shed_idle; }
#else
void gba_bridge_set_replay_delay(int frames) { (void)frames; }   /* local: no replay */
void gba_bridge_replay_info(int *delay, unsigned *late, unsigned *rebased, unsigned *catchup) {
    if (delay) *delay = 0;
    if (late) *late = 0;
    if (rebased) *rebased = 0;
    if (catchup) *catchup = 0;
}
unsigned gba_bridge_replay_shed(void) { return 0; }
#endif
unsigned gba_bridge_up_hold_repeats(void) { return s_up_hold_repeats; }
void gba_bridge_dnq_stats(unsigned *enq, unsigned *deq, unsigned *drops, int *depth) {
    portENTER_CRITICAL(&s_mux);
    *enq = s_dq_enq; *deq = s_dq_deq; *drops = s_dq_drops; *depth = (s_dq_head - s_dq_tail + SW_Q_N) % SW_Q_N;
    portEXIT_CRITICAL(&s_mux);
}
void gba_bridge_mirror_stats(unsigned *n, uint16_t *w0, uint16_t *w1, unsigned *age_ms) {
    *n = s_mir_n; *w0 = s_mir_w0; *w1 = s_mir_w1;
    *age_ms = s_mir_us ? (unsigned)((esp_timer_get_time() - s_mir_us) / 1000) : 0;
}
void gba_bridge_upq_stats(unsigned *enq, unsigned *deq, unsigned *drops, int *depth) {
    portENTER_CRITICAL(&s_mux);
#ifdef KZ_ONLINE_PACING
    if (!s_battle) {
        if (enq)   *enq   = s_up_fr.enq;
        if (deq)   *deq   = s_up_fr.deq;
        if (drops) *drops = s_up_fr.drops;
        if (depth) *depth = s_up_fr_init ? fr_depth(&s_up_fr) : 0;
        portEXIT_CRITICAL(&s_mux);
        return;
    }
#endif
    if (enq)   *enq   = s_gq_enq;
    if (deq)   *deq   = s_gq_deq;
    if (drops) *drops = s_gq_drops;
    if (depth) *depth = (s_gq_head - s_gq_tail + GBA_Q_N) % GBA_Q_N;
    portEXIT_CRITICAL(&s_mux);
}

void gba_bridge_put_host_slot(int mpid, const uint8_t slot14[14]) {
    /* tr-1.10: mpId = slot index in the Switch's gRecvCmds (gbaframe.h:35, "N x 14-byte gRecvCmds"), matching
     * the SAME 5-row layout our own build_recv uses (row0=parent/Switch slot, row1=GBA echo, rows2-4=other
     * players). We relay only ONE row down to the GBA (row0 = the Switch's real content). ldn_brain.c's
     * eng_feed loops over EVERY slot in a single incoming Switch frame and called this once per slot with NO
     * mpid filtering -> a frame containing e.g. slot0=real content + slot1=empty (the GBA's own echo bounced
     * back, or an unused other-player row) overwrote the SAME shared s_switch_words buffer twice in a row,
     * stomping the real value with zero moments later, and (post-tr-1.09) triggering s_uni_new_pending TWICE
     * per incoming frame instead of once -- root cause of the rapid real-value/zero alternation observed on
     * hardware ([dn] SW->GBA w0=8905 / w0=0000 / w0=8906 / w0=0000 ..., tr-1.09 test). Only mpid==0 (the
     * Switch's own slot) is meaningful to relay; ignore everything else. */
    if (mpid == 1) {
        /* tr-1.60 DIAGNOSTIC only: the host's row1 = its mirror of OUR child slots (owner 0x81) — the
         * direct measure of which of the GBA's frames the host actually latched. Log it (change-gated,
         * throttled) during the post-save re-entry so a future stall shows exactly where the host's
         * receive side stopped. NOT relayed to the GBA (tr-1.59 tried that; regression). */
        /* Kizuna: log ALWAYS (change-gated, capped), not only post-save. This row is the
         * Switch's gRecvCmds[1] = the last child slot its tag check ACCEPTED (tag stripped). If it stops
         * advancing while [up] keeps emitting, the Switch is discarding our slots (rolling-tag desync). */
        {
            static uint16_t m0, m1, m2; static int mdbg;
            uint16_t w0 = get_u16le(slot14), w1 = get_u16le(slot14 + 2), w2 = get_u16le(slot14 + 4);
            if (s_battle && w0 && (w0 != s_mir_w0 || w1 != s_mir_w1)) {   /* uncapped summary for the health line */
                s_mir_w0 = w0; s_mir_w1 = w1; s_mir_n++; s_mir_us = esp_timer_get_time();
            }
            bool key_repeat = s_battle && (w0 & 0xFF00u) == 0xBE00u && (m0 & 0xFF00u) == 0xBE00u && (w1 & 0xFFu) == (m1 & 0xFFu);
            if (mdbg < 600 && (w0 != m0 || w1 != m1 || w2 != m2) && !key_repeat && (!s_battle || w0 || m0 != 0xBE00u)) {
                mdbg++; m0 = w0; m1 = w1; m2 = w2;
                KZ_TRACE("  [mirror] SW row1 (accepted from GBA) w0=%04x w1=%04x w2=%04x\n", w0, w1, w2);
            }
        }
        return;
    }
    if (mpid != 0) return;
    uint16_t w0 = get_u16le(slot14), w1 = get_u16le(slot14 + 2), w2 = get_u16le(slot14 + 4);
    /* tr-1.64 DOWN COUNTER TRANSLATION (see s_sc_ctr_offset decl): the host's 0x6600 carries ITS
     * resendExitStandbyCount, which is `offset` ahead of the GBA's real one after a C6-driven takeover.
     * Translate to the GBA's domain BEFORE any internal use or relay, and patch the slot copy the GBA
     * receives so link_rfu_2.c:1179 (w1 must equal the receiver's own counter) matches on both sides.
     * Gated on !s_sc_takeover: while our own takeover echo is in flight both sides already compare in the
     * HOST's native domain (see the echo handler below), so translating that traffic too would break the
     * very loop the offset is derived from. */
    uint8_t xslot[14];
    const uint8_t *sl = slot14;
    if ((w0 & 0xFF00u) == 0x6600u && s_sc_ctr_offset != 0 && !s_sc_takeover) {
        for (int i = 0; i < 14; i++) xslot[i] = slot14[i];
        w1 = (uint16_t)(w1 - (uint16_t)s_sc_ctr_offset);
        put_u16le(xslot + 2, w1);
        sl = xslot;
        static int xdbg;
        if (xdbg < 30) { xdbg++;
            printf("  [sc-x] DOWN 0x6600 count %u -> %u (offset %d)\n",
                   (unsigned)(uint16_t)(w1 + s_sc_ctr_offset), (unsigned)w1, s_sc_ctr_offset); }
    }
    /* tr-1.51 save-chain continuation (see s_sc_* decl). tr-1.53b: RE-ARM per trade. The save-chain state is
     * per-TRADE, but gba_bridge_reset/note_gba_join_selected only fire per JOIN. Without re-arming, trade #2 in
     * the same session would keep s_sc_reexchanged=true from trade #1 -> the takeover gate stays off -> trade
     * #2's save chain deadlocks (the original bug). Re-arm on a CONFIRM_FINISH only when we're NOT mid-chain
     * (first arm: !s_sc_active; subsequent: s_sc_reexchanged, i.e. the previous trade's chain already ended) so
     * a repeated 0xDCBA within one trade never resets mid-save-chain. */
    if (w1 == 0xDCBAu && (!s_sc_active || s_sc_reexchanged)) {
        s_sc_active = true; s_sc_reexchanged = false; s_sc_takeover = false;
        s_sc_gba_count = 0; s_sc_gba_rounds = 0; s_sc_sw_count = 0;
        s_sc_take_count = 0; s_sc_take_emitted = 0xFFFF; s_sc_take_emit_us = 0;
        printf("  [sc] CONFIRM_FINISH -> save chain (re-)armed\n");
    }
    if (s_sc_active && !s_sc_reexchanged) {
        uint16_t hi = (uint16_t)(w0 & 0xFF00u);
        if (hi == 0x6600u) {                           /* Switch echoed a standby count */
            s_sc_sw_count = w1;
            if (s_sc_takeover) {                        /* our C6-driven round: advance in lockstep, SUPPRESS to GBA */
                if (w1 == s_sc_take_count) { s_sc_take_count++;
                    /* tr-1.64: this round was completed by the host on the GBA's behalf (it's parked) — the
                     * host's persistent counter is now one further ahead of the GBA's real one. Track it so
                     * every 0x6600 checkpoint AFTER this takeover (this trade's own end, or any future
                     * trade's) gets translated back into agreement (see s_sc_ctr_offset decl). */
                    s_sc_ctr_offset++;
                    printf("  [sc] Switch echoed count=%u -> drive %u (ctr offset now %d)\n",
                           (unsigned)w1, (unsigned)s_sc_take_count, s_sc_ctr_offset); }
                return;                                 /* do NOT relay our own takeover echo to the parked GBA */
            }
        } else if (hi == 0xA100u) {                     /* Switch re-pulled the party -> chain over, hand back */
            if (s_sc_takeover || s_sc_gba_rounds > 0) {
                if (!s_sc_reexchanged) printf("  [sc] Switch re-exchange (0xA100) -> ending chain, relaying to GBA\n");
                s_sc_reexchanged = true; s_sc_takeover = false;
            }
        }
    }
    portENTER_CRITICAL(&s_mux);
    s_dn_in++;
    bool changed = false;
    for (int i = 0; i < 7; i++) {
        uint16_t v = get_u16le(sl + i * 2);   /* tr-1.64: `sl` = counter-translated view (== slot14 otherwise) */
        if (v != s_switch_words[i]) changed = true;
        s_switch_words[i] = v;
    }
    s_switch_len = 7;
    if (!changed) s_dn_same++;
    /* tr-1.21: ALSO queue it, so a burst of block fragments can't overwrite each other before the GBA's next
     * 0x26 poll. Skip all-zero idle slots -- only real content needs once-only sequenced delivery. */
    if (changed) {
        bool nonzero = false;
        for (int i = 0; i < 7; i++) if (s_switch_words[i]) { nonzero = true; break; }
        if (!nonzero) s_dn_zero++;
        if (nonzero) {
            if ((s_switch_words[0] & 0xFF00u) == 0xBE00u) s_dn_be++;
            /* tr-1.26: coalesce back-to-back SEND_HELD_KEYS (0xBE00). In the seat phase the host emits one
             * per VBlank (60Hz, SendKeysToRfu link_rfu_2.c:1069-1080 — the high byte is a rolling nonce so
             * every one is "changed") while the GBA drains one slot per ~2 frames; without coalescing the
             * FIFO overflows and drop-OLDEST can eat a critical one-shot (0x6600 round, block INIT/REQ, the
             * host's READY 0x16). Freshest-wins is the correct semantic for held keys: the game stores only
             * the latest (gLinkPartnersHeldKeys[i] = gRecvCmds[i][1], :1217) and reads the low-byte key; a
             * slower real link would equally deliver only the freshest. Only a 0xBE00 replaces a 0xBE00 —
             * every other opcode still gets lossless once-only FIFO order. */
            /* NO folding of 0xBE00 here any more. Folding kept only the newest held-keys frame, which destroyed the
             * Switch player's press lengths before they even reached the relay; the GBA side now replays every frame on
             * the GBA's own clock (the GBA side's uni_echo -> frame_replay). The drain runs at the brain's 60 Hz (rfu1_bridge).
             * Thinning instead: the GBA latches the partner's keys only on a SEND_HELD_KEYS command (link_rfu_2.c:1217) and
             * keeps them otherwise, so only a frame whose KEY BYTE changes carries information; the high byte is the
             * Switch's per-frame counter, which the GBA side uses to place it in time. One keepalive per DN_KEEP frames
             * keeps that counter unambiguous. 60 messages/s down starved the GBA board's socket. */
            /* (battle): never thinned or merged -- every Switch slot reaches the GBA once, in order (a merged READY
             * left the GBA waiting forever; merged directions desynced the avatars). */
            bool thin = false;
#ifdef KZ_ONLINE_PACING
            static uint8_t s_dn_last_key, s_dn_last_ctr; static bool s_dn_have_key;
            if (s_battle) s_dn_have_key = false;
            else if ((s_switch_words[0] & 0xFF00u) == 0xBE00u) {
                uint8_t key = (uint8_t)s_switch_words[1], ctr = (uint8_t)(s_switch_words[1] >> 8);
                if (s_dn_have_key && key == s_dn_last_key && (uint8_t)(ctr - s_dn_last_ctr) < DN_KEEP) { thin = true; s_dn_fold++; }
                else { s_dn_have_key = true; s_dn_last_key = key; s_dn_last_ctr = ctr; }
            } else s_dn_have_key = false;        /* any other command: the next held-keys frame goes through */
#else
            /* local (switch-trade-v2.4, tr-1.26): a held-keys slot replaces a queued held-keys slot at the newest position
             * (freshest wins); every other opcode appends. Not in battle. */
            {
                int newest = (s_dq_head + SW_Q_N - 1) % SW_Q_N;
                if (!s_battle && s_dq_head != s_dq_tail
                    && (s_dq[newest][0] & 0xFF00u) == 0xBE00u && (s_switch_words[0] & 0xFF00u) == 0xBE00u) {
                    for (int i = 0; i < 7; i++) s_dq[newest][i] = s_switch_words[i];
                    thin = true; s_dn_fold++;
                }
            }
#endif
            if (!thin) {
                int nxt = (s_dq_head + 1) % SW_Q_N;
                int depth = (s_dq_head - s_dq_tail + SW_Q_N) % SW_Q_N;
                if (depth >= (s_battle ? SW_Q_N : SW_Q_TRADE) - 1) { s_dq_tail = (s_dq_tail + 1) % SW_Q_N; s_dq_drops++; }  /* full -> drop-oldest */
                for (int i = 0; i < 7; i++) s_dq[s_dq_head][i] = s_switch_words[i];
                s_dq_head = nxt; s_dq_enq++;
            }
        }
    }
    portEXIT_CRITICAL(&s_mux);
    /* DIAGNOSTIC: log each DISTINCT Switch->GBA slot (the host's actual trade content), rate-capped.
     * tr-1.27: cap raised 120->600 — the tr-1.26 run exhausted 120 lines on the seat-phase held-keys flood
     * within ~3s of the walk-in, blinding the log to the entire party exchange + trade menu that followed. */
    /* a held key that only moved its nonce (high byte of w1) is not logged — those repeats used up the
     * whole 600-line budget in the room, so the battle start was never seen. */
    static uint16_t last0, last1, last2; static int dbg;
    /* held-keys walking (0xBE00) keeps a cap, but every other slot is always logged, with a timestamp, so the
     * trade-computer exchange can be read off even in a long session. (battle): a held key that only moved its
     * nonce is not logged, and battle blocks/standby keep a 4000-line cap. */
    bool dn_routine = (w0 & 0xFF00u) == 0xBE00u;
    bool key_repeat = s_battle && dn_routine && (last0 & 0xFF00u) == 0xBE00u && (w1 & 0xFFu) == (last1 & 0xFFu);
    bool not_keys = s_battle && w0 && !dn_routine;
    bool log_it = s_battle ? (!key_repeat && (not_keys ? dbg < 4000 : dbg < 600)) : (!dn_routine || dbg < 3000);
    if ((w0 != last0 || w1 != last1 || w2 != last2) && log_it) { if (s_battle || dn_routine) dbg++;
        KZ_TRACE("  [dn %lld] SW->GBA w0=%04x w1=%04x w2=%04x\n", (long long)(esp_timer_get_time() / 1000), w0, w1, w2);
        last0 = w0; last1 = w1; last2 = w2;
    }
    /* tr-1.52 DIAGNOSTIC: uncapped-during-phase trace of the menu-re-entry party re-exchange (post save-chain),
     * to diff vs frlgsim (which does REQ type 1x3 + mail(3) + ribbons(4)). The normal [dn] cap is exhausted by
     * this point in a full trade. Gated on s_sc_reexchanged so it only fires in the target phase. */
    if (s_sc_reexchanged) { static uint16_t r0, r1, r2; static int rdbg;
        if (rdbg < 400 && (w0 != r0 || w1 != r1 || w2 != r2)) { rdbg++; r0 = w0; r1 = w1; r2 = w2;
            printf("  [re-dn] SW->GBA %04x %04x %04x\n", w0, w1, w2); } }
}

int gba_bridge_ni_stage(void) { return s_ni_stage; }
