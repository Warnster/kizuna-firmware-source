/* SPDX-License-Identifier: AGPL-3.0-only
 * Derived from frlg-ldn-trade-esp32 (https://github.com/easyworld/frlg-ldn-trade-esp32), Copyright (c) Easy World and the frlg-ldn-trade-esp32 authors.
 * Changes Copyright (c) 2026 Kizuna. See NOTICE.
 */
/* gba_bridge — the seam between the C6's PARLIO GBA command-phase (GBA task) and the LDN trade brain
 * (brain_task). Two separate joins exist (see docs): GBA<->C6-adapter (RFU NI join, answered locally by the
 * command phase) and C6-sim<->Switch (Pia NI, driven by the sim). Once both are up, TRADE UNI slots are
 * relayed here: GBA 0x24/0x25 -> sim engine->tick; sim engine->feed_in_frame -> GBA 0x26.
 *
 * STAGE 1 (instrument + safe plumbing): capture the real GBA data-phase traffic, buffer the Switch's slot
 * for 0x26, keep the GBA->sim tick IDLE (don't feed the Switch malformed slots and regress pc_conn).
 * Thread-safe: the GBA task and brain_task touch the shared slots under a short critical section. */
#ifndef GBA_BRIDGE_H
#define GBA_BRIDGE_H
#include <stdint.h>
#include <stdbool.h>

/* ---- GBA command-phase side (called from the GBA task, gba_login_main.c) ---- */
/* A data-carrying command from the GBA completed (0x24/0x25 = send its slot). data = the LL words. */
void gba_bridge_on_gba_cmd(uint8_t cmd, const uint32_t *data, int ndata);
/* Build the 0x26 (data-phase pull) reply words from the Switch's latest slot. Returns nout (>=1);
 * out[0] is the RFU count/length header. Returns a single count=0 word when no host slot is ready. */
int  gba_bridge_build_recv(uint32_t *out, int max_out);
void gba_bridge_clear_join_selected(void);   /* tr-1.31: session re-arm clears the 0x1f room-selected latch */
/* tr-0.85 host-NI RELAY: the brain tees the Switch's REAL parent-NI subframes here (eng_feed); build_recv
 * serves them verbatim to the GBA. peek/advance split so a frame preempted by an inline child-ack is
 * re-staged, never dropped: the dp task calls recv_advance() only after the ISR confirms the staged frame
 * was actually served (sm->staged_served). */
void gba_bridge_push_host_ni(int state, int n, int phase, const uint8_t *payload, int len);
void gba_bridge_recv_advance(void);
/* M1: the GBA finished its own NI (NI_END) -> flip 0x26 replies from count=0 keepalive to UNI idle/relay. */
void gba_bridge_set_gba_ni_done(bool v);
/* tr-0.94: the child sent a UNI slot (state 4) -> CHILD_JOINED, its 70B UNI recv buffer is up -> safe to send
 * PARENT UNI frames. This is the real gate (NI_END was too early -> ERR_RECV_BUFF_OVER reset). */
void gba_bridge_set_gba_uni_started(bool v);

/* ---- brain side (called from ldn_brain.c engine callbacks) ---- */
/* engine->tick: hand the sim the GBA's current outgoing trade slot (7 u16). Returns false = idle. */
bool gba_bridge_take_gba_slot(uint16_t words[7]);
/* engine->feed_in_frame: the Switch sent us a slot for the GBA (14-byte gRecvCmd, mpId=slot index). */
void gba_bridge_put_host_slot(int mpid, const uint8_t slot14[14]);

/* ---- GBA identity (from its 0x16 GAME_CONFIG broadcast) — fed to the sim so the Switch's NI join shows
 * the REAL trainer, not the "EMU" default. Japanese carts have a kana name that only the raw FRLG-charmap
 * bytes can carry, so we pass name_raw (8 bytes) + language. ---- */
void gba_bridge_set_gba_identity(uint16_t tid, const uint8_t name_raw[8], uint8_t language, uint8_t activity);
bool gba_bridge_get_gba_identity(uint16_t *tid, uint8_t name_raw[8], uint8_t *language, uint8_t *activity);

/* ---- Join gate: the Switch must only get a join request once the GBA PLAYER selects its room (the GBA
 * sends 0x1f connect to the presented peer). Until then the C6 stays beacon-only (present the room, no join),
 * so "Emu wants to join" doesn't pop on the Switch the instant the GBA opens the wireless menu. ---- */
void gba_bridge_note_gba_join_selected(void);   /* command phase saw the GBA's 0x1f connect to the peer */
bool gba_bridge_gba_join_selected(void);         /* has the GBA chosen to join the Switch's room? */

/* Release the GBA from its join-wait ("Le") into the trade: set true once the REAL Switch approves
 * (host_uni_seen || ni_recv.status==JOIN_GROUP_OK). Until then the parent NI feeds status=0x00 (hold);
 * once released it feeds 0x05 (JOIN_GROUP_OK). Level-triggered — call every brain tick. */
void gba_bridge_set_join_released(bool released);
bool gba_bridge_join_released(void);   /* read by the command phase to pace itself during the approval wait */

/* RFU1 path: the child's NI subframes are acked by the parent. ni_scan_child (in on_gba_cmd)
 * records the last unacked START/NI/END; this hands it out ONCE as the 3-byte parent ack LLSF (state | ack=1 |
 * n | phase | slot0) to be sent as its own HOST_SEND right after the child's frame. Returns false = nothing
 * pending. (The C6 served it ISR-side in the very next 0x26; the RP2040 queues NI frames in order.) */
bool gba_bridge_take_child_ack(uint8_t llsf[3]);

void gba_bridge_reset(void);
int  gba_bridge_ni_stage(void);   /* 0-4 NI join-status, 5 = UNI trade phase (for data-phase logging) */

/* UP-FIFO (GBA->Switch) diagnostics: enqueued / dequeued / dropped-on-overflow counts + current depth. */
unsigned gba_bridge_upq_coalesced(void);
/* count of changes in the host's accepted copy of our slot (row1), its latest value, ms since it changed. */
/* Colosseum room (activity 1/2) -> battle relay rules; trade rooms keep the trade rules. */
void gba_bridge_set_battle(bool on);
bool gba_bridge_battle(void);
void gba_bridge_dnq_stats(unsigned *enq, unsigned *deq, unsigned *drops, int *depth);
void gba_bridge_mirror_stats(unsigned *n, uint16_t *w0, uint16_t *w1, unsigned *age_ms);
void gba_bridge_upq_stats(unsigned *enq, unsigned *deq, unsigned *drops, int *depth);
/* row1 echo FIFO health (GBA slots echoed back to the GBA, one per delivered parent frame). */
void gba_bridge_echo_stats(unsigned *enq, unsigned *deq, unsigned *drops, unsigned *coalesced);

#endif /* GBA_BRIDGE_H */

/* Up-FIFO held-keys merging (same key byte, always on). The setter is kept for the online op; [upq] coal= counts merges. */
void gba_bridge_set_coalesce_up_keys(bool on);
