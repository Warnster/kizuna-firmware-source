/* SPDX-License-Identifier: AGPL-3.0-only
 * Derived from frlg-ldn-trade-esp32 (https://github.com/easyworld/frlg-ldn-trade-esp32), Copyright (c) Easy World and the frlg-ldn-trade-esp32 authors.
 * Changes Copyright (c) 2026 Kizuna. See NOTICE.
 */
/* ldn_brain.h — the standalone trade brain: cport/sim.c driven by the ESP32-C6's own LDN I/O.
 * Once started (after the LDN join + IPs are up), it runs the Pia handshake, NI registration and
 * RFU slot exchange with the Switch on-chip — no PC in the trade path. On the C6 the GBA-facing
 * engine is STUBBED (idle all-zero slots): this task wires the real Switch-side handshake so the
 * Switch reaches the trade join Yes/No prompt; the real GBA cartridge slots are a later task. */
#pragma once
#include <stdint.h>
#include <stdbool.h>

struct sim_engine;
/* Inject a sim engine (option B: a cable-link image passes cable_air's engine). Call BEFORE
 * ldn_brain_start. When set, the built-in gba_bridge engine/identity/join-release are skipped. */
void ldn_brain_set_engine(struct sim_engine *e);
/* The Switch room's activity (4 trade, 1/2 battle, 0 unknown): what we join as when the cart's own is not known. */
void ldn_brain_set_room_activity(uint8_t activity);

bool ldn_brain_active(void);
/* Allocate the brain's heap state (sim + connection, ~28 KB) now; true on success. Call early at boot in the join role. */
bool ldn_brain_prealloc(void);
bool ldn_brain_prealloc_sim(void);  /* just the ~25 KB sim state */
void ldn_brain_release(void);       /* free the connection + sim scratch while stopped */
void ldn_brain_free_all(void);      /* ldn_brain_release() plus the sim state: everything, while stopped */
/* ssid(16), our_ip(4), host_ip(4) octets, our_mac(6)=station MAC, host_mac(6)=Switch BSSID.
 * Returns 0 on success. */
int  ldn_brain_start(const uint8_t ssid[16], const uint8_t our_ip[4], const uint8_t host_ip[4],
                     const uint8_t our_mac[6], const uint8_t host_mac[6], const char *joiner_name);
void ldn_brain_stop(void);
/* Minimum spacing between GBA slots taken toward the Switch (default 15 ms; online sets 12 ms). */
void ldn_brain_set_up_floor_us(int64_t us);
/* Why the session ended (diagnostics): note BEFORE ldn_brain_stop(); the first note per session wins, default "external". */
void ldn_brain_note_stop(const char *why);
const char *ldn_brain_stop_reason(void);
/* Feed one inbound LDN datagram (from ldn_udp_poll). src_ip = 4 octets. */
void ldn_brain_on_datagram(const uint8_t src_ip[4], const uint8_t *data, int len);
/* Advance one VBlank; call at ~60 Hz. */
void ldn_brain_tick(void);
/* Kizuna: the tick loop fell behind by n VBlanks (a stall). Advances the sim's clock so
 * its RTO/ack timers track wall time, but emits NOTHING for the missed periods: catch-up ticks that each
 * emitted an UP slot put two child slots inside one console poll, which the FRLG parent treats as a dropped
 * rolling tag (fatal after five). */
void ldn_brain_skip_ticks(int n);
/* reliable UP window for the sim. Call before or after ldn_brain_start (a value set before start is
 * applied at start). Local trade: leave the default (24, 2). Online (RP2040 across the relay): (6, 1). */
void ldn_brain_set_window(int max_inflight, int silence_ceil);

/* True once the Switch has accepted our join (NI host_status == JOIN_GROUP_OK, i.e. the human tapped
 * "yes" on the Switch's join prompt). a cable-link image feeds this into cable_air's confirmation gate
 * so the GBA holds its "Press A" until the Switch is actually in. Stays true once set. */
bool ldn_brain_join_accepted(void);
