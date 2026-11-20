/* SPDX-License-Identifier: AGPL-3.0-only
 * Derived from frlg-ldn-trade (https://github.com/tornadus/frlg-ldn-trade), Copyright (c) the frlg-ldn-trade authors.
 * Derived from pokeldn (https://github.com/Decryptu/pokeldn), Copyright (c) the pokeldn authors.
 * Derived from frlg-ldn-trade-esp32 (https://github.com/easyworld/frlg-ldn-trade-esp32), Copyright (c) Easy World and the frlg-ldn-trade-esp32 authors.
 * Changes Copyright (c) 2026 Kizuna. See NOTICE.
 */
/* ldn_brain.c — cport/sim.c (the FRLG<->LDN trade brain) driven by the ESP32-C6's own LDN I/O.
 *
 * Inbound LDN datagrams are fed to sim_feed_datagram(); a ~60 Hz tick drives sim_tick(); sim's send
 * callback goes straight to ldn_udp_send(). The sim engine is BRIDGED to the REAL GBA over the C6's own
 * PARLIO link (gba_bridge): eng_tick pulls the GBA's outgoing trade slot (0x24/0x25) up to the Switch, and
 * eng_feed relays the Switch's UNI slots down to the GBA (0x26). The real cartridge identity (tid/name/
 * language/activity) is fed from its 0x16 broadcast. So the C6 carries the ENTIRE trade data layer end-to-end;
 * completion depends on NI->UNI convergence on hardware (see TEST_ON_WAKE.md). [Historical: this engine was
 * once a stub during the S3 port — it is fully wired now.] */
#include "ldn_join.h"
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"   /* vTaskPrioritySet — single-core fairness boost during the approval wait */
#include "esp_random.h"
#include "esp_heap_caps.h"
#include "ldn_brain.h"
#include "ldn_udp.h"
#include "sim.h"
#include "ni.h"        /* ni_sender_done, NI_STATUS_JOIN_GROUP_OK */
#include "crypto.h"    /* FRLG_GAME_KEY, pia_crypto_init, pia_decrypt, pia_is_pia */
#include "pia_zstd.h"  /* pia_decompress (DG decrypt/decompress diagnostic) */
#include "gba_bridge.h"
#include "esp_timer.h" /* the GBA<->sim trade-slot seam */

/* ~28 KB of join-role state: heap objects (not statics) so an image that also has the host role pays for them only when the join role
 * runs. ldn_brain_prealloc() takes them early in the boot, before Wi-Fi/Bluetooth fragment the heap; ldn_brain_start() allocates
 * on demand if nobody did. */
static sim_t     *s_simp;
static pc_conn_t *s_connp;
#define s_sim  (*s_simp)
#define s_conn (*s_connp)
static sim_engine s_engine;
static pia_crypto s_pc;
static bool       s_active;

/* Optional engine override (option B): a cable-link image injects cable_air's sim_engine here so
 * the brain drives the CABLE translator instead of the gba_bridge (RFU-wireless) engine. When set, the
 * built-in gba_bridge engine setup, identity capture and join-release are all skipped (cable_air owns
 * them). NULL => the wireless-trade path, unchanged. */
static sim_engine *s_engine_override;
static const char *s_stop_why;
void ldn_brain_set_engine(sim_engine *e) { s_engine_override = e; }
static uint8_t s_room_activity;
static int64_t s_up_floor_us;
static bool    s_battle_floor;
static int64_t s_floor_before_battle;
void ldn_brain_set_room_activity(uint8_t activity) {
    s_room_activity = activity;
    bool battle = activity == 1 || activity == 2;
    gba_bridge_set_battle(battle);   /* Colosseum rooms use the battle relay rules */
    /* battle mode merges nothing, so the up FIFO must drain at the GBA's 60/s. At the local 15 ms floor
     * the ~16.7 ms loop's jitter deferred 151 of 430 slots in the room (hw run 8: depth 126, 4 drops); the online
     * path's 12 ms floor keeps one slot per tick. Trade rooms keep whatever floor was set. */
    if (battle && !s_battle_floor) { s_floor_before_battle = s_up_floor_us; s_up_floor_us = 12000; s_battle_floor = true; }
    else if (!battle && s_battle_floor) { s_up_floor_us = s_floor_before_battle; s_battle_floor = false; }
}

/* progress diagnostics (printed once each, over the C6's own USB) */
static int s_seen_accepted, s_seen_ni_done, s_seen_uni, s_seen_conn;
static int s_win_inflight, s_win_ceil;   /* 0 = sim defaults */

/* ---- engine: bridged to the real GBA over the C6's own PARLIO command phase (gba_bridge). ----
 * FAITHFUL RELAY: forward every GBA slot to the Switch exactly ONCE, in order (the GBA itself generates the
 * seat held-keys, establishment LinkPlayer blocks and party blocks — we do NOT generate any of that, we only
 * carry it). gba_bridge's UP FIFO guarantees no drop/duplicate; empty -> all-zero idle. */
/* Kizuna: wall-clock spacing of consecutive NON-IDLE up slots. The FRLG parent keeps one
 * child slot per poll (~16.7 ms); two inside one poll = a dropped rolling tag, fatal on the fifth. Any gap
 * under ~15 ms here is a violation candidate; the counts print with the [upq] health line. */
static int64_t  s_up_last_us;
#define UP_SLOT_MIN_GAP_US 15000
/* Online (host link) runs a 12 ms floor: at 15 ms, tick jitter around the 16.7 ms loop deferred
 * ~10 takes/s, so the drain was ~50/s against the GBA's 60/s and the up FIFO overflowed in the held-keys room.
 * The clumping the floor guards against was 1-5 ms (two slots in one poll); 12 ms still excludes it. */
static int64_t s_up_floor_us = UP_SLOT_MIN_GAP_US;   /* declared above ldn_brain_set_room_activity */
void ldn_brain_set_up_floor_us(int64_t us) { s_up_floor_us = us; }
static uint32_t s_up_gaps_short, s_up_gap_min_us = 0xFFFFFFFFu, s_up_fresh, s_up_deferred;
static void eng_tick(void *ctx, uint16_t words[7]) {
    (void)ctx;
    /* One GBA slot per Switch poll enforced HERE, where the slot is taken, not only at tick start:
     * on the C3 the per-tick datagram work before this point varies 1-12 ms, so two ticks 15 ms apart took
     * their slots as little as 8.5 ms apart (bench: under15ms=39, the Switch stopped acking after
     * the LinkPlayer exchange, NETWORK_DESTROYED). Too soon -> this tick sends an idle slot (the parent skips
     * idle slots) and the GBA slot stays queued for the next tick. */
    int64_t take_now = esp_timer_get_time();
    bool too_soon = s_up_last_us && (take_now - s_up_last_us) < s_up_floor_us;
    bool fresh = too_soon ? false : gba_bridge_take_gba_slot(words);
    if (too_soon) s_up_deferred++;
    if (!fresh) for (int i = 0; i < 7; i++) words[i] = 0;      /* nothing queued -> idle (never a stale repeat) */
    if (fresh) {
        int64_t now = esp_timer_get_time();
        if (s_up_last_us) {
            uint32_t gap = (uint32_t)(now - s_up_last_us);
            if (gap < s_up_gap_min_us) s_up_gap_min_us = gap;
            if (gap < 15000) s_up_gaps_short++;
        }
        s_up_last_us = now; s_up_fresh++;
    }
    /* DIAGNOSTIC: log each DISTINCT slot forwarded up to the Switch (0xbe00=seat held-keys, block cmds, etc.),
     * plus a periodic FIFO health line so drops/lag are visible. */
    /* hw the 600-line cap hid the whole room-exit sequence (run 2). Always log a frame that is not a
     * plain 'no key' held-keys slot (0xBE00 with key byte 0x11); the routine held-keys stream keeps the cap. */
    static uint16_t last0, last1, last2; static int updbg, tickn;
    bool routine = (words[0] & 0xFF00u) == 0xBE00u && (words[1] & 0x00FFu) == 0x11u;
    if (fresh && (words[0] != last0 || words[1] != last1 || words[2] != last2) && (!routine || updbg < 600)) { if (routine) updbg++;
        printf("  [up] GBA->SW w0=%04x w1=%04x w2=%04x\n", words[0], words[1], words[2]);
        last0 = words[0]; last1 = words[1]; last2 = words[2];
    }
    /* health line every ~5 s of wall time (was every 300 eng_tick calls, which during an idle gap is minutes) */
    static int64_t s_health_us;
    { int64_t now = esp_timer_get_time();
      if (now - s_health_us > 5000000) { s_health_us = now;
        unsigned enq, deq, drops; int depth;
        gba_bridge_upq_stats(&enq, &deq, &drops, &depth);
        printf("  [upq] enq=%u deq=%u drops=%u coal=%u depth=%d | up-gap: fresh=%u min=%uus under15ms=%u deferred=%u\n",
               enq, deq, drops, gba_bridge_upq_coalesced(), depth, (unsigned)s_up_fresh,
               s_up_gap_min_us == 0xFFFFFFFFu ? 0u : (unsigned)s_up_gap_min_us, (unsigned)s_up_gaps_short,
               (unsigned)s_up_deferred);
        if (gba_bridge_battle()) {   /* battle-mode health line */
        unsigned mn, mage; uint16_t mw0, mw1;
        gba_bridge_mirror_stats(&mn, &mw0, &mw1, &mage);
        unsigned denq, ddeq, ddrops; int ddepth;
        gba_bridge_dnq_stats(&denq, &ddeq, &ddrops, &ddepth);
        printf("  [mir] changes=%u last=%04x/%04x age=%ums | up last=%04x/%04x | dnq enq=%u deq=%u drops=%u depth=%d\n",
               mn, mw0, mw1, mage, last0, last1, denq, ddeq, ddrops, ddepth);
        }
      } }
    (void)tickn;
}
/* tr-1.27: lets the sim's drive_reliable drain a small burst (SIM_UP_BURST_MAX) when the GBA relay queue has
 * more real slots pending — the brain tick runs at the GBA poll rate (~26/s), exactly the production rate, so
 * a 1/tick drain has zero margin and the queue only ever grows (tr-1.26 hw: depth pegged 30/32, drops=5,
 * ~1.1s standing UP latency). */
static int eng_has_backlog(void *ctx) {
    (void)ctx;
    int depth = 0;
    gba_bridge_upq_stats(NULL, NULL, NULL, &depth);
    return depth > 0;
}
static void eng_feed(void *ctx, const gba_in *rec) {
    (void)ctx;
    /* DIAGNOSTIC: log the host's NI sub-frames (what the Switch sends at the RFU/NI layer). */
    static int nidbg;
    if (rec->has_ni && nidbg < 40) { nidbg++;
        printf("  [brain] HOSTNI st=%d ack=%d n=%d ph=%d sz=%d p0=%02x p1=%02x\n",
               rec->llsf_state, rec->ni_ack, rec->ni_n, rec->ni_phase, rec->ni_size,
               rec->ni_payload_len > 0 ? rec->ni_payload[0] : 0,
               rec->ni_payload_len > 1 ? rec->ni_payload[1] : 0);
    }
    /* M1 (tr-0.86): the tr-0.85 host-NI tee is RETIRED — a parent does NOT stream NI data to a child
     * (librfu establishment is child->parent only; the host's NI here is only acks). What the GBA needs
     * DOWN is the Switch's UNI stream, relayed below via gba_bridge_put_host_slot. */
    /* Relay the Switch's UNI trade slots down to the GBA (via 0x26). mpId = slot index; the host is 0. */
    for (int i = 0; i < rec->nslots && i < 5; i++)
        gba_bridge_put_host_slot(i, rec->slots[i]);
}

/* ---- transport send: sim -> LDN UDP ---- */
static void brain_send(void *ctx, const uint8_t *dg, size_t len, const uint8_t dst_ip[4]) {
    (void)ctx;
    ldn_udp_send(dst_ip, dg, (int)len);
}

bool ldn_brain_active(void) { return s_active; }

/* Only the sim state, the one large (~25 KB) piece. Taken at boot by a mode whose session heap is too fragmented to find
 * it later (after Bluetooth stops the C3 can be left with no 25 KB block next to the zstd decoder's 31 KB). */
bool ldn_brain_prealloc_sim(void) {
    if (!s_simp) s_simp = heap_caps_calloc(1, sizeof *s_simp, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    return s_simp != NULL;
}

/* Give back everything but the sim state between sessions (the connection state and sim's batch scratch, ~16.5 KB),
 * so a mode that idles on Bluetooth gets it back; the next ldn_brain_start() takes them again. Only when stopped. */
void ldn_brain_release(void) {
    if (s_active) return;
    if (s_connp) { heap_caps_free(s_connp); s_connp = NULL; }
    sim_release();
}

/* Everything, the boot-time sim reserve included: lent to a Bluetooth image push, which ends in a restart either way. */
void ldn_brain_free_all(void) {
    if (s_active) return;
    ldn_brain_release();
    if (s_simp) { heap_caps_free(s_simp); s_simp = NULL; }
}

bool ldn_brain_prealloc(void) {
    if (!s_simp)  s_simp  = heap_caps_calloc(1, sizeof *s_simp,  MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!s_connp) s_connp = heap_caps_calloc(1, sizeof *s_connp, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    return s_simp && s_connp && sim_prealloc();
}

/* The Switch accepted our join (human tapped "yes"): NI reply status == JOIN_GROUP_OK. host_uni_seen
 * is the same milestone one step later (the trade link came up), so either one means "in". */
bool ldn_brain_join_accepted(void) {
    if (!s_simp) return false;
    return (s_sim.ni_recv.has_status && s_sim.ni_recv.status == NI_STATUS_JOIN_GROUP_OK)
        || s_sim.host_uni_seen;
}

int ldn_brain_start(const uint8_t ssid[16], const uint8_t our_ip[4], const uint8_t host_ip[4],
                    const uint8_t our_mac[6], const uint8_t host_mac[6], const char *joiner_name) {
    if (!ldn_brain_prealloc()) { printf("  [brain] out of memory for the sim state\n"); return -1; }
    pia_crypto_init(&s_pc, ssid, FRLG_GAME_KEY);

    const char *jn = (joiner_name && joiner_name[0]) ? joiner_name : "Kizuna";
    uint8_t random4[4];
    esp_fill_random(random4, sizeof(random4));
    pc_conn_init(&s_conn, our_mac, host_mac, our_ip, host_ip, PC_DEFAULT_OUR_VAR,
                 jn, random4, NULL);

    /* cable path (B): use the injected engine (cable_air) verbatim — it owns identity + slots. */
    sim_engine *eng = s_engine_override;
    if (!eng) {
        /* STUB engine identity: the sim's built-in LinkPlayer defaults (version_low=5, tid=0x8822,
         * name "EMU", activity=TRADE) apply for any field left zero — see sim.c. The real cartridge
         * identity (tid/name/version/activity from its 0x16 broadcast) is wired with the real GBA seam. */
        memset(&s_engine, 0, sizeof(s_engine));
        s_engine.tick          = eng_tick;
        s_engine.has_backlog   = eng_has_backlog;   /* tr-1.27: burst-drain the UP relay queue */
        s_engine.feed_in_frame = eng_feed;
        s_engine.poll_send_done = NULL;
        s_engine.in_seat_phase = 0;
        s_engine.established    = 0;
        s_engine.ni_started     = 1;   /* default True (sim.c also enforces this) */

        /* Feed the REAL cartridge identity (captured from its 0x16 broadcast by the command phase) so the
         * Switch's NI join shows the actual trainer + game, not the "EMU" default. JP cart => raw kana name. */
        uint16_t gtid; uint8_t gname[8], glang, gact;
        if (gba_bridge_get_gba_identity(&gtid, gname, &glang, &gact)) {
            s_engine.lp_trainer_id     = gtid;
            memcpy(s_engine.lp_name_raw, gname, 8);
            s_engine.lp_name_raw_valid = 1;
            s_engine.lp_language       = glang;
            s_engine.lp_version_low    = 4;   /* FireRed = 4 (Emerald=3/FireRed=4/LeafGreen=5) */
            if (gact) { s_engine.ni_activity = gact; s_engine.ni_started = 1; }
            printf("  [brain] identity tid=0x%04x name=%02x%02x%02x%02x lang=%u act=0x%02x (from cart)\n",
                   gtid, gname[0], gname[1], gname[2], gname[3], glang, gact);
        } else {
            printf("  [brain] no cart identity yet -> joining as Kizuna default\n");
        }
        /* Without the cart's own activity, join as the room's: a battle room refuses a trade join. */
        if (!s_engine.ni_activity && s_room_activity) { s_engine.ni_activity = s_room_activity; s_engine.ni_started = 1; }
        eng = &s_engine;
    } else {
        printf("  [brain] using injected engine (cable_air)\n");
    }

    uint8_t connect_id[2];
    esp_fill_random(connect_id, sizeof(connect_id));
    if (!connect_id[0] && !connect_id[1]) connect_id[0] = 1;

    uint64_t nonce = 0;
    esp_fill_random(&nonce, sizeof(nonce));
    if (nonce == 0) nonce = 1;

    sim_init(&s_sim, &s_pc, eng, brain_send, NULL, our_ip, host_ip, &s_conn,
             connect_id, 2, nonce);

    s_seen_accepted = s_seen_ni_done = s_seen_uni = s_seen_conn = 0;
    if (s_win_inflight > 0) sim_set_window(&s_sim, s_win_inflight, s_win_ceil);
    s_stop_why = NULL;
    s_active = true;
    printf("  [brain] STARTED (GBA bridged via PARLIO) us=%u.%u.%u.%u host=%u.%u.%u.%u joiner='%s'\n",
           our_ip[0], our_ip[1], our_ip[2], our_ip[3],
           host_ip[0], host_ip[1], host_ip[2], host_ip[3], jn);
    return 0;
}

void ldn_brain_note_stop(const char *why) { if (!s_stop_why) s_stop_why = why; }
const char *ldn_brain_stop_reason(void) { return s_stop_why ? s_stop_why : "external"; }

void ldn_brain_stop(void) {
    if (!s_active) return;
    s_active = false;
    printf("  [brain] STOPPED why=%s rx=%d tx=%d\n", ldn_brain_stop_reason(), s_sim.rx_count, s_sim.tx_count);
}

void ldn_brain_on_datagram(const uint8_t src_ip[4], const uint8_t *data, int len) {
    if (!s_active) return;
    static int dbg;
    if (dbg < 4) { dbg++;
        static const char *H = "0123456789abcdef";
        char hx[256]; int n = len < 120 ? len : 120;
        for (int i = 0; i < n; i++) { hx[2*i] = H[data[i] >> 4]; hx[2*i+1] = H[data[i] & 15]; }
        hx[2*n] = 0;
        printf("  [brain] DG in len=%d src=%u.%u.%u.%u dg=%s\n", len,
               src_ip[0], src_ip[1], src_ip[2], src_ip[3], hx);
        /* (tr-0.34 DIAG confirmed decrypt=80/decompress=166 succeed — the RX stall was the brain-task
         * stack overflow, fixed by the 24K stack. No per-datagram decode here; sim does it.) */
    }
    sim_feed_datagram(&s_sim, data, len, src_ip);
}

void ldn_brain_set_window(int max_inflight, int silence_ceil) {
    s_win_inflight = max_inflight; s_win_ceil = silence_ceil;
    if (s_active && max_inflight > 0) sim_set_window(&s_sim, max_inflight, silence_ceil);
    printf("  [brain] reliable window: inflight=%d ceil=%d%s\n", max_inflight, silence_ceil, s_active ? "" : " (applied at start)");
}

void ldn_brain_skip_ticks(int n) {
    if (!s_active || n <= 0) return;
    sim_skip_ticks(&s_sim, n);
}

void ldn_brain_tick(void) {
    if (!s_active) return;
    sim_tick(&s_sim);

    /* JOIN-RELEASE COORDINATION (S3 307dff7): hold the GBA at "Le" until the REAL Switch approves, then
     * release it into the trade. Level-triggered every tick.
     * tr-1.15 FIX: user directly observed on real hardware that the GBA was released before a human could
     * possibly have clicked the Switch's real accept/decline prompt. `host_uni_seen` was dropped from this
     * condition -- this project's build log (tr-0.25) documents its ORIGINAL observed sequence as "NI
     * status -> host_uni_seen -> Switch shows the trade join Yes/No prompt": host_uni_seen was ALWAYS known to
     * fire before the prompt even appears, so it can never represent a genuine accept. ni_recv.status is the
     * field the Switch actually uses to carry its real decision (refreshed from each received NI subframe,
     * ni.c:169) -- NI_STATUS_JOIN_GROUP_OK(5) is "yes"; any other value already trips ni_rejected downstream
     * (sim.c:309), so "no" was already wired, we just weren't correctly waiting for the real "yes". */
    bool approved = s_sim.ni_recv.has_status && s_sim.ni_recv.status == NI_STATUS_JOIN_GROUP_OK;
    if (!s_engine_override) {
        gba_bridge_set_join_released(approved);   /* cable path: cable_air gates forwarding itself */
    }

    /* (No priority juggling — the SINGLE LOOP ticks this inline from the GBA command phase, so there is no
     * separate task to prioritize. approved just gates the join-release above.) */
    (void)approved;

    /* Surface milestones (once each) so progress is visible over the C6 USB. */
    if (!s_seen_conn && sim_connected(&s_sim)) { s_seen_conn = 1; printf("  [brain] connected\n"); }
    if (!s_seen_accepted && s_sim.gba_accepted) { s_seen_accepted = 1; printf("  [brain] host_accepted\n"); }
    if (!s_seen_ni_done && s_sim.ni_done) { s_seen_ni_done = 1; printf("  [brain] ni_done\n"); }
    if (!s_seen_uni && s_sim.host_uni_seen) { s_seen_uni = 1; printf("  [brain] host_uni (trade ready)\n"); }
    if (s_sim.ni_rejected) { printf("  [brain] ni_REJECTED\n"); ldn_brain_note_stop("ni_rejected"); ldn_brain_stop(); return; }
    if (s_sim.host_disconnected) { printf("  [brain] host_disconnect\n"); ldn_brain_note_stop("host_d"); ldn_brain_stop(); return; }
    if (s_sim.d_sent && s_sim.tick_count - s_sim.d_tick >= SIM_LEAVE_GRACE_TICKS) {
        printf("  [brain] child_leave (our 'D' sent %d ticks ago) -> leaving the room\n", (int)(s_sim.tick_count - s_sim.d_tick));
        ldn_brain_note_stop("child_leave"); ldn_brain_stop(); return;
    }

    /* NI/UNI handshake visibility: whether OUR NI-send to the Switch is complete, whether the Switch
     * acked it, whether it sent its join-status, and whether it reached UNI. Prints on any change so
     * the exact stall point in the next hardware run is legible. */
    {
        static int last_sig = -1;
        int sig = (s_sim.gba_accepted        ? 1 : 0)
                | (ni_sender_done(&s_sim.ni)  ? 2 : 0)
                | (s_sim.has_host_ni_ack_key  ? 4 : 0)
                | (s_sim.ni_recv.has_status   ? 8 : 0)
                | (s_sim.host_uni_seen        ? 16 : 0)
                | (s_sim.ni_done              ? 32 : 0);
        if (sig != last_sig) { last_sig = sig;
            printf("  [brain] NI accepted=%d our_ni_sent=%d host_acked=%d host_status=%d(val=%d) host_uni=%d ni_done=%d\n",
                   s_sim.gba_accepted, ni_sender_done(&s_sim.ni), s_sim.has_host_ni_ack_key,
                   s_sim.ni_recv.has_status, s_sim.ni_recv.has_status ? s_sim.ni_recv.status : -1,
                   s_sim.host_uni_seen, s_sim.ni_done);
        }
        /* NI-advance diagnostic: the sim only advances our NI send when the host's ack (state,n,phase)
         * EXACTLY matches the sub-frame we're awaiting (sim.c:357). Log both so a mismatch is visible. */
        static int last_aw = -1;
        int aw = (s_sim.ni_aw_state << 8) | (s_sim.ni_aw_n << 4) | s_sim.ni_aw_phase
               | (s_sim.hnak_state << 20) | (s_sim.hnak_n << 16) | (s_sim.hnak_phase << 12)
               | (s_sim.has_ni_awaiting ? 1 << 24 : 0) | (s_sim.has_host_ni_ack_key ? 1 << 25 : 0);
        if (aw != last_aw) { last_aw = aw;
            printf("  [brain] NIadv awaiting(st=%d n=%d ph=%d have=%d) vs hostAck(st=%d n=%d ph=%d have=%d) -> %s\n",
                   s_sim.ni_aw_state, s_sim.ni_aw_n, s_sim.ni_aw_phase, s_sim.has_ni_awaiting,
                   s_sim.hnak_state, s_sim.hnak_n, s_sim.hnak_phase, s_sim.has_host_ni_ack_key,
                   (s_sim.has_host_ni_ack_key && s_sim.hnak_state == s_sim.ni_aw_state &&
                    s_sim.hnak_n == s_sim.ni_aw_n && s_sim.hnak_phase == s_sim.ni_aw_phase) ? "MATCH(advance)" : "mismatch(wait)");
        }
    }

    /* PC_CONN STALL TRACE: the join is gated on pc_conn reaching PC_ST_CONNECTED (NET->FINALIZE->
     * CONNECTED, pia_connect.c). Print the whole connect vector on any change so ONE capture pins the
     * exact stall: is the Switch's datagram decrypting (rx_count up / rx_fail flat)? did we learn the
     * host var + send our join (has_host_var/has_join_sent)? are we transmitting responses (tx)? */
    {
        static int last = -1;
        int sig = s_conn.state
                | (s_conn.has_host_var  ? 1 << 4 : 0)
                | (s_conn.has_join_sent ? 1 << 5 : 0)
                | (s_sim.learned        ? 1 << 6 : 0)
                | ((s_sim.tx_count > 0) ? 1 << 7 : 0)
                | ((s_sim.rx_fail  > 0) ? 1 << 8 : 0);
        if (sig != last) { last = sig;
            printf("  [brain-pc] state=%d(%s) learned_hostvar=%d/%d join_sent=%d rx_ok=%d rx_fail=%d tx=%d\n",
                   s_conn.state,
                   s_conn.state == 0 ? "NET" : s_conn.state == 1 ? "FINALIZE" : "CONNECTED",
                   s_sim.learned, s_conn.has_host_var, s_conn.has_join_sent,
                   s_sim.rx_count, s_sim.rx_fail, s_sim.tx_count);
        }
    }

    /* ~1s heartbeat so we can see whether the Switch's Pia datagrams are arriving. */
    static int hb;
    if (++hb % 60 == 0 || (s_sim.rx_count && s_sim.rx_count < 4)) {
        printf("  [brain-hb] ipfwd=%u rx=%d rxfail=%d tx=%d pia=%d acc=%d ni=%d uni=%d\n",
               (unsigned)ldn_join_ip_frames(), s_sim.rx_count, s_sim.rx_fail, s_sim.tx_count,
               s_conn.state, s_sim.gba_accepted, s_sim.ni_done, s_sim.host_uni_seen);
    }
}
