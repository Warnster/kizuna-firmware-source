/* SPDX-License-Identifier: AGPL-3.0-only
 * Derived from pokeldn (https://github.com/Decryptu/pokeldn), Copyright (c) the pokeldn authors.
 * Changes Copyright (c) 2026 Kizuna. See NOTICE.
 */
/* pia_activity.h — what pia_hsess needs from the thing running on top of the RFU leader (a Mystery
 * Gift engine, a trade engine). Mirrors the attributes pokeldn's HostSession reads off `activity`:
 * feed_child_slot(), tick(), disconnect_requested, mark_disconnect_sent(), and the "console's
 * LinkPlayer arrived" test behind HostPeerProtocol's established check.
 *
 * Every callback receives `ctx` back. All are required. */
#ifndef PIA_ACTIVITY_H
#define PIA_ACTIVITY_H
#include <stdint.h>

typedef struct pia_activity {
    void *ctx;
    /* One new child UNI command row (idle rows too). */
    void (*feed_child_slot)(void *ctx, const uint8_t slot[14]);
    /* The parent's gSendCmd row for this VBlank: seven u16 words (all zero = idle). */
    void (*tick)(void *ctx, uint16_t words[7]);
    /* 1 once the engine wants the session closed (after its READY_CLOSE_LINK went out). */
    int  (*disconnect_requested)(void *ctx);
    /* The 'D' frame went out. */
    void (*mark_disconnect_sent)(void *ctx);
    /* 1 once the console's LinkPlayer block has arrived (HostPeerProtocol: property started). */
    int  (*established)(void *ctx);
} pia_activity;

#endif
