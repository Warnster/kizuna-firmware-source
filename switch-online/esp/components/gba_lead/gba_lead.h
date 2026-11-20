/* SPDX-License-Identifier: AGPL-3.0-only
 * Derived from GB-Link-Switch-LDN (https://github.com/GB-Link/GB-Link-Switch-LDN), Copyright (c) the GB-Link-Switch-LDN authors.
 * Changes Copyright (c) 2026 Kizuna. See NOTICE.
 */
/* gba_lead.h -- the GBA LEADS a Direct Corner group (Trade Center or Colosseum, "Become the leader") and the
 * Switch joins it. The board hosts an LDN room that mirrors the GBA's group (ldn_ap + dist_udp), plays the Switch's Pia
 * session host (glp_host, GB-Link's) and relays each game's own RFU frames between them:
 *
 *   GBA HOST_SEND (its parent UNI/NI frame)  -> lead shim -> glp_host_parent_frame -> the Switch
 *   the Switch's child frame                 -> lead shim -> RFU1 CLIENT_SEND      -> the GBA
 *
 * The Switch's join is passed on as an RFU1 CONNECT_REQ, so the GBA's own "accept?" decides it. A port of GB-Link-Switch-
 * LDN firmware/common/pia_bridge.c's BR_HOST role (commit c972db4, AGPL-3.0; see NOTICE). Runs on the brain task: call
 * gba_lead_poll once per 60 Hz tick, after rfu1_bridge_poll. The session restarts the board when it is over, for a clean
 * heap, as every other session does. */
#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "ldn_advert.h"

/* A beacon (the GBA's 0x16 data, 24 bytes) worth hosting: a Trade Center or Colosseum group (activity 4, 1 or 2). The
 * Union Room's beacons carry other activities and are ignored. */
bool gba_lead_wanted(const uint8_t beacon[24]);
/* Wi-Fi initialised; the join role stopped (no ldn_join, no promiscuous scan). Keys are borrowed for the session. */
bool gba_lead_start(const ldn_keys *keys, const uint8_t beacon[24], uint16_t gba_devid);
bool gba_lead_active(void);
/* Once per brain tick (59.7 Hz nominal): UDP in, the Pia tick, the GBA connect/ack/close timers. */
void gba_lead_poll(void);
/* One JSON fragment for the status line ("lead":{...}), without braces around it. */
void gba_lead_status(char *out, size_t cap);

/* cable stand-in (switch-online, GBA_LEAD_STANDIN): this board leads a FireRed trade group for the Switch on behalf
 * of a GBA in a Cable Club far away (ct_leader + ct_reverse + ct_link). tx sends one GB-Link-style message to the phone
 * (ch 1 = 64-byte LINK_DATA, ch 2 = a LINK_STATUS u16); the phone forwards the relay's LINK_CMD bytes and LINK_DATA back
 * through gba_lead_standin_rx (0xE0 / 0xE1 on ch 0 = the peer joined / left the relay room). */
typedef void (*gba_lead_tx_fn)(uint8_t ch, const uint8_t *p, size_t n);
bool gba_lead_start_standin(const ldn_keys *keys, gba_lead_tx_fn tx);
bool gba_lead_standin_active(void);
void gba_lead_standin_resend(void);   /* the current link status, for a phone that just took over the relay seat */
void gba_lead_standin_rx(uint8_t ch, const uint8_t *p, size_t n);
