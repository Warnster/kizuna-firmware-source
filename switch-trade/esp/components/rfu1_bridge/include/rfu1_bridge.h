/* rfu1_bridge — the GBA seam on the two-chip board.
 *
 * The RP2040 (GB-Link mode 0x07) emulates the wireless adapter toward the GBA and relays RFU1 network packets
 * over the GB UART. It is a TRANSPARENT adapter: it runs login, the 2 MHz command phase and all word timing,
 * but no parent protocol. So this component is the RFU PARENT the GBA child talks to — the same role the C6's
 * command phase + gba_bridge played — with a packet transport instead of 0x26 polls:
 *
 *   GBA -> us : BROADCAST (the GBA's own 0x16 beacon while it hosts in the Union Room) -> identity + beacon
 *               template; CONNECT_REQ(our devid) -> "the GBA selected the Switch's room"; CLIENT_SEND -> the
 *               child's 0x24/0x25 words -> gba_bridge_on_gba_cmd (UP FIFO to the Switch, NI acks).
 *   us -> GBA : BROADCAST(our room = the Switch, 4 Hz) while the room is present and we are a member;
 *               CONNECT_ACK; HOST_SEND carrying exactly the parent link-layer frame bytes gba_bridge builds
 *               (child acks, the JOIN_GROUP_OK NI once the Switch approves, then the UNI stream).
 *
 * Threading: UART bytes are queued from the gblink RX task; everything else runs in the caller's tick
 * (brain_task, 60 Hz), so gba_bridge's "GBA task" state stays single-threaded as designed. */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "rfu1.h"

/* Bring up the GB UART (tx/rx = ESP GPIOs to RP2040 GP9/GP8), check the RP2040 answers GetFirmwareInfo and
 * enter mode 0x07. Returns false if the RP2040 is absent/silent (the app then falls back to console triggers). */
bool rfu1_bridge_start(int tx_gpio, int rx_gpio);   /* tx_gpio < 0: host link over the USB console (gblink) */
bool rfu1_bridge_up(void);
int  rfu1_bridge_login_state(void);
int64_t rfu1_bridge_gba_quiet_us(void);     /* how long the GBA side has been silent (no link activity at all) */        /* RP2040 login telemetry state: >= 2 = a GBA is past login (browsing or linked) */
void rfu1_bridge_isr_stats(void);           /* GetRfuIsrStats (0x4C); the reply prints as [rp-isr] */
/* Call every ~16.7 ms from the brain task. Drains RX, feeds gba_bridge, sends beacons/acks/HOST_SENDs. */
void rfu1_bridge_poll(void);
/* Leave mode 0x07 (Cancel) and forget the session; start() again to resume. */
void rfu1_bridge_stop(void);
/* One JSON object for the status line. */
void rfu1_bridge_status(char *out, size_t cap);

/* Hooks the app provides (trade_session): */
void rfu1_bridge_on_gba_browsing(void);      /* first RFU1 traffic / login seen -> associate to the Switch's room */
void rfu1_bridge_on_gba_selected(void);      /* CONNECT_REQ for our devid -> start the sim join */
void rfu1_bridge_on_gba_gone(void);          /* DISCONNECT / login restart -> end the session, re-arm */
bool rfu1_bridge_room_visible(void);         /* may we beacon the Switch's room to the GBA right now? */
bool rfu1_bridge_room_identity(char name[9], uint16_t *tid);   /* the Switch host's name/tid for the beacon */
bool rfu1_bridge_room_identity_raw(uint8_t name_raw[8]);     /* host's in-game name, FRLG charmap, if decoded */
uint8_t rfu1_bridge_room_activity(void);                       /* the Switch room's activity for the beacon: 4 trade, 1/2 battle, 0 unknown (weak default 0) */

/* the GBA leads (Direct Corner "Become the leader"): it hosts, so the RP2040 relays its BROADCASTs. */
/* The GBA's own group, if a BROADCAST arrived in the last second: its 0x16 data (24 bytes, the game's byte order) and devid. */
bool rfu1_bridge_gba_hosting(uint8_t beacon[24], uint16_t *devid);
/* Once set, every packet from the RP2040 goes to fn (called from rfu1_bridge_poll) and none to the Switch-leads path. */
void rfu1_bridge_set_lead_sink(void (*fn)(const rfu1_pkt_t *p));
/* One complete RFU1 packet to the RP2040. */
bool rfu1_bridge_send_raw(const uint8_t *pkt, uint32_t len);
