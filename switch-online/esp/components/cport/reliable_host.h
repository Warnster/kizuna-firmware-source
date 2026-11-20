/* SPDX-License-Identifier: AGPL-3.0-only
 * Derived from pokeldn (https://github.com/Decryptu/pokeldn), Copyright (c) the pokeldn authors.
 * Changes Copyright (c) 2026 Kizuna. See NOTICE.
 */
/* reliable_host.h — the LEADER side of Pia Reliable protocol 10. Portable C port of pokeldn
 * ldn/reliable.py HostReliableSession and the current upstream ReliableLink it drives (the peer's
 * hole is fast-retransmitted at once, then one round trip apart, where cport reliable.c — the older
 * frlgsim joiner window — resends it once). Wire helpers (rel_build_reliable, rel_build_bulk_ack,
 * rel_parse_bulk_ack) come from reliable.c.
 *
 * Both streams start at FFF0. The leader opens its stream with the RFU 'A' as an Initialized DATA
 * frame (flagsA 15), later frames are 7; a DATA frame is acked by a bulk ack (seq FFF0, flagsA 0,
 * Pia message flag 0x40) at most every ack_period_ms; retransmits carry message flag 0x20.
 * Received DATA is delivered in order, de-duplicated, through a callback.
 *
 * Bounded RAM (pokeldn's dicts are unbounded): the send window is RELH_WINDOW frames (pokeldn
 * MAX_INFLIGHT 128; HostSession pauses at max_inflight anyway), and out-of-order DATA waits in
 * RELH_RX_SLOTS slots of RELH_RX_INNER bytes. An out-of-order frame with no room is treated as
 * lost: not acked, so the peer resends it. Time is double milliseconds (pokeldn passes now*1000.0);
 * no clocks inside. */
#ifndef RELIABLE_HOST_H
#define RELIABLE_HOST_H
#include <stdint.h>
#include <stddef.h>

#define RELH_START            0xFFF0
#define RELH_WINDOW           48    /* send ring = the max_inflight ceiling */
#define RELH_INNER_MAX        96    /* RFUL_FRAME_MAX: a parent UNI 'T' is 88 */
#ifndef RELH_RX_SLOTS
#define RELH_RX_SLOTS         64    /* child frames held for in-order delivery */
#endif
#define RELH_RX_INNER         64    /* the console's frames are <= 46 (metadata) */
#define RELH_RTT_WINDOW       7
#define RELH_RTO_BASE_MS      33.0  /* also the delayed-ack interval */
#define RELH_RTO_RTT_FACTOR   1.4
#define RELH_FAST_RETX_MIN_MS 25.0

#define RELH_FLAGSA_GBA  0x07
#define RELH_FLAGSA_INIT 0x0f
#define RELH_FLAGSA_CTRL 0x00

typedef struct {
    double ack_period_ms;      /* 33 */
    int    max_inflight;       /* <= RELH_WINDOW */
    int    has_rto_bootstrap;  /* 1: RTO before the first RTT sample */
    double rto_bootstrap_ms;   /* 200 */
    int    dup_nack_threshold; /* 1 */
    int    retransmit_limit;   /* -1 = none; HostSession passes HOST_RTX_LIMIT 2 */
} relh_cfg;
extern const relh_cfg RELH_DEFAULT_CFG;   /* HostReliableSession() defaults, max_inflight clamped */

/* ReliableEmission. payload points into the session (send ring / ack buffer): valid until the
 * next relh_receive / relh_poll / relh_send. */
typedef struct {
    uint16_t seq;
    uint8_t  flagsA;
    uint16_t ack;              /* window base sampled at emission */
    uint8_t  retransmitted;
    const uint8_t *payload;
    int      len;
} relh_emission;

int relh_message_flags(const relh_emission *e);                 /* 0x40 ack, 0x20 resend, -1 none */
int relh_serialize(const relh_emission *e, uint8_t *out);       /* 8 + len bytes */

typedef struct {
    uint8_t flagsA, len, acked;
    uint16_t resends;
    double  last_tx;
    uint8_t inner[RELH_INNER_MAX];
} relh_tx;

typedef struct { uint8_t used, flagsA, len; uint16_t seq; uint8_t payload[RELH_RX_INNER]; } relh_rx;

typedef void (*relh_deliver_fn)(void *ctx, uint16_t seq, uint8_t flagsA, const uint8_t *p, int len);

typedef struct {
    relh_cfg cfg;
    uint16_t start_seq;
    /* ReliableLink: unacked = [window_lo, out_seq), always contiguous, ring at `head` */
    uint16_t out_seq, window_lo;
    int      head;
    relh_tx  tx[RELH_WINDOW];
    uint16_t recv_next;        /* ack accounting == delivery order (pokeldn keeps two equal counters) */
    double   rtt[RELH_RTT_WINDOW]; int rtt_n;
    int      has_peer_gap; uint16_t peer_gap; int gap_nacks;
    /* HostReliableSession */
    int      local_opened, peer_opened, ack_owed;
    int      has_next_ack; double next_ack_ms;
    relh_rx  rx[RELH_RX_SLOTS]; int rx_n;
    uint8_t  ack_buf[20];
    int      rx_unheld;        /* out-of-order frames left unacked for want of a slot (must stay 0) */
} relh_t;

void relh_init(relh_t *r, const relh_cfg *cfg);    /* cfg NULL = RELH_DEFAULT_CFG */
int  relh_inflight(const relh_t *r);
int  relh_outstanding(const relh_t *r);             /* unacked, not yet peer-acked */
uint16_t relh_send_low(const relh_t *r);
/* The unacked entry for seq (acked or not), or NULL: carry-forward re-reads payloads here. */
const relh_tx *relh_find(const relh_t *r, uint16_t seq);

/* Idempotent: 0 if already opened, else 1 and *out (flagsA |= Initialized). */
int  relh_open(relh_t *r, const uint8_t *payload, int len, double now_ms, uint8_t flagsA, relh_emission *out);
/* 1 and *out, -1 not opened, -2 window full, -3 not a DATA flagsA. */
int  relh_send(relh_t *r, const uint8_t *payload, int len, double now_ms, uint8_t flagsA, relh_emission *out);
void relh_note_rtt(relh_t *r, double rtt_ms);
/* One Reliable(10) message payload. DATA newly in order goes to `deliver` (in order); returns how
 * many were delivered. Bulk acks update the window and RTT; they are never acked. */
int  relh_receive(relh_t *r, const uint8_t *wire, int len, double now_ms, relh_deliver_fn deliver, void *ctx);
/* Due retransmits (oldest first, stamped, retransmit_limit) then the delayed ack. Returns count. */
int  relh_poll(relh_t *r, double now_ms, relh_emission *out, int max_out);

#endif
