/* SPDX-License-Identifier: AGPL-3.0-only
 * Derived from frlg-ldn-trade (https://github.com/tornadus/frlg-ldn-trade), Copyright (c) the frlg-ldn-trade authors.
 * Changes Copyright (c) 2026 Kizuna. See NOTICE.
 */
/* Pia MESSAGE tiling + Reliable(10) sub-header + the reliable sliding window.
 * C port of frlgsim/reliable.py (host-testable; ESP32-bound). The one transport layer that
 * had no C port. Times are in MILLISECONDS. Defaults are console-faithful. */
#ifndef RELIABLE_H
#define RELIABLE_H
#include <stdint.h>
#include <stddef.h>

#define REL_PROTO_RELIABLE 10
#define REL_FLAGSA_GBA  0x07   /* AppData|Start|End  — a complete single-fragment data frame */
#define REL_FLAGSA_INIT 0x0f   /* +Initialized — stream-opening frame */
#define REL_FLAGSA_CTRL 0x00   /* no AppData — payload is bulk-ack data */
#define REL_STATION_HOST   0x7620
#define REL_STATION_JOINER 0xc493

/* Sized for the ESP's DRAM: the sliding window is MAX_INFLIGHT=24, and a reliable inner is always
 * one small gbaframe (wrapped 14-byte RFU slot ~20B, metadata 46B, K 16B) — never a bulk block.
 * 48 entries gives ample slack over the 24 window; 96B inner covers every frame. */
#define REL_MAX_INFLIGHT 48
#define REL_MAXINNER     96    /* max reliable inner payload (Pia message) we buffer */
#define REL_RTT_WINDOW   7

/* --- stateless serialization --- */

/* Build a self-contained message header (size+proto always present) + payload. msgflags<0 => omit.
 * Returns total bytes written to out. */
int rel_build_message(uint8_t proto, const uint8_t *payload, int plen, int msgflags, uint8_t *out);

/* Reliable(10) 8-byte sub-header + inner. Returns total bytes. */
int rel_build_reliable(uint16_t seq, uint16_t ack, const uint8_t *inner, int inner_len,
                       uint8_t flagsA, uint8_t *out);

/* Bulk-ack payload (20 bytes): stream_id, 01, next_expected(BE2), mask(16). */
int rel_build_bulk_ack(uint16_t next_expected, const uint8_t mask[16], uint8_t stream_id, uint8_t *out);

/* Parse a bulk-ack payload -> ack_id + 16-byte mask. Returns 0 ok, -1 too short. */
int rel_parse_bulk_ack(const uint8_t *payload, int len, uint16_t *ack_id, uint8_t mask[16]);

typedef struct { uint8_t flagsA; uint16_t seq; uint16_t ack; const uint8_t *payload; int payload_len; } rel_reliable_t;
/* Parse Reliable(10) sub-header. Returns 0 ok (payload points into input), -1 too short. */
int rel_parse_reliable(const uint8_t *payload, int len, rel_reliable_t *out);

/* Concatenate already-built message bytes + station footer(BE2) + pad(0xff*pad). Returns total. */
int rel_build_app(const uint8_t *msgs, int msgs_len, uint16_t station_id, int pad, uint8_t *out);

typedef struct { uint8_t flags; uint8_t msgflags; int proto; int size; const uint8_t *payload; int hdr_len; } rel_message_t;
/* Tile a Pia blob into messages (payload ptrs point into data). Returns count; *consumed set. */
int rel_parse_messages(const uint8_t *data, int n, rel_message_t *msgs, int max_msgs, int *consumed);
/* Full blob -> messages + station(-1 if none) + tail_len. Returns message count. */
int rel_parse_app(const uint8_t *data, int n, rel_message_t *msgs, int max_msgs,
                  int *station, int *tail_len);

/* --- ReliableLink sliding window --- */
typedef struct { int used; uint16_t seq; uint8_t flagsA; uint8_t inner[REL_MAXINNER]; int inner_len;
                 uint32_t last_tx; int resends; int acked; } rel_entry_t;
typedef struct { int used; uint16_t seq; uint8_t payload[REL_MAXINNER]; int len; } rel_recv_t;

typedef struct {
    uint16_t out_seq, window_lo, recv_next;
    rel_entry_t unacked[REL_MAX_INFLIGHT];
    rel_recv_t  recv_buf[REL_MAX_INFLIGHT];
    uint16_t recv_ooo[REL_MAX_INFLIGHT]; int recv_ooo_n;
    double rtt[REL_RTT_WINDOW]; int rtt_n;
    /* console-faithful knobs (see reliable.py) */
    double rtt_jitter_k; int dup_nack_threshold;
    int has_rto_ceil; double rto_ceil_ms;
    double rto_backoff;
    int has_rto_bootstrap; double rto_bootstrap_ms;
    /* peer-gap dup-NACK tracking */
    int has_peer_gap; uint16_t peer_gap; int gap_nacks;
} rel_link_t;

void rel_link_init(rel_link_t *L, uint16_t start);
uint16_t rel_queue(rel_link_t *L, const uint8_t *inner, int inner_len, uint8_t flagsA, uint32_t now_ms);
uint16_t rel_send_low(const rel_link_t *L);
int rel_inflight(const rel_link_t *L);
int rel_outstanding(const rel_link_t *L);
/* 1 if an unacked (not-yet-peer-acked) entry exists for seq (carry-forward eligibility). */
int rel_entry_active(const rel_link_t *L, uint16_t seq);
/* Const lookup of an unacked entry by seq, or NULL (for carry-forward inner bytes). */
const rel_entry_t *rel_find(const rel_link_t *L, uint16_t seq);
void rel_add_rtt_sample(rel_link_t *L, double rtt_ms);
/* Returns 1 and sets *out_rto if a timeout is armed; 0 if None (no samples/bootstrap). */
int rel_rto(const rel_link_t *L, double *out_rto);
/* Process peer bulk-ack. Pass mask=all-zero for cumulative-only. has_now=1 to sample RTT. */
void rel_on_ack(rel_link_t *L, uint16_t ack_id, const uint8_t mask[16], int has_now, uint32_t now_ms);

typedef struct { uint16_t seq; uint8_t flagsA; const uint8_t *inner; int inner_len; } rel_due_t;
/* Frames due to resend (oldest-gap first), stamps them resent. limit<0 = all. Returns count. */
int rel_due_retransmits(rel_link_t *L, uint32_t now_ms, int limit, rel_due_t *out, int max_out);

/* Accept a received DATA frame; copies deliverable-in-order payloads into `deliv`. Returns count. */
int rel_on_data(rel_link_t *L, uint16_t seq, const uint8_t *payload, int plen,
                rel_recv_t *deliv, int max_deliv);
/* Live-path selective-repeat accounting (no in-order delivery). */
void rel_note_received(rel_link_t *L, uint16_t seq);
/* Build our bulk-ack (cumulative recv_next + selective mask of recv_ooo). Writes 20 bytes. */
int rel_ack_payload(rel_link_t *L, uint8_t *out);

#endif
