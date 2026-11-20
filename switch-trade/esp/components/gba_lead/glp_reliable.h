/* SPDX-License-Identifier: AGPL-3.0-only
 * Derived from GB-Link-Switch-LDN (https://github.com/GB-Link/GB-Link-Switch-LDN), Copyright (c) the GB-Link-Switch-LDN authors.
 * Changes Copyright (c) 2026 Kizuna. See NOTICE.
 */
/* glp_reliable.h -- copied from GB-Link-Switch-LDN firmware/common/pia_reliable.h (commit c972db4, AGPL-3.0,
 * https://github.com/GB-Link/GB-Link-Switch-LDN); see NOTICE. Kizuna: identifiers renamed pia_ -> glp_, PIA_ -> GLP_,
 * bin_ -> glb_ so they sit beside cport's own Pia host in one image. */
#pragma once
#include "glp_bin.h"

/* Pia reliable stream: selective repeat, 128-bit ack bitmap, RTO from recent round
   trips, fast retransmit on a repeated gap report. */

#define GLP_RELIABLE_SLOTS 64
#define GLP_RELIABLE_PAYLOAD 256
#define GLP_RELIABLE_OOO 64
#define GLP_RTT_SAMPLES 7

typedef struct
{
    uint8_t data[GLP_RELIABLE_PAYLOAD];
    uint16_t length;
    uint16_t seq;
    uint8_t flags;
    double time;
    int resends;
    bool acked;
    bool used;
} glp_pending_t;

typedef struct
{
    uint16_t seq;
    uint8_t flags;
    const uint8_t *data;
    uint16_t length;
} glp_packet_t;

typedef struct
{
    uint16_t next, low, receive_next;
    glp_pending_t pending[GLP_RELIABLE_SLOTS];
    int pending_count;
    uint16_t out_of_order[GLP_RELIABLE_OOO];
    int ooo_count;
    double samples[GLP_RTT_SAMPLES];
    int sample_count;
    int gap, gap_count;
    bool receive_started;
} glp_reliable_t;

void glp_reliable_init(glp_reliable_t *r);
double glp_reliable_rto(const glp_reliable_t *r);
void glp_reliable_add_rtt(glp_reliable_t *r, double ms);
uint16_t glp_reliable_send_low(const glp_reliable_t *r);
bool glp_reliable_has_gap(const glp_reliable_t *r);
int glp_reliable_pending(const glp_reliable_t *r);

/* Queue a frame for transmission. Returns its sequence, or -1 when the window is full. */
int glp_reliable_queue(glp_reliable_t *r, const uint8_t *data, size_t len, uint8_t flags, double time);
void glp_reliable_acknowledge(glp_reliable_t *r, const uint8_t *data, size_t len, double time);
int glp_reliable_retransmit(glp_reliable_t *r, double time, int limit, glp_packet_t *out, int max);
void glp_reliable_receive(glp_reliable_t *r, uint16_t seq, uint16_t advertised_low);
void glp_reliable_ack(const glp_reliable_t *r, uint8_t out[20]);
/* Frame header: flags, length, sequence, send-window base. */
int glp_reliable_wrap(const glp_reliable_t *r, const glp_packet_t *p, uint8_t *out, size_t cap);
