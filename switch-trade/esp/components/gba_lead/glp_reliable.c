/* SPDX-License-Identifier: AGPL-3.0-only
 * Derived from GB-Link-Switch-LDN (https://github.com/GB-Link/GB-Link-Switch-LDN), Copyright (c) the GB-Link-Switch-LDN authors.
 * Changes Copyright (c) 2026 Kizuna. See NOTICE.
 */
/* glp_reliable.c -- copied from GB-Link-Switch-LDN firmware/common/pia_reliable.c (commit c972db4, AGPL-3.0,
 * https://github.com/GB-Link/GB-Link-Switch-LDN); see NOTICE. Kizuna: identifiers renamed pia_ -> glp_, PIA_ -> GLP_,
 * bin_ -> glb_ so they sit beside cport's own Pia host in one image. */
#include "glp_reliable.h"

static glp_pending_t *slot_of(glp_reliable_t *r, uint16_t seq)
{
    glp_pending_t *e = &r->pending[seq % GLP_RELIABLE_SLOTS];
    return e->used && e->seq == seq ? e : NULL;
}

void glp_reliable_init(glp_reliable_t *r)
{
    memset(r, 0, sizeof(*r));
    r->next = r->low = r->receive_next = 0xfff0;
    r->gap = -1;
}

void glp_reliable_add_rtt(glp_reliable_t *r, double ms)
{
    if (ms < 0) return;
    if (r->sample_count < GLP_RTT_SAMPLES) r->samples[r->sample_count++] = ms;
    else
    {
        memmove(r->samples, r->samples + 1, sizeof(double) * (GLP_RTT_SAMPLES - 1));
        r->samples[GLP_RTT_SAMPLES - 1] = ms;
    }
}

double glp_reliable_rto(const glp_reliable_t *r)
{
    if (r->sample_count == 0) return 200;
    double sorted[GLP_RTT_SAMPLES];
    memcpy(sorted, r->samples, sizeof(double) * (size_t)r->sample_count);
    for (int i = 1; i < r->sample_count; ++i)
    {
        double v = sorted[i];
        int j = i - 1;
        while (j >= 0 && sorted[j] > v) { sorted[j + 1] = sorted[j]; --j; }
        sorted[j + 1] = v;
    }
    double median = sorted[r->sample_count / 2], deviation = 0;
    for (int i = 0; i < r->sample_count; ++i)
        deviation += sorted[i] > median ? sorted[i] - median : median - sorted[i];
    deviation /= (double)r->sample_count;
    double rto = 33 + 1.4 * median + 4 * deviation;
    return rto < 670 ? rto : 670;
}

uint16_t glp_reliable_send_low(const glp_reliable_t *r) { return r->pending_count == 0 ? r->next : r->low; }
bool glp_reliable_has_gap(const glp_reliable_t *r) { return r->ooo_count != 0; }
int glp_reliable_pending(const glp_reliable_t *r) { return r->pending_count; }

int glp_reliable_queue(glp_reliable_t *r, const uint8_t *data, size_t len, uint8_t flags, double time)
{
    if (len > GLP_RELIABLE_PAYLOAD) return -1;
    glp_pending_t *e = &r->pending[r->next % GLP_RELIABLE_SLOTS];
    if (e->used) return -1;                      /* window full */
    memcpy(e->data, data, len);
    e->length = (uint16_t)len;
    e->seq = r->next;
    e->flags = flags;
    e->time = time;
    e->resends = 0;
    e->acked = false;
    e->used = true;
    ++r->pending_count;
    int seq = r->next;
    r->next = (uint16_t)(r->next + 1);
    return seq;
}

void glp_reliable_acknowledge(glp_reliable_t *r, const uint8_t *data, size_t len, double time)
{
    if (len < 20) return;
    uint16_t ack = glb_b16(data + 2);
    bool any = false;
    for (int i = 4; i < 20; ++i) if (data[i]) { any = true; break; }

    for (uint16_t seq = r->low; seq != r->next; ++seq)
    {
        glp_pending_t *e = slot_of(r, seq);
        if (!e || e->acked) continue;
        uint16_t bit = (uint16_t)(seq - ack - 1);
        bool arrived = glb_less(seq, ack) || (bit < 128 && (data[4 + (bit >> 3)] & (1 << (bit & 7))) != 0);
        if (!arrived) continue;
        if (e->resends == 0) glp_reliable_add_rtt(r, time - e->time);
        e->acked = true;
    }

    for (;;)
    {
        glp_pending_t *first = slot_of(r, r->low);
        if (!first || !first->acked) break;
        first->used = false;
        --r->pending_count;
        r->low = (uint16_t)(r->low + 1);
    }

    glp_pending_t *missing = slot_of(r, ack);
    if (any && missing && !missing->acked)
    {
        if (r->gap == (int)ack) ++r->gap_count;
        else { r->gap = (int)ack; r->gap_count = 1; }
    }
    else { r->gap = -1; r->gap_count = 0; }
}

int glp_reliable_retransmit(glp_reliable_t *r, double time, int limit, glp_packet_t *out, int max)
{
    int count = 0;
    double rto = glp_reliable_rto(r);
    for (uint16_t seq = r->low; seq != r->next && count < limit && count < max; ++seq)
    {
        glp_pending_t *e = slot_of(r, seq);
        if (!e) continue;
        if (e->acked) continue;
        /* Fast retransmit only after the peer reports this frame missing three times. */
        bool fast = r->gap == (int)seq && e->resends == 0 && r->gap_count >= 3;
        if (!fast && time - e->time < rto) break;
        e->time = time;
        ++e->resends;
        out[count].seq = seq;
        out[count].flags = e->flags;
        out[count].data = e->data;
        out[count].length = e->length;
        ++count;
    }
    return count;
}

static bool ooo_remove(glp_reliable_t *r, uint16_t seq)
{
    for (int i = 0; i < r->ooo_count; ++i)
        if (r->out_of_order[i] == seq)
        {
            r->out_of_order[i] = r->out_of_order[--r->ooo_count];
            return true;
        }
    return false;
}

void glp_reliable_receive(glp_reliable_t *r, uint16_t seq, uint16_t advertised_low)
{
    /* Start from the peer's advertised window base, not the first arrival, so an early
       reorder cannot hide a gap. */
    if (!r->receive_started) { r->receive_next = advertised_low; r->receive_started = true; }

    if (seq == r->receive_next)
    {
        r->receive_next = (uint16_t)(r->receive_next + 1);
        while (ooo_remove(r, r->receive_next)) r->receive_next = (uint16_t)(r->receive_next + 1);
    }
    else if (glb_less(r->receive_next, seq) && (uint16_t)(seq - r->receive_next) < 4096)
    {
        for (int i = 0; i < r->ooo_count; ++i) if (r->out_of_order[i] == seq) return;
        if (r->ooo_count < GLP_RELIABLE_OOO) r->out_of_order[r->ooo_count++] = seq;
    }
}

void glp_reliable_ack(const glp_reliable_t *r, uint8_t out[20])
{
    memset(out, 0, 20);
    out[1] = 1;
    glb_wb16(out + 2, r->receive_next);
    for (int i = 0; i < r->ooo_count; ++i)
    {
        uint16_t bit = (uint16_t)(r->out_of_order[i] - r->receive_next - 1);
        if (bit < 128) out[4 + (bit >> 3)] |= (uint8_t)(1 << (bit & 7));
    }
}

int glp_reliable_wrap(const glp_reliable_t *r, const glp_packet_t *p, uint8_t *out, size_t cap)
{
    if ((size_t)p->length + 8 > cap) return -1;
    out[0] = p->flags;
    glb_wb16(out + 1, p->length);
    glb_wb16(out + 3, p->seq);
    glb_wb16(out + 5, glp_reliable_send_low(r));
    out[7] = 0;
    memcpy(out + 8, p->data, p->length);
    return (int)(8 + p->length);
}
