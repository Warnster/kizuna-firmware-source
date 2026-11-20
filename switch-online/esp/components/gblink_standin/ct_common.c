/* SPDX-License-Identifier: AGPL-3.0-only */
/* ct_common.c -- see ct_common.h. Ported from GB-Link-Switch-LDN web/js/cable/translator.js (KeyRuns, KeyQueue,
 * sizeFromCount, sizeFromRequest), commit c972db4, AGPL-3.0 (see NOTICE). Copyright (c) the GB-Link-Switch-LDN authors;
 * modified for kizuna 2026-11-02 (C port). */
#include "ct_common.h"

#include <string.h>

static const uint8_t MAGIC[14] = { 'G', 'a', 'm', 'e', 'F', 'r', 'e', 'a', 'k', ' ', 'i', 'n', 'c', '.' };

bool ct_is_magic(const uint8_t *b) { return memcmp(b, MAGIC, sizeof MAGIC) == 0; }

size_t ct_rfu1_frame(uint8_t *f, uint32_t type, uint32_t header, size_t size)
{
    memset(f, 0, size);
    f[0] = 0x52; f[1] = 0x46; f[2] = 0x55; f[3] = 0x31;
    ct_wbe32(f + 4, type);
    ct_wbe32(f + 8, header);
    return size;
}

uint16_t ct_size_from_count(unsigned count)
{
    switch (count) {
    case 17: return 200;
    case 9: return 100;
    case 19: return 220;
    case 4: return 40;
    case 2: return 20;
    }
    return (uint16_t)(count * CT_FRAG_BYTES);
}

uint16_t ct_size_from_request(unsigned type)
{
    switch (type) {
    case 2: return 100;
    case 3: return 220;
    case 4: return 40;
    }
    return 200;
}

/* ---- KeyRuns */

#define RUN(k, i) (((k)->head + (i)) % CT_KEY_QUEUE)

void ct_keyruns_clear(ct_keyruns_t *k) { k->head = 0; k->n = 0; }

void ct_keyruns_push(ct_keyruns_t *k, uint8_t code)
{
    if (k->n && k->code[RUN(k, k->n - 1)] == code) { k->count[RUN(k, k->n - 1)] += 1; return; }
    if (k->n == CT_KEY_QUEUE) { k->head = (uint8_t)((k->head + 1) % CT_KEY_QUEUE); k->n--; }   /* runs.shift() */
    unsigned at = RUN(k, k->n);
    k->code[at] = code;
    k->count[at] = 1;
    k->n++;
}

uint8_t ct_keyruns_pop(ct_keyruns_t *k, double step)
{
    if (!k->n) return CT_KEY_EMPTY;
    double backlog = 0;
    for (unsigned i = 0; i < k->n; i++) backlog += k->count[RUN(k, i)];
    if (backlog > CT_HOST_KEY_LAG) step += (backlog - CT_HOST_KEY_LAG) / 4;
    unsigned h = k->head;
    k->count[h] -= step;
    uint8_t code = k->code[h];
    if (k->count[h] <= 0) {
        double over = k->count[h];
        k->head = (uint8_t)((k->head + 1) % CT_KEY_QUEUE);
        k->n--;
        /* The overshoot shortens the next run, which still plays at least once. */
        if (k->n) {
            double c = k->count[k->head] + over;
            k->count[k->head] = c > 0.01 ? c : 0.01;
        }
    }
    return code;
}

bool ct_keyruns_has(const ct_keyruns_t *k, uint8_t a, uint8_t b)
{
    for (unsigned i = 0; i < k->n; i++) {
        uint8_t c = k->code[RUN(k, i)];
        if (c == a || c == b) return true;
    }
    return false;
}

/* ---- KeyQueue */

void ct_keyqueue_clear(ct_keyqueue_t *q) { q->head = 0; q->n = 0; }

void ct_keyqueue_push(ct_keyqueue_t *q, uint8_t code)
{
    if (q->n >= CT_KEY_QUEUE) { q->head = (uint8_t)((q->head + 1) % CT_KEY_QUEUE); q->n--; }
    q->code[(q->head + q->n) % CT_KEY_QUEUE] = code;
    q->n++;
}

uint8_t ct_keyqueue_pop(ct_keyqueue_t *q)
{
    if (!q->n) return CT_KEY_EMPTY;
    uint8_t c = q->code[q->head];
    q->head = (uint8_t)((q->head + 1) % CT_KEY_QUEUE);
    q->n--;
    return c;
}
