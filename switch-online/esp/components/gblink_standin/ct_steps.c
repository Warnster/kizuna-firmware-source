/* SPDX-License-Identifier: AGPL-3.0-only */
/* ct_steps.c -- StepFollower, ported from GB-Link-Switch-LDN web/js/cable/steps.js (commit c972db4,
 * https://github.com/GB-Link/GB-Link-Switch-LDN). Copyright (c) the GB-Link-Switch-LDN authors. Modified for kizuna
 * 2026-11-02: C port. AGPL-3.0, see NOTICE. */
#include "ct_steps.h"

#include <math.h>

enum { STEP_FRAMES = 16, WINDOW_FRAMES = 4, MAX_WINDOW = 14, MAX_GAP = 8 };
#define DRIFT_PER_STEP 1.1

static bool is_direction(uint8_t c) { return c >= 0x12 && c <= 0x15; }
static bool is_link_state(uint8_t c) { return c == CT_KEY_READY || c == CT_KEY_EXIT_ROOM; }

void ct_steps_init(ct_steps_t *s, ct_key_out_fn out, void *ctx)
{
    s->out = out;
    s->ctx = ctx;
    ct_steps_reset(s);
}

void ct_steps_reset(ct_steps_t *s)
{
    s->left = 0;
    s->age = 0;
    s->dir = 0;
    s->window = 0;
    s->chain = 0;
    s->counter = -1;
    s->last = CT_KEY_EMPTY;
}

static void frame(ct_steps_t *s, uint8_t code)
{
    if (is_link_state(code)) {
        s->out(s->ctx, code);   /* taking a seat or leaving goes out as it comes */
    } else if (s->left == 0) {
        if (is_direction(code)) {
            /* A step straight after the last one continues the walk. */
            s->chain = s->age == STEP_FRAMES ? s->chain + 1 : 1;
            s->left = STEP_FRAMES;
            s->age = 0;
            s->dir = code;
            /* Math.round on a non-negative value */
            uint32_t w = WINDOW_FRAMES + (uint32_t)floor(DRIFT_PER_STEP * (double)(s->chain - 1) + 0.5);
            s->window = w < MAX_WINDOW ? w : MAX_WINDOW;
        } else {
            s->chain = 0;
        }
        s->out(s->ctx, code);
    } else {
        /* Mid-step the Switch ignores keys; the GBA gets the step's direction while it may still be starting it. */
        s->out(s->ctx, s->age < s->window ? s->dir : CT_KEY_EMPTY);
    }
    if (s->left > 0) s->left--;
    s->age++;
}

void ct_steps_report(ct_steps_t *s, uint8_t code, uint8_t counter)
{
    unsigned frames = 1;
    if (s->counter >= 0) {
        unsigned gap = (unsigned)(counter - s->counter) & 0xff;
        frames = gap >= 1 && gap <= MAX_GAP ? gap : 1;
    }
    s->counter = counter;
    /* Frames the board did not pass on held the last key. */
    for (unsigned i = 1; i < frames; i++) frame(s, s->last);
    frame(s, code);
    s->last = code;
}
