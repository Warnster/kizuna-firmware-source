/* SPDX-License-Identifier: AGPL-3.0-only
 * Derived from GB-Link-Switch-LDN (https://github.com/GB-Link/GB-Link-Switch-LDN), Copyright (c) the GB-Link-Switch-LDN authors.
 * Changes Copyright (c) 2026 Kizuna. See NOTICE.
 */
/* ct_steps.h -- StepFollower: the Switch player's held keys as a cable GBA should see them. C port of GB-Link-Switch-LDN
 * web/js/cable/steps.js (commit c972db4, AGPL-3.0; see NOTICE).
 *
 * In a link room a direction key reaching an idle avatar starts a 16-frame step; this follows the Switch's own avatar
 * through its key reports and passes each step's direction on for a start window (4 frames, growing 1.1 per step of a
 * walk, at most 14), then no key for the rest of the step. READY (0x16) and EXIT (0x17) pass as they come. */
#ifndef CT_STEPS_H
#define CT_STEPS_H

#include "ct_common.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*ct_key_out_fn)(void *ctx, uint8_t code);

typedef struct {
    ct_key_out_fn out;
    void *ctx;
    uint32_t left, age, window, chain;
    int16_t counter;     /* -1: none yet */
    uint8_t dir, last;
} ct_steps_t;

void ct_steps_init(ct_steps_t *s, ct_key_out_fn out, void *ctx);
void ct_steps_reset(ct_steps_t *s);
/* One report: the key code and the Switch's 8-bit frame counter (0xBE00 word 1 high byte). */
void ct_steps_report(ct_steps_t *s, uint8_t code, uint8_t counter);

#ifdef __cplusplus
}
#endif
#endif
