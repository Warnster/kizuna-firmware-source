/* SPDX-License-Identifier: AGPL-3.0-only
 * Derived from GB-Link-Switch-LDN (https://github.com/GB-Link/GB-Link-Switch-LDN), Copyright (c) the GB-Link-Switch-LDN authors.
 * Changes Copyright (c) 2026 Kizuna. See NOTICE.
 */
/* gl_lead_shim.h -- lead-role trade shim from GB-Link-Switch-LDN trade_shim.h (commit c972db4, AGPL-3.0); see NOTICE.
 * The GBA leads and the Switch is its child. The extra post-trade round is the Switch's, and the GBA never answers it. */
#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* New session. */
void gls_reset(void);
/* Child frame (16 bytes from the Switch's adapter) toward the GBA, rewritten in place with its own sequence tag. The extra
   round, or a barrier the GBA already left, is answered with a parent frame (73 bytes) in `reply`. `forward` stays true. */
size_t gls_lead_child(uint8_t *payload, size_t length, int64_t now_ms, uint8_t *reply, size_t reply_capacity, bool *forward);
/* Parent frame from the GBA toward the Switch: standby numbers mapped in place, the GBA's block kept for a repeat. */
void gls_lead_parent(uint8_t *payload, size_t length, int64_t now_ms);
/* Once per tick: next parent frame (73 bytes) of the GBA's held or unanswered party request and its block, else 0. */
size_t gls_lead_inject(int64_t now_ms, uint8_t *out, size_t capacity);
/* Parent frame the next one supersedes. */
bool gls_parent_sheddable(const uint8_t *payload, size_t length);
