/* Copyright (c) 2026 mgba_ldn contributors (rfu-wrapper-air.c, the translator this file is ported from)
 * Modified: ported to C for the Kizuna adapters (see git history).
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 * Upstream: https://github.com/Gr3nSkyDragon/mgba_LDN (branch mgba-ldn), src/gba/sio/rfu-wrapper-air.c */
/* cable_air — RFU Cable Wrapper translator. See cable_air.h.
 *
 * Faithful port of mgba_LDN src/gba/sio/rfu-wrapper-air.c (branch mgba-ldn). Function bodies are kept
 * as close to the reference as possible (air.c:NNN refs in comments) so the two stay diffable. The
 * mGBA plumbing is replaced by two plain edges:
 *   - Switch edge: struct CableAirSwitch (was struct GBASIORFUBackend) + on_* event entries (was the
 *     GBASIORFU event queue). LLSF child frames go out via sw.send_frame; host frames + connection
 *     events are pushed in by the transport (ldn_join/cport, wired in slice 4).
 *   - Cable edge: the peer API cable_air_game_command/next_command/reset (was GBASIORFUWrapperPeer),
 *     driven by gba_cable (slice 5). The real Ruby/Sapphire cart is the cable master ("the game").
 * AIRLOG becomes an optional trace hook (host tests print; device no-op), with no printf format
 * attribute so it is not -Wformat-checked. */
#include "cable_air.h"
#include <string.h>
#include <stdarg.h>
#include <stdio.h>

/* reference-name aliases so the ported bodies read like the original */
#define LINKCMD_SEND_LINK_TYPE     CA_LINKCMD_SEND_LINK_TYPE
#define LINKCMD_READY_EXIT_STANDBY CA_LINKCMD_READY_EXIT_STANDBY
#define LINKCMD_READY_CLOSE_LINK   CA_LINKCMD_READY_CLOSE_LINK
#define LINKCMD_CONT_BLOCK         CA_LINKCMD_CONT_BLOCK
#define LINKCMD_INIT_BLOCK         CA_LINKCMD_INIT_BLOCK
#define LINKCMD_SEND_HELD_KEYS     CA_LINKCMD_SEND_HELD_KEYS
#define LINKCMD_SEND_BLOCK_REQ     CA_LINKCMD_SEND_BLOCK_REQ
#define _le16(p)                   cable_air_le16(p)
#define _fragCount                 cable_air_frag_count
#define _sizeFromCount             cable_air_size_from_count
#define _sizeFromRequest           cable_air_size_from_request
#define _keyPush                   cable_air_key_push
#define _keyPop                    cable_air_key_pop
#define _cablePushCmd              cable_air_push_cmd
#define _cablePushBlock            cable_air_push_block

/* trace hook — NOT marked __attribute__((format)) so custom specifiers are not -Wformat-checked. */
static void airlog(struct CableAir* air, const char* fmt, ...) {
	if (!air->trace) {
		return;
	}
	char buf[192];
	va_list ap;
	va_start(ap, fmt);
	vsnprintf(buf, sizeof(buf), fmt, ap);
	va_end(ap);
	air->trace(air->traceCtx, buf);
}
#define AIRLOG(air, ...) airlog((air), "AIR " __VA_ARGS__)

static void _put16(uint8_t* p, uint16_t v) {
	p[0] = (uint8_t) v;
	p[1] = (uint8_t) (v >> 8);
}

/* ======================= slice 1: self-contained helpers (host-tested) ======================= */

uint16_t cable_air_le16(const uint8_t* p) { return (uint16_t)(p[0] | (p[1] << 8)); }

unsigned cable_air_frag_count(unsigned bytes) {
	unsigned count = (bytes + FRAG_BYTES - 1) / FRAG_BYTES;
	return count ? count : 1;
}

unsigned cable_air_size_from_count(unsigned count) {
	switch (count) {
	case 17: return 200;
	case 9:  return 100;
	case 19: return 220;
	case 4:  return 40;
	case 2:  return 20;
	}
	return count * FRAG_BYTES;
}

unsigned cable_air_size_from_request(unsigned type) {
	switch (type) {
	case 2: return 100;
	case 3: return 220;
	case 4: return 40;
	default: return 200;
	}
}

void cable_air_key_push(struct KeyQueue* q, uint8_t code) {
	if (q->count >= KEY_QUEUE) {
		q->head = (q->head + 1) % KEY_QUEUE;
		--q->count;
	}
	q->codes[(q->head + q->count) % KEY_QUEUE] = code;
	++q->count;
}

uint8_t cable_air_key_pop(struct KeyQueue* q) {
	if (!q->count) {
		return LINK_KEY_CODE_EMPTY;
	}
	uint8_t code = q->codes[q->head];
	q->head = (q->head + 1) % KEY_QUEUE;
	--q->count;
	return code;
}

/* ---- leader -> game held keys, as timed runs (see struct HostKeyRuns) ---- */
static void _hkReset(struct HostKeyRuns* k) {
	memset(k, 0, sizeof(*k));
	k->step = KEY_Q8 * 3 / 2;
}

static void _hkPush(struct HostKeyRuns* k, uint8_t code, bool counting) {
	if (k->n) {
		struct KeyRun* last = &k->runs[(k->head + k->n - 1) % KEY_QUEUE];
		if (last->code == code) {
			last->count += KEY_Q8;
			if (counting) ++k->pushes;
			return;
		}
	}
	if (k->n >= KEY_QUEUE) {           /* drop the oldest run, never the newest press */
		k->head = (k->head + 1) % KEY_QUEUE;
		--k->n;
	}
	k->runs[(k->head + k->n) % KEY_QUEUE] = (struct KeyRun){ code, KEY_Q8 };
	++k->n;
	if (counting) ++k->pushes;
}

static uint8_t _hkPop(struct HostKeyRuns* k) {
	if (++k->pops >= KEY_RATE_WINDOW) {
		int32_t step = k->pops ? (int32_t)((k->pushes * KEY_Q8) / k->pops) : KEY_Q8;
		k->step = step < KEY_Q8 ? KEY_Q8 : step > 3 * KEY_Q8 ? 3 * KEY_Q8 : step;
		k->pushes = k->pops = 0;
	}
	if (!k->n) {
		return LINK_KEY_CODE_EMPTY;
	}
	int32_t backlog = 0;
	for (unsigned i = 0; i < k->n; ++i) backlog += k->runs[(k->head + i) % KEY_QUEUE].count;
	int32_t step = k->step;
	if (backlog > HOST_KEY_LAG * KEY_Q8) step += (backlog - HOST_KEY_LAG * KEY_Q8) / 4;
	struct KeyRun* head = &k->runs[k->head];
	uint8_t code = head->code;
	head->count -= step;
	if (head->count <= 0) {
		int32_t over = head->count;
		k->head = (k->head + 1) % KEY_QUEUE;
		--k->n;
		if (k->n) {                    /* the overshoot shortens the next run, which still plays once */
			struct KeyRun* next = &k->runs[k->head];
			next->count += over;
			if (next->count < 3) next->count = 3;
		}
	}
	return code;
}

/* air.c:680 _cablePush */
static bool _cablePush(struct CableAir* air, const uint16_t cmd[CMD_WORDS]) {
	if (air->cqCount >= CABLE_QUEUE) {
		AIRLOG(air, "cable queue full, command %04X dropped", cmd[0]);
		return false;
	}
	unsigned at = (air->cqHead + air->cqCount) % CABLE_QUEUE;
	memcpy(air->cq[at], cmd, sizeof(air->cq[at]));
	++air->cqCount;
	return true;
}

void cable_air_push_cmd(struct CableAir* air, uint16_t c0, uint16_t c1, uint16_t c2) {
	uint16_t cmd[CMD_WORDS] = { c0, c1, c2, 0, 0, 0, 0, 0 };
	_cablePush(air, cmd);
}

static void _cablePushIdle(struct CableAir* air, unsigned n) {
	uint16_t z[CMD_WORDS] = { 0, 0, 0, 0, 0, 0, 0, 0 };
	while (n--) {
		_cablePush(air, z);
	}
}

/* (the pkts=59 death): PACE THE BLOCK, don't burst it. The game consumes link commands at
 * 60 Hz (one per VBlank), but the cable delivers ~166 packets/s — INIT + CONTs pushed back-to-back
 * overflow the game's receive queue mid-block-exchange (freeze-ring showed our CONT payload landing
 * byte-perfect, acc 971/973, yet the master errored right at the room transition where LP delivery +
 * card request + card chained ~17 dense packets). The stub path that reached the trade menu (cb-3.73)
 * paces exactly this way: 3 idles after INIT (the decomp's 3-frame post-INIT delay, cb-3.57) and 1
 * idle between CONTs (cb-3.65's proven value — gap=4 desyncs, gap=0 is this death). */
void cable_air_push_block(struct CableAir* air, const uint8_t* data, unsigned size) {
	cable_air_push_cmd(air, CA_LINKCMD_INIT_BLOCK, (uint16_t) size, 0 + 128);
	_cablePushIdle(air, CA_BLOCK_INIT_GAP);
	unsigned pos;
	for (pos = 0; pos < size; pos += (CMD_WORDS - 1) * 2) {
		uint16_t cmd[CMD_WORDS] = { CA_LINKCMD_CONT_BLOCK, 0, 0, 0, 0, 0, 0, 0 };
		unsigned i;
		for (i = 0; i < CMD_WORDS - 1; ++i) {
			unsigned at = pos + i * 2;
			uint8_t lo = at < size ? data[at] : 0;
			uint8_t hi = at + 1 < size ? data[at + 1] : 0;
			cmd[1 + i] = (uint16_t)(lo | (hi << 8));
		}
		_cablePush(air, cmd);
		_cablePushIdle(air, CA_BLOCK_CONT_GAP);
	}
}

bool cable_air_cq_pop(struct CableAir* air, uint16_t out[CMD_WORDS]) {
	if (!air->cqCount) {
		return false;
	}
	memcpy(out, air->cq[air->cqHead], sizeof(air->cq[air->cqHead]));
	air->cqHead = (air->cqHead + 1) % CABLE_QUEUE;
	--air->cqCount;
	return true;
}

void cable_air_build_lp_for_host(struct CableAir* air, uint8_t out[LP_BUFFER_SIZE]) {
	memset(out, 0, LP_BUFFER_SIZE);
	memcpy(out, air->rubyLP, LINK_PLAYER_BLOCK_SIZE);
	memset(&out[LP_LINK_TYPE_OFFSET], 0, 4);
	_put16(&out[LP_VERSION_OFFSET], 0x4000 | VERSION_EMERALD);   /* shown to the Switch as Emerald */
	out[LP_PROGRESS_OFFSET] = PROGRESS_CLEARED;
	out[LP_PROGRESS_OFFSET + 1] = 0;
	out[LP_PROGRESS_OFFSET + 2] = PROGRESS_CLEARED;
}

void cable_air_lp_for_game(struct CableAir* air, uint8_t out[LINK_PLAYER_BLOCK_SIZE]) {
	memcpy(out, air->hostLP, LINK_PLAYER_BLOCK_SIZE);
	memcpy(&out[LP_LINK_TYPE_OFFSET], &air->rubyLP[LP_LINK_TYPE_OFFSET], 4);
}

void cable_air_build_host_card(struct CableAir* air, uint8_t out[CARD_SIZE]) {
	memset(out, 0, CARD_SIZE);
	const uint8_t* lp = air->hostLP;
	out[0x00] = lp[16 + 0x13];
	out[0x02] = 1;
	out[0x0E] = lp[LP_TRAINER_ID_OFFSET];
	out[0x0F] = lp[LP_TRAINER_ID_OFFSET + 1];
	unsigned i;
	for (i = 0; i < 7 && lp[LP_NAME_OFFSET + i] != 0xFF; ++i) {
		out[0x30 + i] = lp[LP_NAME_OFFSET + i];
	}
	out[0x30 + i] = 0xFF;
}

void cable_air_stamp_card_for_host(const uint8_t* rs_card, uint8_t out[CARD_SIZE]) {
	memset(out, 0, CARD_SIZE);
	memcpy(out, rs_card, CARD_VERSION_OFFSET);
	out[CARD_VERSION_OFFSET] = VERSION_EMERALD;
}

/* ============================= 1. wire helpers (air.c:352) ============================= */

static void _slotToWords(const uint8_t* slot, uint16_t words[7]) {
	unsigned i;
	for (i = 0; i < 7; ++i) {
		words[i] = _le16(&slot[i * 2]);
	}
}

static bool _slotIdle(const uint8_t* slot) {
	unsigned i;
	for (i = 0; i < SLOT_BYTES; ++i) {
		if (slot[i]) {
			return false;
		}
	}
	return true;
}

/* ============================= 2. block receive/send (air.c:369) ============================= */

static void _recvReset(struct Recv* r) {
	memset(r, 0, sizeof(*r));
	r->lastIndex = -1;
}

static void _recvInit(struct Recv* r, unsigned count) {
	if (count == 0 || count > MAX_FRAGS) {
		return;
	}
	if (!r->receiving || r->done || count != r->count) {
		_recvReset(r);
		r->count = count;
		r->receiving = true;
	}
}

static bool _recvBlock(struct Recv* r, unsigned index, const uint8_t* frag) {
	if (!r->receiving || index >= r->count) {
		return false;
	}
	bool wasDone = r->done;
	r->lastIndex = (int) index;
	r->flags |= 1u << index;
	memcpy(&r->buf[index * FRAG_BYTES], frag, FRAG_BYTES);
	if (r->flags == ((1u << r->count) - 1)) {
		r->done = true;
	}
	return r->done && !wasDone;
}

static void _feedRecv(struct Recv* r, const uint16_t words[7], const uint8_t* slot, bool* completed) {
	unsigned op = words[0] & RFUCMD_MASK;
	*completed = false;
	if (op == RFUCMD_SEND_BLOCK_INIT) {
		_recvInit(r, words[1]);
	} else if (op == RFUCMD_SEND_BLOCK) {
		*completed = _recvBlock(r, words[0] & 0x1F, &slot[2]);
	}
}

static void _sendStart(struct Send* s, const uint8_t* data, unsigned size, bool isLinkPlayer) {
	memset(s, 0, sizeof(*s));
	s->active = true;
	s->state = SEND_INIT;
	s->count = _fragCount(size);
	memcpy(s->data, data, size);
	s->isLinkPlayer = isLinkPlayer;
}

static void _blockWords(const struct Send* s, unsigned index, uint16_t words[7]) {
	words[0] = (uint16_t)(RFUCMD_SEND_BLOCK | (index & 0x1F));
	unsigned i;
	for (i = 0; i < 6; ++i) {
		words[1 + i] = _le16(&s->data[index * FRAG_BYTES + i * 2]);
	}
}

static void _sendTick(struct Send* s, const struct Recv* ack, uint16_t words[7]) {
	memset(words, 0, 7 * sizeof(uint16_t));
	if (!s->active) {
		return;
	}
	if (s->state == SEND_INIT) {
		++s->initSends;
		if (ack->receiving && ack->count == s->count) {
			s->state = SEND_STREAM;
			s->index = 0;
		} else {
			words[0] = RFUCMD_SEND_BLOCK_INIT;
			words[1] = (uint16_t) s->count;
			words[2] = 1 | OWNER_FLAG;
			return;
		}
	}
	if (s->state == SEND_STREAM) {
		unsigned index = s->index;
		_blockWords(s, index, words);
		if (index >= s->count - 1) {
			s->state = SEND_HOLD;
			s->holdSends = 0;
		} else {
			++s->index;
		}
		return;
	}
	if (s->state == SEND_HOLD) {
		++s->holdSends;
		unsigned last = s->count - 1;
		uint32_t full = (1u << s->count) - 1;
		if (ack->lastIndex == (int) last && ack->count == s->count) {
			if (ack->flags == full) {
				s->state = SEND_DONE;
				s->active = false;
				return;
			}
			unsigned missing[MAX_FRAGS];
			unsigned n = 0, i;
			for (i = 0; i < s->count; ++i) {
				if (!((ack->flags >> i) & 1)) {
					missing[n++] = i;
				}
			}
			if (n) {
				s->rr = (s->rr + 1) % n;
				_blockWords(s, missing[s->rr], words);
				return;
			}
		}
		_blockWords(s, last, words);
	}
}

/* ============================= 3. barriers (air.c:485) ============================= */

static void _barrierInit(struct Barrier* b) {
	memset(b, 0, sizeof(*b));
	b->hostCount = -1;
	b->burstFor = -1;
}

static void _barrierInitiate(struct Barrier* b, int kind) {
	if (b->mode == kind) {
		return;
	}
	b->mode = kind;
	b->initiated = true;
	b->hostCount = -1;
	b->sinceHost = 0;
	b->sinceInitiate = 0;
	b->burstFor = -1;
}

static bool _barrierHostStandby(struct CableAir* air, unsigned count) {
	struct Barrier* b = &air->bar;
	b->sinceHost = 0;
	int prev = b->hostCount;
	b->hostCount = (int) count;
	if (b->initiated && b->mode == BAR_STANDBY) {
		if (count == b->localCount) {
			++b->localCount;
			++b->rounds;
			b->mode = BAR_IDLE;
			b->initiated = false;
			return true;
		}
		return false;
	}
	if (count < b->localCount) {
		return false;
	}
	if (b->mode != BAR_STANDBY) {
		b->mode = BAR_STANDBY;
		b->initiated = false;
		AIRLOG(air, "barrier: leader 0x6600 count=%u -> mirroring", count);
	} else if (prev >= 0 && (int) count != prev) {
		++b->rounds;
	}
	b->localCount = count;
	b->sinceInitiate = 0;
	return false;
}

static void _barrierHostClose(struct CableAir* air, unsigned count) {
	struct Barrier* b = &air->bar;
	b->sinceHost = 0;
	b->hostCount = (int) count;
	b->localCount = count;
	if (b->mode != BAR_CLOSE) {
		b->mode = BAR_CLOSE;
		b->initiated = false;
		AIRLOG(air, "barrier: leader 0x5F00 count=%u -> mirroring", count);
	}
}

static bool _barrierObserve(struct CableAir* air, bool sawBarrier) {
	struct Barrier* b = &air->bar;
	if (b->mode == BAR_CLOSE && b->initiated) {
		if (b->sinceInitiate && b->sinceInitiate % 60 == 0) {
			b->burstN = 0;
		}
		if (++b->sinceInitiate > 300) {
			b->mode = BAR_IDLE;
			b->initiated = false;
			AIRLOG(air, "barrier: close unanswered, ending the link anyway");
			return true;
		}
		return false;
	}
	if (b->mode != BAR_STANDBY) {
		return false;
	}
	if (sawBarrier) {
		b->sinceHost = 0;
		b->sinceInitiate = 0;
		return false;
	}
	++b->sinceHost;
	if (b->initiated) {
		if (b->sinceInitiate && b->sinceInitiate % 60 == 0) {
			b->burstN = 0;
		}
		/* JOIN GATE (B): a standby round is the game's "Press A" confirmation. Do
		 * NOT let it time-release until the Switch has accepted our join — otherwise the game races
		 * past the confirmation before the Switch is in, and the trade desyncs (the crash-after-2nd-
		 * confirmation). While the Switch is not ready we keep re-arming the barrier burst and hold,
		 * so the game sits at the confirmation. A safety cap (JOIN_HOLD_CAP frames) still releases it
		 * eventually so a join that never lands can't hang the cart forever. */
		if (!air->switchReady && b->mode == BAR_STANDBY) {
			b->sinceInitiate = 0;              /* keep the standby alive, don't count toward timeout */
			b->burstN = 0;                     /* re-emit READY_EXIT_STANDBY so the game keeps waiting */
			if (!air->joinHoldLogged) {
				air->joinHoldLogged = true;
				AIRLOG(air, "confirmation reached: holding 'Press A' until the Switch accepts the join");
			}
			if (++air->joinHoldFrames > JOIN_HOLD_CAP) {
				air->joinHoldFrames = 0;
				b->mode = BAR_IDLE;
				b->initiated = false;
				AIRLOG(air, "join gate: Switch never accepted after %u frames, releasing anyway", JOIN_HOLD_CAP);
				return true;
			}
			return false;
		}
		air->joinHoldFrames = 0;
		/* bench: never pass a round we started until the leader answers it. Releasing after
		 * INITIATE_TIMEOUT told the game the round was done (the second sit-down round: 5FFF to the cart, which
		 * relinked and began the trade menu) while the Switch had not reached its standby yet; when it did, we
		 * had stopped asking and both sides waited forever. The Switch can take long to get there (its own
		 * screens, a player reading messages), so keep asking every 60 frames, as GB-Link's translator does. */
		if (++b->sinceInitiate == INITIATE_TIMEOUT) {
			AIRLOG(air, "barrier: standby count=%u unanswered for %u frames, still asking", b->localCount,
			       INITIATE_TIMEOUT);
		}
	} else if (b->sinceHost > IDLE_TIMEOUT) {
		++b->localCount;
		++b->rounds;
		b->mode = BAR_IDLE;
		AIRLOG(air, "barrier: leader stopped 0x6600, round passed (count now %u)", b->localCount);
		return true;
	}
	return false;
}

static bool _barrierWant(struct Barrier* b, uint16_t words[7]) {
	if (b->mode == BAR_IDLE) {
		return false;
	}
	if (b->burstFor != (int) b->localCount) {
		b->burstFor = (int) b->localCount;
		b->burstN = 0;
	}
	if (b->burstN >= BARRIER_EMITS) {
		return false;
	}
	++b->burstN;
	memset(words, 0, 7 * sizeof(uint16_t));
	words[0] = b->mode == BAR_STANDBY ? RFUCMD_READY_EXIT_STANDBY : RFUCMD_READY_CLOSE_LINK;
	words[1] = (uint16_t) b->localCount;
	return true;
}

/* ============================= 4. NI handshake (air.c:614) ============================= */

static uint16_t _childLLSF(unsigned state, unsigned n, unsigned phase, unsigned ack, unsigned size) {
	return (uint16_t)(((state & 0xF) << 10) | ((ack & 1) << 9) | ((n & 3) << 7) | ((phase & 3) << 5) |
	                  (size & 0x1F));
}

static void _niInit(struct NI* ni, const uint8_t* src) {
	memset(ni, 0, sizeof(*ni));
	memcpy(ni->src, src, 26);
	ni->header[0] = 1;
	_put16(&ni->header[1], 12);
	ni->header[3] = 26;
	ni->remain = 7;
}

static unsigned _niNext(struct NI* ni, uint8_t* out) {
	const unsigned payload = 12;
	unsigned i;
	if (ni->state == 0) {
		unsigned size = (unsigned) ni->remain < payload ? (unsigned) ni->remain : payload;
		_put16(out, _childLLSF(LCOM_NI_START, 1, 0, 0, size));
		memcpy(&out[2], ni->header, size);
		ni->state = 1;
		ni->phase = 0;
		for (i = 0; i < 4; ++i) {
			ni->n[i] = 1;
			ni->now[i] = payload * i;
		}
		ni->remain = 26;
		return 2 + size;
	}
	if (ni->state == 1) {
		while (ni->now[ni->phase] >= 26) {
			ni->phase = (ni->phase + 1) % 4;
		}
		unsigned off = ni->now[ni->phase];
		unsigned size = 26 - off < payload ? 26 - off : payload;
		_put16(out, _childLLSF(LCOM_NI, ni->n[ni->phase], ni->phase, 0, size));
		memcpy(&out[2], &ni->src[off], size);
		ni->remain -= (int) size;
		ni->now[ni->phase] += payload << 2;
		ni->phase = (ni->phase + 1) % 4;
		if (ni->remain <= 0) {
			ni->state = 2;
			ni->phase = 0;
		}
		return 2 + size;
	}
	if (ni->state == 2) {
		_put16(out, _childLLSF(LCOM_NI_END, 0, 0, 0, 0));
		ni->state = 3;
		return 2;
	}
	if (ni->state == 3) {
		_put16(out, _childLLSF(LCOM_NULL, 1, 0, 0, 0));
		ni->state = 4;
		return 2;
	}
	return 0;
}

/* ============================= 6. translator (air.c:713) ============================= */

static void _releaseHeld(struct CableAir* air);
static void _exitClosePassed(struct CableAir* air);
static void _airResetLink(struct CableAir* air);
static void _flushPendingStandby(struct CableAir* air);
static void _handleHostFrame(struct CableAir* air, const uint8_t* data, unsigned length);

static void _childQueuePush(struct CableAir* air, const uint8_t* data, unsigned size, bool isLinkPlayer) {
	if (air->childCount >= CHILD_QUEUE || size > MAX_BLOCK_BYTES) {
		AIRLOG(air, "child block queue full, %u bytes dropped", size);
		return;
	}
	struct ChildBlock* block = &air->childQueue[(air->childHead + air->childCount) % CHILD_QUEUE];
	memset(block, 0, sizeof(*block));
	memcpy(block->data, data, size);
	block->size = size;
	block->isLinkPlayer = isLinkPlayer;
	++air->childCount;
}

static void _deliverLinkPlayerToGame(struct CableAir* air) {
	if (!air->haveHostLP || !air->sessionRubyLP || air->p0LPDelivered) {
		return;
	}
	uint8_t block[LINK_PLAYER_BLOCK_SIZE];
	cable_air_lp_for_game(air, block);
	_cablePushBlock(air, block, sizeof(block));
	air->p0LPDelivered = true;
	air->readyAt = air->cablePackets + SESSION_SETTLE_PACKETS;
	if (!air->cardRequested && !air->roomClosed && air->closeCount == 0 && !air->cancelReturn) {
		air->cardArmed = true;
		air->cardAt = air->cablePackets + 15;
	}
	if (air->cancelReturn) {
		air->cancelReturn = false;
		air->roomClosed = false;
		air->exitKeySeen = false;
		air->keysActive = true;
		AIRLOG(air, "room re-entered after the cancelled trade: keys resume");
	}
	AIRLOG(air, "leader's LinkPlayer delivered to the game (link type %04X)", _le16(&block[LP_LINK_TYPE_OFFSET]));
	/* bench (Sapphire vs LeafGreen, first full trade): after the trade the game closes (#3, two rounds)
	 * and relinks for the trade menu (link type 1122), then waits for its master (us) to pull its party. The Switch
	 * reopens its trade menu with one more standby round, which the cable game never asks for, and as the wireless
	 * leader it waits for ours first: both screens sat at "communicating" for good. Ask for that round ourselves
	 * (the proven FRLG path needed the same, run B: "drove 10, Switch echoed"). Withdrawn if the leader
	 * pulls a block instead (_hostPull). */
	if (air->closeCount >= 3 && _le16(&air->rubyLP[LP_LINK_TYPE_OFFSET]) == 0x1122 && air->bar.mode == BAR_IDLE) {
		air->returnRound = true;
		air->purpose = PURPOSE_NONE;
		_barrierInitiate(&air->bar, BAR_STANDBY);
		AIRLOG(air, "back at the trade menu after a trade: standby round %u for the Switch's trade menu", air->bar.localCount);
	}
}

static void _maybeStartRound0(struct CableAir* air) {
	if (air->round0Started || !air->lpSendDone || !air->p0LPDelivered) {
		return;
	}
	air->round0Started = true;
	air->purpose = PURPOSE_ROUND0;
	_barrierInitiate(&air->bar, BAR_STANDBY);
	AIRLOG(air, "LinkPlayers exchanged both ways: standby round %u", air->bar.localCount);
}

static bool _sessionReady(const struct CableAir* air) {
	return !air->linkClosing && air->sessionRubyLP && air->p0LPDelivered && air->cablePackets >= air->readyAt;
}

static void _requestFromGame(struct CableAir* air, unsigned type) {
	air->expect = EXPECT_PULL;
	air->pullType = (int) type;
	air->pullRetryAt = air->cablePackets + PULL_RETRY_PACKETS;   /* cb-3.82: re-request if unanswered */
	_cablePushCmd(air, LINKCMD_SEND_BLOCK_REQ, (uint16_t) type, 0);
}

static void _releaseHeld(struct CableAir* air) {
	if (!_sessionReady(air)) {
		return;
	}
	if (air->pendingPull >= 0 && air->expect != EXPECT_PULL) {
		AIRLOG(air, "cable link ready: forwarding the leader's pull of type %d", air->pendingPull);
		_requestFromGame(air, (unsigned) air->pendingPull);
		air->pendingPull = -1;
	}
	unsigned i;
	for (i = 0; i < air->heldCount; ++i) {
		AIRLOG(air, "cable link ready: delivering a held block of %u bytes", air->held[i].size);
		_cablePushBlock(air, air->held[i].data, air->held[i].size);
	}
	air->heldCount = 0;
}

static void _requestEarlyCard(struct CableAir* air) {
	if (air->expect != EXPECT_NONE) {
		air->cardArmed = true;
		air->cardAt = air->cablePackets + 10;
		return;
	}
	/* The game leaves its link-up screen only once it has its partner's trainer card. The leader sends its real card
	 * after its own walk into the room (~5 s), so with the real card the two walk-ins run one after the other.
	 * CABLE_AIR_EARLY_HOST_CARD (default OFF: the real card readable in the room beats ~5 s): hand
	 * the game a card built from the leader's
	 * LinkPlayer now (name, trainer id, gender; no badges or play time), so both walk in together; the leader's real
	 * card is dropped when it arrives. The game's own room-entry standby is still held until the leader has our card
	 * (_gameStandby), so the leader is never left waiting. (Off: the bench order, the real card only. The
	 * early card was first removed while both sides were player 0, which is fixed.) */
	air->cardRequested = true;
	air->expect = EXPECT_CARD;
	_cablePushCmd(air, LINKCMD_SEND_BLOCK_REQ, 2, 0);
#if CABLE_AIR_EARLY_HOST_CARD
	{
		uint8_t card[CARD_SIZE];
		cable_air_build_host_card(air, card);
		air->cardSynth = true;
		_cablePushBlock(air, card, CARD_SIZE);
		AIRLOG(air, "game's trainer card asked for; the leader's card (from its LinkPlayer) goes to the game now");
	}
#else
	air->cardSynth = false;
	AIRLOG(air, "game's trainer card asked for; the leader's own card goes to the game when it arrives");
#endif
}

static void _queueCard(struct CableAir* air) {
	_childQueuePush(air, air->rubyCard, CARD_SIZE, false);
	air->childQueue[(air->childHead + air->childCount + CHILD_QUEUE - 1) % CHILD_QUEUE].isCard = true;
	air->cardQueued = true;
}

static void _hostPull(struct CableAir* air, unsigned type) {
	AIRLOG(air, "leader pulls block type %u", type);
	if (air->returnRound && air->bar.mode == BAR_STANDBY && air->bar.initiated) {
		air->returnRound = false;
		air->bar.mode = BAR_IDLE;
		air->bar.initiated = false;
		AIRLOG(air, "the leader pulls instead: our trade-menu standby is withdrawn");
	}
	if (type == 2 && air->cardRequested) {
		if (air->cardQueued) {
			return;
		}
		if (air->haveRubyCard) {
			_queueCard(air);
		} else {
			air->cardPullWaiting = true;
		}
		return;
	}
	if (!air->lpSentToHost && type <= 1) {
		if (!air->haveRubyLP) {
			/* DEFER, don't drop. The Switch pulls the LP once, right after host_uni —
			 * typically many seconds before the game commits its LinkPlayer (first-confirmation
			 * join). Dropping the pull meant the Switch never got the game's LP even after the
			 * exchange completed. Answered in _gameBlock when the game's LP arrives (the
			 * cardPullWaiting pattern). */
			air->lpPullWaiting = true;
			AIRLOG(air, "pull before we know the game's LinkPlayer: deferred");
			return;
		}
		uint8_t buffer[LP_BUFFER_SIZE];
		cable_air_build_lp_for_host(air, buffer);
		_childQueuePush(air, buffer, LP_BUFFER_SIZE, true);
		air->lpSentToHost = true;
		return;
	}
	if (air->expect == EXPECT_PULL || air->pendingPull >= 0) {
		return;
	}
	/* The Switch repeats a pull until its block arrives: one answered but not yet streamed is a repeat. */
	if (air->childCount || (air->send.active && air->send.state != SEND_HOLD)) {
		return;
	}
	if (!_sessionReady(air)) {
		air->pendingPull = (int) type;
		AIRLOG(air, "cable link not ready: pull of type %u held", type);
		return;
	}
	_requestFromGame(air, type);
}

static void _hostBlock(struct CableAir* air, unsigned count, const uint8_t* data) {
	unsigned size = _sizeFromCount(count);
	if (count == 17 && !air->haveHostLP && !memcmp(data, "GameFreak inc.", 14)) {
		memcpy(air->hostLP, data, LINK_PLAYER_BLOCK_SIZE);
		air->haveHostLP = true;
		AIRLOG(air, "leader's LinkPlayer received (version %04X, link type %04X)", _le16(&data[LP_VERSION_OFFSET]),
		       _le16(&data[LP_LINK_TYPE_OFFSET]));
		_deliverLinkPlayerToGame(air);
		_maybeStartRound0(air);
		return;
	}
	/* bench: after the first standby round the Switch pulls type 0 again and sends a 200-byte block. If that
	 * block is its LinkPlayer again (magic), it is re-exchanging LinkPlayers: do not hand it to the game, and answer
	 * the pull with our LinkPlayer, not the game's gBlockSendBuffer (its trainer card), which would leave the Switch
	 * with a garbage LinkPlayer for us. */
	if (count == 17 && air->haveHostLP && !memcmp(data, "GameFreak inc.", 14)) {
		AIRLOG(air, "leader's LinkPlayer again (version %04X, link type %04X): re-exchange, ours goes back",
		       _le16(&data[LP_VERSION_OFFSET]), _le16(&data[LP_LINK_TYPE_OFFSET]));
		air->hostLPRepeat = true;
		return;
	}
	if (count == 9) {
		if (air->cardSynth) {
			AIRLOG(air, "leader's trainer card received: not needed, the game already has one");
			air->cardSynth = false;
			return;
		}
		AIRLOG(air, "leader's trainer card received");
	} else if (count == 2) {
		unsigned command = _le16(data);
		AIRLOG(air, "leader's trade-menu block %04X received", command);
		if (command == 0xEEBB) {
			air->cancelPending = true;
		}
	} else {
		AIRLOG(air, "leader's block of %u bytes received", size);
	}
	if (!_sessionReady(air)) {
		if (air->heldCount < HELD_BLOCKS && size <= MAX_BLOCK_BYTES) {
			struct ChildBlock* h = &air->held[air->heldCount++];
			h->size = size;
			memcpy(h->data, data, size);
			AIRLOG(air, "cable link not ready: block of %u bytes held", size);
		}
		return;
	}
	_cablePushBlock(air, data, size);
}

static void _gameBlock(struct CableAir* air, unsigned size, const uint8_t* data) {
	if (air->expect == EXPECT_LINK_PLAYER && size >= LINK_PLAYER_BLOCK_SIZE) {
		memcpy(air->rubyLP, data, LINK_PLAYER_BLOCK_SIZE);
		air->haveRubyLP = true;
		air->sessionRubyLP = true;
		air->expect = EXPECT_NONE;
		AIRLOG(air, "game's LinkPlayer received (version %04X, link type %04X)", _le16(&data[LP_VERSION_OFFSET]),
		       _le16(&data[LP_LINK_TYPE_OFFSET]));
		if (air->lpPullWaiting && !air->lpSentToHost) {
			/* answer the leader's LP pull that arrived before the game committed (see _hostPull) */
			air->lpPullWaiting = false;
			uint8_t buffer[LP_BUFFER_SIZE];
			cable_air_build_lp_for_host(air, buffer);
			_childQueuePush(air, buffer, LP_BUFFER_SIZE, true);
			air->lpSentToHost = true;
			AIRLOG(air, "deferred leader LP pull answered now that the game's LinkPlayer is in");
		}
		_deliverLinkPlayerToGame(air);
		_maybeStartRound0(air);
		return;
	}
	if (air->expect == EXPECT_CARD && size >= CARD_SIZE) {
		memset(air->rubyCard, 0, sizeof(air->rubyCard));
		memcpy(air->rubyCard, data, CARD_VERSION_OFFSET);
		air->rubyCard[CARD_VERSION_OFFSET] = VERSION_EMERALD;
		air->haveRubyCard = true;
		air->expect = EXPECT_NONE;
		AIRLOG(air, "game's trainer card received");
		if (air->cardPullWaiting) {
			air->cardPullWaiting = false;
			_queueCard(air);
		}
		return;
	}
	uint8_t block[MAX_BLOCK_BYTES];
	memset(block, 0, sizeof(block));
	unsigned use = size;
	if (air->expect == EXPECT_PULL && air->hostLPRepeat) {
		air->expect = EXPECT_NONE;
		air->hostLPRepeat = false;
		uint8_t lp[LP_BUFFER_SIZE];
		cable_air_build_lp_for_host(air, lp);
		_childQueuePush(air, lp, LP_BUFFER_SIZE, true);
		AIRLOG(air, "game answered pull %u; the leader re-exchanges LinkPlayers, so ours goes instead", air->pullType);
		return;
	}
	if (air->expect == EXPECT_PULL) {
		use = _sizeFromRequest((unsigned) air->pullType);
		air->expect = EXPECT_NONE;
		AIRLOG(air, "game answered pull %u with %u bytes", air->pullType, size);
	}
	memcpy(block, data, size < sizeof(block) ? size : sizeof(block));
	/* bench (block dumps): the Switch swaps trainer cards as a type-0 (200-byte) pull after the first
	 * standby round, never a type-2 one; the game's answer is its card. Count it as our card so the game's held
	 * standby goes out as soon as the leader has it (no CARD_PULL_WAIT_PACKETS wait). */
	bool asCard = air->cardRequested && !air->cardSendDone && air->lpSentToHost && size >= CARD_VERSION_OFFSET
	              && memcmp(data, "GameFreak inc.", 14) != 0;
	if (use == CARD_SIZE) {
		memset(&block[CARD_VERSION_OFFSET], 0, sizeof(block) - CARD_VERSION_OFFSET);
		block[CARD_VERSION_OFFSET] = VERSION_EMERALD;
	}
	if (size == 2) {
		/* the game's own trade-menu command (AABB ready, DDDD set-mons, CCDD start, DCBA finish,
		 * EEBB cancel) — this is the GBA-side "confirmation". Logged so we can time the join to it. */
		AIRLOG(air, "game's trade-menu block %04X sent to the leader", _le16(data));
	}
	_childQueuePush(air, block, use, false);
	if (asCard && air->childCount) {
		air->childQueue[(air->childHead + air->childCount + CHILD_QUEUE - 1) % CHILD_QUEUE].isCard = true;
		AIRLOG(air, "the game's answer is its trainer card: the held standby follows the leader's ack");
	}
}

static void _gameStandby(struct CableAir* air) {
	if (air->cancelReturn) {
		air->cancelReturn = false;
		air->roomClosed = false;
		air->exitKeySeen = false;
		air->keysActive = true;
		AIRLOG(air, "game standby (2FFE) on re-entering the room: answered locally, keys resume");
		_cablePushCmd(air, LINKCMD_READY_EXIT_STANDBY, 0, 0);
		return;
	}
	if (air->cardRequested && !air->cardSendDone && !air->roomClosed) {
		air->pendingGameStandby = true;
		air->pendingStandbyAt = air->cablePackets;
		AIRLOG(air, "game standby (2FFE) is early: waiting for the leader's card pull");
		return;
	}
	AIRLOG(air, "game standby (2FFE) -> wireless standby round %u", air->bar.localCount);
	air->purpose = PURPOSE_GAME_STANDBY;
	_barrierInitiate(&air->bar, BAR_STANDBY);
}

static void _flushPendingStandby(struct CableAir* air) {
	if (air->pendingGameStandby && air->cardSendDone && air->bar.mode == BAR_IDLE) {
		air->pendingGameStandby = false;
		AIRLOG(air, "the leader's card is through: the game's standby now goes to the wireless side");
		_gameStandby(air);
	}
}

static void _gameClose(struct CableAir* air) {
	if (air->exitKeySeen && !air->cancelPending && !air->roomClosed) {
		if (air->exitKeyQueued && air->keysActive) {
			AIRLOG(air, "game close (5FFF) after EXIT_ROOM: the game's exit key has not reached the leader yet, sending it first");
			air->exitCloseDeferred = true;
			air->exitDeferFrames = 0;
			return;
		}
		AIRLOG(air, "game close (5FFF) after EXIT_ROOM: closing the wireless link");
		air->keysActive = false;
		air->purpose = PURPOSE_EXIT_CLOSE;
		if (air->hostClosedSeen) {
			_exitClosePassed(air);
		} else {
			_barrierInitiate(&air->bar, BAR_CLOSE);
		}
		return;
	}
	AIRLOG(air, "game close (5FFF) #%u: wireless standby rounds, then the cable link is answered", air->closeCount + 1);
	air->exitKeyQueued = false;
	air->keysActive = false;
	air->roomClosed = true;
	air->purpose = PURPOSE_GAME_CLOSE;
	air->cancelClose = air->cancelPending;
	air->cancelPending = false;
	air->closeRoundsLeft = (air->closeCount % 2 == 0) ? 2 : 1;
	if (air->cancelClose) {
		air->closeRoundsLeft = 2;
		air->closeCount = 0;
	} else {
		++air->closeCount;
	}
	_barrierInitiate(&air->bar, BAR_STANDBY);
}

static void _exitLingerFinish(struct CableAir* air, const char* why, bool disconnect) {
	AIRLOG(air, "wireless link ended: %s", why);
	air->exitLinger = 0;
	if (disconnect && air->sw.disconnect) {
		air->sw.disconnect(air->sw.ctx, 0);
	}
	_airResetLink(air);
	air->link = -1;
}

static void _exitClosePassed(struct CableAir* air) {
	AIRLOG(air, "room exit complete: answering the game; the wireless link stays up until the leader has taken our close");
	_cablePushCmd(air, LINKCMD_READY_CLOSE_LINK, 0, 0);
	air->exitLinger = 1;
	air->haveRubyLP = false;
	air->closeCount = 0;
	air->roomClosed = false;
	air->cancelPending = false;
	air->cancelClose = false;
	air->cancelReturn = false;
	air->exitKeySeen = false;
	air->hostClosedSeen = false;
	air->purpose = PURPOSE_NONE;
}

static void _roundPassed(struct CableAir* air) {
	AIRLOG(air, "standby round passed (count now %u)", air->bar.localCount);
	air->returnRound = false;
	switch (air->purpose) {
	case PURPOSE_EXIT_CLOSE:
		_exitClosePassed(air);
		break;
	case PURPOSE_GAME_STANDBY:
		if (!air->roomClosed) {
			air->keysActive = true;
		}
		air->purpose = PURPOSE_NONE;
		_cablePushCmd(air, LINKCMD_READY_EXIT_STANDBY, 0, 0);
		break;
	case PURPOSE_GAME_CLOSE:
		if (air->closeRoundsLeft > 1) {
			--air->closeRoundsLeft;
			_barrierInitiate(&air->bar, BAR_STANDBY);
		} else {
			air->closeRoundsLeft = 0;
			air->purpose = PURPOSE_NONE;
			_cablePushCmd(air, LINKCMD_READY_CLOSE_LINK, 0, 0);
			if (air->cancelClose) {
				air->cancelClose = false;
				air->cancelReturn = true;
				AIRLOG(air, "trade cancelled: waiting for the game to re-enter the room");
			}
		}
		break;
	default:
		air->purpose = PURPOSE_NONE;
		break;
	}
	_flushPendingStandby(air);
}

/* ============================= 7. air (air.c:1111) ============================= */

static void _buildGameData(struct CableAir* air, uint8_t* out) {
	const uint8_t* lp = air->rubyLP;
	memset(out, 0, 26);
	_put16(&out[0], 2);
	unsigned compat = 2 | NI_COMPAT_NATIONAL | (VERSION_EMERALD << 10);
	_put16(&out[2], (uint16_t) compat);
	_put16(&out[4], _le16(&lp[LP_TRAINER_ID_OFFSET]));
	out[12] = 4 | 0x80;
	uint8_t* uname = &out[17];
	unsigned i;
	for (i = 0; i < 7 && lp[LP_NAME_OFFSET + i] != 0xFF; ++i) {
		uname[i] = lp[LP_NAME_OFFSET + i];
	}
	uname[i] = 0xFF;
}

static void _airSend(struct CableAir* air, const uint8_t* data, unsigned length) {
	if (air->sw.send_frame) {
		air->sw.send_frame(air->sw.ctx, data, length);
	}
	++air->childFrames;
}

static void _airResetLink(struct CableAir* air) {
	air->niStarted = false;
	air->hostUni = false;
	air->hostNiDone = false;
	air->tag = 0;
	air->keyCount = 0;
	_recvReset(&air->rx0);
	_recvReset(&air->rx1);
	memset(&air->send, 0, sizeof(air->send));
	air->childCount = 0;
	air->childHead = 0;
	_barrierInit(&air->bar);
	air->lpSentToHost = false;
	air->lpPullWaiting = false;
	air->lpSendDone = false;
	air->cardArmed = false;
	air->cardRequested = false;
	air->haveRubyCard = false;
	air->cardSynth = false;
	air->cardQueued = false;
	air->cardPullWaiting = false;
	air->cardSendDone = false;
	air->pendingGameStandby = false;
	air->round0Started = false;
	air->keysActive = false;
	memset(&air->rubyKeys, 0, sizeof(air->rubyKeys));
	_hkReset(&air->hostKeys);
	air->lastHostKeyCount = -1;
	air->exitKeyQueued = false;
	air->exitCloseDeferred = false;
	air->exitDeferFrames = 0;
	air->purpose = PURPOSE_NONE;
	air->hostFrames = 0;
	air->childFrames = 0;
	air->silentFrames = 0;
}

static void _startSearch(struct CableAir* air) {
	air->link = AIR_SEARCH;
	air->linkFrames = 0;
	if (air->sw.search_start) {
		air->sw.search_start(air->sw.ctx);
	}
	AIRLOG(air, "searching for a leader");
}

static void _logHostOp(struct CableAir* air, unsigned op, const uint16_t words[7]) {
	if (op == RFUCMD_SEND_BLOCK || op == RFUCMD_SEND_HELD_KEYS || op == 0) {
		return;
	}
	if (op == air->lastHostOp && op != RFUCMD_SEND_BLOCK_REQ) {
		return;
	}
	air->lastHostOp = op;
	AIRLOG(air, "host %04X %04X %04X %04X", words[0], words[1], words[2], words[3]);
}

static void _chooseSlot(struct CableAir* air, uint16_t words[7]) {
	memset(words, 0, 7 * sizeof(uint16_t));
	if (_barrierWant(&air->bar, words)) {
		return;
	}
	if (air->bar.mode != BAR_IDLE) {
		return;
	}
	if (!air->send.active && air->childCount) {
		struct ChildBlock* block = &air->childQueue[air->childHead];
		_sendStart(&air->send, block->data, block->size, block->isLinkPlayer);
		air->send.isCard = block->isCard;
		_recvReset(&air->rx1);
		AIRLOG(air, "sending a block of %u bytes (%u fragments)", block->size, air->send.count);
		air->childHead = (air->childHead + 1) % CHILD_QUEUE;
		--air->childCount;
	}
	if (air->send.active) {
		bool wasLP = air->send.isLinkPlayer;
		bool wasCard = air->send.isCard;
		_sendTick(&air->send, &air->rx1, words);
		if (!air->send.active && wasCard) {
			air->cardSendDone = true;
			AIRLOG(air, "trainer card acknowledged by the leader");
			_flushPendingStandby(air);
		} else if (!air->send.active && wasLP) {
			air->lpSendDone = true;
			AIRLOG(air, "LinkPlayer block acknowledged by the leader");
			_maybeStartRound0(air);
		} else if (!air->send.active) {
			AIRLOG(air, "block acknowledged by the leader");
		}
		if (words[0]) {
			return;
		}
	}
	if (air->keysActive) {
		uint8_t code = _keyPop(&air->rubyKeys);
		words[0] = RFUCMD_SEND_HELD_KEYS;
		words[1] = (uint16_t)((air->keyCount++ << 8) | code);
		air->statRubyKeyTx++;                           /* diag: GBA->leader held-keys uplink */
		if (code != LINK_KEY_CODE_EMPTY) air->statLastRubyKey = code;
		if (code == LINK_KEY_CODE_EXIT_ROOM) {
			AIRLOG(air, "the game's EXIT_ROOM key sent to the leader");
			air->exitKeyQueued = false;
			if (air->exitCloseDeferred) {
				air->exitCloseDeferred = false;
				_gameClose(air);
			}
		}
	}
}

static void _sendUni(struct CableAir* air, uint16_t words[7]) {
	uint8_t frame[2 + SLOT_BYTES];
	if (words[0]) {
		words[0] |= (uint16_t)(air->tag << 5);
		air->tag = (air->tag + 1) & 7;
	}
	_put16(frame, (uint16_t)((LCOM_UNI << 10) | SLOT_BYTES));
	unsigned i;
	for (i = 0; i < 7; ++i) {
		_put16(&frame[2 + i * 2], words[i]);
	}
	_airSend(air, frame, sizeof(frame));
}

/* The UNI slot0/slot1 dispatch, shared by _handleHostFrame (whole-child mode) and the engine feed
 * (option-B mode, where cport/sim delivers the parsed leader slots). slot0 = leader command,
 * slot1 = the leader's reflection of our slot (block acks). Returns true when the link ended (the
 * leader took our exit-linger close) and the caller must stop. */
static bool _uniSlots(struct CableAir* air, const uint8_t* slot0, const uint8_t* slot1) {
	uint16_t words0[7];
	_slotToWords(slot0, words0);
	bool sawBarrier = false;
	bool completed;
	if (!_slotIdle(slot0)) {
		unsigned op = words0[0] & RFUCMD_MASK;
		_logHostOp(air, op, words0);
		switch (op) {
		case RFUCMD_SEND_PLAYER_IDS:
			break;
		case RFUCMD_SEND_BLOCK_REQ:
			_hostPull(air, words0[1]);
			break;
		case RFUCMD_SEND_BLOCK_INIT:
		case RFUCMD_SEND_BLOCK:
			_feedRecv(&air->rx0, words0, slot0, &completed);
			if (completed) {
				_hostBlock(air, air->rx0.count, air->rx0.buf);
				air->rx0.receiving = false;
				air->rx0.done = false;
				air->rx0.flags = 0;
			}
			break;
		case RFUCMD_SEND_HELD_KEYS:
			air->statHostKeyRx++;                       /* diag: a 0xBE00 from the leader reached us */
			if (air->lastHostKeyCount != (int)(words0[1] >> 8)) {
				air->lastHostKeyCount = words0[1] >> 8;
				_hkPush(&air->hostKeys, words0[1] & 0xFF, air->keysActive);
				air->statHostKeyPush++;
				air->statLastHostKey = words0[1] & 0xFF;
				if ((words0[1] & 0xFF) == LINK_KEY_CODE_EXIT_ROOM) {
					air->exitKeySeen = true;
				}
			}
			break;
		case RFUCMD_READY_EXIT_STANDBY:
			sawBarrier = true;
			if (_barrierHostStandby(air, words0[1])) {
				_roundPassed(air);
			}
			break;
		case RFUCMD_READY_CLOSE_LINK:
			sawBarrier = true;
			_barrierHostClose(air, words0[1]);
			air->hostClosedSeen = true;
			if (air->purpose == PURPOSE_EXIT_CLOSE) {
				_exitClosePassed(air);
			}
			break;
		case RFUCMD_DISCONNECT:
			AIRLOG(air, "leader sent DISCONNECT");
			break;
		default:
			break;
		}
	}
	if (!_slotIdle(slot1)) {
		uint16_t words1[7];
		_slotToWords(slot1, words1);
		_feedRecv(&air->rx1, words1, slot1, &completed);
		if (air->exitLinger && (words1[0] & RFUCMD_MASK) == RFUCMD_READY_CLOSE_LINK) {
			_exitLingerFinish(air, "the leader took our close", true);
			return true;
		}
	}
	if (_barrierObserve(air, sawBarrier)) {
		_roundPassed(air);
	}
	return false;
}

static void _handleHostFrame(struct CableAir* air, const uint8_t* data, unsigned length) {
	++air->hostFrames;
	if (length < 3) {
		if (air->link == AIR_NI && air->ni.state < 4) {
			uint8_t frame[16];
			unsigned len = _niNext(&air->ni, frame);
			if (len) {
				_airSend(air, frame, len);
			}
		}
		return;
	}
	uint32_t header = data[0] | (data[1] << 8) | (data[2] << 16);
	unsigned state = (header >> 14) & 0xF;
	unsigned ack = (header >> 13) & 1;
	unsigned n = (header >> 11) & 3;
	unsigned phase = (header >> 9) & 3;

	if (state != LCOM_UNI) {
		if (air->link != AIR_NI) {
			return;
		}
		if (air->hostFrames == 1 || (state == LCOM_NI_START && !ack)) {
			AIRLOG(air, "host NI frame state=%u ack=%u n=%u phase=%u len=%u", state, ack, n, phase, length);
		}
		uint8_t frame[16];
		unsigned len = 0;
		if (air->ni.state < 4) {
			len = _niNext(&air->ni, frame);
		} else if (!ack && (state == LCOM_NI_START || state == LCOM_NI || state == LCOM_NI_END)) {
			_put16(frame, _childLLSF(state, n, phase, 1, 0));
			len = 2;
			if (state == LCOM_NI_END) {
				air->hostNiDone = true;
			}
		} else if (!ack && state == LCOM_NULL) {
			air->hostNiDone = true;
		}
		if (len) {
			_airSend(air, frame, len);
		}
		return;
	}

	if (length < HOST_FRAME_BYTES) {
		return;
	}
	if (air->link == AIR_NI) {
		if (air->ni.state < 4) {
			return;
		}
		air->link = AIR_UNI;
		air->hostUni = true;
		AIRLOG(air, "host entered UNI: link layer up (%u host frames)", air->hostFrames);
	}

	if (_uniSlots(air, &data[3], &data[3 + SLOT_BYTES])) {
		return;   /* link ended (exit-linger close taken) */
	}
	uint16_t out[7];
	_chooseSlot(air, out);
	_sendUni(air, out);
}

/* ============================= public API ============================= */

void cable_air_init(struct CableAir* air) {
	memset(air, 0, sizeof(*air));
	_barrierInit(&air->bar);
	_recvReset(&air->rx0);
	_recvReset(&air->rx1);
	air->link = -1;
	air->pendingPull = -1;
	air->lastHostKeyCount = -1;
	_hkReset(&air->hostKeys);
}

void cable_air_set_switch(struct CableAir* air, const struct CableAirSwitch* sw) {
	air->sw = *sw;
}

void cable_air_set_trace(struct CableAir* air, cable_air_trace_fn fn, void* ctx) {
	air->trace = fn;
	air->traceCtx = ctx;
}

/* Join-confirmation gate (option B). Driven by the mode from the brain's join status. Once the Switch
 * has accepted (host_status == JOIN_GROUP_OK) the standby-confirmation hold in _barrierObserve stops
 * biting and the game's "Press A" is answered — releasing it into the trade. */
void cable_air_set_switch_ready(struct CableAir* air, bool ready) {
	if (ready && !air->switchReady) {
		AIRLOG(air, "Switch accepted the join: releasing the confirmation");
	}
	air->switchReady = ready;
}

/* True while the game is parked at a standby confirmation waiting on the join (Switch not yet ready).
 * The mode uses this to know the game has reached its first confirmation. */
bool cable_air_join_pending(const struct CableAir* air) {
	return !air->switchReady && air->bar.mode == BAR_STANDBY && air->bar.initiated && air->joinHoldLogged;
}

/* air.c:1500 _peerReset — a new cable session (the game opened its link). */
void cable_air_reset(struct CableAir* air) {
	air->linkClosing = false;
	air->cablePackets = 0;
	air->readyAt = 0;
	air->announced = false;
	air->announceRetryAt = 0;
	air->pullRetryAt = 0;
	air->expect = EXPECT_NONE;
	air->sessionRubyLP = false;
	air->p0LPDelivered = false;
	air->rxSize = 0;
	air->rxPos = 0;
	air->cqHead = 0;
	air->cqCount = 0;
	++air->sessions;
}

/* air.c:1515 _peerGameCommand — the game finished a packet with a non-zero command. */
void cable_air_game_command(struct CableAir* air, const uint16_t command[CMD_WORDS]) {
	unsigned i;
	switch (command[0]) {
	case LINKCMD_SEND_LINK_TYPE:
		/* (the pkts=599 park): the game announcing its link type IS its command-phase
		 * entry — the moment our own 2222 announce must be on the wire so the game InitBlockSends
		 * its LinkPlayer. The one-shot packet-count announce (next_command, cablePackets >
		 * ANNOUNCE_PACKETS=30) fires ~30 packets after the C6's packet engine starts pumping —
		 * but with the first-confirmation join hold the game parks for many seconds of CRC-churn
		 * packets first, so the announce went out hundreds of packets before the game arrived and
		 * was never seen (vocab 2222:1, no bbbb, both sides idling 0000 forever). Re-arm here so
		 * next_command re-announces on the very next packet, synchronized to the game's real entry.
		 * Guarded on !haveRubyLP so a completed exchange is never re-triggered. */
		if (!air->haveRubyLP) {
			air->announced = false;
			AIRLOG(air, "game announced link type %04X: re-announcing ours next packet", command[1]);
		}
		break;
	case LINKCMD_INIT_BLOCK:
		air->rxSize = command[1] <= MAX_BLOCK_BYTES ? command[1] : 0;
		air->rxPos = 0;
		break;
	case LINKCMD_CONT_BLOCK:
		if (air->rxSize) {
			for (i = 0; i < CMD_WORDS - 1; ++i) {
				unsigned at = air->rxPos + i * 2;
				if (at + 1 < sizeof(air->rxBuf)) {
					_put16(&air->rxBuf[at], command[1 + i]);
				}
			}
			air->rxPos += (CMD_WORDS - 1) * 2;
			if (air->rxPos >= air->rxSize) {
				unsigned size = air->rxSize;
				air->rxSize = 0;
				_gameBlock(air, size, air->rxBuf);
			}
		}
		break;
	case LINKCMD_SEND_HELD_KEYS:
		/* cb-3.78: the cart drives the in-room held-key loop with 0xCAFE. The Celio/FRLG
		 * port gates keysActive behind a 2FFE GAME_STANDBY round (_gameStandby -> _roundPassed), but
		 * Sapphire (RSE) enters the room and goes STRAIGHT to 0xCAFE with no 2FFE — so keysActive stays
		 * off, we drop every held-key (OUT-cafe n=0), and the unanswered cart crashes ~200ms into the
		 * room (bench: cafe:87, active=0, be00=0). Engage the loop on the game's own 0xCAFE
		 * if nothing else has. No-op on FRLG, where the 2FFE round already set keysActive before any
		 * 0xCAFE; guarded so a closing/exiting room never re-arms keys. */
		/* bench: the cb-3.78 "engage on the game's 0xCAFE" is gone. It covered for the made-up card
		 * (above), which let Ruby into its room early; engaging here let the game run while the Switch was still
		 * outside its room. Keys start when a standby round passes, as on the Switch's own side. */
		if ((command[1] & 0xFF) != LINK_KEY_CODE_EMPTY) {
			_keyPush(&air->rubyKeys, command[1] & 0xFF);
		}
		if ((command[1] & 0xFF) == LINK_KEY_CODE_EXIT_ROOM) {
			air->exitKeySeen = true;
			air->exitKeyQueued = true;
		}
		break;
	case LINKCMD_READY_EXIT_STANDBY:
		_gameStandby(air);
		break;
	case LINKCMD_READY_CLOSE_LINK:
		air->linkClosing = true;
		_gameClose(air);
		break;
	default:
		break;
	}
}

/* air.c:1563 _peerNextCommand — fill the virtual player's next command; false to send nothing. */
bool cable_air_next_command(struct CableAir* air, uint16_t command[CMD_WORDS]) {
	++air->cablePackets;
	if (air->heldCount || air->pendingPull >= 0) {
		_releaseHeld(air);
	}
	if (air->cardArmed && air->cablePackets >= air->cardAt) {
		air->cardArmed = false;
		_requestEarlyCard(air);
	}
	if (air->pendingGameStandby && !air->cardSendDone
	    && air->cablePackets - air->pendingStandbyAt > CARD_PULL_WAIT_PACKETS) {
		air->cardSendDone = true;   /* the leader is not going to pull the card: see CARD_PULL_WAIT_PACKETS */
		AIRLOG(air, "no card pull from the leader after %u packets: releasing the game's standby", CARD_PULL_WAIT_PACKETS);
	}
	if (air->pendingGameStandby && air->cardSendDone) {
		_flushPendingStandby(air);   /* goes out once the wireless barrier is idle */
	}
	/* One announce per cable session. The C6's cb-3.81 retry re-sent 2222 whenever the game's
	 * LinkPlayer was unknown, for frames its bit-banged PHY lost. _exitClosePassed clears haveRubyLP, so on the room
	 * exit the retry put a 2222 in front of our 5FFF and the cart showed "Communication error". The RP2040 link does
	 * not lose frames, and GB-Link's translator announces once too. */
	if (!air->announced && air->cablePackets > ANNOUNCE_PACKETS) {
		air->announced = true;
		air->expect = EXPECT_LINK_PLAYER;
		memset(command, 0, CMD_WORDS * sizeof(uint16_t));
		command[0] = LINKCMD_SEND_LINK_TYPE;
		command[1] = 0x1133;
		return true;
	}
	if (air->cqCount) {
		memcpy(command, air->cq[air->cqHead], CMD_WORDS * sizeof(uint16_t));
		air->cqHead = (air->cqHead + 1) % CABLE_QUEUE;
		--air->cqCount;
		return true;
	}
	/* (The C6's cb-3.82 block-request retry is gone for the same reason: no lost frames on the RP2040 link, and a
	 * stray CCCC is as harmful as the stray 2222 above. GB-Link's translator has neither.) */
	if (air->keysActive) {
		uint8_t k = _hkPop(&air->hostKeys);
		memset(command, 0, CMD_WORDS * sizeof(uint16_t));
		command[0] = LINKCMD_SEND_HELD_KEYS;
		command[1] = k;
		air->statCafeReply++;                           /* diag: 0xCAFE reply built toward the GBA */
		if (k != LINK_KEY_CODE_EMPTY) air->statCafeReplyKey++;   /* ...carrying a real key press */
		return true;
	}
	return false;
}

/* air.c:1452 _airFrame — once per emulated frame. Events are pushed in via the on_* entries, so the
 * reference's _drainEvents is gone; the rest of the per-frame housekeeping stays. */
void cable_air_frame(struct CableAir* air) {
	if (air->sw.poll) {
		air->sw.poll(air->sw.ctx);
	}
	if (air->sw.frame) {
		air->sw.frame(air->sw.ctx);
	}
	if (air->link < 0 && air->haveRubyLP) {
		_startSearch(air);
	}
	if (air->exitCloseDeferred && ++air->exitDeferFrames > 120) {
		AIRLOG(air, "the game's exit key never got out: closing the link anyway");
		air->exitCloseDeferred = false;
		air->exitKeyQueued = false;
		_gameClose(air);
	}
	if (air->exitLinger && air->exitLinger % 60 == 0) {
		air->bar.burstN = 0;
	}
	if (air->exitLinger && ++air->exitLinger > EXIT_LINGER_FRAMES) {
		_exitLingerFinish(air, "the leader never took our close, giving up", true);
	}
	if (air->link == AIR_NI && air->hostFrames == 0 && air->ni.state == 0 && ++air->linkFrames % 8 == 0) {
		struct NI copy = air->ni;
		uint8_t frame[16];
		unsigned len = _niNext(&copy, frame);
		if (len) {
			_airSend(air, frame, len);
		}
	}
	if (air->link == AIR_NI || air->link == AIR_UNI) {
		if (++air->silentFrames > HOST_SILENT_FRAMES) {
			AIRLOG(air, "the leader went quiet");
			_airResetLink(air);
			if (air->sw.disconnect) {
				air->sw.disconnect(air->sw.ctx, 0);
			}
			_startSearch(air);
		}
	}
}

/* ---- switch edge events (were the RFU_EVENT_* cases of _drainEvents) ---- */

void cable_air_on_broadcast(struct CableAir* air, uint16_t device_id, int slot, const uint32_t words[6]) {
	/* RfuGameData: word 3's low 7 bits are the activity (ACTIVITY_TRADE = 4). */
	if (air->link == AIR_SEARCH && slot != 0xFF && (words[3] & 0x7F) == 4) {
		AIRLOG(air, "leader %04X found: joining", device_id);
		if (air->sw.search_stop) {
			air->sw.search_stop(air->sw.ctx);
		}
		air->hostId = device_id;
		air->link = AIR_CONNECTING;
		air->linkFrames = 0;
		if (air->sw.connect) {
			air->sw.connect(air->sw.ctx, device_id);
		}
	}
}

void cable_air_on_connect_result(struct CableAir* air, bool accepted, int slot) {
	if (air->link != AIR_CONNECTING) {
		return;
	}
	if (accepted) {
		AIRLOG(air, "leader accepted us (slot %d): NI handshake", slot);
		uint8_t src[26];
		_buildGameData(air, src);
		_airResetLink(air);
		_niInit(&air->ni, src);
		air->niStarted = true;
		air->link = AIR_NI;
		air->linkFrames = 0;
	} else {
		AIRLOG(air, "leader refused or did not answer: searching again");
		_startSearch(air);
	}
}

void cable_air_on_host_frame(struct CableAir* air, const uint8_t* data, unsigned len) {
	if (len && (air->link == AIR_NI || air->link == AIR_UNI)) {
		air->silentFrames = 0;
		_handleHostFrame(air, data, len);
	}
}

void cable_air_on_disconnected(struct CableAir* air) {
	AIRLOG(air, "disconnected from the leader");
	if (air->exitLinger) {
		_exitLingerFinish(air, "the leader disconnected us", false);
	} else if (air->link == AIR_NI || air->link == AIR_UNI) {
		_airResetLink(air);
		_startSearch(air);
	}
}

/* ============================= option-B engine seam (drives cport/sim) ============================= */
/* In option B, the proven cport/sim owns NI + reliable + Pia to the Switch; cable_air is
 * sim's engine at the trade-slot level. sim does its own Switch NI (from the identity below), then
 * pulls our next UNI slot via _tick and hands back the leader's parsed slots via _feed_slots. These
 * are the same _chooseSlot / _uniSlots the whole-child path uses, so they inherit the flow-test
 * coverage; sim's rfu layer adds the wire framing (no _sendUni/LLSF wrap here). */

/* engine->tick: fill the next child UNI trade slot (idle = all-zero). */
void cable_air_engine_tick(struct CableAir* air, uint16_t words[CMD_WORDS - 1]) {
	_chooseSlot(air, words);
}

/* engine->feed_in_frame: the leader's slot0 (command) + slot1 (our-block ack reflection). */
void cable_air_engine_feed_slots(struct CableAir* air, const uint8_t slot0[SLOT_BYTES],
                                 const uint8_t slot1[SLOT_BYTES]) {
	_uniSlots(air, slot0, slot1);
}

/* [air-keys] held-key bridge summary — call from the drive-loop heartbeat (OFF the cable hot path).
 * IN  = held-keys arriving from the leader (Switch) -> hostKeys -> the GBA's 0xCAFE replies.
 * OUT-cafe = 0xCAFE replies we build to the GBA; 'key' = how many carried a real (non-0x11) press.
 * UP-ruby  = the GBA's own keys we forward up to the leader. If IN.rx stays 0 while the Switch is
 * driving its trade menu, the leader's 0xBE00 held-keys are not reaching cable_air (upstream/LDN or
 * wrong inbound layer) — compare with gba_bridge.c:862, which DOES consume the host 0xBE00 flood. */
void cable_air_keystats_dump(const struct CableAir* air) {
	printf("[air-keys] active=%d IN(rx=%lu push=%lu last=%02x q=%u lastCnt=%d) OUT-cafe(n=%lu key=%lu) UP-ruby(n=%lu last=%02x)\n",
	       (int)air->keysActive,
	       (unsigned long)air->statHostKeyRx, (unsigned long)air->statHostKeyPush,
	       air->statLastHostKey, air->hostKeys.n, air->lastHostKeyCount,
	       (unsigned long)air->statCafeReply, (unsigned long)air->statCafeReplyKey,
	       (unsigned long)air->statRubyKeyTx, air->statLastRubyKey);
}

/* The cart's identity for sim's Switch-side NI game-data (from the game's LinkPlayer). Returns false
 * until the game's LinkPlayer has been received. name8 = raw FRLG-charmap bytes (0xFF-terminated). */
bool cable_air_get_identity(const struct CableAir* air, uint16_t* tid, uint8_t name8[8]) {
	if (!air->haveRubyLP) {
		return false;
	}
	if (tid) {
		*tid = cable_air_le16(&air->rubyLP[LP_TRAINER_ID_OFFSET]);
	}
	if (name8) {
		memcpy(name8, &air->rubyLP[LP_NAME_OFFSET], 8);
	}
	return true;
}
