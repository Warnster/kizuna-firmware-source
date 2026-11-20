/* SPDX-License-Identifier: AGPL-3.0-only */
/* ct_common.h -- shared constants and helpers of the gblink_standin component.
 *
 * Ported from GB-Link-Switch-LDN web/js/cable/translator.js (constants, sizeFromCount, sizeFromRequest, KeyRuns, KeyQueue),
 * commit c972db4, https://github.com/GB-Link/GB-Link-Switch-LDN. Copyright (c) the GB-Link-Switch-LDN authors.
 * Modified for kizuna (2026-11-02): ported to C, fixed-size storage. See NOTICE.
 *
 * This program is free software: you can redistribute it and/or modify it under the terms of the GNU Affero General Public
 * License as published by the Free Software Foundation, version 3 of the License. It is distributed WITHOUT ANY WARRANTY;
 * see <https://www.gnu.org/licenses/agpl-3.0.html>.
 */
#ifndef CT_COMMON_H
#define CT_COMMON_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum {
    /* Gen-3 Cable Club commands (pokeemerald link.h) */
    CT_LINKCMD_SEND_LINK_TYPE = 0x2222,
    CT_LINKCMD_READY_EXIT_STANDBY = 0x2ffe,
    CT_LINKCMD_READY_CLOSE_LINK = 0x5fff,
    CT_LINKCMD_CONT_BLOCK = 0x8888,
    CT_LINKCMD_INIT_BLOCK = 0xbbbb,
    CT_LINKCMD_SEND_HELD_KEYS = 0xcafe,
    CT_LINKCMD_SEND_BLOCK_REQ = 0xcccc,

    /* RFU link-layer commands (pokefirered link_rfu.h) */
    CT_RFUCMD_MASK = 0xff00,
    CT_RFUCMD_READY_CLOSE_LINK = 0x5f00,
    CT_RFUCMD_READY_EXIT_STANDBY = 0x6600,
    CT_RFUCMD_SEND_PLAYER_IDS = 0x7700,
    CT_RFUCMD_SEND_BLOCK_INIT = 0x8800,
    CT_RFUCMD_SEND_BLOCK = 0x8900,
    CT_RFUCMD_SEND_BLOCK_REQ = 0xa100,
    CT_RFUCMD_SEND_HELD_KEYS = 0xbe00,
    CT_RFUCMD_DISCONNECT = 0xed00,

    /* RFU1 frame types between the C3 and the RP2040 / glp_host side (rfu1.h) */
    CT_RFU1_BROADCAST = 0,
    CT_RFU1_CONNECT_REQ = 1,
    CT_RFU1_CONNECT_ACK = 2,
    CT_RFU1_CONNECT_NACK = 3,
    CT_RFU1_DISCONNECT = 4,
    CT_RFU1_HOST_SEND = 5,
    CT_RFU1_CLIENT_SEND = 6,
    CT_RFU1_FRAME_MAX = 104,
    CT_RFU1_SEND_DATA = 92,

    CT_CMD_WORDS = 8,           /* one cable command: 8 u16 */
    CT_BATCH = 4,               /* commands per LINK_DATA frame: 4 x 8 x u16 = 64 bytes */
    CT_DATA_BYTES = CT_BATCH * CT_CMD_WORDS * 2,
    CT_SLOT_BYTES = 14,
    CT_FRAG_BYTES = 12,
    CT_MAX_FRAGS = 24,
    CT_MAX_BLOCK_BYTES = CT_MAX_FRAGS * CT_FRAG_BYTES,   /* 288 */

    CT_KEY_EMPTY = 0x11,
    CT_KEY_READY = 0x16,
    CT_KEY_EXIT_ROOM = 0x17,
    CT_KEY_QUEUE = 48,
    CT_HOST_KEY_LAG = 12,
    CT_KEY_RATE_WINDOW = 40,
};

/* One RFU1 frame out (16, 36 or 104 bytes, magic "RFU1" included). */
typedef void (*ct_frame_fn)(void *ctx, const uint8_t *frame, size_t len);
/* Randomness (the leader's devids): any 32-bit value; esp_random() on the board. */
typedef uint32_t (*ct_rand_fn)(void *ctx);
/* Optional trace hook, never needed for correctness: an event id and two arguments. */
typedef void (*ct_trace_fn)(void *ctx, uint16_t event, uint32_t a, uint32_t b);

/* The game side a ct_link drives (translator.js / reverse.js: gameCommand, nextCommand, cableReset, ready). */
typedef struct {
    void *ctx;
    void (*game_command)(void *ctx, const uint16_t words[CT_CMD_WORDS]);
    bool (*next_command)(void *ctx, uint16_t words[CT_CMD_WORDS]);   /* false: none this packet */
    void (*reset)(void *ctx);
    bool (*ready)(void *ctx);
} ct_game_t;

static inline uint16_t ct_le16(const uint8_t *b) { return (uint16_t)(b[0] | (b[1] << 8)); }
static inline void ct_put16(uint8_t *b, uint16_t v) { b[0] = (uint8_t)v; b[1] = (uint8_t)(v >> 8); }
static inline uint32_t ct_be32(const uint8_t *b) { return ((uint32_t)b[0] << 24) | ((uint32_t)b[1] << 16) | ((uint32_t)b[2] << 8) | b[3]; }
static inline void ct_wbe32(uint8_t *b, uint32_t v) { b[0] = (uint8_t)(v >> 24); b[1] = (uint8_t)(v >> 16); b[2] = (uint8_t)(v >> 8); b[3] = (uint8_t)v; }

/* An RFU1 frame: magic, BE type, BE header, zeroed payload; returns the frame's size. */
size_t ct_rfu1_frame(uint8_t *f, uint32_t type, uint32_t header, size_t size);

uint16_t ct_size_from_count(unsigned count);
uint16_t ct_size_from_request(unsigned type);
bool ct_is_magic(const uint8_t *b);   /* "GameFreak inc." at b */

/* KeyRuns (translator.js): runs of one key code played back by time. Counts are doubles so the playback matches the JS
 * exactly (soft-float on the C3; at most ~40 pops a second). */
typedef struct {
    double count[CT_KEY_QUEUE];
    uint8_t code[CT_KEY_QUEUE];
    uint8_t head, n;
} ct_keyruns_t;
void ct_keyruns_clear(ct_keyruns_t *k);
void ct_keyruns_push(ct_keyruns_t *k, uint8_t code);
uint8_t ct_keyruns_pop(ct_keyruns_t *k, double step);
bool ct_keyruns_has(const ct_keyruns_t *k, uint8_t a, uint8_t b);   /* a run of code a or b is queued */

/* KeyQueue (translator.js): one code per event, oldest dropped past 48. */
typedef struct {
    uint8_t code[CT_KEY_QUEUE];
    uint8_t head, n;
} ct_keyqueue_t;
void ct_keyqueue_clear(ct_keyqueue_t *q);
void ct_keyqueue_push(ct_keyqueue_t *q, uint8_t code);
uint8_t ct_keyqueue_pop(ct_keyqueue_t *q);

#ifdef __cplusplus
}
#endif
#endif
