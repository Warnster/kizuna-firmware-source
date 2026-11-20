/* SPDX-License-Identifier: AGPL-3.0-only
 * Derived from frlg-ldn-trade (https://github.com/tornadus/frlg-ldn-trade), Copyright (c) the frlg-ldn-trade authors.
 * Changes Copyright (c) 2026 Kizuna. See NOTICE.
 */
/* rfu.h — FRLG AgbRfu 14-byte command-slot layer, portable C port of frlgsim/rfu.py.
 *
 * Pure/deterministic: no hardware, no allocation. Compiles + unit-tests on the host,
 * and later on the ESP32. This is "the seam" — the 14-byte slot the child emits per VBlank.
 * See frlgsim/rfu.py for the pokefirered (link_rfu_2.c / librfu) citations. */
#ifndef RFU_H
#define RFU_H
#include <stdint.h>
#include <stddef.h>

#define RFU_SLOT_LEN     14   /* 7 x u16 LE */
#define RFU_UNI_SLOT_LEN 16   /* LLSF header(2) + slot(14) */

/* OUT opcodes (high byte of word0) */
#define RFU_IDLE               0x0000
#define RFU_SEND_BLOCK_INIT    0x8800
#define RFU_SEND_BLOCK         0x8900
#define RFU_SEND_HELD_KEYS     0xBE00
#define RFU_READY_EXIT_STANDBY 0x6600
#define RFU_READY_CLOSE_LINK   0x5F00
#define RFU_SEND_BLOCK_REQ     0xA100   /* IN-only */
#define RFU_SEND_PLAYER_IDS    0x7700   /* IN-only */
#define RFU_DISCONNECT         0xED00   /* IN-only */

#define RFU_RFUCMD_MASK     0xFF00
#define RFU_FRAG_INDEX_MASK 0x1F        /* SEND_BLOCK index = low 5 bits; tag = bits 5-7 */
#define RFU_OWNER_FLAG      0x80        /* owner word2 = mpId | 0x80 */
/* librfu LLSF command states */
#define RFU_LCOM_NULL       0
#define RFU_LCOM_NI_START   1
#define RFU_LCOM_NI         2
#define RFU_LCOM_NI_END     3
#define RFU_LCOM_UNI        4

/* Serialize 7 u16 words -> 14-byte slot (LE). */
void rfu_words_to_slot(const uint16_t words[7], uint8_t out[RFU_SLOT_LEN]);
void rfu_idle_slot(uint8_t out[RFU_SLOT_LEN]);

/* Word builders (pre-tag): fill a 7-int gSendCmd. */
void rfu_init_words(uint16_t count, uint16_t owner, uint16_t out[7]);
void rfu_send_block_words(uint16_t index, const uint8_t chunk12[12], uint16_t out[7]);
void rfu_held_keys_words(uint16_t keycode, uint16_t out[7]);
void rfu_exit_standby_words(uint16_t count, uint16_t out[7]);
void rfu_close_link_words(uint16_t count, uint16_t out[7]);

/* Wrap a 14-byte cmd in a CHILD UNI link-layer sub-frame -> 16 bytes. */
void rfu_uni_slot(const uint8_t cmd14[RFU_SLOT_LEN], uint8_t out[RFU_UNI_SLOT_LEN]);

/* Rolling-tag slot builder (one per outgoing link = the child's childSendCmdId). */
typedef struct { uint8_t tag; } rfu_slot_builder;
void rfu_slot_builder_init(rfu_slot_builder *sb);
void rfu_slot_builder_build(rfu_slot_builder *sb, const uint16_t words[7],
                            uint8_t out[RFU_SLOT_LEN]);

/* Decode a 2-byte CHILD LLSF header (rfu.py parse_llsf_child). */
typedef struct { int state, ack, n, phase, size; } rfu_child_llsf;
void rfu_parse_llsf_child(const uint8_t slot[2], rfu_child_llsf *out);

/* Build a 2-byte CHILD LLSF header (rfu.py child_ni_llsf):
 * (state&0xF)<<10 | ack<<9 | (n&3)<<7 | (phase&3)<<5 | (size&0x1F), LE. */
void rfu_child_ni_llsf(int state, int n, int phase, int ack, int size, uint8_t out2[2]);

/* Parse a received (IN) 14-byte slot. valid==0 for an empty/idle slot. */
typedef struct {
    int      valid;
    uint16_t word0;
    uint16_t op;
    const char *name;
    uint8_t  low;
    uint16_t count;      /* INIT / EXIT_STANDBY / CLOSE_LINK */
    uint8_t  owner_raw;  /* INIT */
    uint8_t  peer;       /* INIT: mpId */
    uint8_t  index;      /* SEND_BLOCK */
    uint8_t  frag[12];   /* SEND_BLOCK */
    uint16_t reqtype;    /* SEND_BLOCK_REQ */
    uint16_t keycode;    /* HELD_KEYS */
} rfu_parsed;
void rfu_parse_slot(const uint8_t slot[RFU_SLOT_LEN], rfu_parsed *out);

const char *rfu_op_name(uint16_t op);

#endif /* RFU_H */
