/* SPDX-License-Identifier: AGPL-3.0-only
 * Derived from frlg-ldn-trade (https://github.com/tornadus/frlg-ldn-trade), Copyright (c) the frlg-ldn-trade authors.
 * Changes Copyright (c) 2026 Kizuna. See NOTICE.
 */
/* rfu.c — portable C port of frlgsim/rfu.py. See rfu.h. */
#include "rfu.h"
#include <string.h>

static void put_u16le(uint8_t *p, uint16_t v) { p[0] = (uint8_t)(v & 0xFF); p[1] = (uint8_t)(v >> 8); }
static uint16_t get_u16le(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }

void rfu_words_to_slot(const uint16_t words[7], uint8_t out[RFU_SLOT_LEN]) {
    for (int i = 0; i < 7; i++) put_u16le(out + i * 2, words[i]);
}

void rfu_idle_slot(uint8_t out[RFU_SLOT_LEN]) { memset(out, 0, RFU_SLOT_LEN); }

void rfu_init_words(uint16_t count, uint16_t owner, uint16_t out[7]) {
    memset(out, 0, 7 * sizeof(uint16_t));
    out[0] = RFU_SEND_BLOCK_INIT;
    out[1] = count;
    out[2] = (uint16_t)((owner | RFU_OWNER_FLAG) & 0xFFFF);
}

void rfu_send_block_words(uint16_t index, const uint8_t chunk12[12], uint16_t out[7]) {
    memset(out, 0, 7 * sizeof(uint16_t));
    out[0] = (uint16_t)(RFU_SEND_BLOCK | (index & RFU_FRAG_INDEX_MASK));
    for (int i = 0; i < 6; i++) out[1 + i] = get_u16le(chunk12 + i * 2);
}

void rfu_held_keys_words(uint16_t keycode, uint16_t out[7]) {
    memset(out, 0, 7 * sizeof(uint16_t));
    out[0] = RFU_SEND_HELD_KEYS;
    out[1] = keycode;
}

void rfu_exit_standby_words(uint16_t count, uint16_t out[7]) {
    memset(out, 0, 7 * sizeof(uint16_t));
    out[0] = RFU_READY_EXIT_STANDBY;
    out[1] = count;
}

void rfu_close_link_words(uint16_t count, uint16_t out[7]) {
    memset(out, 0, 7 * sizeof(uint16_t));
    out[0] = RFU_READY_CLOSE_LINK;
    out[1] = count;
}

void rfu_uni_slot(const uint8_t cmd14[RFU_SLOT_LEN], uint8_t out[RFU_UNI_SLOT_LEN]) {
    uint16_t llsf = (uint16_t)((RFU_LCOM_UNI << 10) | RFU_SLOT_LEN); /* 0x100e */
    put_u16le(out, llsf);
    memcpy(out + 2, cmd14, RFU_SLOT_LEN);
}

void rfu_slot_builder_init(rfu_slot_builder *sb) { sb->tag = 0; }

void rfu_slot_builder_build(rfu_slot_builder *sb, const uint16_t words[7],
                            uint8_t out[RFU_SLOT_LEN]) {
    /* Idle (word0==0) emits 14 zeros and does NOT advance the tag. */
    if ((words[0] & 0xFFFF) == 0) { rfu_idle_slot(out); return; }
    uint16_t w[7];
    memcpy(w, words, sizeof(w));
    w[0] = (uint16_t)((w[0] | (sb->tag << 5)) & 0xFFFF);
    rfu_words_to_slot(w, out);
    sb->tag = (uint8_t)((sb->tag + 1) & 7);
}

void rfu_parse_llsf_child(const uint8_t slot[2], rfu_child_llsf *out) {
    uint16_t f = (uint16_t)(slot[0] | (slot[1] << 8));
    out->state = (f >> 10) & 0xF;
    out->ack   = (f >> 9) & 1;
    out->n     = (f >> 7) & 3;
    out->phase = (f >> 5) & 3;
    out->size  = f & 0x1F;
}

void rfu_child_ni_llsf(int state, int n, int phase, int ack, int size, uint8_t out2[2]) {
    uint16_t f = (uint16_t)(((state & 0xF) << 10) | ((ack & 1) << 9) |
                            ((n & 3) << 7) | ((phase & 3) << 5) | (size & 0x1F));
    out2[0] = (uint8_t)f;
    out2[1] = (uint8_t)(f >> 8);
}

const char *rfu_op_name(uint16_t op) {
    switch (op) {
        case 0x0000: return "IDLE";
        case 0x2F00: return "SEND_PACKET";
        case 0x5F00: return "READY_CLOSE_LINK";
        case 0x6600: return "READY_EXIT_STANDBY";
        case 0x7700: return "SEND_PLAYER_IDS";
        case 0x7800: return "SEND_PLAYER_IDS_NEW";
        case 0x8800: return "SEND_BLOCK_INIT";
        case 0x8900: return "SEND_BLOCK";
        case 0xA100: return "SEND_BLOCK_REQ";
        case 0xBE00: return "SEND_HELD_KEYS";
        case 0xED00: return "DISCONNECT";
        case 0xEE00: return "DISCONNECT_PARENT";
        default:     return "UNKNOWN";
    }
}

void rfu_parse_slot(const uint8_t slot[RFU_SLOT_LEN], rfu_parsed *out) {
    memset(out, 0, sizeof(*out));
    int all_zero = 1;
    for (int i = 0; i < RFU_SLOT_LEN; i++) if (slot[i]) { all_zero = 0; break; }
    if (all_zero) { out->valid = 0; out->name = "IDLE"; return; }

    out->valid = 1;
    out->word0 = get_u16le(slot);
    out->op    = (uint16_t)(out->word0 & RFU_RFUCMD_MASK);
    out->name  = rfu_op_name(out->op);
    out->low   = (uint8_t)(out->word0 & 0xFF);

    switch (out->op) {
        case RFU_SEND_BLOCK_INIT:
            out->count     = get_u16le(slot + 2);
            out->owner_raw = slot[4];
            out->peer      = (uint8_t)(slot[4] & 0x7F);
            break;
        case RFU_SEND_BLOCK:
            out->index = (uint8_t)(out->word0 & RFU_FRAG_INDEX_MASK);
            memcpy(out->frag, slot + 2, 12);
            break;
        case RFU_SEND_BLOCK_REQ:
            out->reqtype = get_u16le(slot + 2);
            break;
        case RFU_SEND_HELD_KEYS:
            out->keycode = get_u16le(slot + 2);
            break;
        case RFU_READY_EXIT_STANDBY:
        case RFU_READY_CLOSE_LINK:
            out->count = get_u16le(slot + 2);
            break;
        default: break;
    }
}
