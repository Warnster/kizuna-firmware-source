/* SPDX-License-Identifier: AGPL-3.0-only
 * Derived from pokeldn (https://github.com/Decryptu/pokeldn), Copyright (c) the pokeldn authors.
 * Changes Copyright (c) 2026 Kizuna. See NOTICE.
 */
#include "wonder_card.h"
#include <string.h>

/* FRLG text charmap: index = ASCII - 0x20 (0x20..0x7e). 0xFF = unmapped (also the pad/terminator).
 * Extracted from pokeldn frlg/text/charmap (space=0x00, 0-9=0xA1.., A-Z=0xBB.., a-z=0xD5..). */
static const uint8_t FRLG_MAP[95] = {
    0x00,0xab,0xff,0xff,0xff,0x5b,0x2d,0xff,0x5c,0x5d,0xff,0x2e,0xb8,0xae,0xad,0xba,
    0xa1,0xa2,0xa3,0xa4,0xa5,0xa6,0xa7,0xa8,0xa9,0xaa,0xf0,0x36,0x85,0x35,0x86,0xac,
    0xff,0xbb,0xbc,0xbd,0xbe,0xbf,0xc0,0xc1,0xc2,0xc3,0xc4,0xc5,0xc6,0xc7,0xc8,0xc9,
    0xca,0xcb,0xcc,0xcd,0xce,0xcf,0xd0,0xd1,0xd2,0xd3,0xd4,0xff,0xff,0xff,0xff,0xff,
    0xff,0xd5,0xd6,0xd7,0xd8,0xd9,0xda,0xdb,0xdc,0xdd,0xde,0xdf,0xe0,0xe1,0xe2,0xe3,
    0xe4,0xe5,0xe6,0xe7,0xe8,0xe9,0xea,0xeb,0xec,0xed,0xee,0xff,0xff,0xff,0xff,
};

void frlg_encode_card_text(uint8_t out[WC_TEXT_LEN], const char *s) {
    memset(out, 0xFF, WC_TEXT_LEN);          /* pad/terminator */
    if (!s) return;
    for (int i = 0; i < WC_TEXT_LEN - 1 && s[i]; i++) {
        unsigned char c = (unsigned char)s[i];
        out[i] = (c >= 0x20 && c <= 0x7e) ? FRLG_MAP[c - 0x20] : 0xFF;
    }
}

static void put_u16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static void put_u32(uint8_t *p, uint32_t v) { for (int i = 0; i < 4; i++) p[i] = (uint8_t)(v >> (8*i)); }

int wonder_card_build(const wonder_card_params *p, uint8_t out[WONDER_CARD_SIZE]) {
    if (p->flag_id == 0) return -1;
    if (!(p->card_type < 3 && p->bg_type < 8 && p->send_type <= 2 && p->max_stamps <= 7)) return -1;

    uint32_t display_id = (p->id_number == 0) ? (p->flag_id % 100) : p->id_number;
    uint8_t bitfield = (uint8_t)((p->card_type & 0x3) | ((p->bg_type & 0xF) << 2) | ((p->send_type & 0x3) << 6));

    memset(out, 0, WONDER_CARD_SIZE);
    put_u16(out + 0, p->flag_id);
    put_u16(out + 2, p->icon_species);
    put_u32(out + 4, display_id);
    out[8] = bitfield;
    out[9] = p->max_stamps;
    frlg_encode_card_text(out + 10, p->title);
    frlg_encode_card_text(out + 50, p->subtitle);
    frlg_encode_card_text(out + 90,  p->body[0]);
    frlg_encode_card_text(out + 130, p->body[1]);
    frlg_encode_card_text(out + 170, p->body[2]);
    frlg_encode_card_text(out + 210, p->body[3]);
    frlg_encode_card_text(out + 250, p->footer1);
    frlg_encode_card_text(out + 290, p->footer2);
    out[330] = 0x00; out[331] = 0x00;        /* alignment pad */
    return 0;
}
