/* SPDX-License-Identifier: AGPL-3.0-only
 * Derived from pokeldn (https://github.com/Decryptu/pokeldn), Copyright (c) the pokeldn authors.
 * Changes Copyright (c) 2026 Kizuna. See NOTICE.
 */
/* wonder_card.h — build the 332-byte FRLG `struct WonderCard` on-chip (C port of pokeldn
 * frlg/gift/wonder_card.py build_wonder_card). Layout [pokefirered include/global.h:655];
 * must pass ValidateWonderCard [src/mystery_gift.c:191]. Text fields are FRLG-charmap encoded,
 * 40 bytes each, 0xFF-padded. This is the payload Sammy/the distributor hands over. */
#ifndef WONDER_CARD_H
#define WONDER_CARD_H
#include <stdint.h>
#include <stddef.h>

#define WONDER_CARD_SIZE 332
#define WC_TEXT_LEN 40

/* Encode ASCII text into a 40-byte FRLG text field (unmapped chars & padding -> 0xFF; max 39
 * usable chars). Apostrophes and a few symbols are unmapped in FRLG -> avoid them. */
void frlg_encode_card_text(uint8_t out[WC_TEXT_LEN], const char *s);

typedef struct {
    uint16_t flag_id;       /* must be != 0 (ValidateWonderCard) */
    uint16_t icon_species;  /* card icon only */
    uint32_t id_number;     /* 0 -> card displays flag_id % 100 */
    uint8_t  card_type;     /* < 3 */
    uint8_t  bg_type;       /* < 8 */
    uint8_t  send_type;     /* 0..2 */
    uint8_t  max_stamps;    /* <= 7 */
    const char *title;
    const char *subtitle;
    const char *body[4];
    const char *footer1;
    const char *footer2;
} wonder_card_params;

/* Build the 332-byte card into out. Returns 0 on success, -1 if a field is out of range. */
int wonder_card_build(const wonder_card_params *p, uint8_t out[WONDER_CARD_SIZE]);

#endif
