/* gift_pack.h — the Event Gifts payload as the adapter stores it: one KZG1 blob per gift, built
 * on the phone/PC from a pokeldn .pokegift, a pokemon-gen3-mysterygift-tool WonderCard.bin +
 * Script.bin pair, or a GB-Link payload. Parsing never copies:
 * the mg_gift points into the blob, so the blob must outlive it. Nothing in a pack executes on
 * the adapter; card scripts (and any ARM code inside them) run on the console.
 *
 * Layout (little-endian):
 *   0  "KZG1"   4 u8 version (1)   5 u8 nvariants (1..4)   6 u16 total length incl. CRC
 *   8  char name[48]  (UTF-8, NUL-padded; display only)
 *   56 variants, each: char game_code[4] (all zero = any cartridge), u8 require_revision,
 *      u8 software_version, u16 script_len (1..995), u8 card[332], u8 script[script_len]
 *   end u32 CRC-32 (IEEE) of every byte before it
 * Integrity only. */
#ifndef GIFT_PACK_H
#define GIFT_PACK_H
#include <stdint.h>
#include <stddef.h>
#include "mg_server.h"

#define GIFT_PACK_MAGIC        "KZG1"
#define GIFT_PACK_VERSION      1
#define GIFT_PACK_HEADER_SIZE  56
#define GIFT_PACK_NAME_LEN     48
#define GIFT_PACK_VARIANT_HDR  8
#define GIFT_PACK_MAX_SIZE     (GIFT_PACK_HEADER_SIZE + MG_GIFT_MAX_VARIANTS * \
                                (GIFT_PACK_VARIANT_HDR + MG_CARD_SIZE + MG_RAM_SCRIPT_MAX) + 4)

typedef enum {
    GIFT_PACK_OK = 0,
    GIFT_PACK_ERR_SIZE = -1,      /* shorter than its header, or total length disagrees */
    GIFT_PACK_ERR_MAGIC = -2,
    GIFT_PACK_ERR_VERSION = -3,
    GIFT_PACK_ERR_CRC = -4,
    GIFT_PACK_ERR_LAYOUT = -5,    /* variant count or lengths do not tile the blob */
    GIFT_PACK_ERR_CARD = -6,      /* the console's ValidateWonderCard would refuse it */
    GIFT_PACK_ERR_VARIANTS = -7,  /* duplicate cartridge, or "any" mixed with others */
} gift_pack_result;

typedef struct {
    mg_gift gift;
    char    name[GIFT_PACK_NAME_LEN + 1];
} gift_pack;

/* ValidateWonderCard [mystery_gift.c:191]: flagId != 0, type < 3, sendType <= 2, bgType < 8,
 * maxStamps <= 7. Returns 1 if the console would accept the card. */
int wonder_card_valid(const uint8_t card[MG_CARD_SIZE]);

gift_pack_result gift_pack_parse(const uint8_t *blob, size_t len, gift_pack *out);
const char      *gift_pack_strerror(gift_pack_result r);

#endif
