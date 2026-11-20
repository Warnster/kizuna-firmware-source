/* gift_pack.c — KZG1 gift pack parser. See gift_pack.h. */
#include "gift_pack.h"
#include "crypto.h"
#include <string.h>

static uint16_t rd16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static uint32_t rd32(const uint8_t *p) { return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24; }

int wonder_card_valid(const uint8_t card[MG_CARD_SIZE]) {
    uint8_t bits = card[8];   /* type:2 | bgType:4 | sendType:2 */
    return rd16(card) != 0 && (bits & 3) < 3 && ((bits >> 2) & 0xF) < 8 && (bits >> 6) <= 2 && card[9] <= 7;
}

gift_pack_result gift_pack_parse(const uint8_t *blob, size_t len, gift_pack *out) {
    memset(out, 0, sizeof(*out));
    if (len < GIFT_PACK_HEADER_SIZE + 4 || len > GIFT_PACK_MAX_SIZE) return GIFT_PACK_ERR_SIZE;
    if (memcmp(blob, GIFT_PACK_MAGIC, 4)) return GIFT_PACK_ERR_MAGIC;
    if (blob[4] != GIFT_PACK_VERSION) return GIFT_PACK_ERR_VERSION;
    if (rd16(blob + 6) != len) return GIFT_PACK_ERR_SIZE;
    if (crc32_ieee(blob, len - 4) != rd32(blob + len - 4)) return GIFT_PACK_ERR_CRC;

    int n = blob[5];
    if (n < 1 || n > MG_GIFT_MAX_VARIANTS) return GIFT_PACK_ERR_LAYOUT;
    memcpy(out->name, blob + 8, GIFT_PACK_NAME_LEN);
    out->name[GIFT_PACK_NAME_LEN] = 0;

    size_t off = GIFT_PACK_HEADER_SIZE, end = len - 4;
    for (int i = 0; i < n; i++) {
        if (off + GIFT_PACK_VARIANT_HDR + MG_CARD_SIZE > end) return GIFT_PACK_ERR_LAYOUT;
        const uint8_t *v = blob + off;
        size_t script_len = rd16(v + 6);
        if (script_len < 1 || script_len > MG_RAM_SCRIPT_MAX) return GIFT_PACK_ERR_LAYOUT;
        if (off + GIFT_PACK_VARIANT_HDR + MG_CARD_SIZE + script_len > end) return GIFT_PACK_ERR_LAYOUT;
        mg_gift_variant *gv = &out->gift.variants[i];
        memcpy(gv->game_code, v, 4);
        gv->require_revision = v[4] != 0;
        gv->software_version = v[5];
        gv->card = v + GIFT_PACK_VARIANT_HDR;
        gv->ram_script = gv->card + MG_CARD_SIZE;
        gv->ram_script_len = script_len;
        if (!wonder_card_valid(gv->card)) return GIFT_PACK_ERR_CARD;
        off += GIFT_PACK_VARIANT_HDR + MG_CARD_SIZE + script_len;
    }
    if (off != end) return GIFT_PACK_ERR_LAYOUT;
    out->gift.nvariants = n;
    if (mg_gift_check(&out->gift) != 0) return GIFT_PACK_ERR_VARIANTS;
    return GIFT_PACK_OK;
}

const char *gift_pack_strerror(gift_pack_result r) {
    switch (r) {
    case GIFT_PACK_OK:           return "ok";
    case GIFT_PACK_ERR_SIZE:     return "wrong size";
    case GIFT_PACK_ERR_MAGIC:    return "not a KZG1 gift pack";
    case GIFT_PACK_ERR_VERSION:  return "unsupported gift pack version";
    case GIFT_PACK_ERR_CRC:      return "damaged (CRC mismatch)";
    case GIFT_PACK_ERR_LAYOUT:   return "malformed variant table";
    case GIFT_PACK_ERR_CARD:     return "Wonder Card the game would refuse";
    case GIFT_PACK_ERR_VARIANTS: return "duplicate or conflicting cartridge variants";
    }
    return "unknown error";
}
