/* SPDX-License-Identifier: AGPL-3.0-only
 * Derived from pokeldn (https://github.com/Decryptu/pokeldn), Copyright (c) the pokeldn authors.
 * Changes Copyright (c) 2026 Kizuna. See NOTICE.
 */
/* linkplayer.c — portable C port of pokeldn linkplayer.py + the trade.py / mon.py helpers. See linkplayer.h. */
#include "linkplayer.h"
#include <string.h>

static const uint8_t GAMEFREAK_MAGIC[16] = { 'G','a','m','e','F','r','e','a','k',' ','i','n','c','.',0,0 };

const uint16_t LP_DEFAULT_QUOTE[LP_EASYCHAT_WORDS] = { 2063, 2611, 514, 526 };

/* ---- charmap: the ASCII subset of pokeldn's international table ---- */
static int enc_char(char c) {
    if (c >= 'A' && c <= 'Z') return 0xBB + (c - 'A');
    if (c >= 'a' && c <= 'z') return 0xD5 + (c - 'a');
    if (c >= '0' && c <= '9') return 0xA1 + (c - '0');
    switch (c) {
        case ' ': return 0x00;
        case '!': return 0xAB;
        case '?': return 0xAC;
        case '.': return 0xAD;
        case '-': return 0xAE;
        case ',': return 0xB8;
        case '/': return 0xBA;
        case '&': return 0x2D;
        case '+': return 0x2E;
        case '=': return 0x35;
        case ';': return 0x36;
        case '%': return 0x5B;
        case '(': return 0x5C;
        case ')': return 0x5D;
        case '<': return 0x85;
        case '>': return 0x86;
        case ':': return 0xF0;
        default:  return -1;
    }
}

static char dec_byte(uint8_t b) {
    if (b >= 0xA1 && b <= 0xAA) return (char)('0' + (b - 0xA1));
    if (b >= 0xBB && b <= 0xD4) return (char)('A' + (b - 0xBB));
    if (b >= 0xD5 && b <= 0xEE) return (char)('a' + (b - 0xD5));
    switch (b) {
        case 0x00: return ' ';
        case 0xAB: return '!';
        case 0xAC: return '?';
        case 0xAD: return '.';
        case 0xAE: return '-';
        case 0xB8: return ',';
        case 0xBA: return '/';
        case 0x2D: return '&';
        case 0x2E: return '+';
        case 0x35: return '=';
        case 0x36: return ';';
        case 0x5B: return '%';
        case 0x5C: return '(';
        case 0x5D: return ')';
        case 0x85: return '<';
        case 0x86: return '>';
        case 0xF0: return ':';
        default:   return '.';
    }
}

size_t lp_encode_name(const char *name, int width, uint8_t pad, uint8_t *out) {
    size_t n = 0;
    for (const char *p = name; *p; p++) {
        int b = enc_char(*p);
        if (b < 0) continue;
        if (n < (size_t)(width > 0 ? width - 1 : 0)) out[n] = (uint8_t)b;
        n++;
    }
    if (n > (size_t)(width - 1)) n = (size_t)(width - 1);
    out[n++] = 0xFF;
    while (n < (size_t)width) out[n++] = pad;
    return (size_t)width;
}

void lp_decode_name(const uint8_t *b, size_t n, char *out, size_t out_cap) {
    size_t o = 0;
    for (size_t i = 0; i < n && o + 1 < out_cap; i++) {
        if (b[i] == 0xFF) break;
        out[o++] = dec_byte(b[i]);
    }
    out[o] = 0;
}

/* ---- LinkPlayer ---- */
void lp_default(lp_link_player *lp) {
    memset(lp, 0, sizeof *lp);
    strcpy(lp->name, "EMU");
    lp->trainer_id = 0x47ED8822u;
    lp->version = LP_VERSION_LEAF_GREEN;
    lp->field2 = LP_FIELD2;
    lp->language = LP_LANGUAGE_ENGLISH;
}

static void put16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static void put32(uint8_t *p, uint32_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24); }
static uint16_t get16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static uint32_t get32(const uint8_t *p) { return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24); }

void lp_pack(const lp_link_player *lp, uint8_t name_pad, uint8_t out[LP_RECORD_SIZE]) {
    put16(out + 0, lp->version);
    put16(out + 2, lp->field2);
    put32(out + 4, lp->trainer_id);
    lp_encode_name(lp->name, 8, name_pad, out + 8);
    out[16] = lp->progress_flags; out[17] = lp->never_read;
    out[18] = lp->progress_flags_copy; out[19] = lp->gender;
    put32(out + 20, lp->link_type);
    put16(out + 24, lp->player_id);
    put16(out + 26, lp->language);
}

void lp_unpack(const uint8_t b[LP_RECORD_SIZE], lp_link_player *out) {
    memset(out, 0, sizeof *out);
    out->version = get16(b + 0);
    out->field2 = get16(b + 2);
    out->trainer_id = get32(b + 4);
    lp_decode_name(b + 8, 8, out->name, sizeof out->name);
    out->progress_flags = b[16]; out->never_read = b[17];
    out->progress_flags_copy = b[18]; out->gender = b[19];
    out->link_type = get32(b + 20);
    out->player_id = get16(b + 24);
    out->language = get16(b + 26);
}

void lp_build_block(const lp_link_player *lp, uint8_t name_pad, uint8_t out[LP_BLOCK_SIZE]) {
    memcpy(out, GAMEFREAK_MAGIC, 16);
    lp_pack(lp, name_pad, out + 16);
    memcpy(out + 44, GAMEFREAK_MAGIC, 16);
}

int lp_parse_block(const uint8_t *b, size_t len, lp_link_player *out) {
    if (len < LP_BLOCK_SIZE) { memset(out, 0, sizeof *out); return 0; }
    lp_unpack(b + 16, out);
    return memcmp(b, GAMEFREAK_MAGIC, 14) == 0 && memcmp(b + 44, GAMEFREAK_MAGIC, 14) == 0;
}

/* ---- trainer card [include/trainer_card.h]; offsets as in linkplayer.py ---- */
#define TC_OFF_GENDER      0x00
#define TC_OFF_HAS_POKEDEX 0x02
#define TC_OFF_TRAINER_ID  0x0E
#define TC_OFF_PLAYER_NAME 0x30
#define TC_OFF_EASY_CHAT   0x28
#define TC_OFF_VERSION     0x38
#define TC_OFF_MON_SPECIES 0x54
#define TC_OFF_WONDER_CARD 0x60

void lp_build_trainer_card(const lp_link_player *lp, uint16_t wonder_card_id,
                           const uint16_t *mon_species, int n, uint8_t name_pad,
                           const uint16_t *quote, uint8_t out[LP_TRAINER_CARD_SIZE]) {
    memset(out, 0, LP_TRAINER_CARD_SIZE);
    out[TC_OFF_GENDER] = lp->gender;
    out[TC_OFF_HAS_POKEDEX] = (lp->progress_flags & 0x0F) ? 1 : 0;
    put16(out + TC_OFF_TRAINER_ID, (uint16_t)(lp->trainer_id & 0xFFFF));
    lp_encode_name(lp->name, 8, name_pad, out + TC_OFF_PLAYER_NAME);
    const uint16_t *q = quote ? quote : LP_DEFAULT_QUOTE;
    for (int i = 0; i < LP_EASYCHAT_WORDS; i++) put16(out + TC_OFF_EASY_CHAT + i * 2, q[i]);
    out[TC_OFF_VERSION] = (uint8_t)(lp->version & 0xFF);
    if (mon_species)
        for (int i = 0; i < n && i < LP_PARTY_SIZE; i++) put16(out + TC_OFF_MON_SPECIES + i * 2, mon_species[i]);
    put16(out + TC_OFF_WONDER_CARD, wonder_card_id);
}

/* ---- trade.py ---- */
const char *trade_linkcmd_name(uint16_t cmd) {
    switch (cmd) {
        case TRADE_READY_TO_TRADE:       return "READY_TO_TRADE";
        case TRADE_SET_MONS_TO_TRADE:    return "SET_MONS_TO_TRADE";
        case TRADE_INIT_BLOCK:           return "INIT_BLOCK";
        case TRADE_START_TRADE:          return "START_TRADE";
        case TRADE_READY_FINISH_TRADE:   return "READY_FINISH_TRADE";
        case TRADE_CONFIRM_FINISH_TRADE: return "CONFIRM_FINISH_TRADE";
        case TRADE_REQUEST_CANCEL:       return "REQUEST_CANCEL";
        case TRADE_READY_CANCEL_TRADE:   return "READY_CANCEL_TRADE";
        case TRADE_PLAYER_CANCEL_TRADE:  return "PLAYER_CANCEL_TRADE";
        case TRADE_BOTH_CANCEL_TRADE:    return "BOTH_CANCEL_TRADE";
        case TRADE_PARTNER_CANCEL_TRADE: return "PARTNER_CANCEL_TRADE";
        default: return NULL;
    }
}

void trade_linkcmd_block(uint16_t cmd, uint16_t cursor, uint8_t out[TRADE_LINKCMD_BLOCK_SIZE]) {
    memset(out, 0, TRADE_LINKCMD_BLOCK_SIZE);
    put16(out, cmd);
    put16(out + 2, cursor);
}

void trade_empty_mail_block(uint8_t out[TRADE_MAIL_BLOCK_SIZE]) {
    uint8_t rec[34];
    memset(rec, 0xFF, 18);                 /* words[9] */
    memset(rec + 18, 0xFF, 8);             /* playerName[8] */
    memset(rec + 26, 0x00, 4);             /* trainerId[4] */
    rec[30] = 0x01; rec[31] = 0x00;        /* SPECIES_BULBASAUR */
    rec[32] = 0x00; rec[33] = 0x00;        /* ITEM_NONE */
    memset(out, 0, TRADE_MAIL_BLOCK_SIZE);
    for (int i = 0; i < 6; i++) memcpy(out + i * 34, rec, 34);
}

int trade_resolve_offered_slots(const int *offered, int n_offered, int trade_slot, int trades,
                                int party_size, int out[6]) {
    if (trades < 1 || trades > 6) return -1;
    if (offered) {
        if (n_offered != trades) return -1;
        for (int i = 0; i < n_offered; i++) {
            if (offered[i] < 0) return -1;
            for (int j = 0; j < i; j++) if (offered[j] == offered[i]) return -1;
            out[i] = offered[i];
        }
        return 0;
    }
    int overflow = 0;
    for (int i = 0; i < trades; i++) if (party_size > 0 && trade_slot + i >= party_size) overflow = 1;
    for (int i = 0; i < trades; i++) out[i] = overflow ? i : trade_slot + i;
    return 0;
}

/* ---- mon.py: species of a party mon ---- */
static const char *const SUBSTRUCT_ORDER[24] = {
    "GAEM", "GAME", "GEAM", "GEMA", "GMAE", "GMEA",
    "AGEM", "AGME", "AEGM", "AEMG", "AMGE", "AMEG",
    "EGAM", "EGMA", "EAGM", "EAMG", "EMGA", "EMAG",
    "MGAE", "MGEA", "MAGE", "MAEG", "MEGA", "MEAG",
};

uint16_t mon_species(const uint8_t raw[MON_PARTY_SIZE]) {
    uint32_t pid = get32(raw), otid = get32(raw + 4), key = pid ^ otid;
    const char *order = SUBSTRUCT_ORDER[pid % 24];
    int g = (int)(strchr(order, 'G') - order);
    return (uint16_t)(get16(raw + 32 + g * 12) ^ (uint16_t)(key & 0xFFFF));
}
