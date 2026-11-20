/* SPDX-License-Identifier: AGPL-3.0-only
 * Derived from pokeldn (https://github.com/Decryptu/pokeldn), Copyright (c) the pokeldn authors.
 * Changes Copyright (c) 2026 Kizuna. See NOTICE.
 */
/* linkplayer.h — the Gen III link records a trade leader sends at room entry, portable C port of
 * pokeldn pokeldn/frlg/link/linkplayer.py (struct LinkPlayer, the 60-byte LinkPlayerBlock, the
 * 100-byte trainer-card block) plus the few pieces of trade.py / save/mon.py they need (linkcmd and
 * mail blocks, offered-slot resolution, the party-mon species read).
 *
 * Pure/deterministic, no allocation. Verified byte-for-byte against the Python in
 * test_trade_host.c (vectors from pokeldn HEAD b5ede2b).
 *
 * Names: ASCII only (up to 31 bytes are kept; the encoder uses the first 7 mappable characters). The encoder maps the ASCII subset of pokeldn's international charmap
 * (letters, digits, space ! ? . - , / & + = ; % ( ) < > :); anything else, including the accented
 * Latin range the Python also maps, is dropped, as the Python drops unmapped characters. */
#ifndef LINKPLAYER_H
#define LINKPLAYER_H
#include <stdint.h>
#include <stddef.h>

#define LP_VERSION_FIRE_RED   0x4004   /* gGameVersion(4) + 0x4000 */
#define LP_VERSION_LEAF_GREEN 0x4005
#define LP_LANGUAGE_ENGLISH   2
#define LP_FIELD2             0x8000
#define LP_HOST_NAME_PAD      0xFF     /* robust host-role terminator fill */
#define LP_RECORD_SIZE        28
#define LP_BLOCK_SIZE         60
#define LP_NAME_MAX           7        /* characters; the field is 8 bytes with the terminator */
#define LP_TRAINER_CARD_SIZE  100      /* BLOCK_REQ_SIZE_100 buffer */
#define LP_PARTY_SIZE         6
#define LP_EASYCHAT_WORDS     4

/* struct LinkPlayer [include/link.h:158-171]. name is NUL-terminated ASCII (<= 7 chars used). */
typedef struct {
    char     name[32];            /* ASCII, NUL-terminated; only the first 7 encodable chars are used */
    uint32_t trainer_id;          /* full 32-bit OT id */
    uint16_t version;
    uint16_t field2;
    uint8_t  progress_flags, never_read, progress_flags_copy, gender;
    uint32_t link_type;
    uint16_t player_id;
    uint16_t language;
} lp_link_player;

/* Python LinkPlayer() defaults: name "EMU", trainer 0x47ED8822, LeafGreen, field2 0x8000, English. */
void   lp_default(lp_link_player *lp);

/* charmap.encode(name, width, pad): drop unmapped chars, truncate to width-1, 0xFF, pad. Returns width. */
size_t lp_encode_name(const char *name, int width, uint8_t pad, uint8_t *out);
/* charmap.decode, stop at 0xFF; unmapped bytes become '.'. */
void   lp_decode_name(const uint8_t *b, size_t n, char *out, size_t out_cap);

void   lp_pack(const lp_link_player *lp, uint8_t name_pad, uint8_t out[LP_RECORD_SIZE]);
void   lp_unpack(const uint8_t b[LP_RECORD_SIZE], lp_link_player *out);
/* GAMEFREAK + record + GAMEFREAK. */
void   lp_build_block(const lp_link_player *lp, uint8_t name_pad, uint8_t out[LP_BLOCK_SIZE]);
/* b: >= 60 bytes. Fills *out; returns 1 if both magics' first 14 bytes read "GameFreak inc.". */
int    lp_parse_block(const uint8_t *b, size_t len, lp_link_player *out);

/* build_trainer_card(link_player, wonder_card_id, mon_species, name_pad, quote). quote: four easy-chat
 * words (NULL = LP_DEFAULT_QUOTE). mon_species: n <= 6 species numbers. */
extern const uint16_t LP_DEFAULT_QUOTE[LP_EASYCHAT_WORDS];   /* hello friend trade pokemon */
void   lp_build_trainer_card(const lp_link_player *lp, uint16_t wonder_card_id,
                             const uint16_t *mon_species, int n, uint8_t name_pad,
                             const uint16_t *quote, uint8_t out[LP_TRAINER_CARD_SIZE]);

/* ---- trade.py helpers ---- */
#define TRADE_READY_TO_TRADE        0xAABB
#define TRADE_SET_MONS_TO_TRADE     0xDDDD
#define TRADE_INIT_BLOCK            0xBBBB
#define TRADE_START_TRADE           0xCCDD
#define TRADE_READY_FINISH_TRADE    0xABCD
#define TRADE_CONFIRM_FINISH_TRADE  0xDCBA
#define TRADE_REQUEST_CANCEL        0xEEAA
#define TRADE_READY_CANCEL_TRADE    0xBBCC
#define TRADE_PLAYER_CANCEL_TRADE   0xDDEE
#define TRADE_BOTH_CANCEL_TRADE     0xEEBB
#define TRADE_PARTNER_CANCEL_TRADE  0xEECC
const char *trade_linkcmd_name(uint16_t cmd);          /* NULL if unknown */

#define TRADE_COUNT_LINKCMD       2
#define TRADE_COUNT_PARTY         17
#define TRADE_COUNT_MAIL          19
#define TRADE_COUNT_RIBBON        4
#define TRADE_COUNT_TRAINER_CARD  9
#define TRADE_BLOCK_REQ_SIZE_NONE 0
#define TRADE_BLOCK_REQ_SIZE_200  1
#define TRADE_BLOCK_REQ_SIZE_100  2
#define TRADE_BLOCK_REQ_SIZE_220  3
#define TRADE_BLOCK_REQ_SIZE_40   4
#define TRADE_DEFAULT_ANIM_FRAMES 1935
#define TRADE_LINKCMD_BLOCK_SIZE  20
#define TRADE_MAIL_BLOCK_SIZE     220

void   trade_linkcmd_block(uint16_t cmd, uint16_t cursor, uint8_t out[TRADE_LINKCMD_BLOCK_SIZE]);
void   trade_empty_mail_block(uint8_t out[TRADE_MAIL_BLOCK_SIZE]);

/* resolve_offered_slots. offered == NULL: seeded from trade_slot (full-party default when it would run past
 * party_size). Returns 0, or -1 on the cases where the Python raises ValueError (wrong count, negative,
 * duplicate). trades 1..6. out receives `trades` slots. */
int    trade_resolve_offered_slots(const int *offered, int n_offered, int trade_slot, int trades,
                                   int party_size, int out[6]);

/* ---- save/mon.py ---- */
#define MON_PARTY_SIZE 100
/* decode_mon()["species"]: the growth substructure's u16 after the PID^OTID decrypt and the PID%24
 * shuffle; no checksum gate (the Python does not gate on it either). */
uint16_t mon_species(const uint8_t raw[MON_PARTY_SIZE]);

#endif
