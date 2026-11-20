/* SPDX-License-Identifier: AGPL-3.0-only
 * Derived from pokeldn (https://github.com/Decryptu/pokeldn), Copyright (c) the pokeldn authors.
 * Changes Copyright (c) 2026 Kizuna. See NOTICE.
 */
/* mg_server.h — FRLG Mystery Gift server (the distributor's side of a Wonder Card session),
 * portable C port of the card path of pokeldn frlg/gift/mg_server.py + mg_script.py
 * [decomp:src/mystery_gift_server.c, src/mystery_gift_scripts.c]. A table-driven interpreter
 * over the console's own server-script shape; it decides WHAT to send. Framing is mg_link.c,
 * pacing and the RFU link are the caller's. Pure/deterministic.
 *
 * Use: mg_server_init(); loop { a = mg_server_next(); SEND -> transmit a->data (a->size bytes on
 * the wire, zero-padded; 0 = full 1024) then mg_server_on_sent(); RECV -> feed the console's
 * message via mg_server_on_received(); DONE -> a->result }. */
#ifndef MG_SERVER_H
#define MG_SERVER_H
#include <stdint.h>
#include <stddef.h>
#include "mg_link.h"

#define MG_GAME_DATA_SIZE    0x64  /* agbcc pads MysteryGiftLinkGameData to 100 B, not 96 */
#define MG_CARD_SIZE         332   /* struct WonderCard */
#define MG_RAM_SCRIPT_MAX    995   /* RamScriptData.script; CLI_SAVE_RAM_SCRIPT clamps to it */
#define MG_GIFT_MAX_VARIANTS 4

/* One cartridge build of a gift. game_code "BPRE" = FireRed (EN), "BPGE" = LeafGreen (EN);
 * all zero = any console (pure event scripts that name no ROM address). */
typedef struct {
    char           game_code[4];
    uint8_t        require_revision; /* 0 = any ROM revision; else must equal software_version */
    uint8_t        software_version; /* RomHeaderSoftwareVersion, for scripts that embed ARM code */
    const uint8_t *card;            /* MG_CARD_SIZE bytes */
    const uint8_t *ram_script;      /* raw delivery-script bytecode */
    size_t         ram_script_len;  /* 1..MG_RAM_SCRIPT_MAX */
} mg_gift_variant;

typedef struct {
    mg_gift_variant variants[MG_GIFT_MAX_VARIANTS];
    int             nvariants;
} mg_gift;

/* Returns 0 if every variant is well formed (sizes, a non-zero card flagId, ...), else -1. */
int mg_gift_check(const mg_gift *g);

/* The fields of struct MysteryGiftLinkGameData the server uses [include/mystery_gift.h:22]. */
typedef struct {
    uint32_t magic;          /* 0x101 */
    uint16_t unk_04;
    uint32_t unk_08;
    uint16_t unk_0c;
    uint32_t version_code;   /* low nibble: 1 FireRed, 2 LeafGreen */
    uint16_t flag_id;        /* 0 = holds no Wonder Card */
    char     game_code[4];
    uint8_t  software_version;
    uint8_t  player_name[7]; /* FRLG charmap, 0xFF-terminated unless 7 characters long */
    uint32_t trainer_id;
} mg_game_data;

int  mg_game_data_parse(const uint8_t *p, size_t len, mg_game_data *out);  /* 0, or -1 if short */
int  mg_game_data_valid(const mg_game_data *gd);  /* MysteryGift_ValidateLinkGameData */

/* SVR_MSG_* results [decomp:include/mystery_gift_server.h]. */
enum {
    MG_SVR_NOTHING_SENT    = 0,
    MG_SVR_CARD_SENT       = 2,
    MG_SVR_HAS_CARD        = 5,  /* the console already holds this card's flagId */
    MG_SVR_CLIENT_CANCELED = 9,  /* the player kept the card they had */
    MG_SVR_CANT_SEND       = 10, /* invalid game data, or no variant for this cartridge */
};

typedef enum { MG_ACT_SEND, MG_ACT_RECV, MG_ACT_DONE, MG_ACT_ERROR } mg_act_kind;

typedef struct {
    mg_act_kind    kind;
    uint16_t       ident;
    const uint8_t *data;   /* SEND: payload */
    size_t         len;    /* SEND: payload bytes */
    size_t         size;   /* SEND: size on the wire, 0 = the full 1024-byte buffer */
    int            result; /* DONE: MG_SVR_* */
} mg_action;

struct mg_svr_cmd;

typedef struct {
    const mg_gift            *gift;
    const mg_gift_variant    *chosen;
    const struct mg_svr_cmd  *script;
    int                       pc;
    int                       param;
    mg_action                 action;
    int                       action_live;
    mg_game_data              game_data;
    int                       have_game_data;
    int                       no_variant;    /* refused: the gift has no build for this console */
    uint8_t                   recv[MG_LINK_BUFFER_SIZE];
    size_t                    recv_len;
    int                       messages_sent, messages_received;
} mg_server;

void             mg_server_init(mg_server *s, const mg_gift *g);
const mg_action *mg_server_next(mg_server *s);
void             mg_server_on_sent(mg_server *s);
/* Returns 0, or -1 if no receive is pending or ident is not the one expected (a fatal link
 * error on the console's side too; end the session). */
int              mg_server_on_received(mg_server *s, uint16_t ident, const uint8_t *p, size_t len);

#endif
