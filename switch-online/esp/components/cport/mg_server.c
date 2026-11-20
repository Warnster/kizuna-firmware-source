/* SPDX-License-Identifier: AGPL-3.0-only
 * Derived from pokeldn (https://github.com/Decryptu/pokeldn), Copyright (c) the pokeldn authors.
 * Changes Copyright (c) 2026 Kizuna. See NOTICE.
 */
/* mg_server.c — portable C port of the card path of pokeldn frlg/gift/mg_server.py. See mg_server.h. */
#include "mg_server.h"
#include <string.h>

/* Client-script instructions [decomp:include/mystery_gift_client.h:18]. */
enum {
    CLI_RETURN = 1, CLI_RECV = 2, CLI_SEND_LOADED = 3, CLI_COPY_RECV = 4, CLI_LOAD_GAME_DATA = 8,
    CLI_SAVE_CARD = 10, CLI_COPY_MSG = 12, CLI_ASK_TOSS = 13, CLI_LOAD_TOSS_RESPONSE = 14,
    CLI_SAVE_RAM_SCRIPT = 17, CLI_SEND_READY_END = 20,
};
/* CLI_MSG_* the console's CLI_RETURN reports [decomp:include/mystery_gift_client.h:45]. */
enum { CLI_MSG_CARD_RECEIVED = 2, CLI_MSG_HAD_CARD = 5, CLI_MSG_CANT_ACCEPT = 10,
       CLI_MSG_BUFFER_FAILURE = 14 };

/* struct MysteryGiftClientCmd: u32 instr, u32 parameter. Every value here fits in a byte. */
#define CMD(i, p) (uint8_t)(i), 0, 0, 0, (uint8_t)(p), 0, 0, 0

/* sClientScript_SendGameData [mystery_gift_scripts.c:20] */
static const uint8_t CS_SEND_GAME_DATA[] = {
    CMD(CLI_LOAD_GAME_DATA, 0), CMD(CLI_SEND_LOADED, 0),
    CMD(CLI_RECV, MG_LINKID_CLIENT_SCRIPT), CMD(CLI_COPY_RECV, 0),
};
/* sClientScript_SaveCard [mystery_gift_scripts.c:42] */
static const uint8_t CS_SAVE_CARD[] = {
    CMD(CLI_RECV, MG_LINKID_CARD), CMD(CLI_SAVE_CARD, 0),
    CMD(CLI_RECV, MG_LINKID_RAM_SCRIPT), CMD(CLI_SAVE_RAM_SCRIPT, 0),
    CMD(CLI_SEND_READY_END, 0), CMD(CLI_RETURN, CLI_MSG_CARD_RECEIVED),
};
/* sClientScript_HadCard [mystery_gift_scripts.c:82] */
static const uint8_t CS_HAD_CARD[] = { CMD(CLI_SEND_READY_END, 0), CMD(CLI_RETURN, CLI_MSG_HAD_CARD) };
/* sClientScript_AskToss [mystery_gift_scripts.c:69]; the answer comes back as MG_LINKID_RESPONSE. */
static const uint8_t CS_ASK_TOSS[] = {
    CMD(CLI_ASK_TOSS, 0), CMD(CLI_LOAD_TOSS_RESPONSE, 0), CMD(CLI_SEND_LOADED, 0),
    CMD(CLI_RECV, MG_LINKID_CLIENT_SCRIPT), CMD(CLI_COPY_RECV, 0),
};
/* sClientScript_DynamicError [union_room_message.c:562]: shows our 64-byte message first. */
static const uint8_t CS_DYNAMIC_ERROR[] = {
    CMD(CLI_RECV, MG_LINKID_DYNAMIC_MSG), CMD(CLI_COPY_MSG, 0),
    CMD(CLI_SEND_READY_END, 0), CMD(CLI_RETURN, CLI_MSG_BUFFER_FAILURE),
};
/* sClientScript_CantAccept [mystery_gift_scripts.c:27] */
static const uint8_t CS_CANT_ACCEPT[] = { CMD(CLI_SEND_READY_END, 0), CMD(CLI_RETURN, CLI_MSG_CANT_ACCEPT) };
/* sText_CanceledReadingCard [union_room_message.c:560], "Canceled reading the Card.", EOS-terminated. */
static const uint8_t TEXT_CANCELED_READING_CARD[] = {
    0xbd, 0xd5, 0xe2, 0xd7, 0xd9, 0xe0, 0xd9, 0xd8, 0x00, 0xe6, 0xd9, 0xd5, 0xd8, 0xdd,
    0xe2, 0xdb, 0x00, 0xe8, 0xdc, 0xd9, 0x00, 0xbd, 0xd5, 0xe6, 0xd8, 0xad, 0xff,
};

/* Server-script opcodes [decomp:include/mystery_gift_server.h]; the subset the card path uses. */
enum {
    SVR_RETURN, SVR_SEND, SVR_RECV, SVR_GOTO, SVR_GOTO_IF_EQ, SVR_COPY_GAME_DATA,
    SVR_CHECK_GAME_DATA, SVR_CHECK_EXISTING_CARD, SVR_READ_RESPONSE, SVR_LOAD_CARD,
    SVR_LOAD_RAM_SCRIPT, SVR_LOAD_CLIENT_SCRIPT, SVR_LOAD_MSG,
};
/* MysteryGift_CompareCardFlags [mystery_gift.c:388] */
enum { HAS_NO_CARD = 0, HAS_SAME_CARD = 1, HAS_DIFF_CARD = 2 };

typedef struct mg_svr_cmd {
    uint8_t     op;
    int32_t     arg;  /* RECV: ident; GOTO_IF_EQ: value; RETURN: result */
    const void *ptr;  /* GOTO*: script; LOAD_CLIENT_SCRIPT / LOAD_MSG: bytes */
    uint16_t    len;  /* LOAD_*: byte count */
} svr_cmd;

#define BYTES(a) (a), (uint16_t)sizeof(a)

/* sServerScript_CantSend [mystery_gift_scripts.c:98] */
static const svr_cmd S_CANT_SEND[] = {
    { SVR_LOAD_CLIENT_SCRIPT, 0, BYTES(CS_CANT_ACCEPT) }, { SVR_SEND, 0, 0, 0 },
    { SVR_RECV, MG_LINKID_READY_END, 0, 0 }, { SVR_RETURN, MG_SVR_CANT_SEND, 0, 0 },
};
/* sServerScript_HasCard [mystery_gift_scripts.c:160] */
static const svr_cmd S_HAS_CARD[] = {
    { SVR_LOAD_CLIENT_SCRIPT, 0, BYTES(CS_HAD_CARD) }, { SVR_SEND, 0, 0, 0 },
    { SVR_RECV, MG_LINKID_READY_END, 0, 0 }, { SVR_RETURN, MG_SVR_HAS_CARD, 0, 0 },
};
/* gServerScript_ClientCanceledCard [union_room_message.c:569] */
static const svr_cmd S_CLIENT_CANCELED[] = {
    { SVR_LOAD_CLIENT_SCRIPT, 0, BYTES(CS_DYNAMIC_ERROR) }, { SVR_SEND, 0, 0, 0 },
    { SVR_LOAD_MSG, 0, BYTES(TEXT_CANCELED_READING_CARD) }, { SVR_SEND, 0, 0, 0 },
    { SVR_RECV, MG_LINKID_READY_END, 0, 0 }, { SVR_RETURN, MG_SVR_CLIENT_CANCELED, 0, 0 },
};
/* sServerScript_SendCard [mystery_gift_scripts.c:140] */
static const svr_cmd S_SEND_CARD[] = {
    { SVR_LOAD_CLIENT_SCRIPT, 0, BYTES(CS_SAVE_CARD) }, { SVR_SEND, 0, 0, 0 },
    { SVR_LOAD_CARD, 0, 0, 0 }, { SVR_SEND, 0, 0, 0 },
    { SVR_LOAD_RAM_SCRIPT, 0, 0, 0 }, { SVR_SEND, 0, 0, 0 },
    { SVR_RECV, MG_LINKID_READY_END, 0, 0 }, { SVR_RETURN, MG_SVR_CARD_SENT, 0, 0 },
};
/* sServerScript_TossPrompt [mystery_gift_scripts.c:150]: the response is TRUE when the player
 * keeps the old card, so FALSE goes on to the gift. */
static const svr_cmd S_TOSS_PROMPT[] = {
    { SVR_LOAD_CLIENT_SCRIPT, 0, BYTES(CS_ASK_TOSS) }, { SVR_SEND, 0, 0, 0 },
    { SVR_RECV, MG_LINKID_RESPONSE, 0, 0 }, { SVR_READ_RESPONSE, 0, 0, 0 },
    { SVR_GOTO_IF_EQ, 0, S_SEND_CARD, 0 }, { SVR_GOTO, 0, S_CLIENT_CANCELED, 0 },
};
/* gMysteryGiftServerScript_SendWonderCard [mystery_gift_scripts.c:185] minus SVR_COPY_SAVED_CARD
 * (the card comes from the gift). SVR_CHECK_GAME_DATA also fails when no variant fits. */
static const svr_cmd S_SEND_WONDER_CARD[] = {
    { SVR_LOAD_CLIENT_SCRIPT, 0, BYTES(CS_SEND_GAME_DATA) }, { SVR_SEND, 0, 0, 0 },
    { SVR_RECV, MG_LINKID_GAME_DATA, 0, 0 }, { SVR_COPY_GAME_DATA, 0, 0, 0 },
    { SVR_CHECK_GAME_DATA, 0, 0, 0 }, { SVR_GOTO_IF_EQ, 0, S_CANT_SEND, 0 },
    { SVR_CHECK_EXISTING_CARD, 0, 0, 0 },
    { SVR_GOTO_IF_EQ, HAS_DIFF_CARD, S_TOSS_PROMPT, 0 },
    { SVR_GOTO_IF_EQ, HAS_NO_CARD, S_SEND_CARD, 0 },
    { SVR_GOTO, 0, S_HAS_CARD, 0 },
};

static uint16_t rd16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static uint32_t rd32(const uint8_t *p) { return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24; }

static int code_is_any(const char c[4]) { return !c[0] && !c[1] && !c[2] && !c[3]; }

int mg_gift_check(const mg_gift *g) {
    if (!g || g->nvariants < 1 || g->nvariants > MG_GIFT_MAX_VARIANTS) return -1;
    for (int i = 0; i < g->nvariants; i++) {
        const mg_gift_variant *v = &g->variants[i];
        if (!v->card || !v->ram_script) return -1;
        if (v->ram_script_len < 1 || v->ram_script_len > MG_RAM_SCRIPT_MAX) return -1;
        if (rd16(v->card) == 0) return -1;            /* ValidateWonderCard: flagId != 0 */
        if (code_is_any(v->game_code) && g->nvariants != 1) return -1;
        for (int j = 0; j < i; j++)
            if (!memcmp(g->variants[j].game_code, v->game_code, 4)) return -1;
    }
    return 0;
}

int mg_game_data_parse(const uint8_t *p, size_t len, mg_game_data *out) {
    if (len < MG_GAME_DATA_SIZE) return -1;
    memset(out, 0, sizeof(*out));
    out->magic = rd32(p + 0x00);
    out->unk_04 = rd16(p + 0x04);
    out->unk_08 = rd32(p + 0x08);
    out->unk_0c = rd16(p + 0x0C);
    out->version_code = rd32(p + 0x10);
    out->flag_id = rd16(p + 0x14);
    memcpy(out->player_name, p + 0x45, 7);
    out->trainer_id = rd32(p + 0x4C);
    memcpy(out->game_code, p + 0x5C, 4);
    out->software_version = p[0x60];
    return 0;
}

/* MysteryGift_ValidateLinkGameData [mystery_gift.c:373] */
int mg_game_data_valid(const mg_game_data *gd) {
    return gd->magic == 0x101 && (gd->unk_04 & 1) && (gd->unk_08 & 1) && (gd->unk_0c & 1)
        && (gd->version_code & 0x0F);
}

static const mg_gift_variant *select_variant(const mg_gift *g, const mg_game_data *gd) {
    for (int i = 0; i < g->nvariants; i++) {
        const mg_gift_variant *v = &g->variants[i];
        if (!code_is_any(v->game_code) && memcmp(v->game_code, gd->game_code, 4)) continue;
        if (v->require_revision && v->software_version != gd->software_version) continue;
        return v;
    }
    return NULL;
}

void mg_server_init(mg_server *s, const mg_gift *g) {
    memset(s, 0, sizeof(*s));
    s->gift = g;
    s->script = S_SEND_WONDER_CARD;
}

static void load(mg_server *s, uint16_t ident, const uint8_t *data, size_t len, size_t size) {
    s->action = (mg_action){ .kind = MG_ACT_SEND, .ident = ident, .data = data, .len = len, .size = size };
}

static void fail(mg_server *s) {
    s->action = (mg_action){ .kind = MG_ACT_ERROR };
    s->action_live = 1;
}

/* One server command; sets action_live when the caller has something to do. */
static void step(mg_server *s) {
    const svr_cmd *c = &s->script[s->pc++];
    switch (c->op) {
    case SVR_RETURN:
        s->action = (mg_action){ .kind = MG_ACT_DONE, .result = c->arg };
        s->action_live = 1;
        break;
    case SVR_SEND:
        if (s->action.kind != MG_ACT_SEND || !s->action.data) { fail(s); break; }
        s->action_live = 1;
        break;
    case SVR_RECV:
        s->action = (mg_action){ .kind = MG_ACT_RECV, .ident = (uint16_t)c->arg };
        s->action_live = 1;
        break;
    case SVR_GOTO:
        s->script = c->ptr; s->pc = 0;
        break;
    case SVR_GOTO_IF_EQ:
        if (s->param == c->arg) { s->script = c->ptr; s->pc = 0; }
        break;
    case SVR_COPY_GAME_DATA:
        s->have_game_data = mg_game_data_parse(s->recv, s->recv_len, &s->game_data) == 0;
        if (s->have_game_data) {
            s->chosen = select_variant(s->gift, &s->game_data);
            s->no_variant = s->chosen == NULL;
        }
        break;
    case SVR_CHECK_GAME_DATA:
        s->param = s->have_game_data && mg_game_data_valid(&s->game_data) && s->chosen != NULL;
        break;
    case SVR_CHECK_EXISTING_CARD: {
        uint16_t ours = rd16(s->chosen->card);
        s->param = s->game_data.flag_id == 0 ? HAS_NO_CARD
                 : s->game_data.flag_id == ours ? HAS_SAME_CARD : HAS_DIFF_CARD;
        break;
    }
    case SVR_READ_RESPONSE:
        /* The raw u32 [mystery_gift_server.c:193]: TRUE = the player kept the old card. */
        s->param = s->recv_len >= 4 ? (int)rd32(s->recv) : -1;
        break;
    case SVR_LOAD_CARD:
        load(s, MG_LINKID_CARD, s->chosen->card, MG_CARD_SIZE, MG_CARD_SIZE);
        break;
    case SVR_LOAD_RAM_SCRIPT:
        /* SVR_COPY_SAVED_RAM_SCRIPT never sets ramScriptSize: always the full 1024-byte buffer. */
        load(s, MG_LINKID_RAM_SCRIPT, s->chosen->ram_script, s->chosen->ram_script_len, 0);
        break;
    case SVR_LOAD_CLIENT_SCRIPT:
        load(s, MG_LINKID_CLIENT_SCRIPT, c->ptr, c->len, c->len);
        break;
    case SVR_LOAD_MSG:
        load(s, MG_LINKID_DYNAMIC_MSG, c->ptr, c->len, c->len);
        break;
    default:
        fail(s);
    }
}

const mg_action *mg_server_next(mg_server *s) {
    while (!s->action_live) step(s);
    return &s->action;
}

void mg_server_on_sent(mg_server *s) {
    if (!s->action_live || s->action.kind != MG_ACT_SEND) return;
    s->messages_sent++;
    s->action_live = 0;
    s->action.data = NULL;
}

int mg_server_on_received(mg_server *s, uint16_t ident, const uint8_t *p, size_t len) {
    if (!s->action_live || s->action.kind != MG_ACT_RECV || ident != s->action.ident) return -1;
    if (len > sizeof(s->recv)) len = sizeof(s->recv);
    memcpy(s->recv, p, len);
    s->recv_len = len;
    s->messages_received++;
    s->action_live = 0;
    return 0;
}
