/* SPDX-License-Identifier: AGPL-3.0-only
 * Derived from pokeldn (https://github.com/Decryptu/pokeldn), Copyright (c) the pokeldn authors.
 * Derived from LDN (https://github.com/kinnay/LDN), Copyright (c) kinnay and the LDN authors.
 * Derived from GB-Link-Switch-LDN (https://github.com/GB-Link/GB-Link-Switch-LDN), Copyright (c) the GB-Link-Switch-LDN authors.
 * Changes Copyright (c) 2026 Kizuna. See NOTICE.
 */
/* ldn_host.h — the pure (no radio) half of HOSTING an LDN network that a Switch running FireRed/
 * LeafGreen (NSO) joins as a Mystery Gift FRIEND client. Our own C, written against kinnay/LDN as
 * vendored in pokeldn (vendor/LDN/ldn/__init__.py: AdvertisementInfoEncoderV2, AuthenticationFrame,
 * ChallengeRequest/Response, APNetwork) and pokeldn frlg/link/{beacon,host_beacon}.py, cross-checked
 * against GB-Link firmware/common/ldn_host.c + pia_bridge.c. Inverse of ldn_advert.c / ldn_auth.c.
 *
 *  - advertisement: AdvertisementInfo plaintext (BE) -> ldn_advert_encrypt -> 0x7F action body
 *  - FRLG app data: 92-byte Pia system header + base85 of the 24-byte room record
 *  - authentication (EtherType 0x88b7 body, starts 00 22 aa 01 02 00): parse + verify the joiner's
 *    request (GCM, key = auth key(client_random); HMAC-SHA256(CHALLENGE_KEY) over the challenge),
 *    build our response (388-byte payload, Switch 2's 16 extra bytes echoed)
 *  - member table: node index i = slot i, host = 0, IP 169.254.subnet.(i+1)
 *
 * Keys only ever arrive as `const ldn_keys *`. Not reentrant (two small static scratch buffers):
 * call from the one task that also owns the aes_backend. */
#ifndef LDN_HOST_H
#define LDN_HOST_H
#include <stdint.h>
#include <stddef.h>
#include "ldn_advert.h"

/* FRLG's network (pokeldn HostTransport; matches the real Switch advert in test_ldn_advert.c). */
#define LDN_HOST_FRLG_COMM_ID     0x01006fa0233f8000ULL
#define LDN_HOST_FRLG_SCENE_ID    22287
#define LDN_HOST_FRLG_APP_VERSION 88
#define LDN_HOST_FRLG_MAX         6      /* max participants, host included */

#define LDN_HOST_MAX_MEMBERS      8
#define LDN_HOST_NAME_MAX         32
#define LDN_HOST_APP_DATA_MAX     384
#define LDN_HOST_ADVERT_PT_MAX    (40 + 48 * LDN_HOST_MAX_MEMBERS + 2 + LDN_HOST_APP_DATA_MAX) /* 810 */
#define LDN_HOST_ADVERT_BODY_MAX  (68 + LDN_HOST_ADVERT_PT_MAX)    /* action body from 0x7F */
#define LDN_HOST_ADVERT_FRAME_MAX (24 + LDN_HOST_ADVERT_BODY_MAX)  /* + 802.11 header */

#define LDN_HOST_AUTH_REQ_PAYLOAD 868    /* AuthenticationRequest, version >= 3 */
#define LDN_HOST_AUTH_REQ_LEN     (6 + 72 + 16 + 868)   /* 962; 976 with the Ethernet header */
#define LDN_HOST_AUTH_RES_LEN     (6 + 72 + 16 + 388)   /* 482, accepted */
#define LDN_HOST_AUTH_ERR_LEN     (6 + 72 + 16 + 132)   /* 226, any non-zero status */

typedef struct {
    uint8_t  used;                       /* slot holds an authenticated member */
    uint8_t  ip[4];
    uint8_t  mac[6];
    uint8_t  platform;                   /* 0 Switch, 1 Switch 2 */
    char     name[LDN_HOST_NAME_MAX + 1];
} ldn_host_member;

typedef struct {
    /* NetworkId */
    uint64_t local_comm_id;
    uint16_t scene_id;
    uint8_t  ssid[16];                   /* session id: NetworkId.ssid, WLAN SSID (hex), Pia key */
    /* AdvertisementInfo */
    uint8_t  server_random[16];
    uint64_t challenge;                  /* BE in the advert, LE as the request's token */
    uint8_t  security;                   /* 1 = production */
    uint8_t  policy;                     /* 0 = accept all */
    uint16_t app_version;
    uint8_t  band;                       /* 2 = 2.4 GHz */
    uint8_t  channel;
    uint8_t  max_participants;
    uint8_t  version;                    /* advert + auth frame version, 4 */
    uint8_t  subnet;                     /* N of 169.254.N.x */
    ldn_host_member members[LDN_HOST_MAX_MEMBERS];   /* [0] = us */
    uint8_t  app_data[LDN_HOST_APP_DATA_MAX];
    uint16_t app_data_len;
    uint32_t nonce;                      /* advert nonce, BE on air; bumped on every content change */
} ldn_host_net;

/* The room's identity, chosen once at start (random ssid / server_random / challenge / nonce). */
typedef struct {
    uint8_t  ssid[16];
    uint8_t  server_random[16];
    uint64_t challenge;
    uint32_t nonce;
    uint8_t  channel;                    /* 1, 6 or 11 */
    uint8_t  subnet;                     /* 1..127 (kinnay) */
    uint8_t  mac[6];                     /* our AP MAC = BSSID = member 0 */
    uint8_t  platform;                   /* ours, 0 */
    const char *name;                    /* our nickname, <= 32 bytes */
} ldn_host_identity;

/* FRLG defaults (comm id, scene 22287, app version 88, security 1, policy 0, band 2, max 6,
 * version 4) + the identity; member 0 = us at 169.254.subnet.1, no app data. 0, or -1 on bad args. */
int  ldn_host_net_init(ldn_host_net *n, const ldn_host_identity *id);

/* Member table. add: an authenticated station takes the lowest free index 1..max-1 (an already
 * present MAC keeps its index, nothing changes). Returns the index, or -1 when the room is full.
 * remove: frees the station's slot (never the host). Returns the freed index or -1.
 * add/remove/set_app_data bump the nonce when the advert content changes; touch bumps it after a
 * direct field edit (policy etc.). count includes the host. */
int  ldn_host_member_add(ldn_host_net *n, const uint8_t mac[6], const char *name, uint8_t platform);
int  ldn_host_member_remove(ldn_host_net *n, const uint8_t mac[6]);
int  ldn_host_member_find(const ldn_host_net *n, const uint8_t mac[6]);
int  ldn_host_member_count(const ldn_host_net *n);
int  ldn_host_set_app_data(ldn_host_net *n, const uint8_t *data, size_t len); /* 1 changed, 0 same, -1 */
void ldn_host_touch(ldn_host_net *n);

/* NetworkId: u64 comm id, pad2, u16 scene, pad4, ssid[16]. le = 0: advert form (and the advertise
 * key input); le = 1: authentication-header form. */
void ldn_host_network_id(const ldn_host_net *n, int le, uint8_t out[32]);

/* AdvertisementInfo (EncoderV2, BE): random16 challenge8 security policy app_version pad8
 * u16(band<<10|ch) max count {ip4 mac6 index platform name32 pad4}*count u16 len app_data.
 * Returns the length, or -1 if cap is short. */
long ldn_host_advert_plaintext(const ldn_host_net *n, uint8_t *out, size_t cap);
/* Full action body (7F 00 22 AA 04 00 01 01 ... tag, ciphertext): the bytes after the 802.11 header. */
long ldn_host_advert_body(const ldn_keys *k, const ldn_host_net *n, uint8_t *out, size_t cap);
/* 24-byte action header (DA ff.., SA = BSSID = members[0].mac, A3 ff.., as a Switch sends it) +
 * body: the whole frame for esp_wifi_80211_tx(WIFI_IF_AP, out, len, true). */
long ldn_host_advert_frame(const ldn_keys *k, const ldn_host_net *n, uint8_t *out, size_t cap);

/* WLAN SSID of the hidden AP: the session id as 32 lowercase hex chars (kinnay ssid.hex()). */
void ldn_host_ssid_text(const uint8_t ssid[16], char out[33]);
/* CCMP pairwise + group key: data key(server_random || FRLG 64-byte password), as ldn_join uses. */
int  ldn_host_data_key(const ldn_keys *k, const ldn_host_net *n, uint8_t out[16]);

/* ---- authentication ---- */
enum {
    LDN_HOST_AUTH_NOT_AUTH        = -1,  /* not an LDN authentication frame: ignore it */
    LDN_HOST_AUTH_OK              = 0,
    LDN_HOST_AUTH_DENIED          = 1,   /* policy / room full: the glue's call, never returned */
    LDN_HOST_AUTH_MALFORMED       = 2,   /* format/size/GCM/network id/server random/is_response */
    LDN_HOST_AUTH_INVALID_VERSION = 4,
    LDN_HOST_AUTH_CHALLENGE_FAIL  = 6,   /* payload not 868, bad HMAC, or wrong challenge token */
};

typedef struct {
    uint8_t  version;                    /* header[0] */
    uint8_t  client_random[16];          /* from the cleartext header, even when the parse fails */
    char     name[LDN_HOST_NAME_MAX + 1];
    uint16_t app_version;
    uint8_t  platform;                   /* 0 Switch, 1 Switch 2 */
    uint64_t nonce, device_id;           /* ChallengeRequest, echoed in the response */
    uint8_t  unk[16];                    /* Switch 2's extra 16 bytes, echoed */
    uint8_t  payload[LDN_HOST_AUTH_REQ_PAYLOAD];     /* decrypted request (valid past GCM) */
} ldn_host_auth_req;

/* body = the 0x88b7 frame after its 14-byte Ethernet header. Returns an LDN_HOST_AUTH_* code;
 * checks run in kinnay APNetwork order. req is ~1 KB: keep it static on the ESP. */
int  ldn_host_auth_parse(const ldn_keys *k, const ldn_host_net *n, const uint8_t *body, size_t len,
                         ldn_host_auth_req *req);
/* Our AuthenticationFrame answer to req: status 0 -> 388-byte payload {our platform, pad 0x83,
 * ChallengeResponse(flags 2, nonce, device, host_device_id, unk)}; status != 0 -> 132-byte payload.
 * GCM under the auth key of req->client_random. Returns the length (482 / 226) or -1. */
long ldn_host_auth_response(const ldn_keys *k, const ldn_host_net *n, const ldn_host_auth_req *req,
                            int status, uint64_t host_device_id, uint8_t *out, size_t cap);

/* ---- FRLG application data (pokeldn host_beacon / GB-Link pia_bridge build_app_data) ---- */
#define LDN_FRLG_PIA_HEADER   92
#define LDN_FRLG_RECORD       24
#define LDN_FRLG_APP_DATA_LEN (LDN_FRLG_PIA_HEADER + LDN_FRLG_RECORD / 4 * 5)   /* 122 */
enum { LDN_FRLG_ACTIVITY_WONDER_CARD = 21, LDN_FRLG_ACTIVITY_WONDER_NEWS = 22,
       /* Trade / Union Room, per ni.h NI_ACTIVITY_TRADE (0x04) and pokeldn's frlg_trade_host.py
        * ACTIVITY_TRADE: the SAME activity byte, same rec[16:18] field, as the RFU/NI layer uses --
        * confirmed by the matching values, not assumed. AP+STA LDN-host feasibility test;
        * ldn_frlg_room_gift()'s has_card/started/tail semantics are UNVERIFIED for this activity --
        * they describe a Gift room, not a Trade room, and have never been tested on hardware. */
       LDN_FRLG_ACTIVITY_TRADE = 4 };
enum { LDN_FRLG_VERSION_FIRE_RED = 4, LDN_FRLG_VERSION_LEAF_GREEN = 5, LDN_FRLG_LANGUAGE_ENGLISH = 2 };

typedef struct {
    char     nickname[65];               /* Pia session name, UTF-8, <= 64 bytes */
    uint8_t  players;                    /* Pia 0x16: 1 waiting, 2 once the console is in */
    uint16_t trainer_id;                 /* rec[0:2] */
    uint8_t  name[8];                    /* rec[2:10], GBA charset, 0xFF-padded */
    uint16_t session_id;                 /* rec[10:12], RFU parent id */
    uint8_t  partner[4];                 /* rec[12:16] */
    uint8_t  activity;                   /* word rec[16:18] bits 0-6 */
    uint8_t  national;                   /* bit 7 (GB-Link: can link nationally) */
    uint8_t  version;                    /* bits 8-10 */
    uint8_t  language;                   /* bits 11-13 */
    uint8_t  has_card;                   /* bit 14 */
    uint8_t  started;                    /* bit 15 */
    uint8_t  tail[6];                    /* rec[18:24] raw: dex/clear/trade type, gender+level, species */
} ldn_frlg_room;

/* A Mystery Gift host room as pokeldn builds it: FireRed, English, bit 7 set, players 1, not
 * started, nickname and record name = group (e.g. "KIZUNA"). */
void ldn_frlg_room_gift(ldn_frlg_room *r, uint8_t activity, const char *group,
                        uint16_t trainer_id, uint16_t session_id);
/* The Direct Corner TRADE room exactly as pokeldn host_beacon.build_trade_app_data builds it: the captured
 * native FireRed trade beacon (activity 4, national bit, tail/partner zero) with the trainer name and TID,
 * the RFU parent id (host_session_id: the same two bytes pia_host_cfg.host_session_id carries, stored raw
 * at rec[10:12], so session_id = hsid[0] | hsid[1] << 8), and the game version/language of the profile.
 * The Pia session name is the trainer name. This is the waiting ("inactive") room: players 1, not started.
 * name: the trainer name (<= 7 GBA characters; unencodable characters are dropped as pokeldn does).
 * version: LDN_FRLG_VERSION_*, language: LDN_FRLG_LANGUAGE_* (low 3 bits of the LinkPlayer values). */
void ldn_frlg_room_trade(ldn_frlg_room *r, const char *name, uint16_t trainer_id,
                         const uint8_t host_session_id[2], uint8_t version, uint8_t language);
/* The same room once the console's LinkPlayer is established (activate_trade_app_data): players 2 and the
 * started bit. Re-encode with ldn_frlg_app_data_build and hand to ldn_ap_set_app_data. */
void ldn_frlg_room_trade_activate(ldn_frlg_room *r);
/* ASCII -> GBA charset, <= 7 chars, 0xFF-padded to 8 (A-Z a-z 0-9 space ! ? . -; else '?'). */
void ldn_frlg_gba_name(const char *ascii, uint8_t out[8]);
long ldn_frlg_app_data_build(const ldn_frlg_room *r, uint8_t *out, size_t cap);  /* 122 or -1 */
int  ldn_frlg_app_data_parse(const uint8_t *d, size_t len, ldn_frlg_room *r);    /* 0 or -1 */

#endif
