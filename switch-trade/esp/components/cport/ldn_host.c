/* SPDX-License-Identifier: AGPL-3.0-only
 * Derived from pokeldn (https://github.com/Decryptu/pokeldn), Copyright (c) the pokeldn authors.
 * Derived from LDN (https://github.com/kinnay/LDN), Copyright (c) kinnay and the LDN authors.
 * Derived from GB-Link-Switch-LDN (https://github.com/GB-Link/GB-Link-Switch-LDN), Copyright (c) the GB-Link-Switch-LDN authors.
 * Changes Copyright (c) 2026 Kizuna. See NOTICE.
 */
/* ldn_host.c — see ldn_host.h. Host side of kinnay/LDN APNetwork (protocol 3) + pokeldn's FRLG
 * room application data; GB-Link ldn_host.c was the cross-check for the wire layouts. */
#include "ldn_host.h"
#include "charmap.h"
#include "aes_backend.h"
#include <string.h>

/* kinnay CHALLENGE_KEY (non-dev): a public LDN constant, also in ldn_auth.c. Not console keys. */
static const uint8_t CHALLENGE_KEY[32] = {
    0xf8,0x4b,0x48,0x7f,0xb3,0x72,0x51,0xc2,0x63,0xbf,0x11,0x60,0x90,0x36,0x58,0x92,
    0x66,0xaf,0x70,0xca,0x79,0xb4,0x4c,0x93,0xc7,0x37,0x0c,0x57,0x69,0xc0,0xf6,0x02 };

/* FRLG network password (public constant, as ldn_join.c / pokeldn GBA_APP_PASSPHRASE). */
static const uint8_t FRLG_PASSWORD[64] = {
    0xfc,0xb6,0xf6,0xad,0xb9,0xdf,0xea,0x66,0xac,0xa9,0xc3,0x26,0x14,0x9d,0x2b,0x3b,
    0x08,0xa7,0x81,0x89,0x5c,0xbf,0x78,0xf7,0x20,0xd7,0x8b,0x85,0xa5,0x75,0x84,0xa9,
    0x96,0x65,0xd2,0x37,0x79,0x7b,0x2a,0x41,0xdd,0xef,0x14,0x06,0x3e,0xc2,0x8d,0x25,
    0x91,0x43,0xaf,0x78,0x32,0xfb,0x3c,0xbc,0xf2,0x75,0x9c,0xbf,0xbd,0xc8,0x1d,0x8c };

static const uint8_t AUTH_MAGIC[6] = { 0x00,0x22,0xaa, 0x01,0x02, 0x00 };  /* OUI, 0x102, pad */

static void wbe16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)(v >> 8); p[1] = (uint8_t)v; }
static void wbe32(uint8_t *p, uint32_t v) { for (int i = 0; i < 4; i++) p[i] = (uint8_t)(v >> (24 - 8 * i)); }
static void wbe64(uint8_t *p, uint64_t v) { for (int i = 0; i < 8; i++) p[i] = (uint8_t)(v >> (56 - 8 * i)); }
static void wle16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static void wle32(uint8_t *p, uint32_t v) { for (int i = 0; i < 4; i++) p[i] = (uint8_t)(v >> (8 * i)); }
static void wle64(uint8_t *p, uint64_t v) { for (int i = 0; i < 8; i++) p[i] = (uint8_t)(v >> (8 * i)); }
static uint16_t rbe16(const uint8_t *p) { return (uint16_t)(p[0] << 8 | p[1]); }
static uint32_t rbe32(const uint8_t *p) { return (uint32_t)p[0] << 24 | (uint32_t)p[1] << 16 | (uint32_t)p[2] << 8 | p[3]; }
static uint16_t rle16(const uint8_t *p) { return (uint16_t)(p[0] | p[1] << 8); }
static uint32_t rle32(const uint8_t *p) { return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24; }
static uint64_t rle64(const uint8_t *p) { uint64_t v = 0; for (int i = 7; i >= 0; i--) v = v << 8 | p[i]; return v; }

static size_t bounded_len(const char *s, size_t max) { size_t n = 0; while (s && n < max && s[n]) n++; return n; }
static void copy_name(char *dst, const char *src, size_t max) {
    size_t n = bounded_len(src, max); if (n) memcpy(dst, src, n); dst[n] = 0;
}

/* Constant-time compare for MACs/tags. */
static int ct_equal(const uint8_t *a, const uint8_t *b, size_t n) {
    uint8_t d = 0; for (size_t i = 0; i < n; i++) d |= a[i] ^ b[i]; return d == 0;
}

/* HMAC-SHA256 (RFC 2104) over sha256_digest, key <= 64 B, msg <= 720 B (the challenge body). */
static int hmac_sha256(const uint8_t *key, size_t keylen, const uint8_t *msg, size_t len, uint8_t out[32]) {
    static uint8_t buf[64 + 720];
    uint8_t pad[64], inner[32], outer[64 + 32];
    if (keylen > 64 || len > 720) return -1;
    memset(pad, 0, 64); memcpy(pad, key, keylen);
    for (int i = 0; i < 64; i++) buf[i] = pad[i] ^ 0x36;
    memcpy(buf + 64, msg, len);
    if (sha256_digest(buf, 64 + len, inner)) return -1;
    for (int i = 0; i < 64; i++) outer[i] = pad[i] ^ 0x5c;
    memcpy(outer + 64, inner, 32);
    return sha256_digest(outer, sizeof outer, out) ? -1 : 0;
}

/* ---------------- network + member table ---------------- */

static void set_ip(uint8_t ip[4], uint8_t subnet, int index) {
    ip[0] = 169; ip[1] = 254; ip[2] = subnet; ip[3] = (uint8_t)(index + 1);
}

int ldn_host_net_init(ldn_host_net *n, const ldn_host_identity *id) {
    if (!n || !id || id->channel == 0 || id->subnet == 0) return -1;
    memset(n, 0, sizeof *n);
    n->local_comm_id = LDN_HOST_FRLG_COMM_ID;
    n->scene_id = LDN_HOST_FRLG_SCENE_ID;
    memcpy(n->ssid, id->ssid, 16);
    memcpy(n->server_random, id->server_random, 16);
    n->challenge = id->challenge;
    n->security = 1;
    n->policy = 0;
    n->app_version = LDN_HOST_FRLG_APP_VERSION;
    n->band = 2;
    n->channel = id->channel;
    n->max_participants = LDN_HOST_FRLG_MAX;
    n->version = 4;
    n->subnet = id->subnet;
    n->nonce = id->nonce;
    ldn_host_member *h = &n->members[0];
    h->used = 1;
    set_ip(h->ip, id->subnet, 0);
    memcpy(h->mac, id->mac, 6);
    h->platform = id->platform;
    copy_name(h->name, id->name, LDN_HOST_NAME_MAX);
    return 0;
}

void ldn_host_touch(ldn_host_net *n) { n->nonce++; }

int ldn_host_member_find(const ldn_host_net *n, const uint8_t mac[6]) {
    for (int i = 0; i < LDN_HOST_MAX_MEMBERS; i++)
        if (n->members[i].used && memcmp(n->members[i].mac, mac, 6) == 0) return i;
    return -1;
}

int ldn_host_member_count(const ldn_host_net *n) {
    int c = 0;
    for (int i = 0; i < LDN_HOST_MAX_MEMBERS; i++) c += n->members[i].used ? 1 : 0;
    return c;
}

int ldn_host_member_add(ldn_host_net *n, const uint8_t mac[6], const char *name, uint8_t platform) {
    int i = ldn_host_member_find(n, mac);
    if (i >= 0) return i;
    int max = n->max_participants < LDN_HOST_MAX_MEMBERS ? n->max_participants : LDN_HOST_MAX_MEMBERS;
    for (i = 1; i < max; i++) {
        ldn_host_member *m = &n->members[i];
        if (m->used) continue;
        memset(m, 0, sizeof *m);
        m->used = 1;
        set_ip(m->ip, n->subnet, i);
        memcpy(m->mac, mac, 6);
        m->platform = platform;
        copy_name(m->name, name, LDN_HOST_NAME_MAX);
        ldn_host_touch(n);
        return i;
    }
    return -1;
}

int ldn_host_member_remove(ldn_host_net *n, const uint8_t mac[6]) {
    int i = ldn_host_member_find(n, mac);
    if (i <= 0) return -1;                       /* absent, or the host itself */
    memset(&n->members[i], 0, sizeof n->members[i]);
    ldn_host_touch(n);
    return i;
}

int ldn_host_set_app_data(ldn_host_net *n, const uint8_t *data, size_t len) {
    if (len > LDN_HOST_APP_DATA_MAX || (len && !data)) return -1;
    if (len == n->app_data_len && (len == 0 || memcmp(n->app_data, data, len) == 0)) return 0;
    if (len) memcpy(n->app_data, data, len);
    n->app_data_len = (uint16_t)len;
    ldn_host_touch(n);
    return 1;
}

void ldn_host_network_id(const ldn_host_net *n, int le, uint8_t out[32]) {
    memset(out, 0, 32);
    if (le) { wle64(out, n->local_comm_id); wle16(out + 10, n->scene_id); }
    else    { wbe64(out, n->local_comm_id); wbe16(out + 10, n->scene_id); }
    memcpy(out + 16, n->ssid, 16);
}

/* ---------------- advertisement ---------------- */

long ldn_host_advert_plaintext(const ldn_host_net *n, uint8_t *out, size_t cap) {
    int count = ldn_host_member_count(n);
    size_t len = 40 + 48 * (size_t)count + 2 + n->app_data_len;
    if (n->app_data_len > LDN_HOST_APP_DATA_MAX || len > cap) return -1;
    memset(out, 0, len);
    memcpy(out, n->server_random, 16);
    wbe64(out + 16, n->challenge);
    out[24] = n->security;
    out[25] = n->policy;
    wbe16(out + 26, n->app_version);
    /* [28..36) pad */
    wbe16(out + 36, (uint16_t)(n->band << 10 | n->channel));
    out[38] = n->max_participants;
    out[39] = (uint8_t)count;
    uint8_t *p = out + 40;
    for (int i = 0; i < LDN_HOST_MAX_MEMBERS; i++) {          /* index order, connected only */
        const ldn_host_member *m = &n->members[i];
        if (!m->used) continue;
        memcpy(p, m->ip, 4);
        memcpy(p + 4, m->mac, 6);
        p[10] = (uint8_t)i;
        p[11] = m->platform;
        memcpy(p + 12, m->name, bounded_len(m->name, LDN_HOST_NAME_MAX));  /* + pad4 */
        p += 48;
    }
    wbe16(p, n->app_data_len);
    memcpy(p + 2, n->app_data, n->app_data_len);
    return (long)len;
}

long ldn_host_advert_body(const ldn_keys *k, const ldn_host_net *n, uint8_t *out, size_t cap) {
    static uint8_t pt[LDN_HOST_ADVERT_PT_MAX];
    long len = ldn_host_advert_plaintext(n, pt, sizeof pt);
    if (len < 0 || cap < (size_t)(68 + len)) return -1;
    uint8_t nid[32], nonce4[4];
    ldn_host_network_id(n, 0, nid);
    wbe32(nonce4, n->nonce);
    return ldn_advert_encrypt(k, nid, n->version, nonce4, pt, (size_t)len, out);
}

long ldn_host_advert_frame(const ldn_keys *k, const ldn_host_net *n, uint8_t *out, size_t cap) {
    if (cap < 24) return -1;
    memset(out, 0, 24);
    out[0] = 0xd0;                               /* management / action */
    memset(out + 4, 0xff, 6);                    /* DA broadcast */
    memcpy(out + 10, n->members[0].mac, 6);      /* SA = BSSID */
    memset(out + 16, 0xff, 6);                   /* A3 broadcast (kinnay ActionFrame) */
    long body = ldn_host_advert_body(k, n, out + 24, cap - 24);
    return body < 0 ? -1 : 24 + body;
}

void ldn_host_ssid_text(const uint8_t ssid[16], char out[33]) {
    static const char hex[] = "0123456789abcdef";
    for (int i = 0; i < 16; i++) { out[2 * i] = hex[ssid[i] >> 4]; out[2 * i + 1] = hex[ssid[i] & 15]; }
    out[32] = 0;
}

int ldn_host_data_key(const ldn_keys *k, const ldn_host_net *n, uint8_t out[16]) {
    return ldn_derive_data_key(k, n->server_random, 16, FRLG_PASSWORD, sizeof FRLG_PASSWORD, out);
}

/* ---------------- authentication ---------------- */

/* Response AuthenticationFrame header (72): version, size_lo, status, is_response 1, size_hi,
 * format 1 (GCM), pad2, NetworkId LE, server_random, client_random. GCM nonce = header[0:12],
 * AAD = the whole header. */
static void response_header(const ldn_host_net *n, uint8_t status, size_t size,
                            const uint8_t client_random[16], uint8_t h[72]) {
    memset(h, 0, 72);
    h[0] = n->version;
    h[1] = (uint8_t)size;
    h[2] = status;
    h[3] = 1;
    h[4] = (uint8_t)(size >> 8);
    h[5] = 1;
    ldn_host_network_id(n, 1, h + 8);
    memcpy(h + 40, n->server_random, 16);
    memcpy(h + 56, client_random, 16);
}

int ldn_host_auth_parse(const ldn_keys *k, const ldn_host_net *n, const uint8_t *body, size_t len,
                        ldn_host_auth_req *req) {
    memset(req, 0, sizeof *req);
    if (len < 6 + 72 || memcmp(body, AUTH_MAGIC, 5) != 0) return LDN_HOST_AUTH_NOT_AUTH;
    const uint8_t *h = body + 6;
    req->version = h[0];
    memcpy(req->client_random, h + 56, 16);

    /* AuthenticationFrame.decode: format, exact size, GCM. */
    size_t size = (size_t)(h[1] | h[4] << 8);
    if (h[5] != 1 || len != 6 + 72 + 16 + size || size > LDN_HOST_AUTH_REQ_PAYLOAD)
        return LDN_HOST_AUTH_MALFORMED;
    uint8_t key[16];
    if (ldn_derive_authentication_key(k, req->client_random, 16, key)) return LDN_HOST_AUTH_MALFORMED;
    int bad = aes128_gcm_decrypt(key, h, 12, h, 72, body + 94, size, body + 78, 16, req->payload);
    memset(key, 0, sizeof key);
    if (bad) { memset(req->payload, 0, sizeof req->payload); return LDN_HOST_AUTH_MALFORMED; }

    /* APNetwork._check_authentication_request */
    if (h[0] < 2 || h[0] > 4) return LDN_HOST_AUTH_INVALID_VERSION;
    uint8_t nid[32];
    ldn_host_network_id(n, 1, nid);
    if (h[2] != 0 || memcmp(h + 8, nid, 32) != 0 || memcmp(h + 40, n->server_random, 16) != 0 ||
        h[3] != 0 || size < 64)
        return LDN_HOST_AUTH_MALFORMED;

    /* AuthenticationRequest: name32 u16be app_version u8 platform pad29 pad0x24 challenge[768]. */
    const uint8_t *p = req->payload;
    memcpy(req->name, p, 32); req->name[32] = 0;
    req->app_version = rbe16(p + 32);
    req->platform = p[34];

    /* ChallengeRequest: u32 0, hmac32, pad12, body720 {.. token8 nonce8 device8 unk16 ..} (LE). */
    if (size != LDN_HOST_AUTH_REQ_PAYLOAD) return LDN_HOST_AUTH_CHALLENGE_FAIL;
    const uint8_t *cb = p + 100 + 48;
    uint8_t mac[32];
    if (hmac_sha256(CHALLENGE_KEY, 32, cb, 720, mac) || !ct_equal(mac, p + 104, 32))
        return LDN_HOST_AUTH_CHALLENGE_FAIL;
    if (rle64(cb + 8) != n->challenge) return LDN_HOST_AUTH_CHALLENGE_FAIL;
    req->nonce = rle64(cb + 16);
    req->device_id = rle64(cb + 24);
    memcpy(req->unk, cb + 32, 16);
    return LDN_HOST_AUTH_OK;
}

/* ChallengeResponse (256): u32 0, hmac32, pad12, body208 {u8 0 u8 0 pad2 u32 flags u64 nonce
 * u64 device u64 host_device unk16 unk_host16 pad0x90}, all LE. */
static int challenge_response(const ldn_host_auth_req *req, uint64_t host_device, uint8_t out[256]) {
    memset(out, 0, 256);
    uint8_t *b = out + 48;
    wle32(b + 4, 2);                             /* flags (kinnay APNetwork, GB-Link) */
    wle64(b + 8, req->nonce);
    wle64(b + 16, req->device_id);
    wle64(b + 24, host_device);
    memcpy(b + 32, req->unk, 16);
    return hmac_sha256(CHALLENGE_KEY, 32, b, 208, out + 4);
}

long ldn_host_auth_response(const ldn_keys *k, const ldn_host_net *n, const ldn_host_auth_req *req,
                            int status, uint64_t host_device_id, uint8_t *out, size_t cap) {
    uint8_t pt[388];
    size_t size = status == LDN_HOST_AUTH_OK ? 388 : 132;
    if (status < 0 || status > 255 || cap < 6 + 72 + 16 + size) return -1;
    memset(pt, 0, sizeof pt);
    pt[0] = n->members[0].platform;              /* AuthenticationResponse: platform, pad 0x83 */
    if (size == 388 && challenge_response(req, host_device_id, pt + 132)) return -1;

    uint8_t *h = out + 6, key[16];
    memcpy(out, AUTH_MAGIC, 6);
    response_header(n, (uint8_t)status, size, req->client_random, h);
    if (ldn_derive_authentication_key(k, req->client_random, 16, key)) return -1;
    int bad = aes128_gcm_encrypt(key, h, 12, h, 72, pt, size, out + 94, out + 78);
    memset(key, 0, sizeof key);
    return bad ? -1 : (long)(94 + size);
}

/* ---------------- FRLG application data ---------------- */

static uint8_t b85_char(uint32_t digit) {         /* 0x23.., skipping the backslash */
    uint8_t c = (uint8_t)(0x23 + digit);
    return c >= 0x5c ? (uint8_t)(c + 1) : c;
}
static int b85_digit(uint8_t c) {
    if (c < 0x23 || c == 0x5c || c > 0x78) return -1;
    return c < 0x5c ? c - 0x23 : c - 0x24;
}

void ldn_frlg_gba_name(const char *ascii, uint8_t out[8]) {
    memset(out, 0xff, 8);
    for (int i = 0; i < 7 && ascii && ascii[i]; i++) {
        char c = ascii[i];
        uint8_t g = 0xac;                                     /* '?' */
        if (c >= 'A' && c <= 'Z') g = (uint8_t)(0xbb + c - 'A');
        else if (c >= 'a' && c <= 'z') g = (uint8_t)(0xd5 + c - 'a');
        else if (c >= '0' && c <= '9') g = (uint8_t)(0xa1 + c - '0');
        else if (c == ' ') g = 0x00;
        else if (c == '!') g = 0xab;
        else if (c == '.') g = 0xad;
        else if (c == '-') g = 0xae;
        out[i] = g;
    }
}

void ldn_frlg_room_gift(ldn_frlg_room *r, uint8_t activity, const char *group,
                        uint16_t trainer_id, uint16_t session_id) {
    memset(r, 0, sizeof *r);
    copy_name(r->nickname, group, 64);
    r->players = 1;
    r->trainer_id = trainer_id;
    ldn_frlg_gba_name(group, r->name);
    r->session_id = session_id;
    r->activity = activity;
    r->national = 1;                  /* kept from pokeldn's captured FRLG beacon (word 0x1584) */
    r->version = LDN_FRLG_VERSION_FIRE_RED;
    r->language = LDN_FRLG_LANGUAGE_ENGLISH;
}

void ldn_frlg_room_trade(ldn_frlg_room *r, const char *name, uint16_t trainer_id,
                         const uint8_t host_session_id[2], uint8_t version, uint8_t language) {
    memset(r, 0, sizeof *r);
    copy_name(r->nickname, name, 64);
    r->players = 1;
    r->trainer_id = trainer_id;
    charmap_encode(name, 8, 0xff, r->name);
    r->session_id = (uint16_t)(host_session_id[0] | host_session_id[1] << 8);
    r->activity = LDN_FRLG_ACTIVITY_TRADE;
    r->national = 1;                  /* the captured native trade beacon's word 0x1584 */
    r->version = version & 7;
    r->language = language & 7;
}

void ldn_frlg_room_trade_activate(ldn_frlg_room *r) {
    r->players = 2;
    r->started = 1;
}

static void room_record(const ldn_frlg_room *r, uint8_t rec[24]) {
    wle16(rec, r->trainer_id);
    memcpy(rec + 2, r->name, 8);
    wle16(rec + 10, r->session_id);
    memcpy(rec + 12, r->partner, 4);
    wle16(rec + 16, (uint16_t)((r->activity & 0x7f) | (r->national ? 0x80 : 0) |
                               (r->version & 7) << 8 | (r->language & 7) << 11 |
                               (r->has_card ? 0x4000 : 0) | (r->started ? 0x8000 : 0)));
    memcpy(rec + 18, r->tail, 6);
}

long ldn_frlg_app_data_build(const ldn_frlg_room *r, uint8_t *out, size_t cap) {
    if (cap < LDN_FRLG_APP_DATA_LEN) return -1;
    /* Pia system header (BE): size, sys comm 22, app comm 88, password16, limit on, players,
     * u32 name size, encoding 1 = UTF-8, name64. */
    memset(out, 0, LDN_FRLG_PIA_HEADER);
    wbe16(out, LDN_FRLG_PIA_HEADER);
    out[2] = 22;
    wbe16(out + 3, LDN_HOST_FRLG_APP_VERSION);
    out[0x15] = 1;
    out[0x16] = r->players;
    size_t nl = bounded_len(r->nickname, 64);
    wbe32(out + 0x17, (uint32_t)nl);
    out[0x1b] = 1;
    memcpy(out + 0x1c, r->nickname, nl);
    /* Room record: 4-byte LE groups, 5 base85 digits each, least significant first. */
    uint8_t rec[24], *t = out + LDN_FRLG_PIA_HEADER;
    room_record(r, rec);
    for (int g = 0; g < 6; g++) {
        uint32_t v = rle32(rec + 4 * g);
        for (int d = 0; d < 5; d++, v /= 85) *t++ = b85_char(v % 85);
    }
    return LDN_FRLG_APP_DATA_LEN;
}

int ldn_frlg_app_data_parse(const uint8_t *d, size_t len, ldn_frlg_room *r) {
    if (len < LDN_FRLG_APP_DATA_LEN || rbe16(d) != LDN_FRLG_PIA_HEADER) return -1;
    uint8_t rec[24];
    const uint8_t *t = d + LDN_FRLG_PIA_HEADER;
    for (int g = 0; g < 6; g++) {
        uint64_t v = 0;
        for (int i = 4; i >= 0; i--) {
            int digit = b85_digit(t[5 * g + i]);
            if (digit < 0) return -1;
            v = v * 85 + (uint64_t)digit;
        }
        if (v > 0xffffffffu) return -1;
        wle32(rec + 4 * g, (uint32_t)v);
    }
    memset(r, 0, sizeof *r);
    uint32_t nl = rbe32(d + 0x17);
    if (nl > 64) return -1;
    memcpy(r->nickname, d + 0x1c, nl);
    r->players = d[0x16];
    r->trainer_id = rle16(rec);
    memcpy(r->name, rec + 2, 8);
    r->session_id = rle16(rec + 10);
    memcpy(r->partner, rec + 12, 4);
    uint16_t w = rle16(rec + 16);
    r->activity = w & 0x7f;
    r->national = (w >> 7) & 1;
    r->version = (w >> 8) & 7;
    r->language = (w >> 11) & 7;
    r->has_card = (w >> 14) & 1;
    r->started = (w >> 15) & 1;
    memcpy(r->tail, rec + 18, 6);
    return 0;
}
