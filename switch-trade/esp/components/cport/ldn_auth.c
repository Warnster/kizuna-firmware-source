/* SPDX-License-Identifier: AGPL-3.0-only
 * Derived from pokeldn (https://github.com/Decryptu/pokeldn), Copyright (c) the pokeldn authors.
 * Derived from LDN (https://github.com/kinnay/LDN), Copyright (c) kinnay and the LDN authors.
 * Changes Copyright (c) 2026 Kizuna. See NOTICE.
 */
/* ldn_auth.c — see ldn_auth.h. Port of kinnay/LDN AuthenticationFrame (protocol 3). */
#include "ldn_auth.h"
#include "aes_backend.h"
#include <string.h>

/* kinnay CHALLENGE_KEY (non-dev). */
static const uint8_t CHALLENGE_KEY[32] = {
    0xf8,0x4b,0x48,0x7f,0xb3,0x72,0x51,0xc2,0x63,0xbf,0x11,0x60,0x90,0x36,0x58,0x92,
    0x66,0xaf,0x70,0xca,0x79,0xb4,0x4c,0x93,0xc7,0x37,0x0c,0x57,0x69,0xc0,0xf6,0x02 };

static void wle16(uint8_t *p, uint16_t v){ p[0]=(uint8_t)v; p[1]=(uint8_t)(v>>8); }
static void wle64(uint8_t *p, uint64_t v){ for(int i=0;i<8;i++) p[i]=(uint8_t)(v>>(8*i)); }
static void wbe16(uint8_t *p, uint16_t v){ p[0]=(uint8_t)(v>>8); p[1]=(uint8_t)v; }

/* HMAC-SHA256 built on aes_backend's sha256_digest (key <= 64 bytes here). out = 32 bytes. */
static int hmac_sha256(const uint8_t *key, int keylen, const uint8_t *msg, int msglen, uint8_t out[32]) {
    uint8_t k_ipad[64], k_opad[64];
    static uint8_t inbuf[64 + 1024];   /* 64 + msg (challenge body 720) */
    uint8_t ibuf_inner[32], obuf[64 + 32];
    if (keylen > 64 || msglen > 1024) return -1;
    memset(k_ipad, 0, 64); memset(k_opad, 0, 64);
    memcpy(k_ipad, key, keylen); memcpy(k_opad, key, keylen);
    for (int i = 0; i < 64; i++) { k_ipad[i] ^= 0x36; k_opad[i] ^= 0x5c; }
    memcpy(inbuf, k_ipad, 64); memcpy(inbuf + 64, msg, msglen);
    if (sha256_digest(inbuf, 64 + msglen, ibuf_inner)) return -1;
    memcpy(obuf, k_opad, 64); memcpy(obuf + 64, ibuf_inner, 32);
    if (sha256_digest(obuf, 64 + 32, out)) return -1;
    return 0;
}

/* ChallengeRequest.encode(CHALLENGE_KEY) -> 0x300 (768) bytes. */
static int build_challenge_request(uint64_t token, uint64_t nonce, uint8_t out[768]) {
    uint8_t body[720];
    memset(body, 0, sizeof(body));
    /* header: u8 0,0, len(params1)=0, len(params2)=0, flags=0, pad3 */
    /* (all zero already) */
    wle64(body + 8, token);
    wle64(body + 16, nonce);
    wle64(body + 24, 0);            /* device_id */
    /* unk[16] @32, then pad(0x60), params1 pad(64), params2 pad(512) — all zero */
    uint8_t mac[32];
    if (hmac_sha256(CHALLENGE_KEY, 32, body, 720, mac)) return -1;
    memset(out, 0, 768);
    /* u32(0) @0, mac @4, pad12, body @48 */
    memcpy(out + 4, mac, 32);
    memcpy(out + 48, body, 720);
    return 768;
}

/* AuthenticationRequest.encode(version>=3) -> 868 bytes (BE). */
static int build_auth_request(const char *username, uint16_t app_version,
                              const uint8_t challenge[768], uint8_t out[868]) {
    memset(out, 0, 868);
    int nl = 0; if (username) { while (username[nl] && nl < 32) nl++; }
    memcpy(out, username, (size_t)nl);        /* name padded to 32 */
    wbe16(out + 32, app_version);
    out[34] = 0;                              /* platform PLATFORM_NX */
    /* pad(29) -> [35..64); version>=3: pad(0x24) -> [64..100); challenge[768] @100 */
    memcpy(out + 100, challenge, 768);
    return 868;                               /* 100 + 768 */
}

int ldn_build_auth_frame(const ldn_keys *k, int protocol, int version,
                         uint64_t comm_id, uint16_t scene_id, const uint8_t ssid[16],
                         const uint8_t server_random[16], const uint8_t client_random[16],
                         uint64_t challenge_token, uint64_t challenge_nonce,
                         const char *username, uint16_t app_version,
                         uint8_t *out, int out_cap) {
    if (out_cap < 1024) return -1;
    uint8_t challenge[768];
    if (build_challenge_request(challenge_token, challenge_nonce, challenge) < 0) return -1;
    uint8_t payload[868];
    build_auth_request(username, app_version, challenge, payload);
    const int plen = 868;

    /* frame: u24(0x0022AA) u16(0x102) pad(1) [6] + header[72] + tag[16] + ct[plen] */
    int o = 0;
    out[o++] = 0x00; out[o++] = 0x22; out[o++] = 0xaa;   /* OUI (BE u24) */
    out[o++] = 0x01; out[o++] = 0x02;                    /* 0x102 (BE u16) */
    out[o++] = 0x00;                                     /* pad(1) */

    uint8_t header[72];
    memset(header, 0, sizeof(header));
    header[0] = (uint8_t)version;
    header[1] = (uint8_t)(plen & 0xFF);
    header[2] = 0;                                       /* status_code */
    header[3] = 0;                                       /* is_response (request) */
    header[4] = (uint8_t)(plen >> 8);
    header[5] = (protocol == 1) ? 0 : 1;                 /* format: 1 = AES-GCM */
    /* header[6..8] pad(2) */
    /* NetworkId.encode("<") @8: u64 comm_id LE, pad2, u16 scene LE, pad4, ssid[16] -> 32 bytes */
    wle64(header + 8, comm_id);
    /* pad2 @16..18 */
    wle16(header + 18, scene_id);
    /* pad4 @20..24 */
    memcpy(header + 24, ssid, 16);                       /* ssid @24..40 */
    memcpy(header + 40, server_random, 16);              /* @40..56 */
    memcpy(header + 56, client_random, 16);              /* @56..72 */
    memcpy(out + o, header, 72); o += 72;

    if (protocol == 3) {
        uint8_t auth_key[16];
        if (ldn_derive_authentication_key(k, client_random, 16, auth_key) != 0) return -1;
        uint8_t tag[16];
        /* GCM: nonce = header[:12], AAD = full 72-byte header, pt = payload. */
        if (aes128_gcm_encrypt(auth_key, header, 12, header, 72, payload, plen,
                               out + o + 16, tag) != 0) return -1;
        memcpy(out + o, tag, 16);                        /* tag before ciphertext */
        o += 16 + plen;
    } else {
        memcpy(out + o, payload, plen); o += plen;       /* protocol 1: plaintext (unused for FRLG) */
    }
    return o;                                            /* 6 + 72 + 16 + 868 = 962 */
}
