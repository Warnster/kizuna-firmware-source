/* SPDX-License-Identifier: AGPL-3.0-only
 * Derived from pokeldn (https://github.com/Decryptu/pokeldn), Copyright (c) the pokeldn authors.
 * Derived from LDN (https://github.com/kinnay/LDN), Copyright (c) kinnay and the LDN authors.
 * Changes Copyright (c) 2026 Kizuna. See NOTICE.
 */
#include "ldn_advert.h"
#include "aes_backend.h"
#include <string.h>

/* Per-purpose source constants — hardcoded in the LDN library, NOT from prod.keys.
 * ADVERTISE decrypts the advertisement; DATA_AUTH derives BOTH the WLAN data (CCMP) key
 * and the authentication key (they share this source, only the input data differs). */
static const uint8_t ADVERTISE_SOURCE[16] = {
    0x19,0x18,0x84,0x74,0x3e,0x24,0xc7,0x7d,0x87,0xc6,0x9e,0x42,0x07,0xd0,0xc4,0x38
};
static const uint8_t DATA_AUTH_SOURCE[16] = {
    0xf1,0xe7,0x01,0x84,0x19,0xa8,0x4f,0x71,0x1d,0xa7,0x14,0xc2,0xcf,0x91,0x9c,0x9c
};

/* KeyDerivation._derive_key(data, source) for protocol 3:
 *   k = master_key_12
 *   k = ECB_dec(k, aes_kek_generation_source)
 *   k = ECB_dec(k, source)
 *   k = ECB_dec(k, aes_key_generation_source)
 *   k = ECB_dec(k, sha256(data)[:16])
 * where ECB_dec(kek, block) = AES-128-ECB-decrypt(block) under kek. */
static int derive_key(const ldn_keys *k, const uint8_t source[16],
                      const uint8_t *data, size_t data_len, uint8_t out_key[16]) {
    uint8_t cur[16], sha[32];
    memcpy(cur, k->master_key_12, 16);
    if (aes128_ecb_decrypt_block(cur, k->aes_kek_generation_source, cur)) return -1;
    if (aes128_ecb_decrypt_block(cur, source, cur)) return -1;
    if (aes128_ecb_decrypt_block(cur, k->aes_key_generation_source, cur)) return -1;
    if (sha256_digest(data, data_len, sha)) return -1;
    if (aes128_ecb_decrypt_block(cur, sha, out_key)) return -1;
    return 0;
}

int ldn_derive_advertise_key(const ldn_keys *k, const uint8_t network_id[32], uint8_t out_key[16]) {
    return derive_key(k, ADVERTISE_SOURCE, network_id, 32, out_key);
}

/* Authentication key: data = client_random (used to GCM the AuthenticationFrame during join). */
int ldn_derive_authentication_key(const ldn_keys *k, const uint8_t *client_random,
                                  size_t len, uint8_t out_key[16]) {
    return derive_key(k, DATA_AUTH_SOURCE, client_random, len, out_key);
}

/* WLAN data (CCMP) key: data = server_random || password. Empty password -> pw_len 0. */
int ldn_derive_data_key(const ldn_keys *k, const uint8_t *server_random, size_t sr_len,
                        const uint8_t *password, size_t pw_len, uint8_t out_key[16]) {
    uint8_t buf[256];
    if (sr_len + pw_len > sizeof(buf)) return -1;
    memcpy(buf, server_random, sr_len);
    if (pw_len) memcpy(buf + sr_len, password, pw_len);
    return derive_key(k, DATA_AUTH_SOURCE, buf, sr_len + pw_len, out_key);
}

static uint16_t be16(const uint8_t *p) { return (uint16_t)((p[0] << 8) | p[1]); }

long ldn_advert_decrypt(const ldn_keys *k, const uint8_t *frame, size_t len, uint8_t *out) {
    if (len < 52) return -1;
    if (frame[0] != 0x7F || frame[1] != 0x00 || frame[2] != 0x22 || frame[3] != 0xAA) return -1;
    if (frame[4] != 0x04) return -1;
    if (frame[6] != 0x01 || frame[7] != 0x01) return -1;     /* 0x0101 advertisement */
    if (frame[45] != 0x03) return -1;                        /* format 3 = AES-GCM */

    const uint8_t *network_id = frame + 12;                  /* 32 bytes */
    uint16_t size = be16(frame + 46);
    const uint8_t *header = frame + 12;                      /* 40-byte AAD */
    const uint8_t *tag = frame + 52;                         /* 16-byte GCM tag */
    const uint8_t *ct = frame + 68;
    if (len < (size_t)(68 + size)) return -1;                /* capture must be complete */

    uint8_t key[16], nonce[12];
    if (ldn_derive_advertise_key(k, network_id, key)) return -1;
    memcpy(nonce, frame + 48, 4);                            /* 4-byte advert nonce ... */
    memset(nonce + 4, 0, 8);                                 /* ... || 8 zero bytes */

    if (aes128_gcm_decrypt(key, nonce, 12, header, 40, ct, size, tag, 16, out) != 0)
        return -1;                                           /* auth failed */
    return size;
}

long ldn_advert_encrypt(const ldn_keys *k, const uint8_t network_id[32], uint8_t version,
                        const uint8_t nonce4[4], const uint8_t *plaintext, size_t pt_len,
                        uint8_t *out) {
    if (pt_len > 0xFFFF) return -1;
    /* 12-byte vendor/LDN header prefix (matches decode) */
    static const uint8_t PREFIX[12] = {0x7F,0x00,0x22,0xAA,0x04,0x00,0x01,0x01,0x00,0x00,0x00,0x00};
    memcpy(out, PREFIX, 12);
    memcpy(out + 12, network_id, 32);
    out[44] = version;
    out[45] = 0x03;                                  /* format 3 = AES-GCM */
    out[46] = (uint8_t)(pt_len >> 8);                /* size, big-endian */
    out[47] = (uint8_t)(pt_len & 0xFF);
    memcpy(out + 48, nonce4, 4);

    uint8_t key[16], nonce[12], tag[16];
    if (ldn_derive_advertise_key(k, network_id, key)) return -1;
    memcpy(nonce, nonce4, 4);
    memset(nonce + 4, 0, 8);
    /* AAD = the 40-byte header (out[12..52]); ct goes at out[68], tag at out[52] */
    if (aes128_gcm_encrypt(key, nonce, 12, out + 12, 40, plaintext, pt_len, out + 68, tag) != 0)
        return -1;
    memcpy(out + 52, tag, 16);
    return (long)(68 + pt_len);
}

uint64_t ldn_advert_local_comm_id(const uint8_t *frame, size_t len) {
    if (len < 44 || frame[0] != 0x7F) return 0;
    uint64_t v = 0;
    for (int i = 0; i < 8; i++) v = (v << 8) | frame[12 + i];  /* big-endian u64 */
    return v;
}

int ldn_advert_ssid(const uint8_t *frame, size_t len, uint8_t out_ssid[16]) {
    if (len < 44 || frame[0] != 0x7F) return -1;
    memcpy(out_ssid, frame + 28, 16);
    return 0;
}
