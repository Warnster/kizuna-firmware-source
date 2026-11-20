/* SPDX-License-Identifier: AGPL-3.0-only
 * Derived from pokeldn (https://github.com/Decryptu/pokeldn), Copyright (c) the pokeldn authors.
 * Derived from LDN (https://github.com/kinnay/LDN), Copyright (c) kinnay and the LDN authors.
 * Changes Copyright (c) 2026 Kizuna. See NOTICE.
 */
/* ldn_advert.h — decode + AES-GCM decrypt a Nintendo LDN advertisement action frame.
 * Port of vendor/LDN/ldn AdvertisementFrame (protocol 3 / AES-GCM) + KeyDerivation.
 * Needs exactly 3 prod.keys entries (see ldn_keys.h). The raw AES/SHA come from aes_backend.h.
 *
 * Frame layout (big-endian), starting at the vendor-specific action byte:
 *   [0]      0x7F  vendor-specific
 *   [1:4]    00 22 AA  Nintendo OUI
 *   [4]      0x04  LDN
 *   [5]      pad
 *   [6:8]    0x0101  advertisement frame
 *   [8:12]   pad
 *   [12:44]  NetworkId (u64 local_comm_id, pad2, u16 scene_id, pad4, 16-byte ssid)
 *   [44]     version   [45] format(3=GCM)   [46:48] plaintext size (BE)   [48:52] nonce(4)
 *   [52:52+16]        GCM tag (16)
 *   [68:68+size]      ciphertext
 * GCM: key = advertise_key(NetworkId), nonce = frame_nonce(4) || 8 zero bytes, AAD = frame[12:52]. */
#ifndef LDN_ADVERT_H
#define LDN_ADVERT_H
#include <stdint.h>
#include <stddef.h>

typedef struct {
    uint8_t master_key_12[16];
    uint8_t aes_kek_generation_source[16];
    uint8_t aes_key_generation_source[16];
} ldn_keys;

/* Derive the 16-byte AES-GCM advertise key from the 32-byte NetworkId. 0 on success. */
int ldn_derive_advertise_key(const ldn_keys *k, const uint8_t network_id[32], uint8_t out_key[16]);

/* Join-side key derivations (same 3 keys, DATA_AUTH source). 0 on success.
 *  - authentication key: GCM key for the AuthenticationFrame; data = client_random.
 *  - data key: the WLAN/CCMP session key; data = server_random || password. */
int ldn_derive_authentication_key(const ldn_keys *k, const uint8_t *client_random,
                                  size_t len, uint8_t out_key[16]);
int ldn_derive_data_key(const ldn_keys *k, const uint8_t *server_random, size_t sr_len,
                        const uint8_t *password, size_t pw_len, uint8_t out_key[16]);

/* Decode + GCM-decrypt a full advertisement frame (frame/len as captured off the air, FCS removed).
 * On success writes the plaintext to out (caller provides >= 512 bytes) and returns its length.
 * Returns -1 on a malformed frame or GCM auth failure (wrong keys / corrupt capture). */
long ldn_advert_decrypt(const ldn_keys *k, const uint8_t *frame, size_t len, uint8_t *out);

/* Inverse of ldn_advert_decrypt: build a full advertisement action frame (for HOSTING) from a
 * plaintext AdvertisementInfo. Encrypts with the advertise key (from network_id) + AES-GCM,
 * nonce = nonce4 || 8 zero bytes, AAD = the 40-byte header. Writes the frame to out (caller
 * provides >= 68 + pt_len bytes) and returns its length, or -1 on error. version is normally 4. */
long ldn_advert_encrypt(const ldn_keys *k, const uint8_t network_id[32], uint8_t version,
                        const uint8_t nonce4[4], const uint8_t *plaintext, size_t pt_len,
                        uint8_t *out);

/* Convenience: pull the plaintext fields that live in the frame header (no keys needed). */
uint64_t ldn_advert_local_comm_id(const uint8_t *frame, size_t len);   /* 0 if malformed */
int      ldn_advert_ssid(const uint8_t *frame, size_t len, uint8_t out_ssid[16]); /* 0 on success */

#endif
