/* aes_backend.h — swappable AES primitive interface.
 * crypto.c calls these; the implementation is provided per platform:
 *   host  -> aes_backend_openssl.c (libcrypto)
 *   ESP32 -> aes_backend_mbedtls.c (mbedTLS, built into ESP-IDF)  [later]
 * The AES itself is never hand-rolled — always a vetted library. */
#ifndef AES_BACKEND_H
#define AES_BACKEND_H
#include <stdint.h>
#include <stddef.h>

/* Single AES-128 ECB block encrypt. Returns 0 on success. */
int aes128_ecb_encrypt_block(const uint8_t key[16], const uint8_t in[16], uint8_t out[16]);

/* Single AES-128 ECB block decrypt (needed by the LDN key-derivation chain). 0 on success. */
int aes128_ecb_decrypt_block(const uint8_t key[16], const uint8_t in[16], uint8_t out[16]);

/* SHA-256 of `data` into out[32]. 0 on success. */
int sha256_digest(const uint8_t *data, size_t len, uint8_t out[32]);

/* AES-128-GCM encrypt. Writes ct (pt_len bytes) and the FULL 16-byte tag. 0 on success. */
int aes128_gcm_encrypt(const uint8_t key[16], const uint8_t *nonce, size_t nonce_len,
                       const uint8_t *aad, size_t aad_len,
                       const uint8_t *pt, size_t pt_len,
                       uint8_t *ct, uint8_t tag16[16]);

/* AES-128-GCM decrypt + verify `tag_len` bytes of tag. Writes pt (ct_len bytes).
 * Returns 0 on auth success, -1 on auth failure. */
int aes128_gcm_decrypt(const uint8_t key[16], const uint8_t *nonce, size_t nonce_len,
                       const uint8_t *aad, size_t aad_len,
                       const uint8_t *ct, size_t ct_len,
                       const uint8_t *tag, size_t tag_len,
                       uint8_t *pt);

/* AES-128-CCM encrypt (802.11 CCMP uses nonce_len=13, tag_len=8). Writes ct (pt_len bytes) and
 * the tag (tag_len bytes). 0 on success. */
int aes128_ccm_encrypt(const uint8_t key[16], const uint8_t *nonce, size_t nonce_len,
                       const uint8_t *aad, size_t aad_len,
                       const uint8_t *pt, size_t pt_len,
                       uint8_t *ct, uint8_t *tag, size_t tag_len);

/* AES-128-CCM decrypt + verify tag_len bytes of tag. Writes pt (ct_len bytes).
 * Returns 0 on auth success, -1 on auth failure. */
int aes128_ccm_decrypt(const uint8_t key[16], const uint8_t *nonce, size_t nonce_len,
                       const uint8_t *aad, size_t aad_len,
                       const uint8_t *ct, size_t ct_len,
                       const uint8_t *tag, size_t tag_len,
                       uint8_t *pt);
#endif
