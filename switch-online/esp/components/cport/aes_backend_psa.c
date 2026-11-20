/* aes_backend_psa.c — aes_backend.h implemented on the PSA Crypto API.
 *
 * ESP-IDF v6.1 ships mbedTLS 4.x (TF-PSA-Crypto), where the classic mbedtls/aes.h, gcm.h, ccm.h
 * headers are private — the public crypto API is PSA. This backend is used on the ESP32 build
 * instead of aes_backend_mbedtls.c (classic API) / aes_backend_openssl.c (host). The AES/SHA
 * are the vetted library primitives; nothing is hand-rolled.
 *
 * Called from the single trade-brain task, so the small static staging buffers below are fine
 * (PSA's one-shot AEAD API concatenates ciphertext||tag, but aes_backend wants them separate).
 */
#include "aes_backend.h"
#include <string.h>
#include "psa/crypto.h"

#define STAGE_MAX 2048  /* >= max Pia datagram + tag */

static int ensure_init(void) {
    static int done = 0;
    if (!done) {
        if (psa_crypto_init() != PSA_SUCCESS) return -1;
        done = 1;
    }
    return 0;
}

static int import_key(const uint8_t key[16], psa_algorithm_t alg,
                      psa_key_usage_t usage, psa_key_id_t *id) {
    psa_key_attributes_t a = PSA_KEY_ATTRIBUTES_INIT;
    psa_set_key_type(&a, PSA_KEY_TYPE_AES);
    psa_set_key_bits(&a, 128);
    psa_set_key_usage_flags(&a, usage);
    psa_set_key_algorithm(&a, alg);
    psa_status_t s = psa_import_key(&a, key, 16, id);
    psa_reset_key_attributes(&a);
    return s == PSA_SUCCESS ? 0 : -1;
}

int aes128_ecb_encrypt_block(const uint8_t key[16], const uint8_t in[16], uint8_t out[16]) {
    if (ensure_init()) return -1;
    psa_key_id_t k;
    if (import_key(key, PSA_ALG_ECB_NO_PADDING, PSA_KEY_USAGE_ENCRYPT, &k)) return -1;
    size_t olen = 0;
    psa_status_t s = psa_cipher_encrypt(k, PSA_ALG_ECB_NO_PADDING, in, 16, out, 16, &olen);
    psa_destroy_key(k);
    return (s == PSA_SUCCESS && olen == 16) ? 0 : -1;
}

int aes128_ecb_decrypt_block(const uint8_t key[16], const uint8_t in[16], uint8_t out[16]) {
    if (ensure_init()) return -1;
    psa_key_id_t k;
    if (import_key(key, PSA_ALG_ECB_NO_PADDING, PSA_KEY_USAGE_DECRYPT, &k)) return -1;
    size_t olen = 0;
    psa_status_t s = psa_cipher_decrypt(k, PSA_ALG_ECB_NO_PADDING, in, 16, out, 16, &olen);
    psa_destroy_key(k);
    return (s == PSA_SUCCESS && olen == 16) ? 0 : -1;
}

int sha256_digest(const uint8_t *data, size_t len, uint8_t out[32]) {
    if (ensure_init()) return -1;
    size_t olen = 0;
    return psa_hash_compute(PSA_ALG_SHA_256, data, len, out, 32, &olen) == PSA_SUCCESS && olen == 32
               ? 0 : -1;
}

int aes128_gcm_encrypt(const uint8_t key[16], const uint8_t *nonce, size_t nonce_len,
                       const uint8_t *aad, size_t aad_len,
                       const uint8_t *pt, size_t pt_len,
                       uint8_t *ct, uint8_t tag16[16]) {
    if (ensure_init() || pt_len + 16 > STAGE_MAX) return -1;
    psa_key_id_t k;
    if (import_key(key, PSA_ALG_GCM, PSA_KEY_USAGE_ENCRYPT, &k)) return -1;
    static uint8_t stage[STAGE_MAX];
    size_t olen = 0;
    psa_status_t s = psa_aead_encrypt(k, PSA_ALG_GCM, nonce, nonce_len, aad, aad_len,
                                      pt, pt_len, stage, sizeof(stage), &olen);
    psa_destroy_key(k);
    if (s != PSA_SUCCESS || olen != pt_len + 16) return -1;
    if (pt_len) memcpy(ct, stage, pt_len);
    memcpy(tag16, stage + pt_len, 16);
    return 0;
}

int aes128_gcm_decrypt(const uint8_t key[16], const uint8_t *nonce, size_t nonce_len,
                       const uint8_t *aad, size_t aad_len,
                       const uint8_t *ct, size_t ct_len,
                       const uint8_t *tag, size_t tag_len,
                       uint8_t *pt) {
    if (ensure_init() || ct_len + tag_len > STAGE_MAX) return -1;
    psa_algorithm_t alg = PSA_ALG_AEAD_WITH_SHORTENED_TAG(PSA_ALG_GCM, tag_len);
    psa_key_id_t k;
    if (import_key(key, alg, PSA_KEY_USAGE_DECRYPT, &k)) return -1;
    static uint8_t stage[STAGE_MAX];
    if (ct_len) memcpy(stage, ct, ct_len);
    memcpy(stage + ct_len, tag, tag_len);
    size_t olen = 0;
    psa_status_t s = psa_aead_decrypt(k, alg, nonce, nonce_len, aad, aad_len,
                                      stage, ct_len + tag_len, pt, ct_len ? ct_len : 1, &olen);
    psa_destroy_key(k);
    return (s == PSA_SUCCESS && olen == ct_len) ? 0 : -1;
}

int aes128_ccm_encrypt(const uint8_t key[16], const uint8_t *nonce, size_t nonce_len,
                       const uint8_t *aad, size_t aad_len,
                       const uint8_t *pt, size_t pt_len,
                       uint8_t *ct, uint8_t *tag, size_t tag_len) {
    if (ensure_init() || pt_len + tag_len > STAGE_MAX) return -1;
    psa_algorithm_t alg = PSA_ALG_AEAD_WITH_SHORTENED_TAG(PSA_ALG_CCM, tag_len);
    psa_key_id_t k;
    if (import_key(key, alg, PSA_KEY_USAGE_ENCRYPT, &k)) return -1;
    static uint8_t stage[STAGE_MAX];
    size_t olen = 0;
    psa_status_t s = psa_aead_encrypt(k, alg, nonce, nonce_len, aad, aad_len,
                                      pt, pt_len, stage, sizeof(stage), &olen);
    psa_destroy_key(k);
    if (s != PSA_SUCCESS || olen != pt_len + tag_len) return -1;
    if (pt_len) memcpy(ct, stage, pt_len);
    memcpy(tag, stage + pt_len, tag_len);
    return 0;
}

int aes128_ccm_decrypt(const uint8_t key[16], const uint8_t *nonce, size_t nonce_len,
                       const uint8_t *aad, size_t aad_len,
                       const uint8_t *ct, size_t ct_len,
                       const uint8_t *tag, size_t tag_len,
                       uint8_t *pt) {
    if (ensure_init() || ct_len + tag_len > STAGE_MAX) return -1;
    psa_algorithm_t alg = PSA_ALG_AEAD_WITH_SHORTENED_TAG(PSA_ALG_CCM, tag_len);
    psa_key_id_t k;
    if (import_key(key, alg, PSA_KEY_USAGE_DECRYPT, &k)) return -1;
    static uint8_t stage[STAGE_MAX];
    if (ct_len) memcpy(stage, ct, ct_len);
    memcpy(stage + ct_len, tag, tag_len);
    size_t olen = 0;
    psa_status_t s = psa_aead_decrypt(k, alg, nonce, nonce_len, aad, aad_len,
                                      stage, ct_len + tag_len, pt, ct_len ? ct_len : 1, &olen);
    psa_destroy_key(k);
    return (s == PSA_SUCCESS && olen == ct_len) ? 0 : -1;
}
