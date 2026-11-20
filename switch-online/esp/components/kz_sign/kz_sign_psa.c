/* Device backend. Compiled only into ESP-IDF builds; the host test uses its own backend (test/test_kz_sign.c). */
#include "kz_sign.h"
#include "psa/crypto.h"

static int c_sha256(const uint8_t *d, size_t n, uint8_t out[32])
{
    size_t l = 0;
    return psa_hash_compute(PSA_ALG_SHA_256, d, n, out, 32, &l) == PSA_SUCCESS && l == 32 ? 0 : -1;
}

static int c_verify(const uint8_t pub[65], const uint8_t hash[32], const uint8_t sig[64])
{
    psa_key_attributes_t a = PSA_KEY_ATTRIBUTES_INIT;
    psa_set_key_type(&a, PSA_KEY_TYPE_ECC_PUBLIC_KEY(PSA_ECC_FAMILY_SECP_R1));
    psa_set_key_bits(&a, 256);
    psa_set_key_usage_flags(&a, PSA_KEY_USAGE_VERIFY_HASH);
    psa_set_key_algorithm(&a, PSA_ALG_ECDSA(PSA_ALG_SHA_256));
    psa_key_id_t id = 0;
    if (psa_import_key(&a, pub, 65, &id) != PSA_SUCCESS) return -1;
    psa_status_t st = psa_verify_hash(id, PSA_ALG_ECDSA(PSA_ALG_SHA_256), hash, 32, sig, 64);
    psa_destroy_key(id);
    return st == PSA_SUCCESS ? 0 : -1;
}

static int c_sign(void *key, const uint8_t hash[32], uint8_t sig[64])
{
    size_t l = 0;
    if (!key) return -1;
    return psa_sign_hash(*(psa_key_id_t *)key, PSA_ALG_ECDSA(PSA_ALG_SHA_256), hash, 32, sig, 64, &l) == PSA_SUCCESS && l == 64 ? 0 : -1;
}

const kz_sign_crypto_t kz_sign_psa_crypto = { c_sha256, c_verify, c_sign };
