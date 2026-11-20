#include "kz_identity.h"
#include <string.h>
#include "nvs.h"
#include "psa/crypto.h"
#include "kz_identity_core.h"
#include "kz_identity_ca_pub.h"

#define NS        "kz_identity"
#define K_PRIV    "priv"     /* 32-byte raw EC private scalar */
#define K_PUB     "pub"      /* 65-byte uncompressed P-256 point, derived from priv at generation time */
#define K_CERT    "cert"     /* KIZUNA-CERT-V1 text */
#define K_CERTSIG "certsig"  /* 86-char base64url signature over cert, by the CA key */

static bool s_have_key;
static uint8_t s_pub[65];
static psa_key_id_t s_priv_id;   /* 0 = not imported into PSA yet this boot */

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

/* Imports the stored private scalar into a live PSA key once per boot, reused after that. Not thread-safe against
 * concurrent callers, same as the rest of this component (the BLE control characteristic serialises access). */
static bool priv_key_id(psa_key_id_t *out)
{
    if (!s_have_key) return false;
    if (s_priv_id != 0) { *out = s_priv_id; return true; }

    nvs_handle_t h;
    if (nvs_open(NS, NVS_READONLY, &h) != ESP_OK) return false;
    uint8_t priv[32];
    size_t n = sizeof priv;
    esp_err_t e = nvs_get_blob(h, K_PRIV, priv, &n);
    nvs_close(h);
    if (e != ESP_OK || n != sizeof priv) return false;

    psa_key_attributes_t a = PSA_KEY_ATTRIBUTES_INIT;
    psa_set_key_type(&a, PSA_KEY_TYPE_ECC_KEY_PAIR(PSA_ECC_FAMILY_SECP_R1));
    psa_set_key_bits(&a, 256);
    psa_set_key_usage_flags(&a, PSA_KEY_USAGE_SIGN_HASH);
    psa_set_key_algorithm(&a, PSA_ALG_ECDSA(PSA_ALG_SHA_256));
    psa_key_id_t id = 0;
    bool ok = psa_import_key(&a, priv, sizeof priv, &id) == PSA_SUCCESS;
    memset(priv, 0, sizeof priv);
    if (!ok) return false;
    s_priv_id = id;
    *out = id;
    return true;
}

int kz_identity_sign(void *unused_key, const uint8_t hash[32], uint8_t sig_out[64])
{
    (void)unused_key;
    psa_key_id_t id;
    if (!priv_key_id(&id)) return -1;
    size_t l = 0;
    return psa_sign_hash(id, PSA_ALG_ECDSA(PSA_ALG_SHA_256), hash, 32, sig_out, 64, &l) == PSA_SUCCESS && l == 64 ? 0 : -1;
}

const kz_sign_crypto_t kz_identity_crypto = { c_sha256, c_verify, kz_identity_sign };

static bool load_pub(void)
{
    nvs_handle_t h;
    if (nvs_open(NS, NVS_READONLY, &h) != ESP_OK) return false;
    size_t n = sizeof s_pub;
    bool ok = nvs_get_blob(h, K_PUB, s_pub, &n) == ESP_OK && n == sizeof s_pub;
    nvs_close(h);
    return ok;
}

static esp_err_t generate_and_store(void)
{
    psa_key_attributes_t a = PSA_KEY_ATTRIBUTES_INIT;
    psa_set_key_type(&a, PSA_KEY_TYPE_ECC_KEY_PAIR(PSA_ECC_FAMILY_SECP_R1));
    psa_set_key_bits(&a, 256);
    psa_set_key_usage_flags(&a, PSA_KEY_USAGE_SIGN_HASH | PSA_KEY_USAGE_EXPORT);
    psa_set_key_algorithm(&a, PSA_ALG_ECDSA(PSA_ALG_SHA_256));
    psa_key_id_t id = 0;
    if (psa_generate_key(&a, &id) != PSA_SUCCESS) return ESP_FAIL;

    uint8_t priv[32], pub[65];
    size_t n = 0;
    bool ok = psa_export_key(id, priv, sizeof priv, &n) == PSA_SUCCESS && n == sizeof priv;
    if (ok) ok = psa_export_public_key(id, pub, sizeof pub, &n) == PSA_SUCCESS && n == sizeof pub && pub[0] == 0x04;
    psa_destroy_key(id);
    if (!ok) { memset(priv, 0, sizeof priv); return ESP_FAIL; }

    nvs_handle_t h;
    esp_err_t e = nvs_open(NS, NVS_READWRITE, &h);
    if (e == ESP_OK) {
        e = nvs_set_blob(h, K_PRIV, priv, sizeof priv);
        if (e == ESP_OK) e = nvs_set_blob(h, K_PUB, pub, sizeof pub);
        if (e == ESP_OK) e = nvs_commit(h);
        nvs_close(h);
    }
    memset(priv, 0, sizeof priv);
    if (e == ESP_OK) memcpy(s_pub, pub, sizeof pub);
    return e;
}

esp_err_t kz_identity_init(void)
{
    if (s_have_key) return ESP_OK;
    psa_crypto_init();   /* idempotent if already initialised elsewhere (ota_update.c); needed for sign/verify too,
                          * not just key generation, so this must run on the "key already exists" path as well */
    if (load_pub()) { s_have_key = true; return ESP_OK; }
    esp_err_t e = generate_and_store();
    if (e == ESP_OK) s_have_key = true;
    return e;
}

bool kz_identity_pub(uint8_t out[65])
{
    if (!s_have_key) return false;
    memcpy(out, s_pub, 65);
    return true;
}

bool kz_identity_device_id(char out[KZ_SIGN_ID_LEN + 1])
{
    if (!s_have_key) return false;
    return kz_sign_device_id(s_pub, &kz_identity_crypto, out) == 0;
}

kz_sign_err_t kz_identity_install_cert(const char *cert, const char *cert_sig)
{
    if (!s_have_key) return KZ_SIGN_UNSIGNED;
    size_t cert_len = cert ? strlen(cert) : 0;
    kz_cert_record_t parsed;
    kz_sign_err_t e = kz_identity_cert_check(cert, cert_len, cert_sig, s_pub,
                                              KZ_IDENTITY_CA_TABLE, KZ_IDENTITY_CA_TABLE_LEN,
                                              &kz_identity_crypto, &parsed);
    if (e != KZ_SIGN_OK) return e;

    nvs_handle_t h;
    esp_err_t we = nvs_open(NS, NVS_READWRITE, &h);
    if (we != ESP_OK) return KZ_SIGN_BAD_CERTIFICATE;
    we = nvs_set_str(h, K_CERT, cert);
    if (we == ESP_OK) we = nvs_set_str(h, K_CERTSIG, cert_sig);
    if (we == ESP_OK) we = nvs_commit(h);
    nvs_close(h);
    return we == ESP_OK ? KZ_SIGN_OK : KZ_SIGN_BAD_CERTIFICATE;
}

bool kz_identity_get_cert(char *cert_out, size_t cert_cap, char *cert_sig_out, size_t cert_sig_cap)
{
    nvs_handle_t h;
    if (nvs_open(NS, NVS_READONLY, &h) != ESP_OK) return false;
    size_t n1 = cert_cap, n2 = cert_sig_cap;
    bool ok = nvs_get_str(h, K_CERT, cert_out, &n1) == ESP_OK && nvs_get_str(h, K_CERTSIG, cert_sig_out, &n2) == ESP_OK;
    nvs_close(h);
    return ok;
}

esp_err_t kz_identity_clear(void)
{
    s_have_key = false;
    memset(s_pub, 0, sizeof s_pub);
    if (s_priv_id != 0) { psa_destroy_key(s_priv_id); s_priv_id = 0; }
    nvs_handle_t h;
    esp_err_t e = nvs_open(NS, NVS_READWRITE, &h);
    if (e == ESP_ERR_NVS_NOT_FOUND) return ESP_OK;
    if (e != ESP_OK) return e;
    nvs_erase_all(h);
    e = nvs_commit(h);
    nvs_close(h);
    return e;
}
