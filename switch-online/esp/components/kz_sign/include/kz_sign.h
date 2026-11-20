#pragma once
/* Signed backup records. Pure C, no ESP-IDF headers; crypto is supplied by the caller.
 *
 *   KIZUNA-SAVE-V1\n device_id\n game\n chip\n size\n sha256\n ts\n slot\n    signed by the device key
 *   KIZUNA-CERT-V1\n ca_key_id\n device_id\n pub_hex\n issued_at\n            signed by the CA key
 *
 * Signature: ECDSA P-256 over SHA-256 of the exact record bytes, raw r||s, base64url without padding (86 chars).
 * The first line is the domain tag; it differs from the firmware record's (KIZUNA-FW-V1), so a signature made
 * for one kind of record never verifies as another. */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define KZ_SIGN_TAG_SAVE "KIZUNA-SAVE-V1"
#define KZ_SIGN_TAG_CERT "KIZUNA-CERT-V1"

#define KZ_SIGN_ID_LEN 16          /* device id: 16 lowercase hex chars */
#define KZ_SIGN_TOKEN_MAX 15       /* chip, slot, CA key id: [A-Za-z0-9_-] */
#define KZ_SIGN_SIG_B64_LEN 86
#define KZ_SIGN_RECORD_MAX 256

typedef struct {
    char device_id[KZ_SIGN_ID_LEN + 1];
    char game[5];                  /* 4 chars [A-Z0-9], or "----" if unknown */
    char chip[KZ_SIGN_TOKEN_MAX + 1];
    uint32_t size;
    char sha256[65];               /* 64 lowercase hex */
    uint32_t ts;                   /* epoch seconds, 0 if unknown (not a trust anchor) */
    char slot[KZ_SIGN_TOKEN_MAX + 1];
} kz_save_record_t;

typedef struct {
    char ca_key_id[KZ_SIGN_TOKEN_MAX + 1];
    char device_id[KZ_SIGN_ID_LEN + 1];
    uint8_t pub[65];               /* uncompressed P-256 point, 0x04 || X || Y */
    uint32_t issued_at;
} kz_cert_record_t;

typedef struct {
    int (*sha256)(const uint8_t *data, size_t len, uint8_t out[32]);                      /* 0 = ok */
    int (*verify)(const uint8_t pub[65], const uint8_t hash[32], const uint8_t sig[64]);  /* 0 = valid */
    int (*sign)(void *key, const uint8_t hash[32], uint8_t sig[64]);                      /* 0 = ok; may be NULL */
} kz_sign_crypto_t;

typedef enum {
    KZ_SIGN_OK = 0,
    KZ_SIGN_UNSIGNED,          /* record, signature or certificate missing */
    KZ_SIGN_BAD_CERTIFICATE,   /* malformed, wrong CA, or does not name the record's device */
    KZ_SIGN_BAD_SIGNATURE,     /* malformed record or signature does not verify */
    KZ_SIGN_HASH_MISMATCH,     /* bytes received do not hash to the signed value or size */
    KZ_SIGN_WRONG_GAME,        /* game code or chip type differs from the inserted cartridge */
} kz_sign_err_t;

/* The names sent to the app: "unsigned", "bad_certificate", "bad_signature", "hash_mismatch", "wrong_game". */
const char *kz_sign_err_name(kz_sign_err_t e);

/* Strict base64url, no padding: 86 chars <-> 64 bytes, trailing bits zero. 0 = ok. */
int kz_sign_sig_decode(const char *s, uint8_t out[64]);
void kz_sign_sig_encode(const uint8_t in[64], char out[KZ_SIGN_SIG_B64_LEN + 1]);

/* Device id = first 8 bytes of SHA-256(pub), 16 lowercase hex. 0 = ok. */
int kz_sign_device_id(const uint8_t pub[65], const kz_sign_crypto_t *c, char out[KZ_SIGN_ID_LEN + 1]);

/* Build the exact signed bytes. Return the length, or 0 if a field is invalid or cap is too small. No NUL is counted. */
size_t kz_save_record_build(const kz_save_record_t *r, char *buf, size_t cap);
size_t kz_cert_record_build(const kz_cert_record_t *r, char *buf, size_t cap);

/* Strict parse of the exact bytes (every line, in order, each ending in \n, nothing after). Returns false on any deviation. */
bool kz_save_record_parse(const char *buf, size_t len, kz_save_record_t *out);
bool kz_cert_record_parse(const char *buf, size_t len, kz_cert_record_t *out);

/* Sign a built record with the device or CA key. 0 = ok. */
int kz_sign_record(const char *rec, size_t len, const kz_sign_crypto_t *c, void *key, char sig_b64[KZ_SIGN_SIG_B64_LEN + 1]);

/* Verify a certificate against the CA public key; fills *cert. */
kz_sign_err_t kz_cert_verify(const char *cert, size_t cert_len, const char *cert_sig,
                             const uint8_t ca_pub[65], const kz_sign_crypto_t *c, kz_cert_record_t *cert_out);

/* Everything the adapter checks before it writes a restore, in this order: certificate against the CA, the certificate names the
 * record's device, signature, hash and size against the received bytes, then game and chip against the cartridge.
 * `cart_game` is the 4-char code of the inserted cartridge. Nothing is written by this function. */
kz_sign_err_t kz_sign_restore_check(const char *rec, size_t rec_len, const char *rec_sig,
                                    const char *cert, size_t cert_len, const char *cert_sig,
                                    const uint8_t ca_pub[65],
                                    const uint8_t *data, size_t data_len,
                                    const char *cart_game, const char *cart_chip,
                                    const kz_sign_crypto_t *c);

/* Device backend over PSA (mbedTLS). `key` for sign is a psa_key_id_t* naming a key held by the caller's key store. */
extern const kz_sign_crypto_t kz_sign_psa_crypto;
