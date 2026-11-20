#pragma once
/* Per-adapter identity: an ECDSA P-256 keypair this adapter generates for itself, and the
 * CA-signed certificate for it once enrolled. The private key and the installed certificate are kept in NVS.
 *


















 * Call kz_identity_init() once, after NVS is initialised (same ordering as kz_keys / ota_update_boot()). It
 * generates a keypair on first boot and is a no-op after that. */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
#include "kz_sign.h"

#define KZ_IDENTITY_CERT_MAX 256   /* KIZUNA-CERT-V1 records are well under this (KZ_SIGN_RECORD_MAX = 256) */

/* Generates a device keypair in NVS if one does not exist yet. Idempotent; safe to call every boot. */
esp_err_t kz_identity_init(void);

/* This device's own public key (65-byte uncompressed P-256 point). False if kz_identity_init() has not
 * succeeded yet. */
bool kz_identity_pub(uint8_t out[65]);

/* 16 lowercase hex chars + NUL -- first 8 bytes of SHA-256(own pubkey), same derivation as kz_sign_device_id(). */
bool kz_identity_device_id(char out[KZ_SIGN_ID_LEN + 1]);

/* Signs a pre-hashed (SHA-256) message with the device's own private key. Raw r||s. 0 = ok. Matches the
 * kz_sign_crypto_t.sign signature (key argument unused/may be NULL -- the key lives in this component, not with
 * the caller) so it can be wired in directly wherever a kz_sign_crypto_t is needed for the device's own key. */
int kz_identity_sign(void *unused_key, const uint8_t hash[32], uint8_t sig_out[64]);

/* The kz_sign_crypto_t to pass anywhere this device needs to sign or verify with its own key / the baked-in CA
 * table (sha256 + verify are the same PSA backend as kz_sign_psa_crypto; sign is kz_identity_sign). */
extern const kz_sign_crypto_t kz_identity_crypto;

/* Verifies `cert`/`cert_sig` against the baked-in CA key table (kz_identity_ca_pub.h) and this device's own
 * public key, then stores it in NVS -- the ONLY way a certificate gets onto the device; nothing here can
 * self-sign one. KZ_SIGN_OK on success; otherwise nothing is stored. */
kz_sign_err_t kz_identity_install_cert(const char *cert, const char *cert_sig);

/* The currently-installed certificate, if any, NUL-terminated into caller buffers of at least
 * KZ_IDENTITY_CERT_MAX / (KZ_SIGN_SIG_B64_LEN + 1) bytes. False if none installed. */
bool kz_identity_get_cert(char *cert_out, size_t cert_cap, char *cert_sig_out, size_t cert_sig_cap);

/* Erases the stored key and certificate (factory reset / before reselling, same spirit as kz_keys_clear()). A
 * fresh keypair is generated the next time kz_identity_init() runs. */
esp_err_t kz_identity_clear(void);
