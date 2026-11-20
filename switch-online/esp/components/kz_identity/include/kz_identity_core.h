#pragma once
/* Portable decision logic for kz_identity (no ESP-IDF headers; crypto supplied by the caller, same
 * kz_sign_crypto_t convention as kz_sign.h). This is the part worth testing off-device: does a presented
 * KIZUNA-CERT-V1 certificate actually vouch for MY OWN device key, signed by a CA key id I recognise? */
#include <stddef.h>
#include <stdint.h>
#include "kz_sign.h"

typedef struct {
    const char *id;          /* CA key id, e.g. "ca1" */
    const uint8_t *pub;      /* 65-byte uncompressed P-256 point */
} kz_identity_ca_entry_t;

/* Verifies `cert`/`cert_sig` against whichever entry in `ca_table` its ca_key_id names (KZ_SIGN_BAD_CERTIFICATE
 * if none matches, same as any other certificate failure -- an attacker should not learn which CA ids exist),
 * and that the certified public key is exactly `own_pub`. On KZ_SIGN_OK, *out is the parsed certificate, so the
 * caller can read device_id/issued_at without re-parsing. */
kz_sign_err_t kz_identity_cert_check(const char *cert, size_t cert_len, const char *cert_sig,
                                      const uint8_t own_pub[65],
                                      const kz_identity_ca_entry_t *ca_table, size_t ca_table_len,
                                      const kz_sign_crypto_t *c, kz_cert_record_t *out);
