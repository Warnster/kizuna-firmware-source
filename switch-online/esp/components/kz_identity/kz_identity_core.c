#include "kz_identity_core.h"
#include <string.h>

kz_sign_err_t kz_identity_cert_check(const char *cert, size_t cert_len, const char *cert_sig,
                                      const uint8_t own_pub[65],
                                      const kz_identity_ca_entry_t *ca_table, size_t ca_table_len,
                                      const kz_sign_crypto_t *c, kz_cert_record_t *out)
{
    if (!cert || !cert_sig || cert_len == 0 || !own_pub || !ca_table || !c) return KZ_SIGN_UNSIGNED;

    kz_cert_record_t parsed;
    if (!kz_cert_record_parse(cert, cert_len, &parsed)) return KZ_SIGN_BAD_CERTIFICATE;

    const uint8_t *ca_pub = NULL;
    for (size_t i = 0; i < ca_table_len; i++) {
        if (ca_table[i].id && ca_table[i].pub && !strcmp(ca_table[i].id, parsed.ca_key_id)) { ca_pub = ca_table[i].pub; break; }
    }
    if (!ca_pub) return KZ_SIGN_BAD_CERTIFICATE;   /* unrecognised CA key id */

    kz_cert_record_t verified;
    kz_sign_err_t e = kz_cert_verify(cert, cert_len, cert_sig, ca_pub, c, &verified);
    if (e != KZ_SIGN_OK) return e;

    if (memcmp(verified.pub, own_pub, 65) != 0) return KZ_SIGN_BAD_CERTIFICATE;   /* valid cert, but for a different device */

    if (out) *out = verified;
    return KZ_SIGN_OK;
}
