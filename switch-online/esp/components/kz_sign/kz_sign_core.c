#include "kz_sign.h"
#include <string.h>

static const char B64[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789-_";

const char *kz_sign_err_name(kz_sign_err_t e)
{
    switch (e) {
    case KZ_SIGN_OK: return "ok";
    case KZ_SIGN_UNSIGNED: return "unsigned";
    case KZ_SIGN_BAD_CERTIFICATE: return "bad_certificate";
    case KZ_SIGN_BAD_SIGNATURE: return "bad_signature";
    case KZ_SIGN_HASH_MISMATCH: return "hash_mismatch";
    case KZ_SIGN_WRONG_GAME: return "wrong_game";
    }
    return "bad_signature";
}

static int b64val(char c)
{
    const char *p = c ? strchr(B64, c) : NULL;
    return p ? (int)(p - B64) : -1;
}

int kz_sign_sig_decode(const char *s, uint8_t out[64])
{
    if (!s || strlen(s) != KZ_SIGN_SIG_B64_LEN) return -1;
    size_t o = 0;
    for (size_t i = 0; i < KZ_SIGN_SIG_B64_LEN; i += 4) {
        int n = (KZ_SIGN_SIG_B64_LEN - i >= 4) ? 4 : 2;
        uint32_t v = 0;
        for (int j = 0; j < n; j++) {
            int d = b64val(s[i + j]);
            if (d < 0) return -1;
            v = (v << 6) | (uint32_t)d;
        }
        if (n == 4) { out[o++] = v >> 16; out[o++] = v >> 8; out[o++] = v; }
        else {
            if (v & 0xF) return -1;
            out[o++] = v >> 4;
        }
    }
    return 0;
}

void kz_sign_sig_encode(const uint8_t in[64], char out[KZ_SIGN_SIG_B64_LEN + 1])
{
    size_t o = 0;
    for (size_t i = 0; i < 64; i += 3) {
        uint32_t v = (uint32_t)in[i] << 16;
        size_t n = 64 - i >= 3 ? 3 : 64 - i;
        if (n > 1) v |= (uint32_t)in[i + 1] << 8;
        if (n > 2) v |= in[i + 2];
        out[o++] = B64[(v >> 18) & 63];
        out[o++] = B64[(v >> 12) & 63];
        if (n > 1) out[o++] = B64[(v >> 6) & 63];
        if (n > 2) out[o++] = B64[v & 63];
    }
    out[o] = 0;
}

static const char HEX[] = "0123456789abcdef";

int kz_sign_device_id(const uint8_t pub[65], const kz_sign_crypto_t *c, char out[KZ_SIGN_ID_LEN + 1])
{
    uint8_t h[32];
    if (!c || !c->sha256 || c->sha256(pub, 65, h) != 0) return -1;
    for (int i = 0; i < 8; i++) { out[2 * i] = HEX[h[i] >> 4]; out[2 * i + 1] = HEX[h[i] & 15]; }
    out[KZ_SIGN_ID_LEN] = 0;
    return 0;
}

static bool is_hex(const char *s, size_t n)
{
    for (size_t i = 0; i < n; i++) if (!((s[i] >= '0' && s[i] <= '9') || (s[i] >= 'a' && s[i] <= 'f'))) return false;
    return true;
}

static bool is_token(const char *s)
{
    size_t n = strlen(s);
    if (n == 0 || n > KZ_SIGN_TOKEN_MAX) return false;
    for (size_t i = 0; i < n; i++) {
        char c = s[i];
        if (!((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '_' || c == '-')) return false;
    }
    return true;
}

static bool is_game(const char *s)
{
    if (strlen(s) != 4) return false;
    if (!strcmp(s, "----")) return true;
    for (int i = 0; i < 4; i++) if (!((s[i] >= 'A' && s[i] <= 'Z') || (s[i] >= '0' && s[i] <= '9'))) return false;
    return true;
}

static bool is_id(const char *s) { return strlen(s) == KZ_SIGN_ID_LEN && is_hex(s, KZ_SIGN_ID_LEN); }

static size_t put(char *buf, size_t cap, size_t at, const char *s)
{
    size_t n = strlen(s);
    if (at + n > cap) return (size_t)-1;
    memcpy(buf + at, s, n);
    return at + n;
}

static size_t put_u32(char *buf, size_t cap, size_t at, uint32_t v)
{
    char t[11]; int n = 0;
    do { t[n++] = (char)('0' + v % 10); v /= 10; } while (v);
    if (at + (size_t)n > cap) return (size_t)-1;
    while (n) buf[at++] = t[--n];
    return at;
}

#define LINE(expr) do { at = (expr); if (at == (size_t)-1) return 0; at = put(buf, cap, at, "\n"); if (at == (size_t)-1) return 0; } while (0)

size_t kz_save_record_build(const kz_save_record_t *r, char *buf, size_t cap)
{
    if (!r || !buf || !is_id(r->device_id) || !is_game(r->game) || !is_token(r->chip) || !is_token(r->slot) ||
        strlen(r->sha256) != 64 || !is_hex(r->sha256, 64)) return 0;
    size_t at = 0;
    LINE(put(buf, cap, at, KZ_SIGN_TAG_SAVE));
    LINE(put(buf, cap, at, r->device_id));
    LINE(put(buf, cap, at, r->game));
    LINE(put(buf, cap, at, r->chip));
    LINE(put_u32(buf, cap, at, r->size));
    LINE(put(buf, cap, at, r->sha256));
    LINE(put_u32(buf, cap, at, r->ts));
    LINE(put(buf, cap, at, r->slot));
    return at;
}

size_t kz_cert_record_build(const kz_cert_record_t *r, char *buf, size_t cap)
{
    if (!r || !buf || !is_token(r->ca_key_id) || !is_id(r->device_id) || r->pub[0] != 0x04) return 0;
    char hex[131];
    for (int i = 0; i < 65; i++) { hex[2 * i] = HEX[r->pub[i] >> 4]; hex[2 * i + 1] = HEX[r->pub[i] & 15]; }
    hex[130] = 0;
    size_t at = 0;
    LINE(put(buf, cap, at, KZ_SIGN_TAG_CERT));
    LINE(put(buf, cap, at, r->ca_key_id));
    LINE(put(buf, cap, at, r->device_id));
    LINE(put(buf, cap, at, hex));
    LINE(put_u32(buf, cap, at, r->issued_at));
    return at;
}

/* Split into exactly `n` lines, each terminated by \n, with nothing after the last. Lines are copied into
 * fixed slots and must fit; this rejects embedded NULs and over-long fields without reading past len. */
#define LINE_MAX 140
static bool split(const char *buf, size_t len, char lines[][LINE_MAX], int n)
{
    if (!buf || len == 0 || len > KZ_SIGN_RECORD_MAX) return false;
    size_t at = 0;
    for (int i = 0; i < n; i++) {
        size_t s = at;
        while (at < len && buf[at] != '\n') at++;
        if (at >= len || at - s >= LINE_MAX) return false;
        memcpy(lines[i], buf + s, at - s);
        lines[i][at - s] = 0;
        if (strlen(lines[i]) != at - s) return false;
        at++;
    }
    return at == len;
}

static bool parse_u32(const char *s, uint32_t *out)
{
    size_t n = strlen(s);
    if (n == 0 || n > 10 || (n > 1 && s[0] == '0')) return false;
    uint64_t v = 0;
    for (size_t i = 0; i < n; i++) {
        if (s[i] < '0' || s[i] > '9') return false;
        v = v * 10 + (uint64_t)(s[i] - '0');
    }
    if (v > 0xFFFFFFFFu) return false;
    *out = (uint32_t)v;
    return true;
}

bool kz_save_record_parse(const char *buf, size_t len, kz_save_record_t *out)
{
    char l[8][LINE_MAX];
    kz_save_record_t r;
    if (!out || !split(buf, len, l, 8) || strcmp(l[0], KZ_SIGN_TAG_SAVE)) return false;
    if (!is_id(l[1]) || !is_game(l[2]) || !is_token(l[3]) || !is_token(l[7])) return false;
    if (strlen(l[5]) != 64 || !is_hex(l[5], 64)) return false;
    memset(&r, 0, sizeof r);
    strcpy(r.device_id, l[1]); strcpy(r.game, l[2]); strcpy(r.chip, l[3]); strcpy(r.sha256, l[5]); strcpy(r.slot, l[7]);
    if (!parse_u32(l[4], &r.size) || !parse_u32(l[6], &r.ts)) return false;
    *out = r;
    return true;
}

bool kz_cert_record_parse(const char *buf, size_t len, kz_cert_record_t *out)
{
    char l[5][LINE_MAX];
    kz_cert_record_t r;
    if (!out || !split(buf, len, l, 5) || strcmp(l[0], KZ_SIGN_TAG_CERT)) return false;
    if (!is_token(l[1]) || !is_id(l[2]) || strlen(l[3]) != 130 || !is_hex(l[3], 130)) return false;
    memset(&r, 0, sizeof r);
    strcpy(r.ca_key_id, l[1]); strcpy(r.device_id, l[2]);
    for (int i = 0; i < 65; i++) {
        int a = (int)(strchr(HEX, l[3][2 * i]) - HEX), b = (int)(strchr(HEX, l[3][2 * i + 1]) - HEX);
        r.pub[i] = (uint8_t)(a << 4 | b);
    }
    if (r.pub[0] != 0x04 || !parse_u32(l[4], &r.issued_at)) return false;
    *out = r;
    return true;
}

int kz_sign_record(const char *rec, size_t len, const kz_sign_crypto_t *c, void *key, char sig_b64[KZ_SIGN_SIG_B64_LEN + 1])
{
    uint8_t h[32], sig[64];
    if (!rec || !c || !c->sha256 || !c->sign) return -1;
    if (c->sha256((const uint8_t *)rec, len, h) != 0 || c->sign(key, h, sig) != 0) return -1;
    kz_sign_sig_encode(sig, sig_b64);
    return 0;
}

static bool sig_ok(const char *rec, size_t len, const char *sig_b64, const uint8_t pub[65], const kz_sign_crypto_t *c)
{
    uint8_t h[32], sig[64];
    if (!c || !c->sha256 || !c->verify) return false;
    if (kz_sign_sig_decode(sig_b64, sig) != 0) return false;
    if (c->sha256((const uint8_t *)rec, len, h) != 0) return false;
    return c->verify(pub, h, sig) == 0;
}

kz_sign_err_t kz_cert_verify(const char *cert, size_t cert_len, const char *cert_sig,
                             const uint8_t ca_pub[65], const kz_sign_crypto_t *c, kz_cert_record_t *cert_out)
{
    kz_cert_record_t cr;
    char id[KZ_SIGN_ID_LEN + 1];
    if (!cert || !cert_sig || cert_len == 0) return KZ_SIGN_UNSIGNED;
    if (!kz_cert_record_parse(cert, cert_len, &cr)) return KZ_SIGN_BAD_CERTIFICATE;
    if (!sig_ok(cert, cert_len, cert_sig, ca_pub, c)) return KZ_SIGN_BAD_CERTIFICATE;
    if (kz_sign_device_id(cr.pub, c, id) != 0 || strcmp(id, cr.device_id)) return KZ_SIGN_BAD_CERTIFICATE;
    if (cert_out) *cert_out = cr;
    return KZ_SIGN_OK;
}

kz_sign_err_t kz_sign_restore_check(const char *rec, size_t rec_len, const char *rec_sig,
                                    const char *cert, size_t cert_len, const char *cert_sig,
                                    const uint8_t ca_pub[65],
                                    const uint8_t *data, size_t data_len,
                                    const char *cart_game, const char *cart_chip,
                                    const kz_sign_crypto_t *c)
{
    kz_cert_record_t cr;
    kz_save_record_t sr;
    uint8_t h[32];
    char hex[65];
    if (!rec || !rec_sig || rec_len == 0 || !cert || !cert_sig) return KZ_SIGN_UNSIGNED;
    if (!c || !c->sha256 || !c->verify || !ca_pub) return KZ_SIGN_BAD_CERTIFICATE;
    kz_sign_err_t e = kz_cert_verify(cert, cert_len, cert_sig, ca_pub, c, &cr);
    if (e != KZ_SIGN_OK) return e;
    if (!kz_save_record_parse(rec, rec_len, &sr)) return KZ_SIGN_BAD_SIGNATURE;
    if (strcmp(sr.device_id, cr.device_id)) return KZ_SIGN_BAD_CERTIFICATE;
    if (!sig_ok(rec, rec_len, rec_sig, cr.pub, c)) return KZ_SIGN_BAD_SIGNATURE;
    if (data_len != sr.size) return KZ_SIGN_HASH_MISMATCH;
    if (c->sha256(data, data_len, h) != 0) return KZ_SIGN_HASH_MISMATCH;
    for (int i = 0; i < 32; i++) { hex[2 * i] = HEX[h[i] >> 4]; hex[2 * i + 1] = HEX[h[i] & 15]; }
    hex[64] = 0;
    if (strcmp(hex, sr.sha256)) return KZ_SIGN_HASH_MISMATCH;
    if (!cart_game || !cart_chip || strcmp(sr.game, cart_game) || strcmp(sr.chip, cart_chip)) return KZ_SIGN_WRONG_GAME;
    return KZ_SIGN_OK;
}
