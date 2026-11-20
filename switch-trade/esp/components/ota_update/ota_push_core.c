#include "ota_push_core.h"
#include <stdio.h>
#include <string.h>
#include "cJSON.h"

const char *ota_push_err_reason(ota_push_err_t e)
{
    switch (e) {
        case OTA_PUSH_OK:          return "ok";
        case OTA_PUSH_BAD_REQUEST: return "bad-request";
        case OTA_PUSH_WRONG_MODE:  return "wrong-mode";
        case OTA_PUSH_BAD_SIG:     return "bad-sig";
        case OTA_PUSH_TOO_BIG:     return "too-big";
    }
    return "bad-request";
}

static bool is_lower(char c) { return c >= 'a' && c <= 'z'; }
static bool is_digit(char c) { return c >= '0' && c <= '9'; }

static bool valid_category(const char *s)
{
    size_t n = strlen(s);
    if (n == 0 || n > OTA_PUSH_CAT_MAX) return false;
    if (!(is_lower(s[0]) || is_digit(s[0]))) return false;
    for (size_t i = 1; i < n; i++) if (!(is_lower(s[i]) || is_digit(s[i]) || s[i] == '-')) return false;
    return true;
}

static bool valid_version(const char *s)
{
    size_t n = strlen(s);
    if (n == 0 || n > OTA_PUSH_VER_MAX) return false;
    for (size_t i = 0; i < n; i++) {
        char c = s[i];
        if (!(is_lower(c) || (c >= 'A' && c <= 'Z') || is_digit(c) || c == '.' || c == '_' || c == '-')) return false;
    }
    return true;
}

static bool valid_sha(const char *s)
{
    if (strlen(s) != 64) return false;
    for (int i = 0; i < 64; i++) if (!(is_digit(s[i]) || (s[i] >= 'a' && s[i] <= 'f'))) return false;
    return true;
}

static int b64v(char c)
{
    if (c >= 'A' && c <= 'Z') return c - 'A';
    if (c >= 'a' && c <= 'z') return c - 'a' + 26;
    if (c >= '0' && c <= '9') return c - '0' + 52;
    if (c == '-') return 62;
    if (c == '_') return 63;
    return -1;
}

int ota_push_sig_decode(const char *s, uint8_t out[64])
{
    if (!s || strlen(s) != 86) return -1;
    uint32_t acc = 0; int bits = 0; size_t o = 0;
    for (int i = 0; i < 86; i++) {
        int v = b64v(s[i]);
        if (v < 0) return -1;
        acc = (acc << 6) | (uint32_t)v; bits += 6;
        if (bits >= 8) { bits -= 8; if (o < 64) out[o++] = (uint8_t)(acc >> bits); acc &= (1u << bits) - 1; }
    }
    return (o == 64 && acc == 0) ? 0 : -1;   /* 4 leftover bits must be zero: one canonical encoding per signature */
}

static bool take_str(cJSON *root, const char *key, char *dst, size_t cap)
{
    cJSON *it = cJSON_GetObjectItemCaseSensitive(root, key);
    if (!cJSON_IsString(it) || !it->valuestring || strlen(it->valuestring) >= cap) return false;
    strcpy(dst, it->valuestring);
    return true;
}

ota_push_err_t ota_push_req_parse(const char *json, size_t len, ota_push_req_t *out)
{
    if (!json || !out || len == 0 || len > OTA_PUSH_INPUT_MAX) return OTA_PUSH_BAD_REQUEST;
    memset(out, 0, sizeof *out);
    cJSON *root = cJSON_ParseWithLength(json, len);
    if (!root || !cJSON_IsObject(root)) { cJSON_Delete(root); return OTA_PUSH_BAD_REQUEST; }
    ota_push_err_t rc = OTA_PUSH_BAD_REQUEST;
    cJSON *sz = cJSON_GetObjectItemCaseSensitive(root, "size");
    uint8_t raw[64];
    if (take_str(root, "category", out->category, sizeof out->category) &&
        take_str(root, "version", out->version, sizeof out->version) &&
        take_str(root, "sha256", out->sha256, sizeof out->sha256) &&
        take_str(root, "sig", out->sig, sizeof out->sig) &&
        cJSON_IsNumber(sz) && sz->valuedouble >= 1 && sz->valuedouble <= 2147483647.0 &&
        sz->valuedouble == (double)(uint32_t)sz->valuedouble &&
        valid_category(out->category) && valid_version(out->version) && valid_sha(out->sha256) &&
        ota_push_sig_decode(out->sig, raw) == 0) {
        out->size = (uint32_t)sz->valuedouble;
        rc = OTA_PUSH_OK;
    }
    cJSON_Delete(root);
    if (rc != OTA_PUSH_OK) memset(out, 0, sizeof *out);
    return rc;
}

size_t ota_push_record_build(const ota_push_req_t *r, char *buf, size_t cap)
{
    int w = snprintf(buf, cap, "KIZUNA-FW-V1\n%s\n%s\n%s\n%u\n", r->category, r->version, r->sha256, (unsigned)r->size);
    return (w > 0 && (size_t)w < cap) ? (size_t)w : 0;
}

ota_push_err_t ota_push_authorise(const ota_push_req_t *r, size_t max_size,
                                  const uint8_t pub[65], bool provisioned, const ota_push_crypto_t *crypto)
{
    if (!r || !pub || !crypto || !crypto->sha256 || !crypto->verify) return OTA_PUSH_BAD_REQUEST;
    /* No signing key provisioned (or an all-zero one): refuse everything. */
    uint8_t any = 0; for (int i = 0; i < 65; i++) any |= pub[i];
    if (!provisioned || any == 0 || pub[0] != 0x04) return OTA_PUSH_BAD_SIG;
    if (r->size == 0 || r->size > max_size) return OTA_PUSH_TOO_BIG;
    uint8_t sig[64];
    if (ota_push_sig_decode(r->sig, sig) != 0) return OTA_PUSH_BAD_REQUEST;
    char rec[160];
    size_t n = ota_push_record_build(r, rec, sizeof rec);
    if (n == 0) return OTA_PUSH_BAD_REQUEST;
    uint8_t h[32];
    if (crypto->sha256((const uint8_t *)rec, n, h) != 0) return OTA_PUSH_BAD_SIG;
    return crypto->verify(pub, h, sig) == 0 ? OTA_PUSH_OK : OTA_PUSH_BAD_SIG;
}

bool ota_push_ack_due(uint32_t prev, uint32_t now, uint32_t total, uint32_t step)
{
    if (step == 0 || now <= prev) return false;
    if (now >= total) return prev < total;
    return (now / step) > (prev / step);
}
