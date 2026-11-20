#include "ota_request.h"
#include <string.h>
#include <stdio.h>
#include "cJSON.h"

const char *ota_request_err_str(ota_req_err_t e)
{
    switch (e) {
    case OTA_REQ_OK:          return "ok";
    case OTA_REQ_BAD_JSON:    return "bad-json";
    case OTA_REQ_BAD_VERSION: return "bad-version";
    case OTA_REQ_BAD_URL:     return "bad-url";
    case OTA_REQ_BAD_SHA:     return "bad-sha";
    }
    return "unknown";
}

static int is_hex(char c) { return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F'); }

static int version_ok(const char *s)
{
    size_t n = strlen(s);
    if (n == 0 || n > OTA_REQ_VERSION_MAX) return 0;
    for (size_t i = 0; i < n; i++) {
        char c = s[i];
        if (!((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '.' || c == '_' || c == '-'))
            return 0;
    }
    return 1;
}

static int url_ok(const char *s)
{
    size_t n = strlen(s), p = strlen(OTA_ALLOWED_URL_PREFIX);
    if (n <= p || n > OTA_REQ_URL_MAX) return 0;                 /* must have something after the prefix */
    if (memcmp(s, OTA_ALLOWED_URL_PREFIX, p) != 0) return 0;
    for (size_t i = 0; i < n; i++)                                /* visible ASCII only: no spaces, control, UTF-8 */
        if ((unsigned char)s[i] < 0x21 || (unsigned char)s[i] > 0x7e) return 0;
    return 1;
}

ota_req_err_t ota_request_check(const ota_request_t *r)
{
    if (!r || !memchr(r->version, 0, sizeof r->version) || !memchr(r->url, 0, sizeof r->url)
        || !memchr(r->sha256, 0, sizeof r->sha256))
        return OTA_REQ_BAD_JSON;
    if (!version_ok(r->version)) return OTA_REQ_BAD_VERSION;
    if (!url_ok(r->url)) return OTA_REQ_BAD_URL;
    if (strlen(r->sha256) != 64) return OTA_REQ_BAD_SHA;
    for (int i = 0; i < 64; i++) if (!is_hex(r->sha256[i])) return OTA_REQ_BAD_SHA;
    return OTA_REQ_OK;
}

ota_req_err_t ota_request_parse(const char *json, size_t len, ota_request_t *out)
{
    if (!json || !out || len == 0 || len > OTA_REQ_INPUT_MAX) return OTA_REQ_BAD_JSON;
    cJSON *root = cJSON_ParseWithLength(json, len);
    if (!root || !cJSON_IsObject(root)) { cJSON_Delete(root); return OTA_REQ_BAD_JSON; }

    const cJSON *v = cJSON_GetObjectItemCaseSensitive(root, "version");
    const cJSON *u = cJSON_GetObjectItemCaseSensitive(root, "url");
    const cJSON *s = cJSON_GetObjectItemCaseSensitive(root, "sha256");
    ota_req_err_t rc = OTA_REQ_OK;

    if (!cJSON_IsString(v) || !v->valuestring || !version_ok(v->valuestring)) rc = OTA_REQ_BAD_VERSION;
    else if (!cJSON_IsString(u) || !u->valuestring || !url_ok(u->valuestring)) rc = OTA_REQ_BAD_URL;
    else if (!cJSON_IsString(s) || !s->valuestring || strlen(s->valuestring) != 64) rc = OTA_REQ_BAD_SHA;
    else for (int i = 0; i < 64; i++) if (!is_hex(s->valuestring[i])) { rc = OTA_REQ_BAD_SHA; break; }

    if (rc == OTA_REQ_OK) {
        memset(out, 0, sizeof(*out));
        strcpy(out->version, v->valuestring);
        strcpy(out->url, u->valuestring);
        for (int i = 0; i < 64; i++) {                            /* normalise to lowercase for the later compare */
            char c = s->valuestring[i];
            out->sha256[i] = (c >= 'A' && c <= 'F') ? (char)(c - 'A' + 'a') : c;
        }
        out->sha256[64] = 0;
    }
    cJSON_Delete(root);
    return rc;
}

static void safe_copy(char *dst, size_t cap, const char *src)
{
    size_t i = 0;
    for (; src && src[i] && i + 1 < cap; i++) {
        unsigned char c = (unsigned char)src[i];
        dst[i] = (c == '"' || c == '\\' || c < 0x20 || c > 0x7e) ? '_' : (char)c;
    }
    dst[i] = 0;
}

size_t ota_status_format_spare(char *buf, size_t cap, const char *fw, const char *mode, const char *phase, const char *err, int push, const char *hw, const char *spare)
{
    if (!buf || cap == 0) return 0;
    char f[48], m[32], p[16], e[40], h[24], hs[40] = "", s[48], ps[72] = "";
    safe_copy(f, sizeof f, fw);
    safe_copy(m, sizeof m, mode);
    safe_copy(p, sizeof p, phase && *phase ? phase : "idle");
    safe_copy(e, sizeof e, err);
    safe_copy(h, sizeof h, hw);
    if (h[0]) snprintf(hs, sizeof hs, ",\"hw\":\"%s\"", h);
    safe_copy(s, sizeof s, spare);
    snprintf(ps, sizeof ps, "%s%s%s%s", push ? ",\"push\":1" : "", s[0] ? ",\"spare\":\"" : "", s, s[0] ? "\"" : "");
    int w;
    if (e[0]) w = snprintf(buf, cap, "{\"fw\":\"%s\",\"mode\":\"%s\"%s,\"link\":\"idle\"%s,\"ota\":{\"phase\":\"%s\",\"error\":\"%s\"}}", f, m, hs, ps, p, e);
    else     w = snprintf(buf, cap, "{\"fw\":\"%s\",\"mode\":\"%s\"%s,\"link\":\"idle\"%s,\"ota\":{\"phase\":\"%s\"}}", f, m, hs, ps, p);
    if (w < 0) { buf[0] = 0; return 0; }
    return (size_t)w < cap ? (size_t)w : cap - 1;
}

size_t ota_status_format_hw(char *buf, size_t cap, const char *fw, const char *mode, const char *phase, const char *err, int push, const char *hw)
{
    return ota_status_format_spare(buf, cap, fw, mode, phase, err, push, hw, NULL);
}

/* Inserts `frag` (a ",\"key\":...\" fragment, no trailing junk) right before the outermost closing '}' an
 * earlier _format_* call already wrote at buf[0..base_len), shifting that '}' and the NUL right. Returns
 * the new length, same truncate-safe contract as every _format_* function here (never overruns `cap`,
 * always NUL-terminated). `base_len` must be the untruncated case (base_len < cap-1) -- callers check
 * that themselves, since what counts as "no room to splice into" differs slightly by caller. */
static size_t splice_before_close(char *buf, size_t cap, size_t base_len, const char *frag, int frag_len)
{
    if (frag_len < 0) return base_len;
    char *close = buf + base_len - 1;   /* the outermost closing brace */
    size_t room = cap - 1 - base_len;   /* bytes free before the NUL's cap-1 slot, after this '}' moves too */
    size_t len = (size_t)frag_len <= room ? (size_t)frag_len : room;
    memmove(close + len, close, 2);     /* shift the '}' and the NUL right by len */
    memcpy(close, frag, len);
    return base_len + len;
}

size_t ota_status_format_wifi(char *buf, size_t cap, const char *fw, const char *mode, const char *phase, const char *err,
                               int push, const char *hw, const char *spare,
                               int wifi_saved, const char *wifi_test, int wifi_rssi, int has_rssi)
{
    size_t base_len = ota_status_format_spare(buf, cap, fw, mode, phase, err, push, hw, spare);
    /* base_len < cap-1 means the full (untruncated) string was written -- _spare's own return convention
     * (see its last line) -- so buf[base_len-1] is guaranteed the outermost closing '}' it always writes.
     * A truncated write has no such guarantee: leave it alone rather than splice into an unknown shape. */
    if (!buf || cap == 0 || base_len == 0 || base_len >= cap - 1) return base_len;

    char t[16] = "", ts[40] = "", rs[24] = "";
    safe_copy(t, sizeof t, wifi_test);
    if (t[0]) snprintf(ts, sizeof ts, ",\"test\":\"%s\"", t);
    if (has_rssi) snprintf(rs, sizeof rs, ",\"rssi\":%d", wifi_rssi);

    char wifi[96];
    int wn = snprintf(wifi, sizeof wifi, ",\"wifi\":{\"saved\":%s%s%s}", wifi_saved ? "true" : "false", ts, rs);
    return splice_before_close(buf, cap, base_len, wifi, wn);
}

size_t ota_status_format_setup(char *buf, size_t cap, const char *fw, const char *mode, const char *phase, const char *err,
                                int push, const char *hw, const char *spare,
                                int wifi_saved, const char *wifi_test, int wifi_rssi, int has_rssi,
                                const char *setup_name)
{
    size_t base_len = ota_status_format_wifi(buf, cap, fw, mode, phase, err, push, hw, spare, wifi_saved, wifi_test, wifi_rssi, has_rssi);
    char s[16] = "";
    safe_copy(s, sizeof s, setup_name);
    if (!s[0] || !buf || cap == 0 || base_len == 0 || base_len >= cap - 1) return base_len;   /* same truncation guard as _wifi */

    char setup[32];
    int sn = snprintf(setup, sizeof setup, ",\"setup\":\"%s\"", s);
    return splice_before_close(buf, cap, base_len, setup, sn);
}

size_t ota_status_format_ex(char *buf, size_t cap, const char *fw, const char *mode, const char *phase, const char *err, int push)
{
    return ota_status_format_hw(buf, cap, fw, mode, phase, err, push, NULL);
}

size_t ota_status_format(char *buf, size_t cap, const char *fw, const char *mode, const char *phase, const char *err)
{
    return ota_status_format_ex(buf, cap, fw, mode, phase, err, 0);
}
