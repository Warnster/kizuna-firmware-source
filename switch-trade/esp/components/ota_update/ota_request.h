#pragma once
/* Validation of an OTA install request received over BLE ({"version","url","sha256"}).
 *
 * The URL must point at the Kizuna firmware server over HTTPS (certificate-verified against the CA bundle).
 *
 * Pure C + cJSON (no ESP-IDF headers). */





#include <stddef.h>

/* Downloads are only accepted from this prefix. Override at build time for a staging server. The trailing
 * path separator matters: "https://updates.kizuna.trading@evil.example/" does not match it. */
#ifndef OTA_ALLOWED_URL_PREFIX
#define OTA_ALLOWED_URL_PREFIX "https://updates.kizuna.trading/v1/fw/"
#endif

#define OTA_REQ_VERSION_MAX 31
#define OTA_REQ_URL_MAX     255
#define OTA_REQ_INPUT_MAX   1024

typedef struct {
    char version[OTA_REQ_VERSION_MAX + 1];
    char url[OTA_REQ_URL_MAX + 1];
    char sha256[65];   /* 64 lowercase hex chars */
} ota_request_t;

typedef enum {
    OTA_REQ_OK = 0,
    OTA_REQ_BAD_JSON,      /* not a JSON object, empty, or oversized */
    OTA_REQ_BAD_VERSION,   /* missing, empty, too long, or characters outside [A-Za-z0-9._-] */
    OTA_REQ_BAD_URL,       /* missing, too long, not on the allowed server, or contains non-printable characters */
    OTA_REQ_BAD_SHA,       /* not exactly 64 hex characters */
} ota_req_err_t;

ota_req_err_t ota_request_parse(const char *json, size_t len, ota_request_t *out);

/* Re-validate an already-parsed request (e.g. one read back from NVS at boot): every field NUL-terminated
 * within its buffer and passing the same rules as ota_request_parse(). */
ota_req_err_t ota_request_check(const ota_request_t *r);
const char *ota_request_err_str(ota_req_err_t e);

/* Format the status-characteristic JSON the app parses:
 *   {"fw":"..","mode":"..","link":"idle","ota":{"phase":"..","error":".."}}   ("error" only when non-empty)
 * Any character that could break the JSON (quote, backslash, control) in fw/mode/phase/err is replaced by '_'.
 * Returns the length written; the output is always NUL-terminated and never overruns cap (truncated JSON is
 * possible only if cap is far too small, and is then still NUL-terminated). */
size_t ota_status_format(char *buf, size_t cap, const char *fw, const char *mode, const char *phase, const char *err);
/* Same, plus "push":1 (between "link" and "ota") when `push` is non-zero: the firmware accepts a signed image over
 * Bluetooth (ota_push.h). Absent otherwise, so the app treats it as unsupported. */
size_t ota_status_format_ex(char *buf, size_t cap, const char *fw, const char *mode, const char *phase, const char *err, int push);
/* As _ex, plus an optional "hw" field (the adapter hardware the image is for, e.g. "c3rp"; NULL/"" = omitted).
 * Lets the app keep a two-chip (ESP32-C3 + RP2040) adapter away from single-chip images. */
size_t ota_status_format_hw(char *buf, size_t cap, const char *fw, const char *mode, const char *phase, const char *err, int push, const char *hw);
/* As _hw, plus an optional "spare" field (after "push"): the version of the complete image in the other OTA slot, which
 * the app can boot instantly with {"op":"boot_spare"}. NULL/"" = omitted (no such image, or the running
 * firmware cannot switch to it). */
size_t ota_status_format_spare(char *buf, size_t cap, const char *fw, const char *mode, const char *phase, const char *err, int push, const char *hw, const char *spare);

/* The adapter's home Wi-Fi, as the app reads it: {"saved":bool} always; "test" (one of ota_request's TEST_* strings)

 * added only once a {"op":"wifi_test"} run has a result, "rssi" only alongside an "ok" result that read
 * one. `test` NULL/"" omits it (no result yet); `has_rssi` false omits "rssi" regardless of `rssi`'s value. */
#define OTA_WIFI_TEST_OK       "ok"         /* joined the network and reached the Kizuna server */
#define OTA_WIFI_TEST_NONE     "none"       /* no network stored -- nothing to test */
#define OTA_WIFI_TEST_WIFI     "wifi"       /* couldn't join the network (wrong password, out of range, off) */
#define OTA_WIFI_TEST_DNS      "dns"        /* joined, but couldn't resolve the Kizuna server's name */
#define OTA_WIFI_TEST_INTERNET "internet"   /* joined and resolved it, but couldn't reach it (firewall / captive portal) */
/* As _spare, plus an optional "wifi" object (after "ota"). */
size_t ota_status_format_wifi(char *buf, size_t cap, const char *fw, const char *mode, const char *phase, const char *err,
                               int push, const char *hw, const char *spare,
                               int wifi_saved, const char *wifi_test, int wifi_rssi, int has_rssi);

/* As _wifi, plus an optional top-level "setup" field: the BLE name this adapter WILL advertise once it
 * restarts into Wi-Fi-setup mode, computable in advance from
 * its own MAC (ota_update.c compute_setup_name) -- lets the app connect straight to it after
 * asking for the restart, instead of re-scanning blind for whatever just appeared. NULL/"" = omitted
 * (firmware too old to report it; the app falls back to scanning, as it always has). */
size_t ota_status_format_setup(char *buf, size_t cap, const char *fw, const char *mode, const char *phase, const char *err,
                                int push, const char *hw, const char *spare,
                                int wifi_saved, const char *wifi_test, int wifi_rssi, int has_rssi,
                                const char *setup_name);
