#pragma once
/* Bluetooth firmware push: the pure-C decision logic (no ESP-IDF headers; host-tested in host_tests/).
 *
 * The BLE service has no pairing, so ANY device in range can try to push an image. An image is therefore
 * accepted only if the writer presents a record signed with OUR private key:
 *
 *     "KIZUNA-FW-V1\n" category "\n" version "\n" sha256_hex "\n" size "\n"      (ECDSA P-256 / SHA-256)
 *
 * The signature binds the image hash and size, so after the stream the device compares the hash it computed
 * with the signed one. The public key is baked in (fw_signing_pub.h); while that key is the all-zero
 * placeholder every push is refused. */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define OTA_PUSH_CAT_MAX 31
#define OTA_PUSH_VER_MAX 31
#define OTA_PUSH_INPUT_MAX 512

typedef struct {
    char category[OTA_PUSH_CAT_MAX + 1];
    char version[OTA_PUSH_VER_MAX + 1];
    char sha256[65];      /* 64 lowercase hex */
    uint32_t size;        /* image bytes */
    char sig[87];         /* 86 base64url chars: raw r||s */
} ota_push_req_t;

typedef enum {
    OTA_PUSH_OK = 0,
    OTA_PUSH_BAD_REQUEST,      /* malformed JSON / field out of range */
    OTA_PUSH_WRONG_MODE,       /* unused since (see ota_push_authorise); kept so the reason code stays defined */
    OTA_PUSH_BAD_SIG,          /* signature does not verify, or no signing key is provisioned */
    OTA_PUSH_TOO_BIG,          /* does not fit the update slot */
} ota_push_err_t;

/* The code sent to the app in {"ev":"fw_error","reason":..} */
const char *ota_push_err_reason(ota_push_err_t e);

/* Parse + validate a {"op":"fw_begin",...} control payload. */
ota_push_err_t ota_push_req_parse(const char *json, size_t len, ota_push_req_t *out);

/* The exact bytes that are signed. Returns the length, or 0 if cap is too small. */
size_t ota_push_record_build(const ota_push_req_t *r, char *buf, size_t cap);

/* Strict base64url (no padding, 86 chars -> 64 bytes, trailing bits zero). 0 = ok. */
int ota_push_sig_decode(const char *s, uint8_t out[64]);

typedef struct {
    int (*sha256)(const uint8_t *data, size_t len, uint8_t out[32]);                       /* 0 = ok */
    int (*verify)(const uint8_t pub[65], const uint8_t hash[32], const uint8_t sig[64]);   /* 0 = valid */
} ota_push_crypto_t;

/* Decide whether this request may start. `max_size` is the slot size. There is deliberately NO check against the mode
 * the adapter is running now: pushing a different mode's image is how the app SWITCHES modes. What an image is for is
 * pinned by the signed record (the category is part of the signed bytes), so only an image we published for that
 * category can ever pass. */
ota_push_err_t ota_push_authorise(const ota_push_req_t *r, size_t max_size,
                                  const uint8_t pub[65], bool provisioned, const ota_push_crypto_t *crypto);

/* Acknowledge every `step` bytes and at completion. */
bool ota_push_ack_due(uint32_t prev_received, uint32_t received, uint32_t total, uint32_t step);
