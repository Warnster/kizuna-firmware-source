#include "ota_push.h"

#if OTA_UPDATE_ENABLED && OTA_PUSH_ENABLED

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/stream_buffer.h"
#include "freertos/task.h"
#include "psa/crypto.h"
#include "fw_signing_pub.h"
#include "ota_push_core.h"

static const char *TAG = "ota_push";

#define SB_BYTES      (17 * 1024)   /* the app never has more than 16 KB unacknowledged; one block of this must be free */
#define WRITE_CHUNK   2048
#define ACK_STEP      8192
#define SILENCE_MS    20000

static portMUX_TYPE s_claim = portMUX_INITIALIZER_UNLOCKED;
static struct {
    volatile bool active;               /* claimed by begin() (before the erase) until teardown */
    volatile bool abort_req;
    volatile const char *fail;          /* set by feed() on a protocol violation */
    ota_push_req_t req;
    ota_push_ops_t ops;
    const esp_partition_t *part;
    esp_ota_handle_t handle;
    psa_hash_operation_t hash;
    StreamBufferHandle_t sb;
    uint32_t fed;                       /* bytes accepted from BLE */
    uint32_t written;                   /* bytes written to flash */
    volatile TickType_t last_rx;
    bool prepared;                      /* ops.prepare() ran: a failed begin() ends in finished(false) */
} S;

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

static const ota_push_crypto_t CRYPTO = { c_sha256, c_verify };

static void emit_error(const char *reason)
{
    char b[64];
    snprintf(b, sizeof b, "{\"ev\":\"fw_error\",\"reason\":\"%s\"}", reason);
    if (S.ops.emit) S.ops.emit(b);
}

static void record_failure(const char *reason)
{
    char r[48];
    snprintf(r, sizeof r, "failed:%s", reason);
    ota_update_record_result(r);
}

bool ota_push_active(void) { return S.active; }
void ota_push_abort(void) { if (S.active) S.abort_req = true; }

void ota_push_feed(const uint8_t *data, size_t len)
{
    if (!S.active || !S.sb || len == 0) return;
    if (S.fed + len > S.req.size) { S.fail = "bad-request"; return; }   /* more bytes than the signed size */
    if (xStreamBufferSend(S.sb, data, len, 0) != len) { S.fail = "write"; return; }   /* app ignored the window */
    S.fed += (uint32_t)len;
    S.last_rx = xTaskGetTickCount();
}

static void teardown(bool restarting, const char *fail_reason)
{
    if (fail_reason) {
        esp_ota_abort(S.handle);
        psa_hash_abort(&S.hash);
        if (strcmp(fail_reason, "aborted") != 0) record_failure(fail_reason);
        emit_error(fail_reason);
        ESP_LOGW(TAG, "push failed: %s (written %u of %u)", fail_reason, (unsigned)S.written, (unsigned)S.req.size);
    }
    StreamBufferHandle_t sb = S.sb;
    S.sb = NULL;
    S.active = false;
    if (sb) vStreamBufferDelete(sb);
    void (*fin)(bool) = S.ops.finished;
    if (fin) fin(restarting);
}

static void writer_task(void *arg)
{
    (void)arg;
    uint8_t *buf = malloc(WRITE_CHUNK);
    const char *fail = buf ? NULL : "write";
    while (!fail && S.written < S.req.size) {
        size_t n = xStreamBufferReceive(S.sb, buf, WRITE_CHUNK, pdMS_TO_TICKS(500));
        if (n > 0) {
            if (esp_ota_write(S.handle, buf, n) != ESP_OK || psa_hash_update(&S.hash, buf, n) != PSA_SUCCESS) { fail = "write"; break; }
            uint32_t prev = S.written;
            S.written += (uint32_t)n;
            if (ota_push_ack_due(prev, S.written, S.req.size, ACK_STEP)) {
                char b[64];
                snprintf(b, sizeof b, "{\"ev\":\"fw_ack\",\"done\":%u}", (unsigned)S.written);
                if (S.ops.emit) S.ops.emit(b);
            }
        }
        if (S.abort_req) { fail = "aborted"; break; }
        if (S.fail) { fail = (const char *)S.fail; break; }
        if ((xTaskGetTickCount() - S.last_rx) > pdMS_TO_TICKS(SILENCE_MS) && S.written < S.req.size) { fail = "timeout"; break; }
    }
    free(buf);

    if (!fail) {
        uint8_t h[32]; size_t hl = 0;
        if (psa_hash_finish(&S.hash, h, sizeof h, &hl) != PSA_SUCCESS || hl != 32) fail = "hash";
        else {
            char hex[65];
            for (int i = 0; i < 32; i++) sprintf(hex + 2 * i, "%02x", h[i]);
            if (strcmp(hex, S.req.sha256) != 0) { fail = "hash"; psa_hash_abort(&S.hash); }
        }
        if (!fail) {
            if (esp_ota_end(S.handle) != ESP_OK) fail = "finish";
            else if (esp_ota_set_boot_partition(S.part) != ESP_OK) fail = "finish";
        }
    }
    if (fail) {
        teardown(false, fail);
    } else {
        char r[48]; snprintf(r, sizeof r, "ok:%s", S.req.version);
        ota_update_record_result(r);
        if (S.ops.emit) S.ops.emit("{\"ev\":\"fw_done\"}");
        ESP_LOGI(TAG, "push verified (%u bytes, sha256 matches the signed record); restarting", (unsigned)S.written);
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_restart();   /* the caller's locks stay held until the restart, like the Wi-Fi OTA trigger */
    }
    vTaskDelete(NULL);
}

/* A refused or failed begin(): record, tell the app, give the session back. */
static bool begin_failed(const char *reason, bool refused)
{
    if (refused) ESP_LOGW(TAG, "push refused: %s", reason);
    else ESP_LOGW(TAG, "push could not start: %s (largest free block %u)", reason,
                  (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
    record_failure(reason);
    emit_error(reason);
    bool fin = S.prepared;
    S.active = false;
    if (fin && S.ops.finished) S.ops.finished(false);
    return false;
}

bool ota_push_begin(const char *json, size_t len, const char *mode, const ota_push_ops_t *ops)
{
    (void)mode;   /* the running mode is intentionally not part of the decision: see ota_push_authorise() */
    if (!ops || !ops->emit) return false;
    /* Claim first. Bench: `active` used to be set only after the multi-second erase, so a second fw_begin
     * in that window wiped S (memset) under the first; its 20 KB stream buffer leaked ("written 0 of 0") and every
     * later push failed to allocate. */
    bool busy;
    taskENTER_CRITICAL(&s_claim);
    busy = S.active;
    if (!busy) { memset(&S, 0, sizeof S); S.active = true; }
    taskEXIT_CRITICAL(&s_claim);
    if (busy) { ops->emit("{\"ev\":\"fw_error\",\"reason\":\"busy\"}"); return false; }
    S.ops = *ops;

    ota_push_req_t req;
    ota_push_err_t e = ota_push_req_parse(json, len, &req);
    const esp_partition_t *part = esp_ota_get_next_update_partition(NULL);
    if (e == OTA_PUSH_OK && !part) e = OTA_PUSH_TOO_BIG;
    if (e == OTA_PUSH_OK) {
        psa_crypto_init();
        e = ota_push_authorise(&req, part->size, KZ_FW_SIGNING_PUB, KZ_FW_SIGNING_PROVISIONED != 0, &CRYPTO);
    }
    if (e != OTA_PUSH_OK) return begin_failed(ota_push_err_reason(e), true);
    S.req = req;
    S.part = part;
    if (S.ops.prepare) { S.prepared = true; S.ops.prepare(); }
    S.sb = xStreamBufferCreate(SB_BYTES, 1);
    if (!S.sb) return begin_failed("write", false);
    ESP_LOGI(TAG, "push accepted: %s %s, %u bytes -> %s; erasing", req.category, req.version, (unsigned)req.size, part->label);
    ota_update_spare_forget();   /* the other mode in that slot is about to be erased */
    if (esp_ota_begin(part, req.size, &S.handle) != ESP_OK || psa_hash_setup(&S.hash, PSA_ALG_SHA_256) != PSA_SUCCESS) {
        vStreamBufferDelete(S.sb); S.sb = NULL;
        return begin_failed("begin", false);
    }
    S.last_rx = xTaskGetTickCount();
    if (xTaskCreate(writer_task, "ota_push", 5120, NULL, 5, NULL) != pdPASS) {
        esp_ota_abort(S.handle); psa_hash_abort(&S.hash);
        vStreamBufferDelete(S.sb); S.sb = NULL;
        return begin_failed("write", false);
    }
    S.ops.emit("{\"ev\":\"fw_ready\"}");
    return true;
}

#else /* push not compiled in */

bool ota_push_begin(const char *json, size_t len, const char *mode, const ota_push_ops_t *ops) { (void)json; (void)len; (void)mode; (void)ops; return false; }
void ota_push_feed(const uint8_t *data, size_t len) { (void)data; (void)len; }
void ota_push_abort(void) { }
bool ota_push_active(void) { return false; }

#endif
