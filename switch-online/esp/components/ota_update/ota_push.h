#pragma once
/* Bluetooth firmware push (ESP side). A phone streams an image over BLE; we write it to the spare OTA slot.
 * It is installed only if the request carries a record signed with our key (ota_push_core.h), and only after the
 * SHA-256 of what we received matches the signed hash. Compiled in only when OTA_PUSH_ENABLED=1 (save-vault).
 *
 *   begin()  worker task: validates + verifies the signature, erases the slot, emits {"ev":"fw_ready"}
 *   feed()   BLE host task: copies image bytes into a stream buffer (never touches flash)
 *   writer   own task: drains the buffer to flash, emits {"ev":"fw_ack","done":N} every 8 KB, then verifies,
 *            switches the boot partition, records "ok:<version>" and restarts.
 * Any failure aborts the slot write, records "failed:<reason>", emits {"ev":"fw_error","reason":...}.
 * 20 s of silence while receiving aborts with "timeout". */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "ota_update.h"

typedef struct {
    void (*emit)(const char *json);      /* send a control-characteristic event (may be called from the writer task) */
    void (*finished)(bool restarting);   /* the session ended; restarting == false lets the caller release its locks */
    /* Optional. Called on the begin() task once the request is authorised, just before the push takes its memory
     * (~17 KB stream buffer, writer task and chunk, the slot handle): a mode can give back a reserve here. Once it has
     * run, the session always ends in finished(): begin() failing after it calls finished(false) too. */
    void (*prepare)(void);
} ota_push_ops_t;

/* Blocking (erases the slot): call from a worker task, not the BLE host task. Returns false if the request was
 * refused (the error event was already emitted and, unless prepare() ran, `finished(false)` is NOT called: the caller
 * still owns its locks).
 * The session is claimed before anything else, so a second begin() during the erase gets "busy". */
bool ota_push_begin(const char *json, size_t len, const char *mode, const ota_push_ops_t *ops);
/* Non-blocking; safe from the BLE host task. Ignored when no push is active. */
void ota_push_feed(const uint8_t *data, size_t len);
/* {"op":"fw_abort"}: the writer task notices within ~0.5 s. */
void ota_push_abort(void);
bool ota_push_active(void);
