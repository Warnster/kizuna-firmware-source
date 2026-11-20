#pragma once
/* Minimal Kizuna BLE service for a mode that has no data protocol of its own.
 *
 *   service 4b494e4a-0000-...   (the app scans for it and the KIZUNA_ name prefix)
 *     ...-0001  status   read + notify   {"fw","mode","link","push"?,"ota":{...}}   (ota_update_status_json)
 *     ...-0002  OTA      write           {"version","url","sha256"} -> stored, adapter restarts and installs over Wi-Fi
 *     ...-0003  control  write + notify  {"op":"wifi_setup"} -> restart into the Wi-Fi setup session
 *                                         {"op":"fw_begin",...} / {"op":"fw_abort"} -> signed image push
 *                                         {"op":"identity",...} / {"op":"install_cert",...} -> device identity
 *                                         / certificate (kz_identity.h)
 *     ...-0004  data     write(+no rsp)  raw bytes for the mode's own transfers (kz_mode_ble_set_handlers)
 *     ...-0005  image    write(+no rsp)  raw image bytes for a Bluetooth push
 *
 * Call after NVS init and ota_update_boot(); it brings up NimBLE and advertises `device_name` ("KIZUNA_xx").
 * `mode` is this firmware's category (e.g. "ds-online"): reported in the status and required of a pushed image. */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

esp_err_t kz_mode_ble_start(const char *mode, const char *device_name);

/* Optional mode protocol, set BEFORE kz_mode_ble_start. Both run on the NimBLE host task: copy what
 * you need and defer real work to your own task.
 *   op    control writes whose "op" the base does not handle; return true if the mode took it
 *   data  writes to the data characteristic (...-0004) */
typedef bool (*kz_mode_ble_op_fn)(const char *json, size_t len);
typedef void (*kz_mode_ble_data_fn)(const uint8_t *data, size_t len);
void kz_mode_ble_set_handlers(kz_mode_ble_op_fn op, kz_mode_ble_data_fn data);

/* Send a {"ev":...} JSON notification on the control characteristic (no-op when nobody is connected). */
void kz_mode_ble_notify(const char *json);

/* Data characteristic notifications (...-0004 now also notifies): one attempt, non-blocking; false when nobody is
 * connected or the mbuf pool is momentarily empty. For per-frame streams (the relay engine). */
bool kz_mode_ble_notify_data(const void *data, size_t len);
bool kz_mode_ble_connected(void);
void kz_mode_ble_link_info(uint16_t *itvl_1p25ms, uint16_t *mtu);
/* The connection parameters requested from the central right after connecting (default 12..24 = 15..30 ms, latency 0, 4 s supervision
 * timeout) and re-requested once if it keeps a longer interval. Units: 1.25 ms (interval), 10 ms (timeout). Applies to the live link too. */
void kz_mode_ble_set_conn_pref(uint16_t itvl_min, uint16_t itvl_max, uint16_t latency, uint16_t supervision_timeout);   /* 0 when not connected / unknown */

/* Last advertising start result (0 = ok, -1 = never tried) and whether the controller is advertising right now. */
int  kz_mode_ble_adv_rc(void);
bool kz_mode_ble_adv_active(void);

/* Optional: while busy() is true the OTA triggers (OTA install, wifi_setup, fw_begin) are refused with an error event instead of restarting the
 * adapter under a live session. NULL (default) = never busy, i.e. the behaviour every other mode has. */
void kz_mode_ble_set_busy_fn(bool (*busy)(void));

/* Optional, for a Bluetooth image push (fw_begin): `lend()` runs once the signed request is accepted, just before the
 * push takes its ~30 KB, so a mode can give back a boot-time reserve; `failed()` runs if that push then fails or is
 * aborted (a successful push restarts into the new image). A mode that lends a reserve restarts in failed() to take it
 * back on a clean heap. NULL (default) = nothing to lend. */
void kz_mode_ble_set_push_fns(void (*lend)(void), void (*failed)(void));

/* Stop/resume advertising and drop the link while the mode needs the shared radio to itself. */
void kz_mode_ble_set_advertising(bool on);

/* Full stop and restart: kz_mode_ble_stop() disconnects, stops advertising and deinitialises the NimBLE
 * host and controller, returning their heap (~52 KB on the ESP32-C3); kz_mode_ble_start() brings it back (same handlers).
 * For a mode whose session cannot share the C3's RAM with Bluetooth (the local GBA<->Switch trade). Idempotent. */
esp_err_t kz_mode_ble_stop(void);
bool kz_mode_ble_running(void);
