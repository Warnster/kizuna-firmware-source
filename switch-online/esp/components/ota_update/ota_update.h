#pragma once
#include <stdbool.h>

/* OTA-firmware branch — BLE Wi-Fi provisioning + OTA firmware updates.
 *





 * SAFETY DESIGN: this whole subsystem is gated OFF by default (OTA_UPDATE_ENABLED below), matching
 * this project's existing convention for staged/unproven features (see gba_login_main.c's
 * PIN_WIGGLE_TEST / STRESS_FLUSH_TEST). With the flag at 0, ota_update_provisioning_boot() returns
 * immediately without touching NVS/netif/Wi-Fi/BLE at all — app_main's executed boot path is
 * provably byte-identical to tr-1.61 until this is explicitly turned on for a hardware test.
 *
 * The provisioning manager's Wi-Fi (default STA netif, DHCP-enabled, for the user's HOME network)
 * is a completely different logical Wi-Fi configuration from the existing LDN link's custom
 * static-IP/no-DHCP STA netif (for the SWITCH's ad-hoc group) set up later in app_main. These must
 * never coexist in one boot — this is why ota_update_provisioning_boot() either (a) returns having
 * touched nothing (not provisioned check didn't trigger anything, or already provisioned and no
 * update requested), or (b) runs to completion and calls esp_restart() itself — it NEVER falls
 * through into the caller with the provisioning manager's Wi-Fi/BLE stack still live. */
#ifndef OTA_PUSH_ENABLED   /* a mode that also accepts signed images over Bluetooth sets -DOTA_PUSH_ENABLED=1 (see ota_push.h) */
#define OTA_PUSH_ENABLED 0
#endif
#ifndef OTA_UPDATE_ENABLED   /* a mode enables OTA for itself with -DOTA_UPDATE_ENABLED=1  */
#define OTA_UPDATE_ENABLED 0
#endif

/* Dev override: when 1, an already-provisioned device still comes up advertising KIZUNA_* in update mode
 * and waits for the app on every boot. The production board has no button for this: in normal use the app asks
 * a running adapter, over Bluetooth, to restart into this mode. (Test toggle, 0 in released images.) */
#define OTA_FORCE_UPDATE_MODE 0

/* Call once, at the very start of app_main(), BEFORE any existing GBA/LDN/Wi-Fi bring-up.
 *
 * Enters BLE provisioning mode if the device has no stored Wi-Fi credentials yet, OR
 * enters BLE update mode (— connects with the ALREADY-stored credentials, waits for an
 * OTA-install command) when OTA_FORCE_UPDATE_MODE is set or the app has requested a session. There is no
 * hardware trigger (no button) on the production board.
 *
 * In EITHER case, on completion this function calls esp_restart() and never returns to the caller.
 * If neither condition applies (normal boot, already provisioned, no session requested), returns
 * immediately having touched nothing — the caller's existing boot sequence proceeds unchanged. */
void ota_update_provisioning_boot(void);

/* ---- Runtime-triggered OTA, for modes that already run their own BLE service (save-vault) ------------------
 * The mode's BLE server receives the request, stores it, and restarts. On the NEXT boot ota_update_boot()
 * performs it with BLE off - so Wi-Fi and BLE never run together, and there is no half-updated live state.
 * One-shot actions live in NVS namespace "kz_ota" (key "boot"); the action is CLEARED BEFORE it runs, so a
 * crash or failure can never boot-loop into it again. The outcome of the last attempt is kept in key "res".
 *
 * Call ota_update_boot() first thing in app_main(), AFTER the caller's own nvs_flash_init() and BEFORE any
 * BLE/Wi-Fi bring-up. With no pending action it returns immediately having only read NVS. */
#include <stddef.h>

void ota_update_boot(void);

/* Validate {"version","url","sha256"} (ota_request.h: HTTPS to our server only) and store it as the pending
 * install. Returns true if stored - the caller should then restart (after answering the BLE write).
 * On rejection the reason is recorded as the last result ("failed:bad-url", ...). */
bool ota_update_request_install(const char *json, size_t len);

/* Arm a Wi-Fi provisioning session (the app sets the home network over BLE) for the next boot. The session
 * ends on success or after a timeout, then the device restarts into its normal mode. */
bool ota_update_request_provision(void);

/* Call once the mode has proven itself healthy (e.g. BLE is advertising). Confirms an OTA-installed image so
 * the bootloader does not roll back to the previous slot on the next reset. */
void ota_update_confirm_boot(void);

/* Write the status-characteristic JSON the app reads:
 *   {"fw":"<version>","mode":"<mode>","link":"idle","ota":{"phase":"idle|rebooting|done|failed","error":"..."}}
 * Returns the length written (always NUL-terminated within cap). */
/* The adapter hardware this image runs on, reported as "hw" in the status JSON (e.g. "c3rp" for the two-chip
 * ESP32-C3 + RP2040 board). Set once at boot; NULL/"" omits the field (single-chip C6 adapters). */
void ota_update_set_hw(const char *hw);
size_t ota_update_status_json(char *buf, size_t cap, const char *mode);
/* Record the outcome of an install ("ok:<version>" / "failed:<why>") so the status characteristic reports it
 * after the restart. Used by the Bluetooth push path (ota_push.c). */
void ota_update_record_result(const char *res);

/* Instant mode switch. The other OTA slot often still holds the mode the adapter ran before its last install.
 * ota_update_enable_spare_switch(): this firmware handles {"op":"boot_spare"}; only then does the status report the
 * spare image (as "spare":"<version>"). ota_update_spare_version(): that version if the slot holds a complete image that
 * was never rolled back (hash-checked once, ~0.3 s, then cached), else NULL. ota_update_boot_spare(): make it the boot
 * image and record "ok:<version>"; the caller restarts. ota_update_spare_forget(): the slot is being rewritten. */
void ota_update_enable_spare_switch(void);
const char *ota_update_spare_version(void);
bool ota_update_boot_spare(void);
void ota_update_spare_forget(void);
