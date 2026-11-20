/*
 * The user's home Wi-Fi, shared by every Kizuna mode.
 *
 * Kept in Kizuna's own NVS namespace ("kz_wifi"), NOT the Wi-Fi driver's ("nvs.net80211"): modes that use the
 * radio for something else (Switch LDN, the DS access point) write the driver's store, so a network kept only
 * there would be lost on a mode switch. `nvs` sits at the same offset in every mode and no flash path erases it
 * (USB flash, OTA and Bluetooth push all write app/table regions only), so the network survives reflashes and
 * mode changes. kz_wifi_connect() runs the driver with WIFI_STORAGE_RAM, so it never writes the driver's store.
 *
 * Not thread-safe: call from one task at a time (the boot path / a mode's own Wi-Fi task).
 */
#pragma once
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define KZ_WIFI_SSID_MAX 32   /* bytes, excluding the NUL (802.11 limit) */
#define KZ_WIFI_PASS_MAX 64

/* Copy the stored network into ssid/pass (NUL-terminated). Returns true only if one is stored. */
bool kz_wifi_get(char ssid[KZ_WIFI_SSID_MAX + 1], char pass[KZ_WIFI_PASS_MAX + 1]);
bool kz_wifi_configured(void);
esp_err_t kz_wifi_set(const char *ssid, const char *pass);
esp_err_t kz_wifi_clear(void);

/* Copy the network the Wi-Fi driver holds into the Kizuna store. overwrite=false only fills an empty store (a
 * device provisioned before kz_wifi existed); overwrite=true is for right after the user set a new network.
 * Requires esp_wifi_init() to have run. Returns true if something was stored. */
bool kz_wifi_adopt_driver_config(bool overwrite);

/* Bring the station up on the stored network and wait for an IP. Initialises netif / event loop / driver as
 * needed. ESP_ERR_NOT_FOUND = no network stored; ESP_ERR_TIMEOUT = couldn't join or get an address in time
 * (always call kz_wifi_stop() afterwards either way). */
esp_err_t kz_wifi_connect(int timeout_ms);

/* Disconnect and de-initialise the driver, returning its RAM (~44 KB on the C6). Safe to call twice. */
void kz_wifi_stop(void);

#ifdef __cplusplus
}
#endif
