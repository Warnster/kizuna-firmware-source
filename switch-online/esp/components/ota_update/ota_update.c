/* OTA-firmware branch — BLE Wi-Fi provisioning + OTA update trigger.
 * UNTESTED ON HARDWARE — see ota_update.h. Gated OFF by OTA_UPDATE_ENABLED (0) until proven.
 *
 * Built against ESP-IDF v6.1's "network_provisioning" component -- the in-tree "wifi_provisioning"
 * component used by older IDF versions was renamed/generalized (now covers Wi-Fi + Thread under one
 * API: network_prov_mgr_*, NETWORK_PROV_*, network_prov_scheme_ble), fetched via the IDF component
 * manager (see idf_component.yml). Do not "fix" these names back to the old wifi_prov_mgr_* ones.
 *
 * One BLE session mechanism serves BOTH flows:
 *   - Not yet provisioned -> the app sets Wi-Fi credentials via the standard network_provisioning
 *     endpoints; the manager auto-stops itself once connected (default behavior, no code needed).
 *   - Already provisioned, update requested (the app asks over Bluetooth, which restarts into this mode) -> the app
 *     skips the Wi-Fi-config endpoints and calls the custom "ota-install" endpoint directly; that
 *     handler calls network_prov_mgr_stop_provisioning() itself, since no Wi-Fi-config event would
 *     otherwise fire to end the session (this is the exact use documented in manager.h's
 *     stop_provisioning() doc comment: "calling this API inside an endpoint handler ... will allow
 *     the response to be sent successfully before the underlying protocomm service is stopped").
 * Either way, ota_update_provisioning_boot() ends in esp_restart() and never falls through into the
 * caller's existing LDN Wi-Fi bring-up in the same boot (see ota_update.h for why that matters).
 *
 * run_https_ota() verifies the downloaded image's SHA256 against what the app supplied (via the
 * granular esp_https_ota_begin/perform API + a repurposed decrypt_cb as a raw-chunk observer) and
 * pins ESP-IDF's built-in CA bundle for the HTTPS connection.

 */
#include "ota_update.h"

#if OTA_UPDATE_ENABLED

#include "sdkconfig.h"   /* the guards below must see CONFIG_* before anything else pulls it in */

/* sdkconfig.defaults only applies to a FRESH sdkconfig. A mode that just turned OTA on, built against an old
 * sdkconfig, would otherwise fail deep inside esp_https_ota with a confusing message (or, for rollback, build
 * fine and silently have no rollback). Say what to do instead. */
#ifndef CONFIG_ESP_HTTPS_OTA_DECRYPT_CB
#error "ota_update needs CONFIG_ESP_HTTPS_OTA_DECRYPT_CB=y. Delete this mode's sdkconfig (or run idf.py fullclean) so its sdkconfig.defaults is applied."
#endif
#ifndef CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE
#error "ota_update needs CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=y (a bad OTA image must roll back). Delete this mode's sdkconfig (or run idf.py fullclean) so its sdkconfig.defaults is applied."
#endif

#include <string.h>
#include <strings.h>   /* strcasecmp */
#include <stdio.h>
#include <stdlib.h>    /* malloc, in observe_chunk_cb */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_ota_ops.h"   /* esp_ota_mark_app_valid_cancel_rollback (CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE) */
#include "esp_image_format.h"   /* esp_image_verify: the spare slot's image is complete */
#include "esp_log.h"
#include "esp_err.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "ota_request.h"
#include "cJSON.h"
#include "esp_app_desc.h"
#include "esp_https_ota.h"
#include "esp_crt_bundle.h"
#include "psa/crypto.h"
#include "network_provisioning/manager.h"
#include "network_provisioning/scheme_ble.h"
#include "kz_wifi.h"

static const char *TAG = "ota_update";

/* This project builds with CONFIG_LOG_MAXIMUM_LEVEL=2 (WARN) -- every
 * ESP_LOGI in this file was COMPILED OUT of the binary. A successful provisioning boot therefore
 * printed exactly ZERO bytes before blocking in network_prov_mgr_wait(), which is what the
 * "bug 3 silent hang" actually was (see OTA_PROVISIONING_BOOT_HANG_HANDOFF.md §resolution): the
 * device was correctly waiting for the phone app, invisibly. Progress markers in this file must be
 * printf (the project convention -- the ==== banner, [hb], [join] lines are all printf), never
 * ESP_LOGI. ESP_LOGE/ESP_LOGW survive the log level and stay as-is. */

/* Proof-of-possession for the Wi-Fi setup session (SECURITY_1). */

#define PROV_POP "kizuna-dev-pop"

typedef struct { char version[32]; char url[256]; char sha256[65]; bool pending; } ota_req_t;
static ota_req_t s_req;
/* STA credentials captured at boot, BEFORE network_prov_mgr_init() can reset the driver config --
 * update mode needs them to reconnect to Wi-Fi for the OTA download. */
static wifi_config_t s_boot_sta_cfg;

/* Runtime-triggered OTA state (see ota_update.h). NVS namespace "kz_ota": "boot" u8 = one-shot action,
 * "req" blob = the pending ota_request_t, "res" string = outcome of the last attempt. */
#define KZ_NVS_NS "kz_ota"
enum { BOOT_NONE = 0, BOOT_INSTALL = 1, BOOT_PROVISION = 2 };
static bool s_force_session;          /* enter the provisioning session even if already provisioned */
static volatile int s_session_timeout_s;   /* 0 = wait forever (fresh provisioning); >0 = give up after this long */

static void record_result(const char *res)
{
    nvs_handle_t h;
    if (nvs_open(KZ_NVS_NS, NVS_READWRITE, &h) != ESP_OK) return;   /* NVS not up: nothing we can do */
    nvs_set_str(h, "res", res);
    nvs_commit(h);
    nvs_close(h);
    printf("[ota] result: %s\n", res);
}

static esp_err_t ep_ota_install_handler(uint32_t session_id, const uint8_t *inbuf, ssize_t inlen,
                                         uint8_t **outbuf, ssize_t *outlen, void *priv_data)
{
    (void)session_id; (void)priv_data;
    const char *resp = "{\"ok\":false}";
    cJSON *root = cJSON_ParseWithLength((const char *)inbuf, inlen);
    if (root) {
        cJSON *v = cJSON_GetObjectItem(root, "version");
        cJSON *u = cJSON_GetObjectItem(root, "url");
        cJSON *s = cJSON_GetObjectItem(root, "sha256");
        if (cJSON_IsString(v) && cJSON_IsString(u) && cJSON_IsString(s)) {
            strncpy(s_req.version, v->valuestring, sizeof(s_req.version) - 1);
            strncpy(s_req.url, u->valuestring, sizeof(s_req.url) - 1);
            strncpy(s_req.sha256, s->valuestring, sizeof(s_req.sha256) - 1);
            s_req.pending = true;
            resp = "{\"ok\":true}";
            printf("[ota] install requested: version=%s url=%s\n", s_req.version, s_req.url);
            /* No Wi-Fi-config event fires in the already-provisioned update-mode case (see file
             * header) -- explicitly end the session ourselves (documented pattern, manager.h). */
            network_prov_mgr_stop_provisioning();
        }
        cJSON_Delete(root);
    }
    *outbuf = (uint8_t *)strdup(resp);
    *outlen = (ssize_t)strlen(resp);
    return ESP_OK;
}

static esp_err_t ep_ota_status_handler(uint32_t session_id, const uint8_t *inbuf, ssize_t inlen,
                                        uint8_t **outbuf, ssize_t *outlen, void *priv_data)
{
    (void)session_id; (void)inbuf; (void)inlen; (void)priv_data;
    const esp_app_desc_t *desc = esp_app_get_description();
    char buf[160];
    int n = snprintf(buf, sizeof(buf), "{\"version\":\"%s\",\"state\":\"%s\"}",
                      desc->version, s_req.pending ? "installing" : "idle");
    *outbuf = (uint8_t *)strdup(buf);
    *outlen = (ssize_t)n;
    return ESP_OK;
}

/* Repurposes the OTA "decrypt" hook as a raw-chunk OBSERVER (pass-through, no actual decryption) so
 * every byte esp_https_ota is about to write to flash also gets folded into a running SHA256 first.
 * This runs BEFORE esp_https_ota_finish() ever commits the new partition as the boot target -- see
 * run_https_ota() below for why that ordering is what makes hash verification meaningful here. */
static esp_err_t observe_chunk_cb(decrypt_cb_arg_t *args, void *user_ctx)
{
    psa_hash_operation_t *hash_op = (psa_hash_operation_t *)user_ctx;
    psa_hash_update(hash_op, (const uint8_t *)args->data_in, args->data_in_len);
    args->data_out = malloc(args->data_in_len);
    if (!args->data_out) return ESP_ERR_NO_MEM;
    memcpy(args->data_out, args->data_in, args->data_in_len);   /* pass-through, unmodified */
    args->data_out_len = args->data_in_len;
    return ESP_OK;
}

/* Downloads the OTA image, verifies its SHA256 against s_req.sha256 (from the app's install
 * command), and only THEN commits it as the boot target -- rejecting it leaves the currently
 * running image untouched (the download landed in the INACTIVE ota_0/ota_1 slot; esp_https_ota_abort
 * never flips the boot pointer to it). Uses the granular begin/perform API (not the one-shot
 * esp_https_ota() convenience function) specifically because the one-shot call commits internally
 * before returning, which would make verify-then-decide impossible. Does not return -- reboots
 * either way. Pins ESP-IDF's built-in CA bundle for the HTTPS connection (esp_crt_bundle_attach). */
static void run_https_ota(const char *url)
{
    psa_crypto_init();   /* idempotent if already initialized elsewhere (cport's aes_backend_psa.c) */
    psa_hash_operation_t hash_op = PSA_HASH_OPERATION_INIT;
    psa_hash_setup(&hash_op, PSA_ALG_SHA_256);

    esp_http_client_config_t http_cfg = { .url = url, .crt_bundle_attach = esp_crt_bundle_attach };
    esp_https_ota_config_t ota_cfg = {
        .http_config = &http_cfg,
        .decrypt_cb = observe_chunk_cb,
        .decrypt_user_ctx = &hash_op,
    };

    esp_https_ota_handle_t handle = NULL;
    esp_err_t err = esp_https_ota_begin(&ota_cfg, &handle);
    if (err != ESP_OK) { ESP_LOGE(TAG, "esp_https_ota_begin failed: %s", esp_err_to_name(err)); record_result("failed:begin"); esp_restart(); }

    do { err = esp_https_ota_perform(handle); } while (err == ESP_ERR_HTTPS_OTA_IN_PROGRESS);

    bool got_all = esp_https_ota_is_complete_data_received(handle);
    uint8_t digest[32]; size_t digest_len = 0;
    psa_hash_finish(&hash_op, digest, sizeof(digest), &digest_len);
    char digest_hex[65];
    for (size_t i = 0; i < digest_len; i++) snprintf(digest_hex + i * 2, 3, "%02x", digest[i]);

    bool hash_ok = (strcasecmp(digest_hex, s_req.sha256) == 0);
    if (err == ESP_OK && got_all && hash_ok) {
        err = esp_https_ota_finish(handle);   /* commits: sets this image as the next boot target */
        if (err == ESP_OK) {
            printf("[ota] OTA succeeded (sha256 verified), rebooting into new image\n");
            char r[48]; snprintf(r, sizeof r, "ok:%s", s_req.version); record_result(r);
        } else {
            ESP_LOGE(TAG, "esp_https_ota_finish failed: %s", esp_err_to_name(err));
            record_result("failed:finish");
        }
    } else {
        ESP_LOGE(TAG, "OTA REJECTED: err=%s got_all=%d hash_ok=%d (got %s want %s) -- leaving current image active",
                 esp_err_to_name(err), got_all, hash_ok, digest_hex, s_req.sha256);
        record_result((err == ESP_OK && got_all && !hash_ok) ? "failed:hash" : "failed:download");
        esp_https_ota_abort(handle);   /* does NOT touch the boot partition pointer */
    }
    esp_restart();
}

/* The home network to use for a download: the Kizuna store (kz_wifi.h) when it has one, since the driver's own copy
 * can be overwritten by a mode that uses the radio for something else. Needs esp_wifi_init() first. */
static void prefer_kz_wifi(void)
{
    char ssid[KZ_WIFI_SSID_MAX + 1], pass[KZ_WIFI_PASS_MAX + 1];
    kz_wifi_adopt_driver_config(false);
    if (!kz_wifi_get(ssid, pass)) return;
    memset(&s_boot_sta_cfg, 0, sizeof(s_boot_sta_cfg));
    memcpy(s_boot_sta_cfg.sta.ssid, ssid, strlen(ssid));
    memcpy(s_boot_sta_cfg.sta.password, pass, strlen(pass));
}

/* Update mode never associates to Wi-Fi -- the app sends NO credentials in that path (they are
 * already stored in NVS from a prior provisioning), so the network_provisioning manager never
 * brings the STA up. Without an association, esp_https_ota fails instantly with ESP_ERR_HTTP_CONNECT
 * Bring the STA up with the stored config and block until an IP
 * arrives (or the timeout elapses) before starting the download. */
static bool wifi_sta_connect_blocking(int timeout_ms)
{
    /* Restore the boot-time snapshot -- the prov manager may have cleared the live config. */
    if (s_boot_sta_cfg.sta.ssid[0]) esp_wifi_set_config(WIFI_IF_STA, &s_boot_sta_cfg);
    printf("[ota] bringing up Wi-Fi for OTA (ssid='%s')\n", (const char *)s_boot_sta_cfg.sta.ssid);

    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_start();     /* tolerated if already started */
    esp_wifi_connect();   /* uses the stored SSID/passphrase */

    esp_netif_t *nif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    for (int i = 0; i < timeout_ms / 100; i++) {
        esp_netif_ip_info_t ip = { 0 };
        if (nif && esp_netif_get_ip_info(nif, &ip) == ESP_OK && ip.ip.addr != 0) {
            printf("[ota] Wi-Fi up for OTA (ip %u.%u.%u.%u)\n",
                   (unsigned)(ip.ip.addr & 0xff), (unsigned)((ip.ip.addr >> 8) & 0xff),
                   (unsigned)((ip.ip.addr >> 16) & 0xff), (unsigned)((ip.ip.addr >> 24) & 0xff));
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    printf("[ota] Wi-Fi did not get an IP within %d ms -- OTA will likely fail\n", timeout_ms);
    return false;
}

/* Periodic "[ota-hb]" wait heartbeat, printed while blocked in network_prov_mgr_wait(). This is
 * what makes the (by-design, possibly infinite) BLE wait state visible on the serial log --
 * without it a provisioning boot is completely silent at this project's WARN log level and reads
 * as a dead device. */

static volatile bool s_wait_hb_run;
static char s_service_name[24];

static void wait_hb_task(void *arg)
{
    (void)arg;
    int waited = 0;
    while (s_wait_hb_run) {
        printf("[ota-hb] BLE provisioning: waiting for the app (service '%s')\n", s_service_name);
        vTaskDelay(pdMS_TO_TICKS(2000));
        waited += 2;
        if (s_session_timeout_s > 0 && waited >= s_session_timeout_s) {
            /* A session the user asked for (from a running mode) must not strand the adapter forever. */
            printf("[ota] Wi-Fi setup session timed out after %ds, ending it\n", waited);
            network_prov_mgr_stop_provisioning();
            break;
        }
    }
    vTaskDelete(NULL);
}

void ota_update_provisioning_boot(void)
{
    /* CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE is on (sdkconfig.defaults, this branch): an image that
     * arrived via OTA boots in PENDING_VERIFY and MUST confirm itself, or the bootloader rolls back
     * to the previous slot on the next reset. Nothing called this before -- every real
     * OTA would have been silently undone. Harmlessly returns an error when the running image's
     * state is already valid/undefined (e.g. a plain esptool flash). Called before the possibly-
     * infinite provisioning wait below so a power cycle mid-wait can't trigger a rollback.
     * NOTE: this also returns ESP_OK on a plain esptool flash (it stamps
     * the UNDEFINED ota state to VALID) -- the log line must not claim the image came via OTA. */
    if (esp_ota_mark_app_valid_cancel_rollback() == ESP_OK)
        printf("[ota] app image marked valid (any pending rollback cancelled)\n");

    bool force_session = OTA_FORCE_UPDATE_MODE || s_force_session;

    /* tr-OTA-2 hw bug: a raw erase_region on the nvs partition (done once, by hand,
     * to clear a stale Wi-Fi SSID left over from an old build's flash-storage config, for testing)
     * leaves NVS needing the same erase+reinit retry gba_login_main.c's own nvs_flash_init() call
     * already handles -- this call didn't, and the device hung completely (no boot banner at all)
     * on the next boot. Match the existing pattern instead of assuming init always succeeds. */
    esp_err_t nvs_err = nvs_flash_init();
    if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES || nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    /* tr-OTA-1 hw bug: network_prov_mgr_init() does NOT bring up the esp_wifi driver
     * itself -- omitting this call crashed on real hardware the first time this code ever ran
     * (ESP_ERR_WIFI_NOT_INIT inside network_prov_mgr_start_provisioning's internal esp_wifi_set_mode,
     * abort() via ESP_ERROR_CHECK). Compile/link-only verification could never have caught this. */
    wifi_init_config_t wifi_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_cfg));

    /* Snapshot the persisted STA credentials NOW, before the provisioning manager can clear them. */
    memset(&s_boot_sta_cfg, 0, sizeof(s_boot_sta_cfg));
    esp_wifi_get_config(WIFI_IF_STA, &s_boot_sta_cfg);
    prefer_kz_wifi();
    printf("[ota] stored STA ssid at boot='%s'\n", (const char *)s_boot_sta_cfg.sta.ssid);

    network_prov_mgr_config_t prov_cfg = {
        .scheme = network_prov_scheme_ble,
        .scheme_event_handler = NETWORK_PROV_EVENT_HANDLER_NONE,
        .app_event_handler = NETWORK_PROV_EVENT_HANDLER_NONE,
    };
    ESP_ERROR_CHECK(network_prov_mgr_init(prov_cfg));

    bool provisioned = false;
    network_prov_mgr_is_wifi_provisioned(&provisioned);

    if (!provisioned || force_session) {
        network_prov_mgr_endpoint_create("ota-install");
        network_prov_mgr_endpoint_create("ota-status");

        uint8_t mac[6] = {0};
        esp_wifi_get_mac(WIFI_IF_STA, mac);
        snprintf(s_service_name, sizeof(s_service_name), "KIZUNA_%02X%02X", mac[4], mac[5]);
        printf("[ota] entering BLE provisioning mode: service '%s' (provisioned=%d forced=%d)\n",
               s_service_name, provisioned, force_session);
        ESP_ERROR_CHECK(network_prov_mgr_start_provisioning(NETWORK_PROV_SECURITY_1, PROV_POP,
                                                             s_service_name, NULL));
        network_prov_mgr_endpoint_register("ota-install", ep_ota_install_handler, NULL);
        network_prov_mgr_endpoint_register("ota-status", ep_ota_status_handler, NULL);

        s_session_timeout_s = s_force_session ? 300 : 0;
        s_wait_hb_run = true;
        xTaskCreate(wait_hb_task, "ota_wait_hb", 2048, NULL, 2, NULL);

        /* Blocks until NETWORK_PROV_END: fresh-provisioning path auto-stops on Wi-Fi cred success
         * (default manager behavior, no code needed here); update-only path is ended explicitly by
         * ep_ota_install_handler's network_prov_mgr_stop_provisioning() call above. This wait is
         * UNBOUNDED BY DESIGN -- an unprovisioned device sits here advertising until
         * the app talks to it. The [ota-hb] heartbeat above keeps that state visible in the log. */
        network_prov_mgr_wait();
        s_wait_hb_run = false;
        network_prov_mgr_deinit();
        kz_wifi_adopt_driver_config(true);   /* a network the user just set: keep it in the Kizuna store too */

        if (s_req.pending) {
            wifi_sta_connect_blocking(15000);   /* update mode never joined Wi-Fi -- do it now */
            run_https_ota(s_req.url);            /* does not return */
        }
        printf("[ota] provisioning session ended, restarting\n");
        esp_restart();   /* never fall through into the caller's existing LDN Wi-Fi bring-up */
    }

    /* Already provisioned, no session requested: nothing to do -- but honor ota_update.h's "returns
     * having touched nothing" contract for real. Fix: this path used to return with
     * the default DHCP STA netif + Wi-Fi driver + default event loop still live, and the caller's
     * LDN bring-up then failed against them (seen on hardware:
     * "esp_netif_new_api: ... duplicate key" -> the LDN static-IP netif was NEVER created, ldn_udp/
     * ldn_join got a NULL netif -- LDN silently broken; plus "wifi_init: Failed to init, WiFi is
     * initialized by esp_wifi_init" -> the LDN path ran on THIS function's wifi init). Tear it all
     * back down so app_main proceeds into a clean slate, exactly as if this feature were disabled.
     * (esp_netif_init() itself has no deinit and nvs stays initialized -- both are idempotent /
     * error-tolerated at their second call sites in gba_login_main.c.) */
    network_prov_mgr_deinit();
    ESP_ERROR_CHECK(esp_wifi_deinit());
    esp_netif_destroy_default_wifi(sta_netif);
    ESP_ERROR_CHECK(esp_event_loop_delete_default());
    printf("[ota] provisioned, no update requested: torn down, continuing normal boot\n");
}

/* ==== Runtime-triggered OTA (ota_update.h) ============================================================== */

static bool boot_action_set(uint8_t act, const ota_request_t *req)
{
    nvs_handle_t h;
    if (nvs_open(KZ_NVS_NS, NVS_READWRITE, &h) != ESP_OK) return false;
    esp_err_t e = ESP_OK;
    if (req) e = nvs_set_blob(h, "req", req, sizeof(*req));
    if (e == ESP_OK) { nvs_erase_key(h, "res"); e = nvs_set_u8(h, "boot", act); }   /* new request: forget the old result */
    if (e == ESP_OK) e = nvs_commit(h);
    nvs_close(h);
    return e == ESP_OK;
}

bool ota_update_request_install(const char *json, size_t len)
{
    ota_request_t r;
    ota_req_err_t e = ota_request_parse(json, len, &r);
    if (e != OTA_REQ_OK) {
        char msg[32]; snprintf(msg, sizeof msg, "failed:%s", ota_request_err_str(e));
        printf("[ota] install request rejected: %s\n", ota_request_err_str(e));
        record_result(msg);
        return false;
    }
    if (!boot_action_set(BOOT_INSTALL, &r)) { printf("[ota] could not store the install request\n"); return false; }
    printf("[ota] install of %s queued for next boot\n", r.version);
    return true;
}

bool ota_update_request_provision(void)
{
    bool ok = boot_action_set(BOOT_PROVISION, NULL);
    printf("[ota] Wi-Fi setup %s for next boot\n", ok ? "queued" : "could not be queued");
    return ok;
}

void ota_update_confirm_boot(void)
{
    if (esp_ota_mark_app_valid_cancel_rollback() == ESP_OK)
        printf("[ota] running image confirmed valid (rollback cancelled)\n");
    const char *spare = ota_update_spare_version();   /* check it now, on the caller's task, not on the first status read */
    printf("[ota] other slot: %s\n", spare ? spare : "no bootable image");
}

/* ---- the other OTA slot --------------------------------------------------------------------------------- */
static bool s_spare_switch;
static volatile int s_spare_state;   /* 0 = not checked, 1 = nothing bootable, 2 = s_spare_ver is bootable */
static char s_spare_ver[sizeof(((esp_app_desc_t *)0)->version)];

void ota_update_enable_spare_switch(void) { s_spare_switch = true; }
void ota_update_spare_forget(void) { s_spare_state = 1; }

const char *ota_update_spare_version(void)
{
    if (s_spare_state == 0) {
        int st = 1;
        const esp_partition_t *p = esp_ota_get_next_update_partition(NULL);
        esp_ota_img_states_t ist;
        esp_app_desc_t d;
        esp_partition_pos_t pos = { .offset = p ? p->address : 0, .size = p ? p->size : 0 };
        esp_image_metadata_t md;
        if (p
            && !(esp_ota_get_state_partition(p, &ist) == ESP_OK && (ist == ESP_OTA_IMG_INVALID || ist == ESP_OTA_IMG_ABORTED))
            && esp_ota_get_partition_description(p, &d) == ESP_OK
            && esp_image_verify(ESP_IMAGE_VERIFY_SILENT, &pos, &md) == ESP_OK) {
            memcpy(s_spare_ver, d.version, sizeof s_spare_ver);
            s_spare_ver[sizeof s_spare_ver - 1] = 0;
            st = 2;
        }
        if (s_spare_state == 0) s_spare_state = st;   /* a push may have started meanwhile: it wins */
    }
    return s_spare_state == 2 ? s_spare_ver : NULL;
}

bool ota_update_boot_spare(void)
{
    const char *v = ota_update_spare_version();
    const esp_partition_t *p = esp_ota_get_next_update_partition(NULL);
    if (!v || !p) return false;
    /* esp_ota_set_boot_partition verifies the image again; with rollback on, the switched-to image boots as "new" and
     * falls back to this one if it never confirms itself. */
    if (esp_ota_set_boot_partition(p) != ESP_OK) { s_spare_state = 1; return false; }
    char r[48]; snprintf(r, sizeof r, "ok:%s", v);
    record_result(r);
    printf("[ota] booting the other slot: %s\n", v);
    return true;
}

/* Bring up just enough of the network stack to associate with the stored Wi-Fi and download. */
static void ota_wifi_bringup(void)
{
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t wifi_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_cfg));
    memset(&s_boot_sta_cfg, 0, sizeof(s_boot_sta_cfg));
    esp_wifi_get_config(WIFI_IF_STA, &s_boot_sta_cfg);   /* the credentials provisioning stored in NVS */
    prefer_kz_wifi();
}

void ota_update_boot(void)
{
    nvs_handle_t h;
    if (nvs_open(KZ_NVS_NS, NVS_READWRITE, &h) != ESP_OK) return;   /* nothing stored yet (or NVS not up) */
    uint8_t act = BOOT_NONE;
    nvs_get_u8(h, "boot", &act);
    if (act == BOOT_NONE) { nvs_close(h); return; }

    /* One-shot: clear BEFORE acting, so no failure/crash below can re-enter this on the next boot. */
    nvs_set_u8(h, "boot", BOOT_NONE);
    nvs_commit(h);

    if (act == BOOT_PROVISION) {
        nvs_close(h);
        printf("[ota] boot action: Wi-Fi setup session (BLE, app sets the network)\n");
        s_force_session = true;
        ota_update_provisioning_boot();   /* runs the session, then esp_restart(): never returns */
        return;
    }

    if (act == BOOT_INSTALL) {
        ota_request_t r;
        size_t sz = sizeof(r);
        esp_err_t e = nvs_get_blob(h, "req", &r, &sz);
        nvs_erase_key(h, "req");
        nvs_commit(h);
        nvs_close(h);
        if (e != ESP_OK || sz != sizeof(r) || ota_request_check(&r) != OTA_REQ_OK) {
            record_result("failed:bad-request");
            esp_restart();
        }
        memset(&s_req, 0, sizeof(s_req));
        strncpy(s_req.version, r.version, sizeof(s_req.version) - 1);
        strncpy(s_req.url, r.url, sizeof(s_req.url) - 1);
        strncpy(s_req.sha256, r.sha256, sizeof(s_req.sha256) - 1);
        s_req.pending = true;
        printf("[ota] boot action: install %s\n", s_req.version);

        ota_wifi_bringup();
        if (!s_boot_sta_cfg.sta.ssid[0]) {
            printf("[ota] no Wi-Fi network is stored - set one up first\n");
            record_result("failed:no-wifi");
            esp_restart();
        }
        if (!wifi_sta_connect_blocking(20000)) {
            record_result("failed:wifi");
            esp_restart();
        }
        run_https_ota(s_req.url);   /* records the outcome and restarts: never returns */
        return;
    }
    nvs_close(h);   /* unknown action value: already cleared above */
}

static const char *s_hw = "";
void ota_update_set_hw(const char *hw) { s_hw = hw ? hw : ""; }

size_t ota_update_status_json(char *buf, size_t cap, const char *mode)
{
    const char *phase = "idle";
    char err[40] = "";
    nvs_handle_t h;
    if (nvs_open(KZ_NVS_NS, NVS_READONLY, &h) == ESP_OK) {
        uint8_t act = 0; nvs_get_u8(h, "boot", &act);
        char res[48]; size_t n = sizeof(res);
        if (act == BOOT_INSTALL) phase = "rebooting";
        else if (nvs_get_str(h, "res", res, &n) == ESP_OK) {
            if (!strncmp(res, "ok:", 3)) phase = "done";
            else if (!strncmp(res, "failed:", 7)) {
                phase = "failed";
                strncpy(err, res + 7, sizeof(err) - 1); err[sizeof(err) - 1] = 0;
            }
        }
        nvs_close(h);
    }
    const char *spare = s_spare_switch && s_spare_state == 2 ? s_spare_ver : NULL;   /* never hash on the BLE host task */
    size_t w = ota_status_format_spare(buf, cap, esp_app_get_description()->version, mode, phase, err, OTA_PUSH_ENABLED, s_hw, spare);
    /* "wifi":{"saved":0|1,"up":0|1} (the app's parseWifi): saved = a network is in the shared kz_wifi store, up = the STA
     * is associated to THAT network right now (switch-online's STA on the Switch's LDN room is not "up"). Never the name:
     * the app keeps the name per adapter itself. Without this the app guessed from its own records. */
    char ssid[KZ_WIFI_SSID_MAX + 1] = { 0 }, pass[KZ_WIFI_PASS_MAX + 1] = { 0 };
    bool saved = kz_wifi_get(ssid, pass);
    memset(pass, 0, sizeof pass);
    bool up = false;
    wifi_ap_record_t ap;
    if (saved && esp_wifi_sta_get_ap_info(&ap) == ESP_OK) up = strncmp((const char *)ap.ssid, ssid, sizeof ap.ssid) == 0;
    if (w > 0 && w + 32 < cap && buf[w - 1] == '}') {
        int k = snprintf(buf + w - 1, cap - (w - 1), ",\"wifi\":{\"saved\":%d,\"up\":%d}}", saved ? 1 : 0, up ? 1 : 0);
        if (k > 0) w = w - 1 + (size_t)k;
    }
    return w;
}

void ota_update_record_result(const char *res) { record_result(res); }

#else /* !OTA_UPDATE_ENABLED */

void ota_update_provisioning_boot(void) { /* disabled -- see ota_update.h */ }
void ota_update_boot(void) { /* disabled -- see ota_update.h */ }
void ota_update_set_hw(const char *hw) { (void)hw; }
bool ota_update_request_install(const char *json, size_t len) { (void)json; (void)len; return false; }
bool ota_update_request_provision(void) { return false; }
void ota_update_confirm_boot(void) { }
void ota_update_record_result(const char *res) { (void)res; }
void ota_update_enable_spare_switch(void) { }
const char *ota_update_spare_version(void) { return NULL; }
bool ota_update_boot_spare(void) { return false; }
void ota_update_spare_forget(void) { }
size_t ota_update_status_json(char *buf, size_t cap, const char *mode)
{
    (void)mode;
    if (buf && cap) buf[0] = 0;
    return 0;
}

#endif
