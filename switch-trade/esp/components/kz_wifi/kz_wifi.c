#include "kz_wifi.h"
#include <string.h>
#include <stdio.h>
#include "nvs.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#define NS "kz_wifi"

bool kz_wifi_get(char ssid[KZ_WIFI_SSID_MAX + 1], char pass[KZ_WIFI_PASS_MAX + 1])
{
    nvs_handle_t h;
    if (nvs_open(NS, NVS_READONLY, &h) != ESP_OK) return false;
    size_t ns = KZ_WIFI_SSID_MAX + 1, np = KZ_WIFI_PASS_MAX + 1;
    bool ok = nvs_get_str(h, "ssid", ssid, &ns) == ESP_OK && ssid[0];
    if (ok && nvs_get_str(h, "pass", pass, &np) != ESP_OK) pass[0] = 0;   /* open network */
    nvs_close(h);
    return ok;
}

bool kz_wifi_configured(void)
{
    char s[KZ_WIFI_SSID_MAX + 1], p[KZ_WIFI_PASS_MAX + 1];
    return kz_wifi_get(s, p);
}

esp_err_t kz_wifi_set(const char *ssid, const char *pass)
{
    if (!ssid || !ssid[0] || strlen(ssid) > KZ_WIFI_SSID_MAX) return ESP_ERR_INVALID_ARG;
    if (pass && strlen(pass) > KZ_WIFI_PASS_MAX) return ESP_ERR_INVALID_ARG;
    nvs_handle_t h;
    esp_err_t e = nvs_open(NS, NVS_READWRITE, &h);
    if (e != ESP_OK) return e;
    e = nvs_set_str(h, "ssid", ssid);
    if (e == ESP_OK) e = nvs_set_str(h, "pass", pass ? pass : "");
    if (e == ESP_OK) e = nvs_commit(h);
    nvs_close(h);
    return e;
}

esp_err_t kz_wifi_clear(void)
{
    nvs_handle_t h;
    esp_err_t e = nvs_open(NS, NVS_READWRITE, &h);
    if (e != ESP_OK) return e;
    nvs_erase_all(h);
    e = nvs_commit(h);
    nvs_close(h);
    return e;
}

bool kz_wifi_adopt_driver_config(bool overwrite)
{
    if (!overwrite && kz_wifi_configured()) return false;
    wifi_config_t c = { 0 };
    if (esp_wifi_get_config(WIFI_IF_STA, &c) != ESP_OK || !c.sta.ssid[0]) return false;
    char ssid[KZ_WIFI_SSID_MAX + 1] = { 0 }, pass[KZ_WIFI_PASS_MAX + 1] = { 0 };
    memcpy(ssid, c.sta.ssid, KZ_WIFI_SSID_MAX);   /* driver fields are fixed-size, not always NUL-terminated */
    memcpy(pass, c.sta.password, KZ_WIFI_PASS_MAX);
    if (kz_wifi_set(ssid, pass) != ESP_OK) return false;
    printf("[kzwifi] stored network '%s'\n", ssid);
    return true;
}

/* ---- connect / stop ---------------------------------------------------------------------------------- */
#define BIT_IP   BIT0
#define BIT_FAIL BIT1
#define MAX_RETRIES 2   /* fail fast when the home network is out of range: boot waits on this */

static EventGroupHandle_t s_ev;
static esp_netif_t *s_netif;
static esp_event_handler_instance_t s_h_wifi, s_h_ip;
static bool s_inited;
static int s_retries;

static void on_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg; (void)data;
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        if (++s_retries <= MAX_RETRIES) esp_wifi_connect();
        else xEventGroupSetBits(s_ev, BIT_FAIL);
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        xEventGroupSetBits(s_ev, BIT_IP);
    }
}

esp_err_t kz_wifi_connect(int timeout_ms)
{
    char ssid[KZ_WIFI_SSID_MAX + 1], pass[KZ_WIFI_PASS_MAX + 1];
    esp_netif_init();
    esp_err_t e = esp_event_loop_create_default();
    if (e != ESP_OK && e != ESP_ERR_INVALID_STATE) return e;   /* INVALID_STATE = already created */
    if (!s_ev) s_ev = xEventGroupCreate();
    if (!s_ev) return ESP_ERR_NO_MEM;
    if (!s_netif) s_netif = esp_netif_create_default_wifi_sta();

    if (!s_inited) {
        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        if ((e = esp_wifi_init(&cfg)) != ESP_OK) return e;
        s_inited = true;
    }
    kz_wifi_adopt_driver_config(false);
    if (!kz_wifi_get(ssid, pass)) return ESP_ERR_NOT_FOUND;

    esp_wifi_set_storage(WIFI_STORAGE_RAM);   /* never write the driver's NVS copy */
    wifi_config_t c = { 0 };
    memcpy(c.sta.ssid, ssid, strlen(ssid));
    memcpy(c.sta.password, pass, strlen(pass));
    c.sta.threshold.authmode = pass[0] ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;
    c.sta.pmf_cfg.capable = true;
    c.sta.sae_pwe_h2e = WPA3_SAE_PWE_BOTH;

    xEventGroupClearBits(s_ev, BIT_IP | BIT_FAIL);
    s_retries = 0;
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, on_event, NULL, &s_h_wifi);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, on_event, NULL, &s_h_ip);
    if ((e = esp_wifi_set_mode(WIFI_MODE_STA)) != ESP_OK) return e;
    if ((e = esp_wifi_set_config(WIFI_IF_STA, &c)) != ESP_OK) return e;
    if ((e = esp_wifi_start()) != ESP_OK) return e;

    EventBits_t b = xEventGroupWaitBits(s_ev, BIT_IP | BIT_FAIL, pdFALSE, pdFALSE, pdMS_TO_TICKS(timeout_ms));
    if (b & BIT_IP) { printf("[kzwifi] connected to '%s'\n", ssid); return ESP_OK; }
    printf("[kzwifi] could not join '%s' (%s)\n", ssid, (b & BIT_FAIL) ? "rejected / not found" : "timed out");
    return ESP_ERR_TIMEOUT;
}

void kz_wifi_stop(void)
{
    if (s_h_wifi) { esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, s_h_wifi); s_h_wifi = NULL; }
    if (s_h_ip)   { esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, s_h_ip); s_h_ip = NULL; }
    if (s_inited) {
        esp_wifi_disconnect();
        esp_wifi_stop();
        esp_wifi_deinit();
        s_inited = false;
    }
    if (s_netif) { esp_netif_destroy_default_wifi(s_netif); s_netif = NULL; }
}
