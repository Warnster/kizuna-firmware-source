#include "kz_keys.h"
#include <string.h>
#include "nvs.h"

#define NS  "kz_keys"
#define KEY "ldn3"

esp_err_t kz_keys_save(const uint8_t keys[KZ_KEYS_LEN])
{
    if (!keys) return ESP_ERR_INVALID_ARG;
    uint8_t any = 0;
    for (int i = 0; i < KZ_KEYS_LEN; i++) any |= keys[i];
    if (!any) return ESP_ERR_INVALID_ARG;
    nvs_handle_t h;
    esp_err_t e = nvs_open(NS, NVS_READWRITE, &h);
    if (e != ESP_OK) return e;
    e = nvs_set_blob(h, KEY, keys, KZ_KEYS_LEN);
    if (e == ESP_OK) e = nvs_commit(h);
    nvs_close(h);
    return e;
}

bool kz_keys_load(uint8_t out[KZ_KEYS_LEN])
{
    memset(out, 0, KZ_KEYS_LEN);
    nvs_handle_t h;
    if (nvs_open(NS, NVS_READONLY, &h) != ESP_OK) return false;
    size_t n = KZ_KEYS_LEN;
    bool ok = nvs_get_blob(h, KEY, out, &n) == ESP_OK && n == KZ_KEYS_LEN;
    nvs_close(h);
    if (!ok) memset(out, 0, KZ_KEYS_LEN);
    return ok;
}

bool kz_keys_present(void)
{
    nvs_handle_t h;
    if (nvs_open(NS, NVS_READONLY, &h) != ESP_OK) return false;
    size_t n = 0;
    bool ok = nvs_get_blob(h, KEY, NULL, &n) == ESP_OK && n == KZ_KEYS_LEN;
    nvs_close(h);
    return ok;
}

esp_err_t kz_keys_clear(void)
{
    nvs_handle_t h;
    esp_err_t e = nvs_open(NS, NVS_READWRITE, &h);
    if (e == ESP_ERR_NVS_NOT_FOUND) return ESP_OK;   /* nothing stored */
    if (e != ESP_OK) return e;
    nvs_erase_all(h);
    e = nvs_commit(h);
    nvs_close(h);
    return e;
}
