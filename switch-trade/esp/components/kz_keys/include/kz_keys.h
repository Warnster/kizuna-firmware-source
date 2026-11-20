#pragma once
/* kz_keys -- the three user-supplied LDN values (master_key_12, aes_kek_generation_source, aes_key_generation_source),
 * kept in NVS namespace "kz_keys" so the adapter works after a power cycle or a firmware update without the phone
 *

 * Rules:
 *  - WRITE-ONLY from outside the adapter. Nothing here (or in any op that uses it) returns, prints or logs the values;
 *    the only things that leave are "present" and "absent".
 *  - kz_keys_clear() erases them (the app's "Remove keys", keys_clear, and any factory reset).
 *  - The image never contains them; they live only in the NVS partition, which OTA does not touch.


 * The struct layout is cport's ldn_keys (3 x 16 bytes); this component only stores the 48 bytes. */
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#define KZ_KEYS_LEN 48

/* Store (replace) the 48 bytes. ESP_ERR_INVALID_ARG for NULL or an all-zero value. */
esp_err_t kz_keys_save(const uint8_t keys[KZ_KEYS_LEN]);
/* Load into out (true) or leave out zeroed (false: none stored / unreadable). */
bool kz_keys_load(uint8_t out[KZ_KEYS_LEN]);
bool kz_keys_present(void);
esp_err_t kz_keys_clear(void);
