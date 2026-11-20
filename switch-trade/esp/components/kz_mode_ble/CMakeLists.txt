# The BLE face every mode shares: status + OTA trigger + Wi-Fi setup + signed image push, on the same GATT
# service/characteristics the app already speaks. Other modes keep its own copy for
# now (it also carries the backup/restore protocol); new modes use this.
idf_component_register(SRCS "kz_mode_ble.c"
                       INCLUDE_DIRS "include"
                       REQUIRES bt nvs_flash ota_update esp_system esp_coex kz_wifi)
