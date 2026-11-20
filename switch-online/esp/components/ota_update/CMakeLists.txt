# OTA-firmware branch: BLE Wi-Fi provisioning + OTA update trigger.
idf_component_register(SRCS "ota_update.c" "ota_request.c" "ota_push_core.c" "ota_push.c"
                       INCLUDE_DIRS "."
                       REQUIRES network_provisioning protocomm esp_https_ota app_update bootloader_support
                                esp_wifi nvs_flash esp_netif esp_event cjson bt mbedtls kz_wifi)
