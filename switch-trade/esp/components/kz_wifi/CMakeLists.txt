# The user's home Wi-Fi, shared by every mode (see include/kz_wifi.h).
idf_component_register(SRCS "kz_wifi.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_wifi esp_netif esp_event nvs_flash)
