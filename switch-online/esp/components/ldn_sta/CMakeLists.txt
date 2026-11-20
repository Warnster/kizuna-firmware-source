# ldn_sta — the Switch LDN STATION role (join a Switch-hosted FRLG room): ldn_join (association, CCMP key
# install, 0x88b7 auth, Pia frame forwarding) + ldn_udp (static-IP UDP transport). From an earlier Kizuna firmware
# components/ldn_join, minus the GBA seam (that is components/trade_brain). Keys arrive at runtime.
idf_component_register(SRCS "ldn_join.c" "ldn_udp.c"
                       INCLUDE_DIRS "."
                       REQUIRES esp_wifi wpa_supplicant esp_event esp_timer esp_hw_support esp_netif lwip cport esp_coex)
# ldn_join.c uses the REAL IDF v6.1 wpa_supplicant driver ABI (struct wpa_funcs, enum key_flag,
# esp_wifi_*_internal) from a PRIVATE header; include its directory rather than a hand-mirrored copy.
# This pins the component to IDF v6.1 (same pin as GB-Link's and pokeldn's bridges). Target-independent:
# the same symbols exist in the esp32c3 and esp32c6 libraries.
target_include_directories(${COMPONENT_LIB} PRIVATE
                           "$ENV{IDF_PATH}/components/wpa_supplicant/esp_supplicant/src")
