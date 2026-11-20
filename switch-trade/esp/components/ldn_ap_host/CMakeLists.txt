# ldn_ap_host — the Switch LDN HOST (access point) role: ldn_ap (hidden-SSID AP, no 4-way handshake,
# the private wpa_supplicant hooks on the AP side) + dist_udp (the Pia UDP transport on the AP
# interface). Vendored from an earlier Kizuna firmware (components/ldn_ap, dist_udp),
# which proved this role in 97 host-checked ldn_host tests but has NEVER run on real hardware.
idf_component_register(SRCS "ldn_ap.c" "dist_udp.c"
                       INCLUDE_DIRS "."
                       REQUIRES esp_wifi wpa_supplicant esp_netif esp_event esp_timer lwip cport)
target_include_directories(${COMPONENT_LIB} PRIVATE
                           "$ENV{IDF_PATH}/components/wpa_supplicant/esp_supplicant/src")
