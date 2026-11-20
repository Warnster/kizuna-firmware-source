# Ruby/Sapphire image (KZ_GBA_LINK=cable): the cable bridge implements rfu1_bridge.h over GB-Link mode 0x01 and
# rfu1_bridge registers its header only. cable_air is required in both images (unused code is dropped at link time).
set(gba_link_srcs)
if(KZ_GBA_LINK STREQUAL "cable")
    set(gba_link_srcs "gba_cable_bridge.c" "cable_air_sim.c")
endif()
idf_component_register(SRCS "switch_trade_main.c" "trade_session.c" "relay_engine.c" ${gba_link_srcs}
                       INCLUDE_DIRS "."
                       REQUIRES ldn_sta trade_brain rfu1_bridge gblink rfu1 cable_air cport kz_keys esp_wifi esp_netif esp_event nvs_flash esp_timer
                                esp_driver_usb_serial_jtag esp_driver_uart esp_hw_support lwip kz_mode_ble bt ldn_ap_host trade_host_session gba_lead)
# the GBA leads and the Switch joins: local RFU image only. switch-online also compiles trade_session.c
# too and stay without it.
if(NOT KZ_GBA_LINK STREQUAL "cable")
    target_compile_definitions(${COMPONENT_LIB} PRIVATE KZ_GBA_LEAD=1)
endif()
