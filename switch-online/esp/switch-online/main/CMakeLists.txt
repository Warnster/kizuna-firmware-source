set(ST "${CMAKE_CURRENT_LIST_DIR}/../../switch-trade/main")
idf_component_register(SRCS "switch_online_main.c" "${ST}/trade_session.c"
                       INCLUDE_DIRS "." "${ST}"
                       REQUIRES ldn_sta trade_brain rfu1_bridge gblink rfu1 cport esp_wifi esp_coex esp_netif esp_event nvs_flash esp_timer
                                esp_driver_usb_serial_jtag lwip kz_mode_ble ota_update bt gba_lead ldn_ap_host)
# the GBA leads, the Switch joins -- online too (trade_session.c's lead trigger, gba_lead polled in brain_task).
target_compile_definitions(${COMPONENT_LIB} PRIVATE KZ_GBA_LEAD=1)
