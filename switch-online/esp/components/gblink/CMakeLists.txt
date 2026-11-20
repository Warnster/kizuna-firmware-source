# Host side of the GB-Link RP2040 control UART.
idf_component_register(SRCS "gblink.c"
                       INCLUDE_DIRS "include"
                       PRIV_REQUIRES esp_driver_uart esp_timer log)
