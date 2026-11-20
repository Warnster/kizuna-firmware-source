# the hosted Direct Corner trade session (ldn_ap room + dist_udp + pia_host + trade_host), shared by the
# join-role/bench project (switch-trade) and the hosted-only image (hosted-trade).
idf_component_register(SRCS "trade_host_session.c"
                       INCLUDE_DIRS "."
                       REQUIRES cport ldn_ap_host
                       PRIV_REQUIRES esp_timer esp_hw_support heap log freertos)
