# gba_lead: the GBA leads a Direct Corner group and the Switch joins it. GB-Link-Switch-LDN's host role
# (pia_host / pia_crypto / pia_reliable as glp_*, the lead half of trade_shim as gls_*, pia_bridge's BR_HOST as gba_lead.c;
# commit c972db4, AGPL-3.0, see NOTICE) on kizuna's ldn_ap + dist_udp + rfu1_bridge. RFU images only: the Ruby/Sapphire
# cable image (KZ_GBA_LINK=cable) has no wireless leader, so it registers the header only.
set(gba_lead_srcs "gba_lead.c" "gl_lead_shim.c" "glp_host.c" "glp_crypto.c" "glp_reliable.c")
if(KZ_GBA_LINK STREQUAL "cable")
    set(gba_lead_srcs)
endif()
idf_component_register(SRCS ${gba_lead_srcs}
                       INCLUDE_DIRS "."
                       REQUIRES cport ldn_ap_host rfu1 rfu1_bridge
                       PRIV_REQUIRES mbedtls esp_timer esp_hw_support esp_wifi heap freertos)
