# The GBA seam on the two-chip board: RFU parent toward the GBA over the RP2040's RFU1 packets.
# switch-trade built for Ruby/Sapphire (KZ_GBA_LINK=cable) implements this API in its own gba_cable_bridge.c
# over the link-cable mode; this component then only provides the header.
set(rfu1_bridge_srcs "rfu1_bridge.c")
if(KZ_GBA_LINK STREQUAL "cable")
    set(rfu1_bridge_srcs)
endif()
idf_component_register(SRCS ${rfu1_bridge_srcs}
                       INCLUDE_DIRS "include"
                       REQUIRES gblink rfu1 trade_brain
                       PRIV_REQUIRES esp_timer esp_hw_support)
