# trade_brain — ldn_brain (drives cport's sim.c, the virtual RFU child toward the Switch) + gba_bridge (the
# parent-side NI/UNI logic toward the GBA, carried over from the C6 PARLIO design). In M3 the
# gba_bridge half is re-targeted at the RP2040's RFU1 frames; until then it is the proven C6 logic verbatim
# and the sim runs with its stub engine (idle slots) when no GBA traffic is wired in.
idf_component_register(SRCS "ldn_brain.c" "gba_bridge.c"
                       INCLUDE_DIRS "."
                       REQUIRES ldn_sta cport esp_timer esp_hw_support)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error=unused-function)
