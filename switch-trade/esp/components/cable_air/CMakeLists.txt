# cable_air: the RFU Cable Wrapper translator ported from mgba_LDN (MPL-2.0, file-level copyleft: these files stay
# under the MPL and their source stays available). Pure logic, no ESP-IDF dependencies. On the two-chip board the
# cable edge is the RP2040 in GB-Link link-cable mode (switch-trade built with -DKZ_GBA_LINK=cable) and the
# Switch edge is cport's sim through ldn_brain's engine override.
idf_component_register(SRCS "cable_air.c"
                       INCLUDE_DIRS ".")
