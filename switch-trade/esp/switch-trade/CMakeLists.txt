cmake_minimum_required(VERSION 3.22)
# Switch Trade mode on the two-chip board: the ESP32-C3 (C6 on the bench) joins the
# Switch's FireRed/LeafGreen LDN room; the GBA link is the RP2040 (GB-Link mode 0x07, M2). M1 = Switch-only
# join from the USB console. Builds for esp32c3 and esp32c6:
#   idf.py -B build-c3 -DIDF_TARGET=esp32c3 -DSDKCONFIG=sdkconfig.c3 build
#   idf.py -B build-c6 -DIDF_TARGET=esp32c6 -DSDKCONFIG=sdkconfig.c6 build
# (sdkconfig.defaults.<target> is picked up automatically next to sdkconfig.defaults.)
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../components")
# Which GBA games this image serves. One source tree, two images:
#   rfu   (default) FireRed/LeafGreen/Emerald: the RP2040 emulates the wireless adapter (GB-Link mode 0x07, rfu1_bridge).
#   cable           Ruby/Sapphire: the RP2040 is the other GBA on the link cable (mode 0x01) and cable_air translates to
#                   the Switch's wireless trade (switch-trade/main/gba_cable_bridge.c). Mode id "switch-trade-rs".
#   idf.py -B build-rs -DIDF_TARGET=esp32c3 -DSDKCONFIG=sdkconfig.c3 -DKZ_GBA_LINK=cable build
set(KZ_GBA_LINK "rfu" CACHE STRING "GBA link: rfu (FRLG/Emerald, mode 0x07) or cable (Ruby/Sapphire, mode 0x01)")
set_property(CACHE KZ_GBA_LINK PROPERTY STRINGS rfu cable)
# Only what this mode uses: gblink + rfu1 + rfu1_bridge are the M2 GBA side; rp2040_swd/gba_multiboot and
# Wi-Fi/OTA/BLE plumbing in M4. Excluding them keeps the image and static RAM measurable for the M1 gate.
set(EXCLUDE_COMPONENTS rp2040_swd gba_multiboot kzn_link)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
# Bluetooth firmware push + Wi-Fi install: every duo mode carries the OTA receiver; the decoder workspace is session-only.
idf_build_set_property(COMPILE_OPTIONS "-DOTA_UPDATE_ENABLED=1" APPEND)
idf_build_set_property(COMPILE_OPTIONS "-DOTA_PUSH_ENABLED=1" APPEND)
idf_build_set_property(COMPILE_OPTIONS "-DPIA_ZSTD_SESSION_HEAP=1" APPEND)
if(KZ_GBA_LINK STREQUAL "cable")
    idf_build_set_property(COMPILE_OPTIONS "-DKZ_GBA_LINK_CABLE=1" APPEND)
elseif(NOT KZ_GBA_LINK STREQUAL "rfu")
    message(FATAL_ERROR "KZ_GBA_LINK must be rfu or cable, got '${KZ_GBA_LINK}'")
endif()
project(kz_switch_trade)
