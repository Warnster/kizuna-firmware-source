# switch-online: the Switch side of the online GBA<->Switch trade. The Switch's local wireless on the C3;
# the GBA's RFU1 frames reach a phone over Bluetooth (kz_mode_ble data characteristic) and the phone carries them to
# the relay (LINK stratum, family gba-rfu). The USB console path ("@GB" lines) stays for
# the bench. Session logic is switch-trade's trade_session.c, compiled from there so both modes share its fixes.
cmake_minimum_required(VERSION 3.22)
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../components")
set(EXCLUDE_COMPONENTS rp2040_swd gba_multiboot kzn_link trade_host_session uni_echo gb_serial gen1_link)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
idf_build_set_property(COMPILE_OPTIONS "-DOTA_UPDATE_ENABLED=1" APPEND)
idf_build_set_property(COMPILE_OPTIONS "-DOTA_PUSH_ENABLED=1" APPEND)
# Production image: compile out the shared trade engine's per-frame and heartbeat bench lines (KZ_TRACE in sim.c,
# rfu1_bridge.c, ldn_brain.c, gba_bridge.c, trade_session.c). switch-trade keeps them.
idf_build_set_property(COMPILE_OPTIONS "-DKZ_QUIET_TRACE=1" APPEND)
# The online relay pacing in trade_brain/gba_bridge.c: the UP replay on the Switch's clock, DN_KEEP thinning, the
# 64-slot down ring. Local images keep switch-trade-v2.4's local path.
idf_build_set_property(COMPILE_OPTIONS "-DKZ_ONLINE_PACING=1" APPEND)
# online: the GBA may lead (gba_lead, this adapter hosts the room for the Switch). Its beacons cross the relay and the
# phone's Bluetooth, so "still leading" and "group closed" get wider windows than on one board (1 s / 3 s).
idf_build_set_property(COMPILE_OPTIONS "-DRFU1_BEACON_FRESH_US=3000000" APPEND)
idf_build_set_property(COMPILE_OPTIONS "-DGBA_LEAD_BEACON_TIMEOUT_MS=10000" APPEND)
# The Pia host must fit in one heap block beside Bluetooth: 32 resend slots instead of 64 (bench: 32,208 B wanted,
# largest block 29,696 B).
idf_build_set_property(COMPILE_OPTIONS "-DGLP_RELIABLE_SLOTS=32" APPEND)
# ... and room for the GBA's frames arriving in bunches (relay + phone) while the Switch drains slower than on one board.
idf_build_set_property(COMPILE_OPTIONS "-DGLP_HOST_PARENT_SLOTS=56" APPEND)
# rfu1_bridge rx queue stays at 32: 80 (+3.2 KB) cut the largest heap block to 26624 B and the GBA-led Pia
# host (27336 B) could not start. Dropped chunks now lose whole frames, never splice them (rfu1_bridge.c chunk_t.gap).
# ... and enough credit to cover the longer round trip (Bluetooth beside the soft-AP): at 6 the Switch starved.
idf_build_set_property(COMPILE_OPTIONS "-DGLP_CREDIT_CAP=10" APPEND)
# the cable stand-in (the GBA in a Cable Club far away, this board leads the Switch's group for it). Online only.
idf_build_set_property(COMPILE_OPTIONS "-DGBA_LEAD_STANDIN=1" APPEND)
project(kz_switch_online)
