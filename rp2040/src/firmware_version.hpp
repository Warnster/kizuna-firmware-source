#pragma once

#include <cstdint>

// Single source for the firmware version. It is reported over the
// protocol (GetFirmwareInfo) and folded into the USB bcdDevice (see usbLayer.cpp).
//
// Why bcdDevice tracks the version: Windows caches a device's MS OS / WinUSB
// driver binding keyed by VID/PID/bcdDevice. If bcdDevice never changes, a host
// that cached an older descriptor keeps the stale binding after a firmware
// update and will not re-read the corrected descriptors. Advancing bcdDevice on
// every release makes Windows treat it as a new device and re-enumerate cleanly.
namespace fw
{
    inline constexpr uint8_t versionMajor = 2;
    inline constexpr uint8_t versionMinor = 2;
    inline constexpr uint8_t versionPatch = 7;

    // Packed BCD nibbles 0x0<major><minor><patch>, e.g. 2.2.1 -> 0x0221.
    // Keep each field <= 9 to stay valid BCD.
    inline constexpr uint16_t bcdDevice =
        (static_cast<uint16_t>(versionMajor) << 8) |
        (static_cast<uint16_t>(versionMinor) << 4) |
        static_cast<uint16_t>(versionPatch);

    // Kizuna fork build number, appended to the GetFirmwareInfo reply as ['K', kizunaBuild] (bytes 6-7) so the
    // adapter's ESP32-C3 can tell this firmware from stock GB-Link, which sends 6 bytes, and put it back over SWD.
    // GB-Link's web apps read only the bytes they know (length >= 4/5/6 checks). Bump on every fork image.
    inline constexpr uint8_t kizunaMarker = 'K';
    inline constexpr uint8_t kizunaBuild = 13;
}
