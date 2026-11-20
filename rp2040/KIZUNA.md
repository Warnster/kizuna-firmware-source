# Kizuna RP2040 firmware: kizuna.13

Complete corresponding source for `gblink.v2.2.7-kizuna.13`, the RP2040 firmware on the Kizuna adapter. It is
a fork of GB-Link's [GBLink-Firmware](https://github.com/GB-Link/GBLink-Firmware) (Wireless-Adapter branch,
v2.2.7) and, like the original, is licensed under the GNU General Public License v3.0 (see `LICENSE`).

| Image | SHA-256 of the released `.uf2` |
|---|---|
| `gblink.v2.2.7-kizuna.13.uf2` | `bbc86b5565c339290b135fffdd63c2b3767f70e69a86218902773cefe6acc811` |

Built from this folder, the instructions below produce that exact file.

## Changes from GB-Link's v2.2.7

All stock modes are unchanged. Kizuna adds:

- **Mode 0x80**: GBA SIO-UART passthrough.
- **Mode 0x02**: `0x33` SetClockKhz and `0x34` SetParkSiHigh commands.
- **Mode 0x08**: a Game Boy-speed slave byte exchange, with a microsecond edge timeout.
- The GBA SI line is released to a pulled-up input after every mode.
- In wireless adapter mode (0x07) the SD soft-reset pins are ignored, because the adapter's 4-wire cable does not carry SD.
- USB identity: product name "Kizuna" and WebUSB landing page https://kizuna.trading. VID 0x2FE3 / PID 0x000A are unchanged.
- GetFirmwareInfo appends `['K', 13]` so the adapter can tell this fork from stock GB-Link firmware.

## Build

Needs Zephyr v3.7 (`v3.7-branch`; the release was built at commit `35ae4966`), the Zephyr SDK 0.16.8 and `west`.
With a Zephyr workspace set up as described in `west.yml`:

```bash
west build -p always -b rpi_pico rp2040
```

The image is `build/zephyr/zephyr.uf2`.

## Install

Flash the `.uf2` to the RP2040 by any UF2 method (in BOOTSEL mode it appears as a USB drive; copy the file onto it),
or write `build/zephyr/zephyr.bin` over SWD.
