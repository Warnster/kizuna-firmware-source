# Kizuna firmware source: switch-trade v2.5, switch-trade-rs v1.5

Complete corresponding source for the Kizuna adapter firmware images below, as required by the
GNU Affero General Public License v3.0. See `LICENSE`, `NOTICE` and `LICENSES/`.

| Image | Version | App | Extra build flags |
|---|---|---|---|
| `switch-trade` | `v2.5` | `esp/switch-trade` | `(none)` |
| `switch-trade-rs` | `v1.5` | `esp/switch-trade` | `-DKZ_GBA_LINK=cable` |

Source commit: `655e1e6be82372d9602662b3ac8cab21dccdb2ec`.

## Build

Needs ESP-IDF v6.1 and an ESP32-C3. From the app folder:

```bash
idf.py -DIDF_TARGET=esp32c3 -DSDKCONFIG=sdkconfig.c3 -DPROJECT_VER=dev-switch-trade-655e1e6 build
idf.py -DIDF_TARGET=esp32c3 -DSDKCONFIG=sdkconfig.c3 -DKZ_GBA_LINK=cable -DPROJECT_VER=dev-switch-trade-rs-655e1e6 build
```

The app's `main/idf_component.yml` pulls managed components from the Espressif component registry at build time.

## Install

The adapter's ESP32-C3 can be flashed over USB with `idf.py -p <port> flash`. The adapter does not use secure
boot or flash encryption, so modified builds run.

## What is not included

- The Nintendo Switch local-wireless key material. It is not part of this source; the adapter receives it at
  run time and keeps it in NVS. Without it the Switch side will not connect.
- Other adapter modes and the RP2040 firmware. The RP2040 source (GPL-3.0) is in [`rp2040/`](../rp2040/).
