# Kizuna firmware source

Source code for the Kizuna adapter firmware that contains open-source (copyleft) code, published as those licences
require: the ESP32-C3 modes below under the GNU Affero General Public License v3.0, and the RP2040 firmware under the
GNU General Public License v3.0.

| Folder | Firmware images | Released versions (tags) |
|---|---|---|
| [`switch-trade/`](switch-trade/) | Switch Trade, Switch Trade for Ruby/Sapphire | `switch-trade-v2.5` (also covers `switch-trade-rs-v1.5`) |
| [`switch-online/`](switch-online/) | Switch Online | `switch-online-v1.1` |
| [`rp2040/`](rp2040/) | RP2040 firmware (fork of GB-Link's GBLink-Firmware); see `rp2040/KIZUNA.md` | `rp2040-kizuna.13` |

Each folder is the complete source of the images it lists. The ESP32-C3 folders hold every file compiled into
their images, the build files, a `NOTICE` naming the upstream projects the code comes from, and build and install
instructions in their `README.md`. `rp2040/` is the full source tree of the fork, with build instructions in `KIZUNA.md`.
Each release replaces its folder and is tagged; check out a tag for the exact source of that version.

Other Kizuna modes, the Kizuna app and the online service are not part of this repository.

This repository is updated from Kizuna's release process. It does not accept pull requests.
