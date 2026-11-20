/* SPDX-License-Identifier: AGPL-3.0-only
 * Derived from pokeldn (https://github.com/Decryptu/pokeldn), Copyright (c) the pokeldn authors.
 * Derived from LDN (https://github.com/kinnay/LDN), Copyright (c) kinnay and the LDN authors.
 * Changes Copyright (c) 2026 Kizuna. See NOTICE.
 */
/* ldn_auth.h — build the LDN AuthenticationFrame (protocol 3 / AES-GCM) that registers a joiner as
 * a session participant, so the host assigns it an IP. C port of kinnay/LDN AuthenticationFrame +
 * AuthenticationRequest + ChallengeRequest (see vendor/LDN/ldn/__init__.py). The frame is sent as
 * an Ethernet-II 0x88b7 data frame (esp_wifi_internal_tx) once the CCMP link is up.
 *
 * All the derived-key/GCM/SHA come from ldn_advert.h + aes_backend.h. */
#ifndef LDN_AUTH_H
#define LDN_AUTH_H
#include <stdint.h>
#include <stddef.h>
#include "ldn_advert.h"

/* Build the ~962-byte authentication-request frame (starts with 00 22 aa 01 02). Inputs are the
 * fields read off the room's advertisement plus fresh randoms. protocol is 3 for FRLG.
 * Returns the frame length, or -1 on error / out too small (out_cap should be >= 1024). */
int ldn_build_auth_frame(const ldn_keys *k, int protocol, int version,
                         uint64_t comm_id, uint16_t scene_id, const uint8_t ssid[16],
                         const uint8_t server_random[16], const uint8_t client_random[16],
                         uint64_t challenge_token, uint64_t challenge_nonce,
                         const char *username, uint16_t app_version,
                         uint8_t *out, int out_cap);

#endif
