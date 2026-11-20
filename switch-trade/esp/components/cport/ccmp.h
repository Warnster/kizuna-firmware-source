/* SPDX-License-Identifier: AGPL-3.0-only
 * Derived from pokeldn (https://github.com/Decryptu/pokeldn), Copyright (c) the pokeldn authors.
 * Derived from LDN (https://github.com/kinnay/LDN), Copyright (c) kinnay and the LDN authors.
 * Changes Copyright (c) 2026 Kizuna. See NOTICE.
 */
/* 802.11 CCMP (AES-CCM) for LDN data frames — C port of the nonce/AAD construction in
 * pokeldn vendor/LDN/ldn/wlan.py (_nonce/_aad/encrypt/decrypt). This is the software-CCMP the
 * ESP32 host needs because esp_wifi cannot install LDN's pre-shared CCMP key (no 4-way
 * handshake). Data key = ldn_derive_data_key(server_random, password). */
#ifndef CCMP_H
#define CCMP_H
#include <stdint.h>
#include <stddef.h>

/* 13-byte CCMP nonce: priority(1) + A2/source(6) + PN(6, big-endian). priority = tid for QoS. */
void ccmp_nonce(uint8_t priority, const uint8_t a2[6], uint64_t pn, uint8_t out[13]);

/* CCMP AAD (matches wlan.py _aad): FC(2 LE) + A1 + A2 + A3 + frag(2)=0 [+ tid(2 LE) if qos].
 * protected bit is always set (a CCMP frame is protected). Returns AAD length (24 or 22). */
int ccmp_aad(int qos, int tods, int fromds, uint16_t tid,
             const uint8_t a1[6], const uint8_t a2[6], const uint8_t a3[6], uint8_t out[32]);

/* Encrypt a data-frame payload. Writes ct(pt_len) followed by the 8-byte MIC into out.
 * Returns out length (pt_len+8), or -1. QoS data frame; priority=tid. */
int ccmp_encrypt(const uint8_t key[16], uint64_t pn, int qos, int tods, int fromds, uint16_t tid,
                 const uint8_t a1[6], const uint8_t a2[6], const uint8_t a3[6],
                 const uint8_t *pt, int pt_len, uint8_t *out);

/* Decrypt + verify (input = ct||8-byte MIC). Writes plaintext (len-8) into out.
 * Returns plaintext length, or -1 on auth failure / bad length. */
int ccmp_decrypt(const uint8_t key[16], uint64_t pn, int qos, int tods, int fromds, uint16_t tid,
                 const uint8_t a1[6], const uint8_t a2[6], const uint8_t a3[6],
                 const uint8_t *ct_mic, int ct_mic_len, uint8_t *out);

#endif
