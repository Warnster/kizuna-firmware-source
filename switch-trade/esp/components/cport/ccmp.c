/* SPDX-License-Identifier: AGPL-3.0-only
 * Derived from pokeldn (https://github.com/Decryptu/pokeldn), Copyright (c) the pokeldn authors.
 * Derived from LDN (https://github.com/kinnay/LDN), Copyright (c) kinnay and the LDN authors.
 * Changes Copyright (c) 2026 Kizuna. See NOTICE.
 */
/* 802.11 CCMP — C port of pokeldn vendor/LDN/ldn/wlan.py nonce/AAD + AES-CCM. */
#include "ccmp.h"
#include "aes_backend.h"
#include <string.h>

#define IEEE80211_FTYPE_DATA 2

void ccmp_nonce(uint8_t priority, const uint8_t a2[6], uint64_t pn, uint8_t out[13]) {
    out[0] = priority;
    memcpy(out + 1, a2, 6);
    /* PN as the low 6 bytes of a big-endian u64 (struct.pack(">Q", pn)[2:]) */
    for (int i = 0; i < 6; i++) out[7 + i] = (uint8_t)(pn >> (8 * (5 - i)));
}

int ccmp_aad(int qos, int tods, int fromds, uint16_t tid,
             const uint8_t a1[6], const uint8_t a2[6], const uint8_t a3[6], uint8_t out[32]) {
    uint16_t fc = IEEE80211_FTYPE_DATA << 2;
    if (qos) fc |= 8 << 4;
    fc |= (uint16_t)(tods << 8);
    fc |= (uint16_t)(fromds << 9);
    fc |= (uint16_t)(1 << 14);   /* protected — a CCMP frame is always protected */
    int n = 0;
    out[n++] = (uint8_t)(fc & 0xff); out[n++] = (uint8_t)(fc >> 8);  /* FC little-endian */
    memcpy(out + n, a1, 6); n += 6;
    memcpy(out + n, a2, 6); n += 6;
    memcpy(out + n, a3, 6); n += 6;
    out[n++] = 0; out[n++] = 0;   /* fragment number */
    if (qos) { out[n++] = (uint8_t)(tid & 0xff); out[n++] = (uint8_t)(tid >> 8); }  /* QoS control (TID) */
    return n;
}

int ccmp_encrypt(const uint8_t key[16], uint64_t pn, int qos, int tods, int fromds, uint16_t tid,
                 const uint8_t a1[6], const uint8_t a2[6], const uint8_t a3[6],
                 const uint8_t *pt, int pt_len, uint8_t *out) {
    uint8_t nonce[13], aad[32];
    ccmp_nonce((uint8_t)(qos ? tid : 0), a2, pn, nonce);
    int aad_len = ccmp_aad(qos, tods, fromds, tid, a1, a2, a3, aad);
    if (aes128_ccm_encrypt(key, nonce, 13, aad, aad_len, pt, pt_len, out, out + pt_len, 8) != 0)
        return -1;
    return pt_len + 8;
}

int ccmp_decrypt(const uint8_t key[16], uint64_t pn, int qos, int tods, int fromds, uint16_t tid,
                 const uint8_t a1[6], const uint8_t a2[6], const uint8_t a3[6],
                 const uint8_t *ct_mic, int ct_mic_len, uint8_t *out) {
    if (ct_mic_len < 8) return -1;
    int ct_len = ct_mic_len - 8;
    uint8_t nonce[13], aad[32];
    ccmp_nonce((uint8_t)(qos ? tid : 0), a2, pn, nonce);
    int aad_len = ccmp_aad(qos, tods, fromds, tid, a1, a2, a3, aad);
    if (aes128_ccm_decrypt(key, nonce, 13, aad, aad_len, ct_mic, ct_len,
                           ct_mic + ct_len, 8, out) != 0)
        return -1;
    return ct_len;
}
