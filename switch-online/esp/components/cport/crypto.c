/* SPDX-License-Identifier: AGPL-3.0-only
 * Derived from frlg-ldn-trade (https://github.com/tornadus/frlg-ldn-trade), Copyright (c) the frlg-ldn-trade authors.
 * Changes Copyright (c) 2026 Kizuna. See NOTICE.
 */
/* crypto.c — portable C port of frlgsim/crypto.py. See crypto.h. */
#include "crypto.h"
#include "aes_backend.h"
#include <string.h>

const uint8_t FRLG_GAME_KEY[16] = {
    0x83, 0xca, 0x7f, 0xab, 0x73, 0x4c, 0x34, 0x63,
    0x3b, 0x10, 0x18, 0x35, 0x26, 0xc1, 0xe8, 0x5b
};

static const uint8_t PIA_MAGIC[4] = {0x32, 0xab, 0x98, 0x64};

uint32_t crc32_ieee(const uint8_t *data, size_t len) {
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int k = 0; k < 8; k++)
            crc = (crc >> 1) ^ ((crc & 1u) ? 0xEDB88320u : 0u);
    }
    return crc ^ 0xFFFFFFFFu;
}

void pia_crypto_init(pia_crypto *pc, const uint8_t ssid[16], const uint8_t game_key[16]) {
    aes128_ecb_encrypt_block(game_key, ssid, pc->session_key);
    pc->net_id = crc32_ieee(ssid + 1, 15);
}

void pia_nonce(const pia_crypto *pc, const uint8_t ip[4],
               const uint8_t hdr_nonce8[8], uint8_t out12[12]) {
    uint32_t ip_be = ((uint32_t)ip[0] << 24) | ((uint32_t)ip[1] << 16) |
                     ((uint32_t)ip[2] << 8)  | (uint32_t)ip[3];
    uint32_t four = pc->net_id ^ ip_be;
    out12[0] = (uint8_t)(four >> 24);
    out12[1] = (uint8_t)(four >> 16);
    out12[2] = (uint8_t)(four >> 8);
    out12[3] = (uint8_t)(four);
    memcpy(out12 + 4, hdr_nonce8, 8);
}

void pia_header_pack(const pia_header *h, uint8_t out[PIA_HDR_PACK_LEN]) {
    memcpy(out, PIA_MAGIC, 4);
    out[4]  = h->enc;
    out[5]  = h->flags;
    out[6]  = (uint8_t)(h->dst >> 8);
    out[7]  = (uint8_t)(h->dst);
    out[8]  = (uint8_t)(h->src >> 8);
    out[9]  = (uint8_t)(h->src);
    out[10] = (uint8_t)(h->pktid >> 8);
    out[11] = (uint8_t)(h->pktid);
    out[12] = h->footer;
    memcpy(out + 13, h->nonce8, 8);
}

void pia_header_unpack(const uint8_t *dg, pia_header *h) {
    h->enc    = dg[4];
    h->flags  = dg[5];
    h->dst    = (uint16_t)((dg[6] << 8) | dg[7]);
    h->src    = (uint16_t)((dg[8] << 8) | dg[9]);
    h->pktid  = (uint16_t)((dg[10] << 8) | dg[11]);
    h->footer = dg[12];
    memcpy(h->nonce8, dg + PIA_NONCE_OFF, 8);
}

int pia_is_pia(const uint8_t *dg, size_t len) {
    return len >= PIA_CT_OFF && memcmp(dg, PIA_MAGIC, 4) == 0;
}

size_t pia_encrypt(const pia_crypto *pc, const uint8_t *pt, size_t pt_len,
                   const uint8_t ip[4], const pia_header *h, uint8_t *out) {
    uint8_t nonce[12], tag16[16];
    pia_nonce(pc, ip, h->nonce8, nonce);
    pia_header_pack(h, out);                                  /* [0:21] */
    if (aes128_gcm_encrypt(pc->session_key, nonce, 12, NULL, 0,
                           pt, pt_len, out + PIA_CT_OFF, tag16) != 0)
        return 0;
    memcpy(out + PIA_TAG_OFF, tag16, PIA_TAG_LEN);            /* [21:29] truncated tag */
    return PIA_CT_OFF + pt_len;
}

long pia_decrypt(const pia_crypto *pc, const uint8_t *dg, size_t dg_len,
                 const uint8_t ip[4], uint8_t *out) {
    if (!pia_is_pia(dg, dg_len)) return -1;
    uint8_t nonce[12];
    pia_nonce(pc, ip, dg + PIA_NONCE_OFF, nonce);
    size_t ct_len = dg_len - PIA_CT_OFF;
    if (aes128_gcm_decrypt(pc->session_key, nonce, 12, NULL, 0,
                           dg + PIA_CT_OFF, ct_len,
                           dg + PIA_TAG_OFF, PIA_TAG_LEN, out) != 0)
        return -1;
    return (long)ct_len;
}
