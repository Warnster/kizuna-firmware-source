/* SPDX-License-Identifier: AGPL-3.0-only
 * Derived from frlg-ldn-trade (https://github.com/tornadus/frlg-ldn-trade), Copyright (c) the frlg-ldn-trade authors.
 * Changes Copyright (c) 2026 Kizuna. See NOTICE.
 */
/* crypto.h — Pia AES-GCM transport crypto, portable C port of frlgsim/crypto.py.
 * Port logic (CRC32, key derivation, nonce, header, GCM framing) lives here; the raw AES
 * comes from aes_backend.h. zstd (compress/decompress) is NOT ported here — separate concern.
 *
 *   session_key = AES-128-ECB(game_key, ssid)   (ssid = one 16-byte block)
 *   net_id      = CRC32(ssid[1:16])
 *   GCM nonce   = (net_id ^ src_ip_be)(4 BE) || header_nonce8(8)   [12 bytes]
 *   AAD         = empty ; tag = first 8 bytes of the 16-byte GCM tag */
#ifndef PIA_CRYPTO_H
#define PIA_CRYPTO_H
#include <stdint.h>
#include <stddef.h>

#define PIA_HDR_PACK_LEN 21   /* bytes returned by pia_header_pack */
#define PIA_NONCE_OFF    13
#define PIA_TAG_OFF      21
#define PIA_CT_OFF       29
#define PIA_TAG_LEN      8

extern const uint8_t FRLG_GAME_KEY[16];

/* zlib-compatible CRC32 (reflected, poly 0xEDB88320). */
uint32_t crc32_ieee(const uint8_t *data, size_t len);

typedef struct { uint8_t session_key[16]; uint32_t net_id; } pia_crypto;
void pia_crypto_init(pia_crypto *pc, const uint8_t ssid[16], const uint8_t game_key[16]);
void pia_nonce(const pia_crypto *pc, const uint8_t ip[4],
               const uint8_t hdr_nonce8[8], uint8_t out12[12]);

typedef struct {
    uint16_t dst, src, pktid;
    uint8_t  nonce8[8];
    uint8_t  enc, flags, footer;
} pia_header;
void pia_header_pack(const pia_header *h, uint8_t out[PIA_HDR_PACK_LEN]);
void pia_header_unpack(const uint8_t *dg, pia_header *h);
int  pia_is_pia(const uint8_t *dg, size_t len);

/* Build a full Pia datagram (header + 8-byte tag + ct) into out. Returns bytes written. */
size_t pia_encrypt(const pia_crypto *pc, const uint8_t *pt, size_t pt_len,
                   const uint8_t ip[4], const pia_header *h, uint8_t *out);

/* Decrypt+verify. Returns plaintext length into out, or -1 on auth failure / not-Pia. */
long pia_decrypt(const pia_crypto *pc, const uint8_t *dg, size_t dg_len,
                 const uint8_t ip[4], uint8_t *out);

#endif
