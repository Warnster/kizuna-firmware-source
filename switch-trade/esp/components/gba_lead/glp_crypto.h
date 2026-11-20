/* SPDX-License-Identifier: AGPL-3.0-only
 * Derived from GB-Link-Switch-LDN (https://github.com/GB-Link/GB-Link-Switch-LDN), Copyright (c) the GB-Link-Switch-LDN authors.
 * Changes Copyright (c) 2026 Kizuna. See NOTICE.
 */
/* glp_crypto.h -- copied from GB-Link-Switch-LDN firmware/common/pia_crypto.h (commit c972db4, AGPL-3.0,
 * https://github.com/GB-Link/GB-Link-Switch-LDN); see NOTICE. Kizuna: identifiers renamed pia_ -> glp_, PIA_ -> GLP_,
 * bin_ -> glb_ so they sit beside cport's own Pia host in one image. */
#pragma once
#include "glp_bin.h"

/* Pia transport crypto and framing: AES-GCM datagram envelope, zstd body compression,
   message encoding. Mirror of the host's PiaCrypto. */

#define GLP_MAX_DATAGRAM 1472
#define GLP_MAX_BODY 2048
/* Decompressed body size. The Switch resends all unacked reliable frames in one datagram;
   largest seen: 74 messages, 7.2 KB, from 381 bytes on the wire. */
#define GLP_MAX_INFLATE 12288
#define GLP_MAX_MESSAGES 16

typedef struct
{
    uint8_t key[16];
    uint32_t net_id;
} glp_crypto_t;

typedef struct
{
    uint8_t protocol;
    uint8_t flags;
    bool has_flags;
    const uint8_t *payload;
    uint16_t length;
} glp_message_t;

/* AES-ECB, one block. `encrypt` false decrypts (host default). */
void glp_ecb(const uint8_t key[16], const uint8_t in[16], uint8_t out[16], bool encrypt);
/* AES-GCM with caller-chosen tag length. Returns false on tag mismatch. */
bool glp_gcm(const uint8_t *key, size_t key_len, const uint8_t *nonce, size_t nonce_len,
             const uint8_t *aad, size_t aad_len, const uint8_t *in, size_t in_len,
             uint8_t *out, uint8_t *tag, size_t tag_len, bool encrypt);
/* AES-CTR with a 4-byte prefix counter (v1 advertisement). */
void glp_ctr(const uint8_t key[16], const uint8_t nonce4[4], const uint8_t *in, size_t len, uint8_t *out);
void glp_sha256(const uint8_t *data, size_t len, uint8_t out[32]);
void glp_hmac_sha256(const uint8_t *key, size_t key_len, const uint8_t *data, size_t len, uint8_t out[32]);

void glp_crypto_init(glp_crypto_t *c, const uint8_t ssid[16]);

/* Decrypt a received datagram into `out`. Returns the plaintext length, or -1. */
int glp_decrypt(const glp_crypto_t *c, const uint8_t *datagram, size_t length,
                const char *source_ip, uint8_t *out, size_t out_cap);
/* Build a datagram from an already-assembled body. Returns its length, or -1. */
int glp_encrypt(const glp_crypto_t *c, const uint8_t *body, size_t body_len, const char *source_ip,
                uint16_t dst, uint16_t src, uint16_t packet, uint64_t nonce, uint8_t flags,
                uint8_t footer, uint8_t *out, size_t out_cap);

/* zstd: the Switch compresses most datagrams, so decoding is required. Sending uses raw
   blocks, which are valid zstd with the header Pia expects. */
/* Allocate the zstd decompression context before Wi-Fi init. It needs ~94 KB contiguous;
   after Wi-Fi init the largest free block is about half that. */
bool glp_crypto_prepare(void);
size_t glp_crypto_dctx_size(void);

/* Returns the decompressed length; -1 when the context is missing or a plain body does
   not fit; -(1000 + ZSTD_ErrorCode) when zstd rejects the frame (1070 = output too small). */
int glp_decompress(const uint8_t *in, size_t len, uint8_t *out, size_t out_cap);
int glp_compress_raw(const uint8_t *in, size_t len, uint8_t *out, size_t out_cap);

/* Message list encoding: [bits][flags?][size:2][protocol][payload]. */
int glp_message_encode(const glp_message_t *m, uint8_t *out, size_t out_cap);
int glp_messages_decode(const uint8_t *data, size_t len, glp_message_t *out, int max);
/* Iterator form of the above, no message count limit. */
typedef struct { const uint8_t *data; size_t len, pos; int size, proto; uint8_t flags; } glp_message_iter_t;
void glp_message_iter_init(glp_message_iter_t *it, const uint8_t *data, size_t len);
bool glp_message_next(glp_message_iter_t *it, glp_message_t *out);
