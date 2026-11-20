/* SPDX-License-Identifier: AGPL-3.0-only
 * Derived from pokeldn (https://github.com/Decryptu/pokeldn), Copyright (c) the pokeldn authors.
 * Changes Copyright (c) 2026 Kizuna. See NOTICE.
 */
/* mg_link.h — FRLG MysteryGiftLink framing, portable C port of pokeldn frlg/gift/mg_link.py +
 * mystery_gift.py [decomp:src/mystery_gift_link.c]. A message is block 0 = {u16 ident; u16 crc;
 * u16 size} then <=252-byte chunks, each sent as its own RFU block. A requested size of 0 means
 * the whole 1024-byte buffer (RAM script, READY_END); the CRC covers the declared size, padding
 * included. We are the server (RFU parent); the console runs MysteryGiftClient. Pure/deterministic. */
#ifndef MG_LINK_H
#define MG_LINK_H
#include <stdint.h>
#include <stddef.h>

#define MG_LINK_BUFFER_SIZE 1024
#define MG_LINK_HEADER_SIZE 6
#define MG_LINK_MAX_CHUNK   252
#define MG_LINK_MAX_BLOCKS  (1 + (MG_LINK_BUFFER_SIZE + MG_LINK_MAX_CHUNK - 1) / MG_LINK_MAX_CHUNK)

/* Message idents [include/mystery_gift_link.h:8-22]. */
enum {
    MG_LINKID_CLIENT_SCRIPT   = 16, /* server -> client: client-script instruction array */
    MG_LINKID_GAME_DATA       = 17, /* client -> server: MysteryGiftLinkGameData */
    MG_LINKID_GAME_STAT       = 18,
    MG_LINKID_RESPONSE        = 19, /* client -> server: u32 yes/no/toss answer */
    MG_LINKID_READY_END       = 20, /* 1024-byte wire payload */
    MG_LINKID_DYNAMIC_MSG     = 21,
    MG_LINKID_CARD            = 22, /* server -> client: struct WonderCard (332 B) */
    MG_LINKID_NEWS            = 23,
    MG_LINKID_STAMP           = 24,
    MG_LINKID_RAM_SCRIPT      = 25, /* server -> client: delivery script bytecode */
    MG_LINKID_EREADER_TRAINER = 26,
};

/* CalcCRC16WithTable [src/util.c:250]: reflected, init 0x1121, poly 0x8408, final complement. */
uint16_t mg_crc16(const uint8_t *data, size_t len);

/* An outgoing message, padded and split. */
typedef struct {
    uint16_t ident, crc, size;           /* size as sent: 1..1024 */
    uint8_t  header[MG_LINK_HEADER_SIZE];
    uint8_t  buf[MG_LINK_BUFFER_SIZE];   /* payload zero-padded to size */
    int      nblocks;                    /* header + chunks */
} mg_msg;

/* size == 0 pads to the full 1024-byte buffer; otherwise size must be >= len and <= 1024.
 * Pass size = len for an exact message. Returns 0, or -1 if the sizes are out of range. */
int mg_msg_build(mg_msg *m, uint16_t ident, const uint8_t *payload, size_t len, size_t size);

/* Block i of the message (0 = header). Returns its length; *out points into m. 0 if i is out of range. */
size_t mg_msg_block(const mg_msg *m, int i, const uint8_t **out);

/* Reassembles one message from the console. RFU blocks arrive padded to 12-byte fragments,
 * so each block is sliced to what the header declared. */
typedef enum {
    MG_RX_PENDING  = 0,
    MG_RX_COMPLETE = 1,
    MG_RX_ERR_SHORT     = -1, /* block shorter than the header or the chunk it must carry */
    MG_RX_ERR_SIZE      = -2, /* declared size 0 or > 1024 */
    MG_RX_ERR_IDENT     = -3, /* not the ident we expect */
    MG_RX_ERR_CRC       = -4,
    MG_RX_ERR_IDLE      = -5, /* fed with no message expected */
} mg_rx_result;

typedef struct {
    int      expected_ident;             /* -1 = none */
    int      in_header;
    uint16_t ident, crc, size;
    uint8_t  buf[MG_LINK_BUFFER_SIZE];
    size_t   len;
    int      blocks;
} mg_rx;

void         mg_rx_init(mg_rx *rx);
void         mg_rx_expect(mg_rx *rx, uint16_t ident);
int          mg_rx_active(const mg_rx *rx);
/* On MG_RX_COMPLETE the payload is rx->buf[0 .. rx->size) and the receiver goes idle. */
mg_rx_result mg_rx_feed(mg_rx *rx, const uint8_t *block, size_t len);

#endif
