/* rfu1 — codec for the "RFU1" network packets GB-Link's wireless-adapter mode (0x07) streams on the GB
 * transport's data channel. This is gpsp's RFU netpacket layout, a public wire format;
 * the code here is written from the field table, not from any GPL/AGPL source.
 *
 * Packet = 12-byte big-endian header { magic "RFU1", ptype, hdata } + a fixed-size payload per ptype.
 * Packets may arrive split across, or packed into, UART frames; rfu1_rx_t reassembles a byte stream. */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define RFU1_MAGIC       0x52465531u   /* 'R' 'F' 'U' '1' */
#define RFU1_HDR_LEN     12
#define RFU1_MAX_LEN     104           /* HOST_SEND / CLIENT_SEND */
#define RFU1_SEND_DATA   92            /* payload bytes carried by the two SEND types */

enum {
    RFU1_BROADCAST    = 0x00,   /* 36 B: 6 BE words of 0x16 broadcast data; hdata = devid | slot<<16 */
    RFU1_CONNECT_REQ  = 0x01,   /* 16 B: hdata = target devid */
    RFU1_CONNECT_ACK  = 0x02,   /* 16 B: hdata = devid | clnum<<16 */
    RFU1_CONNECT_NACK = 0x03,   /* 16 B */
    RFU1_DISCONNECT   = 0x04,   /* 16 B */
    RFU1_HOST_SEND    = 0x05,   /* 104 B: parent->child, hdata bits 0-6 = byte length */
    RFU1_CLIENT_SEND  = 0x06,   /* 104 B: child->parent, hdata = devid | clnum<<16 | len<<24 */
    RFU1_CLIENT_ACK   = 0x07,   /* 16 B: hdata = devid | clnum<<16 */
    RFU1_FLOWCTL      = 0x08,   /* 16 B: hdata bit0 = hold, bits 8-15 = queue depth */
    RFU1_NUM_TYPES    = 9,
};

typedef struct {
    uint32_t ptype;
    uint32_t hdata;
    uint32_t len;                 /* total packet length incl. header */
    uint8_t  payload[RFU1_MAX_LEN - RFU1_HDR_LEN];
} rfu1_pkt_t;

/* Fixed total length for a ptype, 0 if unknown. */
uint32_t rfu1_len_for(uint32_t ptype);
const char *rfu1_name(uint32_t ptype);

/* ---- serializer: writes a complete packet into out (>= rfu1_len_for(ptype)); returns its length. ---- */
uint32_t rfu1_put_u32be(uint8_t *p, uint32_t v);
uint32_t rfu1_build(uint8_t *out, uint32_t ptype, uint32_t hdata, const uint8_t *payload, uint32_t plen);
uint32_t rfu1_build_broadcast(uint8_t *out, uint16_t devid, uint8_t next_slot, const uint32_t words[6]);
uint32_t rfu1_build_connect_ack(uint8_t *out, uint16_t devid, uint8_t clnum, bool ack);
uint32_t rfu1_build_host_send(uint8_t *out, const uint8_t *data, uint32_t len);   /* len <= 92 */
uint32_t rfu1_build_client_ack(uint8_t *out, uint16_t devid, uint8_t clnum);
uint32_t rfu1_build_flowctl(uint8_t *out, bool hold, uint8_t depth);
uint32_t rfu1_build_disconnect(uint8_t *out, uint16_t devid);

/* ---- field helpers ---- */
static inline uint16_t rfu1_devid(uint32_t hdata)     { return (uint16_t)(hdata & 0xFFFF); }
static inline uint8_t  rfu1_clnum(uint32_t hdata)     { return (uint8_t)((hdata >> 16) & 0x3); }
static inline uint8_t  rfu1_bc_slot(uint32_t hdata)   { return (uint8_t)((hdata >> 16) & 0xFF); }
static inline uint32_t rfu1_host_len(uint32_t hdata)  { return hdata & 0x7F; }
static inline uint32_t rfu1_client_len(uint32_t hdata){ return (hdata >> 24) & 0xFF; }
static inline bool     rfu1_fc_hold(uint32_t hdata)   { return (hdata & 1) != 0; }
static inline uint8_t  rfu1_fc_depth(uint32_t hdata)  { return (uint8_t)((hdata >> 8) & 0xFF); }

/* ---- stream reassembler: feed UART data-channel bytes, pull complete packets. ----
 * Resyncs on the magic; zero padding between packets (GB-Link pads chunk tails) is skipped. */
typedef struct {
    uint8_t  buf[RFU1_MAX_LEN];
    uint32_t have;
    uint32_t resyncs, bad_len, packets;
} rfu1_rx_t;

void rfu1_rx_init(rfu1_rx_t *rx);
/* Feed n bytes; calls cb for each complete packet (in order). Returns the number of packets delivered. */
typedef void (*rfu1_pkt_cb_t)(const rfu1_pkt_t *pkt, void *ctx);
int  rfu1_rx_feed(rfu1_rx_t *rx, const uint8_t *data, size_t n, rfu1_pkt_cb_t cb, void *ctx);
/* Parse one complete packet from a buffer (no reassembly). Returns bytes consumed or 0. */
uint32_t rfu1_parse(const uint8_t *data, size_t n, rfu1_pkt_t *out);
