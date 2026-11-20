/* SPDX-License-Identifier: AGPL-3.0-only
 * Derived from frlg-ldn-trade (https://github.com/tornadus/frlg-ldn-trade), Copyright (c) the frlg-ldn-trade authors.
 * Changes Copyright (c) 2026 Kizuna. See NOTICE.
 */
/* ni.h — librfu NI (reliable) handshake, portable C port of frlgsim/ni.py.
 * CHILD single-pass NI sender (RfuGameData delivery), recv-side NI ack, and RfuGameData build. */
#ifndef NI_H
#define NI_H
#include <stdint.h>
#include <stddef.h>
#include "rfu.h"

#define NI_WINDOW_COUNT     4
#define NI_CHILD_FRAME_SIZE 2
#define NI_RFU_SERIAL_GAME  0x0002
#define NI_HEADER_SIZE      7
#define NI_ACTIVITY_TRADE   0x04
#define NI_LANGUAGE_ENGLISH 2
#define NI_LANGUAGE_JAPANESE 1   /* pokefirered: LANGUAGE_JAPANESE = 1 (compat low nibble) */
#define NI_STATUS_JOIN_GROUP_OK 5

/* Build the 26-byte NI game-data src (serialNo + gname[15] + uname[9]). Returns 26. */
size_t ni_build_game_data(int version_low, uint16_t trainer_id, const char *ot_name,
                          int language, int activity, int started,
                          const uint8_t partner_info[4], uint8_t out[26]);

/* As above, but takes the trainer name as RAW FRLG-charmap bytes (8, 0xFF/0x00-padded) instead of
 * an ASCII string. Required for a JAPANESE cartridge: the GBA's own name bytes are kana (e.g.
 * としあき = 14 0c 01 07), which charmap_encode (ASCII-only: A-Z/a-z/0-9) cannot represent — it
 * would drop them. Passing the cartridge's bytes through verbatim is both lossless and correct,
 * since both ends use the same FRLG charmap. Returns 26. */
size_t ni_build_game_data_raw(int version_low, uint16_t trainer_id, const uint8_t ot_name8[8],
                              int language, int activity, int started,
                              const uint8_t partner_info[4], uint8_t out[26]);

/* 7-byte NI_START header: dataType(1) + payload_size(2 LE) + data_size(4 LE). */
void ni_header(int data_type, uint16_t payload_size, uint32_t data_size, uint8_t out7[7]);

typedef enum { NI_SEND_START, NI_SENDING, NI_SEND_LAST, NI_SEND_NULL, NI_DONE } ni_state;

#define NI_EMIT_MAX  12   /* game data tiles to < ~6 sub-frames; 12 is ample */
#define NI_SLOT_MAX  16   /* 2 LLSF + up to 12 payload */
typedef struct { int state, n, phase; uint8_t slot[NI_SLOT_MAX]; int slot_len; } ni_emit_t;

typedef struct {
    uint8_t  src[64];
    size_t   data_size;
    int      data_type;
    int      payload_size;
    ni_state state;
    int      phase;
    int      n[NI_WINDOW_COUNT];
    int      remain;
    int      now[NI_WINDOW_COUNT];
    uint8_t  header[NI_HEADER_SIZE];
    ni_emit_t emitted[NI_EMIT_MAX];   /* every emitted sub-frame, for re-send under loss */
    int      emitted_n;
} ni_sender;
void ni_sender_init(ni_sender *s, const uint8_t *src, size_t src_len,
                    int sub_frame_size, int data_type);
/* Emit the next NI sub-frame slot (LLSF + payload) into out; returns 1, or 0 when done. */
int  ni_sender_next_slot(ni_sender *s, uint8_t *out, size_t *out_len);
int  ni_sender_done(const ni_sender *s);
/* Last emitted sub-frame (state/n/phase + bytes). Returns 1 if any emitted, else 0. */
int  ni_sender_last_emitted(const ni_sender *s, int *state, int *n, int *phase,
                            uint8_t *out, size_t *out_len);
/* The sub-frame emitted AFTER the one matching (state,n,phase). Returns 1 + bytes, else 0. */
int  ni_sender_resend_after(const ni_sender *s, int state, int n, int phase,
                            uint8_t *out, size_t *out_len);

/* CHILD recv-side NI ack (mirror state/n/phase, ack=1, size=0). Returns 2. */
size_t ni_recv_ack_slot(int state, int n, int phase, uint8_t out2[2]);

typedef struct { int complete; int status; int has_status; } ni_receiver;
void   ni_receiver_init(ni_receiver *r);
/* Given a host NI sub-frame, return the ack slot len (writes out2) or 0 (no ack). */
size_t ni_receiver_on_host_ni(ni_receiver *r, int state, int ack, int n, int phase,
                              const uint8_t *payload, size_t payload_len, uint8_t out2[2]);

#endif
