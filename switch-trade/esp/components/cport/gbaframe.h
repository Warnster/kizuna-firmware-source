/* SPDX-License-Identifier: AGPL-3.0-only
 * Derived from frlg-ldn-trade (https://github.com/tornadus/frlg-ldn-trade), Copyright (c) the frlg-ldn-trade authors.
 * Changes Copyright (c) 2026 Kizuna. See NOTICE.
 */
/* gbaframe.h — emulator 0x57 frame layer, portable C port of frlgsim/gbaframe.py.
 * The Switch adapter's wrapper around the RFU command slot: `57 <type> <len:u16 LE> <body>`.
 * Types: C=0x43 connect, A=0x41 accept, T=0x54 slot-data, K=0x4b ack, D=0x44 disconnect. */
#ifndef GBAFRAME_H
#define GBAFRAME_H
#include <stdint.h>
#include <stddef.h>
#include "rfu.h"

#define GBA_MARKER   0x57
#define GBA_TYPE_C   0x43
#define GBA_TYPE_A   0x41
#define GBA_TYPE_T   0x54
#define GBA_TYPE_K   0x4B
#define GBA_TYPE_J   0x4A
#define GBA_TYPE_D   0x44
#define GBA_TYPE_G   0x47
#define GBA_SLOT_LEN 14
#define GBA_MAX_SLOTS 5

/* Builders (return bytes written). `out` must be large enough. */
size_t gba_build_frame(uint8_t ftype, const uint8_t *data, size_t dlen, uint8_t *out);
size_t gba_build_connect(const uint8_t *connect_id, size_t idlen, uint8_t *out);
size_t gba_wrap_t(const uint8_t *slot, size_t slot_len, uint32_t ts, uint8_t *out);
size_t gba_build_k(uint32_t k_seq, uint32_t mid, uint32_t acked_ts, uint8_t out[16]);

/* Parent/leader side (pokeldn gba/gbaframe.py): the host 'T' carries slot_len at body[4]. */
size_t gba_wrap_t_parent(const uint8_t *slot, size_t slot_len, uint32_t ts, uint8_t *out);
/* 'A' body <host_session_id:2><echoed connect_id:2><00 00>. */
size_t gba_build_accept(const uint8_t host_session_id[2], const uint8_t connect_id[2], uint8_t out[10]);
/* 'G' <value:u32>: 0 after the 'A', 1 once the parent has the child's NI (emulator-to-emulator). */
size_t gba_build_link_state(uint32_t value, uint8_t out[8]);
size_t gba_build_disconnect(const uint8_t connect_id[2], uint8_t out[6]);

/* Parse a HOST/parent frame (parse_in). */
typedef struct {
    int      valid;
    uint8_t  type;             /* 'T' 0x54 / 'A' 0x41 / 'K' 0x4b / other raw */
    /* T */
    int      ts_valid;
    uint32_t ts;
    uint8_t  slot_len;
    int      llsf_state;       /* -1 if n/a */
    int      nslots;           /* UNI: N x 14-byte gRecvCmds (mpId = index) */
    uint8_t  slots[GBA_MAX_SLOTS][GBA_SLOT_LEN];
    int      ni_ack, ni_n, ni_phase, ni_size;   /* non-UNI T (NI window); state = llsf_state */
    uint8_t  ni_payload[64];                     /* NI sub-frame payload (slot[3:]); has_ni set */
    int      ni_payload_len;
    int      has_ni;                             /* 1 iff a non-UNI NI sub-frame was parsed */
    /* A */
    uint8_t  host_session_id[2];
    uint8_t  connect_id[2];
    /* K */
    uint32_t k_seq;
    int      acked_ts_valid;
    uint32_t acked_ts;
} gba_in;
int gba_parse_in(const uint8_t *payload, size_t len, gba_in *out);

/* Parse a CHILD/joiner 'T' frame we emit (parse_out) — inverse of wrap_t. */
typedef struct {
    int      valid;
    uint32_t ts;
    uint8_t  slot_len;
    uint8_t  slot[64];
    int      have_llsf;
    rfu_child_llsf llsf;
    int      have_cmd;         /* UNI slot -> 14-byte gSendCmd in cmd[] */
    uint8_t  cmd[GBA_SLOT_LEN];
} gba_out;
int gba_parse_out(const uint8_t *payload, size_t len, gba_out *out);

#endif
