/* SPDX-License-Identifier: AGPL-3.0-only
 * Derived from pokeldn (https://github.com/Decryptu/pokeldn), Copyright (c) the pokeldn authors.
 * Changes Copyright (c) 2026 Kizuna. See NOTICE.
 */
/* Pia connection layer (Net + Session + RTT): the handshake the host completes before it
 * registers us as a peer. C port of the JOINER subset of pokeldn/ldn/pia_connect.py
 * (Pia 6.32+, header 0x1D). The LEADER's builders/parsers (build_net_conn_request, parse_session_join,
 * build_session_update/join_response/leave_response, NET_UPDATE_PROPERTY) are at the end of this
 * header, for pia_host.c; the joiner state machine below never uses them.
 *
 * Station var-ids are ASSIGNED per session: the wire header is [dst_var][src_var] and the
 * footer is the destination var, both learned from the first incoming packet. The joiner's own
 * Session-layer 8-byte constant id is simply the 6-byte MAC + 0x0000 (no permutation); a leader
 * advertises the permuted pc_ldn_constant_id, which the joiner echoes from the Net 0x11.
 *
 * Times/ticks are VBlank counts (as in the Python), matching sim.py. */
#ifndef PIA_CONNECT_H
#define PIA_CONNECT_H
#include <stdint.h>
#include <stddef.h>

#define PC_PROTO_NET      1
#define PC_PROTO_RTT      3
#define PC_PROTO_RELIABLE 10
#define PC_PROTO_SESSION  13

#define PC_SESSION_VAR          0x0001
#define PC_RTT_ORIGINATE_PERIOD 10

#define PC_NET_CONN_REQUEST     0x11
#define PC_NET_CONN_RESPONSE    0x12
#define PC_NET_UPDATE_PROPERTY  0x50
#define PC_NET_UPDATE_PROPERTY_ACK 0x51

#define PC_SESSION_JOIN_REQUEST  0
#define PC_SESSION_JOIN_RESPONSE 2
#define PC_SESSION_UPDATE        5
#define PC_SESSION_UPDATE_ACK    6
#define PC_SESSION_LEFT_SYNC     7

#define PC_DEFAULT_OUR_VAR 0xc493

/* --- stateless builders / parsers (return bytes written, or 0/-1 on failure) --- */

/* The Session/finalize 8-byte constant id: mac(6) + 0x0000. Kept for callers/tests. */
void pc_constant_id8(const uint8_t mac[6], uint8_t out[8]);

/* parse_net: whole body from offset 4. Returns 0 ok (sets *t0,*t1,*body,*blen), -1 too short. */
int pc_parse_net(const uint8_t *p, int n, uint8_t *t0, uint8_t *t1,
                 const uint8_t **body, int *blen);
/* parse a Net 0x11 -> host_var, host_mac(6), seqid. Returns 0 ok, -1 not a conn request. */
int pc_parse_net_conn_request(const uint8_t *p, int n,
                              uint16_t *host_var, uint8_t host_mac[6], uint32_t *seqid);

int pc_build_net_response(uint32_t seqid, uint8_t out[8]);      /* 8 bytes */
int pc_build_net_property_ack(uint32_t seqid, uint8_t out[8]);  /* 8 bytes */

typedef struct { uint8_t type; uint8_t version; uint8_t systime[8];
                 uint8_t subject[2]; int subject_len; } pc_rtt_t;
/* parse_rtt. Returns 0 ok, -1 too short (<16). */
int pc_parse_rtt(const uint8_t *p, int n, pc_rtt_t *out);
int pc_build_rtt_response(const uint8_t *request, int rlen, uint8_t out[21]);
int pc_build_rtt_request(const uint8_t *templ, int tlen, uint64_t systime, uint8_t out[21]);

/* Session join (variable length). out must hold >=160. Returns bytes written. */
int pc_build_session_join(const uint8_t our_mac[6], uint16_t our_var, const uint8_t our_ip[4],
                          const uint8_t host_mac[6], uint16_t host_var,
                          const char *player_name, const uint8_t random4[4],
                          const uint8_t player_id[16], uint8_t *out);
int pc_build_session_finalize(const uint8_t our_mac[6], uint8_t out[15]); /* 15 bytes */

/* parse_session: type + optional count. Returns 0 ok, -1 empty. */
int pc_parse_session(const uint8_t *p, int n, uint8_t *type, uint8_t *count, int *has_count);

/* --- ConnectionManager state machine --- */
typedef enum { PC_ST_NET = 0, PC_ST_FINALIZE, PC_ST_CONNECTED } pc_state_t;

#define PC_OUT_MAXPAYLOAD 160
#define PC_OUT_MAX        8
#define PC_RTT_PENDING_MAX 64
#define PC_RTT_SAMPLES_MAX 32

/* One queued outbound Pia message; sim.c frames it (header/footer/compress/pktid). */
typedef struct {
    uint8_t  proto;
    uint8_t  payload[PC_OUT_MAXPAYLOAD];
    int      payload_len;
    uint16_t dst, src;
    int      compress;      /* bool: zstd the payload */
    int      footer;        /* bool: append station footer */
    int      establishing;  /* bool: connection-setup frame */
    int      unicast;       /* always 1 */
    int      pktid;         /* -1 => per-channel counter; else forced value */
    int      footer_var;    /* -1 => none; else override recipient var */
} pc_out_t;

typedef struct { uint64_t systime; int32_t tick; int used; } pc_rtt_pend_t;

typedef struct {
    uint8_t   our_mac[6], host_mac[6];
    uint8_t   our_ip[4], host_ip[4];
    uint16_t  our_var;
    int       has_host_var; uint16_t host_var;
    uint8_t   player_name[21]; int player_name_len;
    uint8_t   random4[4];
    uint8_t   player_id[16];
    pc_state_t state;

    int       rtt_before_finalize;
    int       join_repeat_ticks;
    int       has_join_sent; int32_t join_sent_tick;

    pc_out_t  outbox[PC_OUT_MAX]; int outbox_n;

    int       has_last_host_rtt; uint8_t last_host_rtt[21];
    uint64_t  rtt_systime;
    int32_t   rtt_orig_tick;
    pc_rtt_pend_t rtt_pending[PC_RTT_PENDING_MAX];
    int32_t   rtt_samples[PC_RTT_SAMPLES_MAX]; int rtt_samples_n;
} pc_conn_t;

/* our_ip/host_ip may be NULL (treated as 0.0.0.0). player_id NULL => default. */
void pc_conn_init(pc_conn_t *c, const uint8_t our_mac[6], const uint8_t host_mac[6],
                  const uint8_t our_ip[4], const uint8_t host_ip[4],
                  uint16_t our_var, const char *player_name, const uint8_t random4[4],
                  const uint8_t player_id[16]);
int  pc_conn_connected(const pc_conn_t *c);
void pc_conn_learn_ids(pc_conn_t *c, const int *our_var, const int *host_var);
/* Feed an inbound Pia message. tick<0 => "no tick" (Python None). */
void pc_conn_on_message(pc_conn_t *c, uint8_t proto, const uint8_t *payload, int len, int32_t tick);
void pc_conn_maybe_originate_rtt(pc_conn_t *c, int32_t tick);
void pc_conn_maybe_repeat_join(pc_conn_t *c, int32_t tick);
/* Hand the caller the queued entries and clear the outbox. Returns count; *out points at internal storage. */
int  pc_conn_drain(pc_conn_t *c, const pc_out_t **out);

/* ===== LEADER (host) side: pokeldn ldn/pia_connect.py + host_pia.build_net_property_update ===== */

#define PC_SESSION_LEAVE_REQUEST  3
#define PC_SESSION_LEAVE_RESPONSE 4
#define PC_PIA_PORT               12345
#define PC_NET_STATION_SIZE       22     /* Pia 6.39 NetStation: 4-byte prefix, 16-byte address, port */
#define PC_APP_HEADER_SIZE        0x5C   /* PIA_APPLICATION_HEADER_SIZE */

#define PC_JOIN_MAX_PROTOCOLS 16
#define PC_JOIN_MAX_PLAYERS   4
#define PC_PLAYER_NAME_MAX    40         /* _parse_player_info rejects longer */

typedef struct { uint8_t player_id[16]; uint8_t encoding; uint8_t name_len; uint8_t name[PC_PLAYER_NAME_MAX]; } pc_player_t;

/* parse_session_join, IPv4 only. */
typedef struct {
    int      nprotocols;
    uint8_t  protocols[PC_JOIN_MAX_PROTOCOLS][2];   /* (id, version) */
    uint8_t  app_ver[2], random4[4];
    uint8_t  source_constant_id[8];
    uint16_t source_var;
    uint8_t  nat_mapping, private_ipv6;
    uint8_t  token[32];
    uint8_t  destination_constant_id[8];
    uint16_t destination_var;
    uint8_t  num_players, num_participants;
    uint8_t  ip[4];
    uint16_t port;
    pc_player_t players[PC_JOIN_MAX_PLAYERS];
} pc_session_join_t;

/* The LDN constant id: mac[2],mac[4],mac[5],mac[3],mac[1],mac[0],0,0 (the joiner echoes it). */
void pc_ldn_constant_id(const uint8_t mac[6], uint8_t out[8]);

/* Net 0x11 carrying all max_stations slots: stations[0..n) occupied (rank = index), the rest empty
 * (rank 0xff). network_id = CRC32(ssid[1:16]). out >= 30 + 22 * max_stations. Returns bytes, or -1
 * unless 1 <= nstations <= max_stations. */
int pc_build_net_conn_request(uint32_t seqid, uint16_t host_var, const uint8_t host_mac[6],
                              uint32_t network_id, const uint8_t (*stations)[4], int nstations,
                              int max_stations, uint8_t *out);

/* Net 0x50 (host_pia.build_net_property_update); nstations = 1 + participants. out >= 38 + app_len. */
int pc_build_net_property_update(uint32_t seqid, uint32_t network_id, uint16_t nstations,
                                 uint16_t max_stations, uint16_t scene_id, uint8_t property_byte,
                                 const uint8_t *app_data, int app_len, uint8_t *out);

/* Returns 0 ok, -1 malformed / not IPv4 / more players or protocols than we hold. */
int pc_parse_session_join(const uint8_t *p, int n, pc_session_join_t *out);

/* The leader's type-5 update, Pia 6.39: one fragment, leader station first, then the joiner's
 * echoed from its join. host_player_id NULL = DEFAULT_PLAYER_ID, host_token NULL = zeros.
 * Returns bytes; out >= 27 + 2 * 57 + 61 * (1 + join->num_players). */
int pc_build_session_update(const pc_session_join_t *join, const uint8_t host_constant8[8],
                            uint16_t host_var, const uint8_t host_ip[4],
                            const uint8_t *host_name, int host_name_len,
                            const uint8_t host_player_id[16], uint16_t sequence_id,
                            const uint8_t host_token[32], uint16_t update_sequence, uint8_t *out);

/* Type 2, 37 bytes; the session version is the join's own protocol-13 version (default 7). */
int pc_build_session_join_response(const pc_session_join_t *join, const uint8_t host_constant8[8],
                                   uint16_t host_var, const uint8_t random4[4], uint8_t out[37]);

/* Type 4 answer to a 6.39 type-3 leave request (22 or 34 bytes): type, random, the request's
 * constant id and variable id -- 15 bytes. -1 if not a leave request. */
int pc_build_session_leave_response(const uint8_t *request, int n, const uint8_t random4[4],
                                    uint8_t out[15]);

#endif
