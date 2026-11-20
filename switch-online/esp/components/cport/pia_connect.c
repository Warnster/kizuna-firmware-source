/* SPDX-License-Identifier: AGPL-3.0-only
 * Derived from pokeldn (https://github.com/Decryptu/pokeldn), Copyright (c) the pokeldn authors.
 * Changes Copyright (c) 2026 Kizuna. See NOTICE.
 */
/* C port of the joiner subset of pokeldn/ldn/pia_connect.py. See pia_connect.h. */
#include "pia_connect.h"
#include <string.h>

/* Pia 6.39 defaults, verbatim from the Python module. */
static const uint8_t PC_DEFAULT_APP_VER[2]   = { 0x00, 0x58 };
static const uint8_t PC_DEFAULT_PLAYER_ID[16] = {
    0,0,0,0,0,0,0,1, 0,0,0,0,0,0,0,0 };
/* [(1,0),(3,5),(5,1),(10,3),(13,7),(15,0)] flattened. */
static const uint8_t PC_DEFAULT_PROTOCOLS[12] = { 1,0, 3,5, 5,1, 10,3, 13,7, 15,0 };
#define PC_DEFAULT_NPROTO 6

/* --- byte helpers --- */
static void wbe16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)(v >> 8); p[1] = (uint8_t)v; }
static void wbe32(uint8_t *p, uint32_t v) {
    p[0]=(uint8_t)(v>>24); p[1]=(uint8_t)(v>>16); p[2]=(uint8_t)(v>>8); p[3]=(uint8_t)v; }
static uint16_t rbe16(const uint8_t *p) { return (uint16_t)((p[0] << 8) | p[1]); }
static uint32_t rbe32(const uint8_t *p) {
    return ((uint32_t)p[0]<<24)|((uint32_t)p[1]<<16)|((uint32_t)p[2]<<8)|p[3]; }

void pc_constant_id8(const uint8_t mac[6], uint8_t out[8]) {
    memcpy(out, mac, 6); out[6] = 0; out[7] = 0;
}

int pc_parse_net(const uint8_t *p, int n, uint8_t *t0, uint8_t *t1,
                 const uint8_t **body, int *blen) {
    if (n < 4) return -1;
    *t0 = p[0]; *t1 = p[1]; *body = p + 4; *blen = n - 4;
    return 0;
}

int pc_parse_net_conn_request(const uint8_t *p, int n,
                              uint16_t *host_var, uint8_t host_mac[6], uint32_t *seqid) {
    uint8_t t0, t1; const uint8_t *body; int blen;
    if (pc_parse_net(p, n, &t0, &t1, &body, &blen)) return -1;
    if (t1 != PC_NET_CONN_REQUEST || blen < 12) return -1;
    *seqid = rbe32(body);
    *host_var = rbe16(body + 4);
    memcpy(host_mac, body + 6, 6);
    return 0;
}

int pc_build_net_response(uint32_t seqid, uint8_t out[8]) {
    out[0] = 0x01; out[1] = PC_NET_CONN_RESPONSE; out[2] = 0; out[3] = 0;
    wbe32(out + 4, seqid);
    return 8;
}

int pc_build_net_property_ack(uint32_t seqid, uint8_t out[8]) {
    out[0] = 0x01; out[1] = PC_NET_UPDATE_PROPERTY_ACK; out[2] = 0; out[3] = 0;
    wbe32(out + 4, seqid);
    return 8;
}

int pc_parse_rtt(const uint8_t *p, int n, pc_rtt_t *out) {
    if (n < 16) return -1;
    out->type = p[0];
    out->version = p[3];
    memcpy(out->systime, p + 8, 8);
    if (n >= 21) { out->subject[0] = p[19]; out->subject[1] = p[20]; out->subject_len = 2; }
    else { out->subject_len = 0; }
    return 0;
}

int pc_build_rtt_response(const uint8_t *request, int rlen, uint8_t out[21]) {
    int take = rlen < 21 ? rlen : 21;
    memset(out, 0, 21);
    memcpy(out, request, (size_t)take);
    out[0] = 1;
    return 21;
}

int pc_build_rtt_request(const uint8_t *templ, int tlen, uint64_t systime, uint8_t out[21]) {
    int i, take = tlen < 21 ? tlen : 21;
    memset(out, 0, 21);
    memcpy(out, templ, (size_t)take);
    out[0] = 0;
    for (i = 0; i < 8; i++) out[8 + i] = (uint8_t)(systime >> (8 * i)); /* little-endian */
    return 21;
}

int pc_build_session_join(const uint8_t our_mac[6], uint16_t our_var, const uint8_t our_ip[4],
                          const uint8_t host_mac[6], uint16_t host_var,
                          const char *player_name, const uint8_t random4[4],
                          const uint8_t player_id[16], uint8_t *out) {
    uint8_t *o = out;
    int i, nmlen;
    const uint8_t *pid = player_id ? player_id : PC_DEFAULT_PLAYER_ID;
    uint8_t zip[4] = { 0, 0, 0, 0 };
    const uint8_t *ip = our_ip ? our_ip : zip;

    *o++ = PC_SESSION_JOIN_REQUEST;
    *o++ = PC_DEFAULT_NPROTO;
    memcpy(o, PC_DEFAULT_PROTOCOLS, sizeof(PC_DEFAULT_PROTOCOLS)); o += sizeof(PC_DEFAULT_PROTOCOLS);
    memcpy(o, PC_DEFAULT_APP_VER, 2); o += 2;
    memcpy(o, random4, 4); o += 4;
    memcpy(o, our_mac, 6); o += 6; *o++ = 0; *o++ = 0;   /* source constant id (8) */
    wbe16(o, our_var); o += 2;                            /* source variable id (2) */
    *o++ = 0; *o++ = 0;                                   /* NAT mapping, is-private-ipv6 */
    memset(o, 0, 32); o += 32;                            /* identification token */
    memcpy(o, host_mac, 6); o += 6; *o++ = 0; *o++ = 0;   /* dest constant id (8) */
    wbe16(o, host_var); o += 2;                           /* dest variable id (2) */
    *o++ = 1; *o++ = 1;                                   /* num players, num participants */
    *o++ = 0; memcpy(o, ip, 4); o += 4;                   /* StationAddress: [0] + IPv4 */
    wbe16(o, 12345); o += 2;                              /* src_port BE */
    /* PlayerInfo */
    nmlen = 0;
    if (player_name) { while (player_name[nmlen] && nmlen < 20) nmlen++; }
    memcpy(o, pid, 16); o += 16;
    wbe32(o, (uint32_t)nmlen); o += 4;
    *o++ = 1;
    for (i = 0; i < nmlen; i++) *o++ = (uint8_t)player_name[i];
    return (int)(o - out);
}

int pc_build_session_finalize(const uint8_t our_mac[6], uint8_t out[15]) {
    /* [6] <our_mac:6> 0000 0000000000 01 */
    out[0] = 6;
    memcpy(out + 1, our_mac, 6);
    memset(out + 7, 0, 7);
    out[14] = 1;
    return 15;
}

int pc_parse_session(const uint8_t *p, int n, uint8_t *type, uint8_t *count, int *has_count) {
    if (n < 1) return -1;
    *type = p[0];
    *has_count = 0;
    if ((p[0] == PC_SESSION_JOIN_REQUEST || p[0] == PC_SESSION_UPDATE) && n > 2) {
        *count = p[1]; *has_count = 1;
    }
    return 0;
}

/* --- ConnectionManager --- */

static pc_out_t *pc_q_new(pc_conn_t *c) {
    if (c->outbox_n >= PC_OUT_MAX) return NULL;   /* best-effort; drop under overflow */
    return &c->outbox[c->outbox_n++];
}

static void pc_q(pc_conn_t *c, uint8_t proto, const uint8_t *payload, int plen,
                 uint16_t dst, uint16_t src, int compress, int footer, int establishing,
                 int pktid, int footer_var) {
    pc_out_t *e = pc_q_new(c);
    if (!e) return;
    if (plen > PC_OUT_MAXPAYLOAD) plen = PC_OUT_MAXPAYLOAD;
    e->proto = proto;
    memcpy(e->payload, payload, (size_t)plen);
    e->payload_len = plen;
    e->dst = dst; e->src = src;
    e->compress = compress; e->footer = footer; e->establishing = establishing;
    e->unicast = 1; e->pktid = pktid; e->footer_var = footer_var;
}

void pc_conn_init(pc_conn_t *c, const uint8_t our_mac[6], const uint8_t host_mac[6],
                  const uint8_t our_ip[4], const uint8_t host_ip[4],
                  uint16_t our_var, const char *player_name, const uint8_t random4[4],
                  const uint8_t player_id[16]) {
    memset(c, 0, sizeof(*c));
    memcpy(c->our_mac, our_mac, 6);
    if (host_mac) memcpy(c->host_mac, host_mac, 6);
    if (our_ip) memcpy(c->our_ip, our_ip, 4);
    if (host_ip) memcpy(c->host_ip, host_ip, 4);
    c->our_var = our_var ? our_var : PC_DEFAULT_OUR_VAR;
    c->has_host_var = 0;
    if (player_name) {
        int i = 0; while (player_name[i] && i < 20) { c->player_name[i] = (uint8_t)player_name[i]; i++; }
        c->player_name[i] = 0; c->player_name_len = i;
    }
    if (random4) memcpy(c->random4, random4, 4);
    memcpy(c->player_id, player_id ? player_id : PC_DEFAULT_PLAYER_ID, 16);
    c->state = PC_ST_NET;
    c->has_join_sent = 0;
    c->has_last_host_rtt = 0;
    c->rtt_systime = 0x10000;
    c->rtt_orig_tick = -1000000000;
}

int pc_conn_connected(const pc_conn_t *c) { return c->state == PC_ST_CONNECTED; }

void pc_conn_learn_ids(pc_conn_t *c, const int *our_var, const int *host_var) {
    if (our_var) c->our_var = (uint16_t)*our_var;
    if (host_var) { c->host_var = (uint16_t)*host_var; c->has_host_var = 1; }
}

/* build the Session join from current state */
static int pc_build_join(pc_conn_t *c, uint8_t *out) {
    uint16_t hv = c->has_host_var ? c->host_var : 0;
    return pc_build_session_join(c->our_mac, c->our_var, c->our_ip, c->host_mac, hv,
                                 (const char *)c->player_name, c->random4, c->player_id, out);
}

static void pc_rtt_pending_add(pc_conn_t *c, uint64_t systime, int32_t tick) {
    int i, free = -1, oldest = -1;
    for (i = 0; i < PC_RTT_PENDING_MAX; i++) {
        if (!c->rtt_pending[i].used) { free = i; break; }
        if (oldest < 0 || c->rtt_pending[i].tick < c->rtt_pending[oldest].tick) oldest = i;
    }
    if (free < 0) free = oldest;   /* full: evict the oldest (Python drops the oldest 32) */
    c->rtt_pending[free].systime = systime;
    c->rtt_pending[free].tick = tick;
    c->rtt_pending[free].used = 1;
}

/* pop a pending probe by systime; returns 0 and sets *tick if found. */
static int pc_rtt_pending_pop(pc_conn_t *c, uint64_t systime, int32_t *tick) {
    int i;
    for (i = 0; i < PC_RTT_PENDING_MAX; i++) {
        if (c->rtt_pending[i].used && c->rtt_pending[i].systime == systime) {
            *tick = c->rtt_pending[i].tick;
            c->rtt_pending[i].used = 0;
            return 0;
        }
    }
    return -1;
}

void pc_conn_on_message(pc_conn_t *c, uint8_t proto, const uint8_t *payload, int len, int32_t tick) {
    uint8_t buf[160];
    int has_tick = (tick >= 0);

    if (proto == PC_PROTO_NET) {
        uint8_t t0, t1; const uint8_t *body; int blen;
        if (pc_parse_net(payload, len, &t0, &t1, &body, &blen) == 0) {
            if (t1 == PC_NET_CONN_REQUEST && c->state == PC_ST_NET) {
                uint16_t host_var; uint8_t host_mac[6]; uint32_t seqid = 2;
                if (pc_parse_net_conn_request(payload, len, &host_var, host_mac, &seqid) == 0) {
                    memcpy(c->host_mac, host_mac, 6);
                    if (!c->has_host_var) { c->host_var = host_var; c->has_host_var = 1; }
                }
                pc_build_net_response(seqid, buf);
                pc_q(c, PC_PROTO_NET, buf, 8, 0, 0, 0, 0, 1, /*pktid*/0, /*footer_var*/-1);
                if (c->has_host_var) {
                    int jl = pc_build_join(c, buf);
                    pc_q(c, PC_PROTO_SESSION, buf, jl, 0, c->our_var, 1, 0, 1, 0, -1);
                    if (!c->has_join_sent && has_tick) { c->join_sent_tick = tick; c->has_join_sent = 1; }
                }
            } else if (t1 == PC_NET_UPDATE_PROPERTY) {
                uint32_t seqid = (blen >= 4) ? rbe32(body) : 1;
                pc_build_net_property_ack(seqid, buf);
                pc_q(c, PC_PROTO_NET, buf, 8, 0, c->our_var, 0, 0, 1, -1, -1);
            }
        }
    } else if (proto == PC_PROTO_SESSION) {
        uint8_t type, count; int has_count;
        if (pc_parse_session(payload, len, &type, &count, &has_count) == 0) {
            if (type == PC_SESSION_UPDATE && c->has_host_var && c->state != PC_ST_CONNECTED) {
                int fl = pc_build_session_finalize(c->our_mac, buf);
                pc_q(c, PC_PROTO_SESSION, buf, fl, c->host_var, c->our_var, 0, 1, 0, -1, -1);
            }
        }
        if (c->state == PC_ST_NET) c->state = PC_ST_FINALIZE;
    } else if (proto == PC_PROTO_RTT || proto == PC_PROTO_RELIABLE) {
        if (c->state == PC_ST_FINALIZE) c->state = PC_ST_CONNECTED;
        if (proto == PC_PROTO_RTT && (c->state != PC_ST_NET || c->rtt_before_finalize)
                && c->has_host_var) {
            pc_rtt_t r;
            if (pc_parse_rtt(payload, len, &r) == 0) {
                if (r.type == 0) {
                    int take = len < 21 ? len : 21;
                    memset(c->last_host_rtt, 0, 21);
                    memcpy(c->last_host_rtt, payload, (size_t)take);
                    c->has_last_host_rtt = 1;
                    pc_build_rtt_response(payload, len, buf);
                    pc_q(c, PC_PROTO_RTT, buf, 21, PC_SESSION_VAR, c->our_var, 0, 1, 0, -1,
                         /*footer_var*/ c->host_var);
                } else if (r.type == 1 && has_tick) {
                    uint64_t systime = 0; int i; int32_t sent;
                    for (i = 0; i < 8; i++) systime |= ((uint64_t)r.systime[i]) << (8 * i);
                    if (pc_rtt_pending_pop(c, systime, &sent) == 0 && tick >= sent) {
                        if (c->rtt_samples_n < PC_RTT_SAMPLES_MAX)
                            c->rtt_samples[c->rtt_samples_n++] = tick - sent;
                    }
                }
            }
        }
    }
}

void pc_conn_maybe_originate_rtt(pc_conn_t *c, int32_t tick) {
    uint8_t buf[21];
    if (!pc_conn_connected(c) || !c->has_last_host_rtt || !c->has_host_var) return;
    if (tick - c->rtt_orig_tick < PC_RTT_ORIGINATE_PERIOD) return;
    c->rtt_orig_tick = tick;
    c->rtt_systime = c->rtt_systime + 1;   /* 64-bit wrap is fine */
    pc_rtt_pending_add(c, c->rtt_systime, tick);
    pc_build_rtt_request(c->last_host_rtt, 21, c->rtt_systime, buf);
    pc_q(c, PC_PROTO_RTT, buf, 21, PC_SESSION_VAR, c->our_var, 0, 1, 0, -1, c->host_var);
}

void pc_conn_maybe_repeat_join(pc_conn_t *c, int32_t tick) {
    uint8_t buf[160]; int jl;
    if (!c->join_repeat_ticks || c->state != PC_ST_NET || !c->has_host_var || !c->has_join_sent)
        return;
    if (tick - c->join_sent_tick < c->join_repeat_ticks) return;
    c->join_sent_tick = tick;
    jl = pc_build_join(c, buf);
    pc_q(c, PC_PROTO_SESSION, buf, jl, 0, c->our_var, 1, 0, 1, 0, -1);
}

int pc_conn_drain(pc_conn_t *c, const pc_out_t **out) {
    int n = c->outbox_n;
    *out = c->outbox;
    c->outbox_n = 0;
    return n;
}

/* ===== LEADER (host) side ===== */

void pc_ldn_constant_id(const uint8_t mac[6], uint8_t out[8]) {
    out[0] = mac[2]; out[1] = mac[4]; out[2] = mac[5]; out[3] = mac[3];
    out[4] = mac[1]; out[5] = mac[0]; out[6] = 0; out[7] = 0;
}

/* _net_station: [state, rank, 0, 0] + 16-byte address (IPv4 first) + port; an empty slot has
 * no address and port 0. */
static uint8_t *pc_net_station(uint8_t *o, const uint8_t *ip, uint8_t rank) {
    memset(o, 0, PC_NET_STATION_SIZE);
    o[1] = rank;
    if (ip) { memcpy(o + 4, ip, 4); wbe16(o + 20, PC_PIA_PORT); }
    return o + PC_NET_STATION_SIZE;
}

int pc_build_net_conn_request(uint32_t seqid, uint16_t host_var, const uint8_t host_mac[6],
                              uint32_t network_id, const uint8_t (*stations)[4], int nstations,
                              int max_stations, uint8_t *out) {
    uint8_t *o = out;
    int i;
    if (nstations < 1 || nstations > max_stations) return -1;
    *o++ = 0x01; *o++ = PC_NET_CONN_REQUEST;
    wbe16(o, (uint16_t)(max_stations * PC_NET_STATION_SIZE)); o += 2;   /* the station array size */
    wbe32(o, seqid); o += 4;
    wbe16(o, host_var); o += 2;
    pc_ldn_constant_id(host_mac, o); o += 8;
    memset(o, 0, 4); wbe32(o + 4, network_id); o += 8;                 /* zero-extended to 8 */
    *o++ = 1;
    wbe16(o, (uint16_t)max_stations); o += 2;
    *o++ = 0;
    for (i = 0; i < nstations; i++) o = pc_net_station(o, stations[i], (uint8_t)i);
    for (; i < max_stations; i++) o = pc_net_station(o, NULL, 0xFF);
    return (int)(o - out);
}

int pc_build_net_property_update(uint32_t seqid, uint32_t network_id, uint16_t nstations,
                                 uint16_t max_stations, uint16_t scene_id, uint8_t property_byte,
                                 const uint8_t *app_data, int app_len, uint8_t *out) {
    int system_len = app_len < PC_APP_HEADER_SIZE ? app_len : PC_APP_HEADER_SIZE;
    uint8_t *o = out;
    *o++ = 0x01; *o++ = PC_NET_UPDATE_PROPERTY;
    wbe16(o, (uint16_t)app_len); o += 2;
    wbe32(o, seqid); o += 4;
    memset(o, 0, 4); wbe32(o + 4, network_id); o += 8;
    wbe16(o, nstations); o += 2;
    wbe16(o, max_stations); o += 2;
    memset(o, 0, 6); o += 6;
    wbe16(o, scene_id); o += 2;
    *o++ = property_byte; *o++ = 0x01;        /* the GBA app writes 01 01; the second's meaning is unknown */
    wbe32(o, (uint32_t)system_len); o += 4;
    wbe32(o, (uint32_t)(app_len - system_len)); o += 4;
    if (app_len > 0) { memcpy(o, app_data, (size_t)app_len); o += app_len; }
    return (int)(o - out);
}

/* _parse_player_info. Returns the end offset, or -1. */
static int pc_parse_player(const uint8_t *p, int n, int off, pc_player_t *pl) {
    uint32_t name_size;
    if (off + 21 > n) return -1;
    name_size = rbe32(p + off + 16);
    if (name_size > PC_PLAYER_NAME_MAX || off + 21 + (int)name_size > n) return -1;
    memcpy(pl->player_id, p + off, 16);
    pl->encoding = p[off + 20];
    pl->name_len = (uint8_t)name_size;
    memcpy(pl->name, p + off + 21, name_size);
    return off + 21 + (int)name_size;
}

int pc_parse_session_join(const uint8_t *p, int n, pc_session_join_t *j) {
    int pos, i, np;
    memset(j, 0, sizeof(*j));
    if (n < 2 || p[0] != PC_SESSION_JOIN_REQUEST) return -1;
    np = p[1];
    pos = 2;
    if (pos + np * 2 + 2 + 4 + 8 + 2 + 2 + 32 + 8 + 2 + 2 > n) return -1;
    if (np > PC_JOIN_MAX_PROTOCOLS) return -1;
    j->nprotocols = np;
    for (i = 0; i < np; i++) { j->protocols[i][0] = p[pos + 2 * i]; j->protocols[i][1] = p[pos + 2 * i + 1]; }
    pos += np * 2;
    memcpy(j->app_ver, p + pos, 2); pos += 2;
    memcpy(j->random4, p + pos, 4); pos += 4;
    memcpy(j->source_constant_id, p + pos, 8); pos += 8;
    j->source_var = rbe16(p + pos); pos += 2;
    j->nat_mapping = p[pos]; j->private_ipv6 = p[pos + 1]; pos += 2;
    memcpy(j->token, p + pos, 32); pos += 32;
    memcpy(j->destination_constant_id, p + pos, 8); pos += 8;
    j->destination_var = rbe16(p + pos); pos += 2;
    j->num_players = p[pos]; j->num_participants = p[pos + 1]; pos += 2;
    if (pos >= n || p[pos] != 0) return -1;                /* StationAddress kind 0 = IPv4 */
    pos += 1;
    if (pos + 6 > n) return -1;
    memcpy(j->ip, p + pos, 4); pos += 4;
    j->port = rbe16(p + pos); pos += 2;
    if (j->num_players > PC_JOIN_MAX_PLAYERS) return -1;
    for (i = 0; i < j->num_players; i++)
        if ((pos = pc_parse_player(p, n, pos, &j->players[i])) < 0) return -1;
    return pos == n ? 0 : -1;
}

static uint8_t *pc_put_player(uint8_t *o, const uint8_t id[16], const uint8_t *name, int len,
                              uint8_t encoding) {
    memcpy(o, id, 16); o += 16;
    wbe32(o, (uint32_t)len); o += 4;
    *o++ = encoding;
    if (len) { memcpy(o, name, (size_t)len); o += len; }
    return o;
}

/* _build_session_station up to its players. */
static uint8_t *pc_put_station(uint8_t *o, const uint8_t constant8[8], uint16_t var, const uint8_t ip[4],
                               uint16_t port, uint8_t station_index, uint16_t join_order,
                               const uint8_t token[32], uint8_t nplayers, uint8_t nparticipants) {
    memcpy(o, constant8, 8); o += 8;
    wbe16(o, var); o += 2;
    memcpy(o, ip, 4); o += 4;
    wbe16(o, port); o += 2;
    *o++ = station_index;
    wbe16(o, join_order); o += 2;
    *o++ = 0; *o++ = 0;                                   /* left-join order / reserved */
    memcpy(o, token, 32); o += 32;
    *o++ = nplayers; *o++ = nparticipants;
    *o++ = 0; *o++ = 0;
    return o;
}

int pc_build_session_update(const pc_session_join_t *join, const uint8_t host_constant8[8],
                            uint16_t host_var, const uint8_t host_ip[4],
                            const uint8_t *host_name, int host_name_len,
                            const uint8_t host_player_id[16], uint16_t sequence_id,
                            const uint8_t host_token[32], uint16_t update_sequence, uint8_t *out) {
    static const uint8_t zero32[32];
    uint8_t *o = out;
    int i;
    if (join->num_players < 1 || host_name_len > PC_PLAYER_NAME_MAX) return -1;
    /* [type, sequence:u16, fragment count, fragment index, offset:u16] */
    *o++ = PC_SESSION_UPDATE; wbe16(o, update_sequence); o += 2;
    *o++ = 1; *o++ = 0; wbe16(o, 3); o += 2;
    memcpy(o, host_constant8, 8); o += 8;
    wbe16(o, host_var); o += 2;
    *o++ = 2; *o++ = 0;                                   /* two stations, none departed */
    wbe16(o, sequence_id); o += 2;
    wbe16(o, update_sequence); o += 2;
    memset(o, 0, 4); o += 4;                              /* 6.39 reserved */
    o = pc_put_station(o, host_constant8, host_var, host_ip, PC_PIA_PORT, 0, 0,
                       host_token ? host_token : zero32, 1, 1);
    o = pc_put_player(o, host_player_id ? host_player_id : PC_DEFAULT_PLAYER_ID, host_name,
                      host_name_len, 1);
    o = pc_put_station(o, join->source_constant_id, join->source_var, join->ip, join->port, 1, 1,
                       join->token, join->num_players, join->num_participants);
    for (i = 0; i < join->num_players; i++)
        o = pc_put_player(o, join->players[i].player_id, join->players[i].name,
                          join->players[i].name_len, join->players[i].encoding);
    return (int)(o - out);
}

int pc_build_session_join_response(const pc_session_join_t *join, const uint8_t host_constant8[8],
                                   uint16_t host_var, const uint8_t random4[4], uint8_t out[37]) {
    uint8_t version = 7;
    int i;
    for (i = 0; i < join->nprotocols; i++)          /* dict(protocols): the last entry wins */
        if (join->protocols[i][0] == PC_PROTO_SESSION) version = join->protocols[i][1];
    out[0] = PC_SESSION_JOIN_RESPONSE; out[1] = PC_PROTO_SESSION; out[2] = version; out[3] = 1;
    memset(out + 4, 0, 4);
    memcpy(out + 8, random4, 4);
    memcpy(out + 12, host_constant8, 8);
    wbe16(out + 20, host_var);
    memcpy(out + 22, join->source_constant_id, 8);
    wbe16(out + 30, join->source_var);
    out[32] = 1; wbe16(out + 33, 1); out[35] = 0; out[36] = 0;
    return 37;
}

int pc_build_session_leave_response(const uint8_t *request, int n, const uint8_t random4[4],
                                    uint8_t out[15]) {
    if ((n != 0x16 && n != 0x22) || request[0] != PC_SESSION_LEAVE_REQUEST) return -1;
    out[0] = PC_SESSION_LEAVE_RESPONSE;
    memcpy(out + 1, random4, 4);
    memcpy(out + 5, request + 5, 10);
    return 15;
}
