/* SPDX-License-Identifier: AGPL-3.0-only
 * Derived from pokeldn (https://github.com/Decryptu/pokeldn), Copyright (c) the pokeldn authors.
 * Changes Copyright (c) 2026 Kizuna. See NOTICE.
 */
/* pia_host.h — the Pia LEADER a Switch running FireRed/LeafGreen (NSO) joins as a Mystery Gift
 * FRIEND client. Portable C port of pokeldn (190f210):
 *
 *  - pia_hsess: frlg/link/host_session.py HostSession — the Reliable host session (reliable_host.c)
 *    + the RFU leader (rfu_leader.c) + the activity (mg_host.c). Opens 'A' only after the bulk ack
 *    covers the child's 'C', pauses the activity on a full send window, holds new frames while the
 *    console is HOST_OUTSTANDING_MAX frames behind (never on the close/disconnect path), and sends
 *    'D' once the activity asks to disconnect after its READY_CLOSE_LINK went out.
 *  - pia_host: ldn/host_pia.py HostPeerProtocol — Net 0x11 probes (broadcast + unicast, 500 ms
 *    until the 0x12), Session join -> type 5 update (broadcast + unicast) + type 2 response every
 *    250 ms until the type 6, RTT answers + our own probe every 315 ms, one Reliable protocol tick
 *    per 1/59.727 s with carry-forward (depth 4) and the 250 ms quiet gate, Net 0x50 every 500 ms
 *    once the activity is established, and type 4 answers to a type 3 leave.
 *
 * No sockets, clocks or randomness inside: the caller passes now_ms, supplies random bytes through
 * cfg.rng and receives datagrams through cfg.send (UDP port 12345). Nonces follow the native
 * sequence (PiaNonceSequence(native=True)) seeded from 8 rng bytes. Outbound zstd is the stored
 * (raw block) form `28b52ffd 00 18 <hdr3> <bytes>`, which any zstd decoder accepts. */
#ifndef PIA_HOST_H
#define PIA_HOST_H
#include <stdint.h>
#include <stddef.h>
#include "reliable_host.h"
#include "rfu_leader.h"
#include "pia_activity.h"
#include "pia_connect.h"
#include "crypto.h"
#include "reliable.h"

/* ---- HostSession ---- */
#define PIA_HOST_RTX_LIMIT       2   /* a ~250 ms TX hiccup must not flood the console with every frame */
#define PIA_HOST_OUTSTANDING_MAX 6   /* below the console's 8-deep RFU receive queue */
#define PIA_HSESS_EMIT_MAX       8   /* RTX_LIMIT resends + ack + one new frame, with slack */

typedef struct {
    relh_t      rel;
    rfu_leader  rfu;
    pia_activity act;
    int has_connect_seq; uint16_t connect_seq;
    int peer_open_ack_sent, connect_ack_sent, close_poll_sent, disconnect_sent, stopped;
    int send_window_full, window_full_ticks, console_backlogged, backlog_ticks;
    int events[RFUL_EV_UNI + 1];   /* rfu_leader events, by kind */
} pia_hsess;

/* rel_cfg NULL = HostReliableSession(retransmit_limit=HOST_RTX_LIMIT). host_session_id: the RFU
 * parent id the beacon advertises (random low byte, high byte 0xf1). */
void pia_hsess_init(pia_hsess *s, const pia_activity *activity, const uint8_t host_session_id[2], const relh_cfg *rel_cfg);
/* One Reliable(10) message from the child. Returns the number of RFU events it produced. */
int  pia_hsess_receive(pia_hsess *s, const uint8_t *wire, int len, double now_ms);
/* At most one RFU slot per call. Returns emissions written to out (max_out >= PIA_HSESS_EMIT_MAX). */
int  pia_hsess_tick(pia_hsess *s, double now_ms, relh_emission *out, int max_out);
void pia_hsess_note_rtt(pia_hsess *s, double rtt_ms);
void pia_hsess_on_ldn_leave(pia_hsess *s);
int  pia_hsess_established(const pia_hsess *s);   /* the console's LinkPlayer arrived */

/* ---- HostPeerProtocol ---- */
#define PIA_HOST_VAR              0x00C6
#define PIA_HOST_NET_RETRY_MS     500
#define PIA_HOST_ACCEPT_RETRY_MS  250
#define PIA_HOST_RTT_PERIOD_MS    315
#define PIA_HOST_CARRY_DEPTH      4
#define PIA_HOST_QUIET_GATE_MS    250
#define PIA_HOST_BATCH_MAX        9
#define PIA_HOST_COMPRESS_MIN     62
#define PIA_HOST_MAX_PARTICIPANTS 7     /* LDN members besides us (LDN_HOST_MAX_MEMBERS - 1) */
#define PIA_HOST_RTT_PENDING      16
#define PIA_HOST_DG_MAX           1600  /* largest datagram accepted from the Switch (and its plaintext) */
#define PIA_HOST_TX_MAX           1152  /* our largest: 9 Reliable messages of an 88-byte 'T' ~ 1040 */
#define PIA_HOST_APP_DATA_MAX     384
#define PIA_HOST_EMIT_MAX         (PIA_HOST_CARRY_DEPTH * PIA_HSESS_EMIT_MAX + PIA_HSESS_EMIT_MAX)
#define PIA_HOST_RX_MSGS          32    /* Pia messages handled per Switch datagram */

typedef void (*pia_host_send_fn)(void *ctx, const uint8_t *dg, size_t len, const uint8_t dst_ip[4]);
typedef void (*pia_host_rng_fn)(void *ctx, uint8_t *out, size_t n);

typedef enum {
    PIA_HOST_EV_NET_ACKED,         /* the Switch answered Net 0x11 with 0x12 */
    PIA_HOST_EV_JOIN,              /* first Session join request accepted for parsing */
    PIA_HOST_EV_FINALIZED,         /* type 6: Reliable may flow */
    PIA_HOST_EV_PROPERTY_STARTED,  /* established: switch the LDN advert to the active app data */
    PIA_HOST_EV_LEAVE,             /* the Switch sent a type 3 leave (answered) */
} pia_host_event;
typedef void (*pia_host_event_fn)(void *ctx, pia_host_event ev);

typedef struct {
    uint8_t  ssid[16];             /* NetworkId.ssid: also the Pia session key input */
    uint8_t  our_mac[6], our_ip[4], broadcast_ip[4];
    uint16_t scene_id;             /* 22287 */
    uint8_t  max_participants;     /* 6 */
    const char *session_name;      /* our Pia player name (profile.session_name), <= 40 bytes */
    const uint8_t *active_app_data;/* Net 0x50 body once established (<= PIA_HOST_APP_DATA_MAX) */
    int      active_app_data_len;
    int      response_first;       /* type 2 before type 5 (the Mystery Gift default: 1) */
    double   protocol_tick_ms;     /* 0 = 1000 / 59.727 */
    uint8_t  host_session_id[2];   /* the RFU parent id: must match the beacon's */
    pia_host_send_fn  send;
    pia_host_rng_fn   rng;
    pia_host_event_fn event;       /* optional */
    void    *ctx;
} pia_host_cfg;

/* What a Direct Corner TRADE host passes to HostPeerProtocol and the LDN host network, from pokeldn
 * config/host.toml + HostTransport (identical to the Mystery Gift host; only the activity and the room app
 * data differ): scene 22287, max participants 6, session_response_first (type 2 before type 5), the native
 * nonce sequence (always on here), protocol tick 1/59.727 s (protocol_tick_ms = 0). Sets only those four
 * fields; the caller still supplies ssid, our_mac/our_ip/broadcast_ip, session_name (pokeldn: the trainer
 * name, == the room's Pia nickname), active_app_data (the room after ldn_frlg_room_trade_activate),
 * host_session_id (the same 2 bytes as ldn_frlg_room_trade), send, rng, event, ctx.
 * Also fixed by pokeldn and set in ldn_host.c, not here: local comm id 0x01006fa0233f8000, app version 88,
 * channel 1/6/11 (pokeldn defaults to 1; any of the three is scanned), LDN nickname = the trainer name. */
void pia_host_cfg_trade_defaults(pia_host_cfg *cfg);

typedef struct { uint16_t seq; uint8_t flagsA; uint16_t ack; uint8_t retransmitted; } pia_host_carry;

typedef struct {
    pia_host_cfg cfg;
    char        session_name[PC_PLAYER_NAME_MAX + 1];
    uint8_t     app_data[PIA_HOST_APP_DATA_MAX];
    pia_hsess   sess;
    pia_crypto  crypto;
    uint8_t     participants[PIA_HOST_MAX_PARTICIPANTS][4];
    int         nparticipants;
    uint64_t    nonce;

    int      joined, net_acked; uint32_t net_sequence; double next_net_send;
    int      property_started, has_next_property; double next_property_send; int property_acks;
    int      session_join_seen, has_join; pc_session_join_t join;
    int      session_accepts, has_next_accept; double next_accept;
    int      session_finalized;
    int      has_guest; uint16_t guest_var; uint8_t guest_ip[4];
    uint16_t session_packet_id, reliable_packet_id;
    int      has_rtt_template; uint8_t rtt_template[21]; int rtt_template_len;
    uint64_t rtt_systime; int has_next_rtt; double next_rtt_send;
    struct { uint64_t systime; double sent; int used; } rtt_pending[PIA_HOST_RTT_PENDING];
    int      rtt_requests_in, rtt_responses_in, rtt_requests_out, rtt_responses_out;
    int      has_next_tick; double next_protocol_tick;
    int      reliable_messages_in, reliable_messages_out, leave_requests_in;
    int      has_last_rx; double last_rx;
    pia_host_carry carry[PIA_HOST_CARRY_DEPTH][PIA_HSESS_EMIT_MAX];
    int      carry_len[PIA_HOST_CARRY_DEPTH], carry_n, carry_head, carried_frames;
    int      rx_datagrams, rx_rejected, tx_datagrams;

    int      time_init; uint32_t last_ms; uint64_t now_ms64;
    relh_emission emit[PIA_HOST_EMIT_MAX];     /* carried + this tick's frames */
    rel_message_t rx_msgs[PIA_HOST_RX_MSGS];   /* kept off the caller's stack */
    pc_session_join_t join_rx;                 /* a join is parsed here, adopted only if valid */
    /* TX: a = tiled messages (stored zstd prefix room ahead), b = the datagram, c = payloads.
     * RX: decrypted plaintext and decompressed messages; a reply mid-datagram never clobbers them. */
    uint8_t  scratch_a[PIA_HOST_TX_MAX], scratch_b[PIA_HOST_TX_MAX], scratch_c[PIA_HOST_TX_MAX];
    uint8_t  rx_pt[PIA_HOST_DG_MAX], rx_app[PIA_HOST_DG_MAX];
} pia_host_t;

/* activity: the vtable of an initialised engine (mg_host_activity(), trade_host_activity()) whose ctx
 * outlives the host. Copied by value. */
void pia_host_init(pia_host_t *h, const pia_host_cfg *cfg, const pia_activity *activity);
/* The LDN members besides us (Net 0x11 stations, unicast copies). */
void pia_host_set_participants(pia_host_t *h, const uint8_t (*ips)[4], int n);
/* The first LDN member authenticated: start the Net 0x11 probes. */
void pia_host_on_participant_joined(pia_host_t *h);
/* A UDP datagram on port 12345. Returns the number of Pia messages handled, -1 if rejected. */
int  pia_host_receive(pia_host_t *h, const uint8_t *dg, size_t len, const uint8_t src_ip[4], uint32_t now_ms);
/* Call often (every ~1-5 ms, at least once per 1/59.727 s): every timer runs from here. */
void pia_host_tick(pia_host_t *h, uint32_t now_ms);
/* The Switch left LDN: stop the session (no more Reliable). */
void pia_host_on_ldn_leave(pia_host_t *h);
/* host_pia.next_deadline: ms until the earliest timer is due (0 = now), at most default_ms --
 * the receive timeout for the glue's loop. */
uint32_t pia_host_next_deadline_ms(pia_host_t *h, uint32_t now_ms, uint32_t default_ms);

/* The stored zstd frame the host sends (exposed for tests). Returns bytes, -1 if out_cap short. */
int  pia_host_zstd_stored(const uint8_t *in, int len, uint8_t *out, int out_cap);

#endif
