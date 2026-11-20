/* SPDX-License-Identifier: AGPL-3.0-only
 * Derived from pokeldn (https://github.com/Decryptu/pokeldn), Copyright (c) the pokeldn authors.
 * Changes Copyright (c) 2026 Kizuna. See NOTICE.
 */
/* trade_host.h — leader-side FRLG Direct Corner trade engine, portable C port of pokeldn
 * pokeldn/frlg/link/host_trade.py HostTradeEngine (pokeldn HEAD b5ede2b) for union_room=False,
 * colosseum=False. Runs the RFU conversation of a Pokemon Center trade room as the PARENT: the
 * console (RFU child) is fed in with trade_host_feed_child_slot(), and trade_host_tick() returns the
 * parent's gSendCmd row (seven u16 words, all zero = idle) once per VBlank.
 *
 * Flow (state names are pokeldn's H_*): LINK_PLAYER (SEND_PLAYER_IDS burst, BLOCK_REQ, swap
 * LinkPlayer blocks) -> ENTRY_CARD (trainer cards) -> ENTRY_SEAT (the held-key walk to the left
 * chair, standby rounds) -> PARTY (three 200-byte party blocks, mail 220, ribbons 40, each gated on
 * the console's link task going quiet) -> SELECT -> CONFIRM -> ANIM -> SAVE (six child-initiated
 * standby rounds) -> back to PARTY, once per trade; then LEAVE_MENU (five-second menu wait, both
 * sides CANCEL) -> CANCEL -> RETURN_FIELD -> EXIT (EXIT_ROOM key) -> CLOSE (READY_CLOSE_LINK
 * retries, 15 s grace) -> DONE.
 *
 * Not ported (out of scope): Union Room (chat, battle, trading board), the Colosseum, and the
 * battle-only echo gating (_echo_owed), which pokeldn scopes to the battle: the trade path needs
 * nothing from the leader's echo bookkeeping. The Python's diagnostics (trace, status reports,
 * slot-run statistics, logging) are not reproduced; behaviour on the wire is.
 *
 * Differences from the Python, all on inputs the Python treats as errors (it raises):
 *  - a child block whose fragment count does not match what the engine expects is counted in
 *    errors and dropped (the Python raises ValueError; the block is already consumed from the
 *    receiver there too);
 *  - the word / block queues are bounded (TH_WORDS_MAX 64, TH_BLOCKS_MAX 6): on overflow the new
 *    entry is dropped and words_dropped / blocks_dropped counts it. The Python deques are unbounded;
 *    a healthy session never queues more than ~13 words or 3 blocks;
 *  - a fragment count > 32 in a child SEND_BLOCK_INIT can never complete (as in the Python: the
 *    all-received mask is never reached).
 *
 * Party format: up to six 100-byte party-mon blobs in pokeldn's Mon.raw form (the wire/.ek3 form:
 * encrypted secure region, party tail at 80..99). pokeldn's Mon.from_pk3() produces it; this engine
 * never re-encrypts. Received mons are returned in the same form.
 *
 * Call rate: trade_host_tick() once per VBlank (1/59.727 s), at most one row per call, exactly as
 * pokeldn's HostSession; feed each NEW child UNI command row (idle rows too) before the tick that
 * follows it. No clocks, no allocation. sizeof(trade_host) is about 6 KB. */
#ifndef TRADE_HOST_H
#define TRADE_HOST_H
#include <stdint.h>
#include <stddef.h>
#include "linkplayer.h"
#include "block.h"
#include "pia_activity.h"

#define TH_PARTY_MAX      6
#define TH_WORDS_MAX      64
#define TH_BLOCKS_MAX     6
#define TH_BLOCK_MAX      224     /* largest block we send: mail, 220 */

typedef enum {
    TH_LINK_PLAYER, TH_ENTRY_CARD, TH_ENTRY_SEAT, TH_PARTY, TH_SELECT, TH_CONFIRM, TH_ANIM,
    TH_SAVE, TH_LEAVE_MENU, TH_CANCEL, TH_RETURN_FIELD, TH_EXIT, TH_CLOSE, TH_DONE,
    TH_STATE_COUNT
} th_state;

/* HostTradeTiming (frames = VBlanks). Defaults are DEFAULT_HOST_TRADE_TIMING; the chat fields are not ported. */
typedef struct {
    int save_barrier_rounds;                /* 6 */
    int save_final_standby_quiet_frames;    /* 75 */
    int party_link_settle_frames;           /* 30 */
    int startup_standby_echo_frames;        /* 4 */
    int player_ids_repeat_frames;           /* 8 */
    int link_player_idle_frames;            /* 12 */
    int entry_final_standby_quiet_frames;   /* 75 */
    int final_menu_ready_frames;            /* 300 */
    int post_cancel_exit_wait_frames;       /* 300 */
    int post_client_close_grace_frames;     /* 900 */
    int close_retry_frames;                 /* 60 */
} th_timing;
void th_timing_default(th_timing *t);

typedef struct {
    lp_link_player lp;                      /* our identity (profile.to_link_player()) */
    uint16_t       card_flag_id;            /* the trainer card's wonder-card u16 */
    uint16_t       quote[LP_EASYCHAT_WORDS];/* trainer-card profile quote (easy-chat word ids) */
    uint8_t        party[TH_PARTY_MAX][MON_PARTY_SIZE];
    int            party_count;             /* 1..6 */
    int            trades;                  /* 1..6 */
    int            has_offered_slots;       /* 1: use offered_slots[0..trades-1] (distinct, < party_count) */
    int            offered_slots[TH_PARTY_MAX];
    int            trade_slot;              /* otherwise seeded from this, see trade_resolve_offered_slots */
    int            anim_delay;              /* VBlanks the trade animation is assumed to take (1935) */
    int            trust_pia;               /* send each block fragment once (Pia carries it reliably) */
    th_timing      timing;
} th_cfg;
/* LinkPlayer defaults, empty party, trades 1, trade_slot 0, anim 1935, trust_pia 1, default timing and quote. */
void th_cfg_default(th_cfg *cfg);

/* The block sender the engine drives (pokeldn BlockSender with owner 0, ack NULL, stream_repeat 1),
 * kept here because the current Python has a HOLD re-send gap the older block.c port lacks.
 * Exposed for tests. */
typedef struct {
    uint8_t  data[TH_BLOCK_MAX];
    int      len, count, owner, trust_pia;
    int      state;                         /* 0 INIT, 1 STREAM, 2 HOLD, 3 DONE */
    int      index, init_sends, hold_sends, hold_gap, frag_sends;
} th_sender;
void th_sender_init(th_sender *s, const uint8_t *data, int len, int owner, int trust_pia);
void th_sender_tick(th_sender *s, uint16_t words[7]);
int  th_sender_done(const th_sender *s);

typedef struct {
    uint16_t w[7];
} th_words;
typedef struct {
    uint8_t  data[TH_BLOCK_MAX];
    uint16_t len;
} th_block;

typedef struct trade_host {
    th_cfg      cfg;
    th_state    state;
    int         offered[TH_PARTY_MAX];
    int         n_party;
    uint8_t     party[TH_PARTY_MAX][MON_PARTY_SIZE];   /* swapped as trades commit */
    uint8_t     received[TH_PARTY_MAX][MON_PARTY_SIZE];
    int         received_count;
    uint8_t     trainer_card[LP_TRAINER_CARD_SIZE];
    uint8_t     child_card[LP_TRAINER_CARD_SIZE];
    int         has_child_card;
    uint8_t     link_player_block[200];
    lp_link_player child_link_player;
    int         has_child_link_player;
    int         rejected_link_players;
    uint8_t     child_party[600];

    int         round, commits;
    int         child_cursor;                           /* -1 = none */
    int         done, disconnect_requested;
    int         errors, words_dropped, blocks_dropped;

    /* expectation of the next completed child block */
    int         expected;                               /* th_expect */
    int         party_pair, party_expected_i;
    int         link_waiting_idle, link_idle_frames;    /* link_completed: 0 none, 1..3 party i+1, 4 mail, 5 ribbons */
    int         link_completed;
    int         link_player_idle;
    int         child_finish, anim_wait;                /* anim_wait < 0 = None */
    int         save_rounds, save_last_count, save_final_standby_seen, save_standby_quiet;
    int         last_child_standby;                     /* -1 = None */
    int         entry_final_standby_seen, entry_standby_quiet;
    int         exit_count;
    int         cancel_standby_count, return_standby_count;   /* -1 = None */
    int         room_exit_wait;                         /* < 0 = None */
    int         child_exit_seen;
    int         leave_menu_wait;                        /* < 0 = None */
    int         host_cancel_ready, child_cancel_requested, select_cancels;
    int         close_retry_wait, close_confirmed, close_grace_wait;   /* grace < 0 = None */

    /* held-key route: runs of (keycode, count); a steady key repeats once the plan is spent */
    struct { uint8_t key; uint16_t count; } held_runs[12];
    int         held_n, held_i, held_left;
    int         held_steady;                            /* -1 = None */
    int         held_count;

    recv_block  rx;
    th_sender   sender;
    int         sender_active;

    th_words    words_q[TH_WORDS_MAX];
    int         wq_head, wq_len;
    th_block    blocks_q[TH_BLOCKS_MAX];
    int         bq_head, bq_len;
} trade_host;

/* 0, or -1 if the configuration is invalid (the cases where the Python raises ValueError: party not
 * 1..6, trades not 1..6, offered slots wrong count / negative / duplicate / past the party). */
int  trade_host_init(trade_host *th, const th_cfg *cfg);
void trade_host_feed_child_slot(trade_host *th, const uint8_t slot[14]);
void trade_host_tick(trade_host *th, uint16_t words[7]);

int  trade_host_established(const trade_host *th);      /* the console's LinkPlayer arrived */
int  trade_host_disconnect_requested(const trade_host *th);
void trade_host_mark_disconnect_sent(trade_host *th);   /* after the 'D' frame went out; -> DONE */
th_state trade_host_state(const trade_host *th);
const char *trade_host_state_name(th_state s);          /* "H_LINK_PLAYER" ... "H_DONE" */
int  trade_host_done(const trade_host *th);
int  trade_host_close_confirmed(const trade_host *th);
int  trade_host_commits(const trade_host *th);
int  trade_host_received_count(const trade_host *th);
/* The i'th mon received (100 bytes, Mon.raw form), or NULL. */
const uint8_t *trade_host_received_mon(const trade_host *th, int i);

/* The pia_activity vtable (ctx = th) for pia_hsess. */
pia_activity trade_host_activity(trade_host *th);

#endif
