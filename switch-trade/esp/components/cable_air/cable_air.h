/* Copyright (c) 2026 mgba_ldn contributors (rfu-wrapper-air.c, the translator this file is ported from)
 * Modified: ported to C for the Kizuna adapters (see git history).
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 * Upstream: https://github.com/Gr3nSkyDragon/mgba_LDN (branch mgba-ldn), src/gba/sio/rfu-wrapper-air.c */
/* cable_air — the RFU Cable Wrapper translator, ported from mgba_LDN
 * (src/gba/sio/rfu-wrapper-air.c, branch mgba-ldn) for the ESP32 (C6 single-chip, then the C3 + RP2040 two-chip board).
 *
 * Sits between the physical Gen-3 link cable (a real Ruby/Sapphire cart; on the two-chip board the RP2040
 * cable child) and the Switch's FRLG wireless trade (spoken by ldn_join/cport as an RFU child). It is
 * a TRANSPARENT TRANSPORT relay: the trade-menu protocol (20-byte link blocks 0xAABB/0xDDDD/0xDCBA…)
 * is byte-identical across Gen-3 families and passes straight through as ordinary blocks. The
 * translator only converts transport — block framing (14-byte cable chunks ⇄ 12-byte RFU fragments),
 * held keys, standby/close barriers, LinkPlayer/trainer-card exchanges, "send block N" pulls — plus a
 * few small RS↔FRLG cross-gen byte fixups (LinkPlayer version/link-type, trainer-card version).
 *
 * PORT STATUS: this file is being ported in vertical slices. Slice 1 (this commit) is the
 * self-contained, host-testable core: size tables, key queue, cable-block framing, and the cross-gen
 * transforms. The stateful translator FSM (host pull/block, game block, barriers, NI) and the RFU
 * wire (wired to cport) land in later slices. Function names mirror the reference for diffability.
 *
 * Pure logic — no ESP-IDF, no RTOS, no hardware — so it builds and runs on the dev host
 * (host_tests/run.sh) against the reference logic and cport's captured-Switch vectors.
 */
#ifndef CABLE_AIR_H
#define CABLE_AIR_H

#ifndef CABLE_AIR_EARLY_HOST_CARD
#define CABLE_AIR_EARLY_HOST_CARD 0   /* see _requestEarlyCard; off: the real card, so it can be read in the room */
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

enum {
	/* RFU opcodes (high byte of a slot's first word; pokefirered link_rfu.h) */
	RFUCMD_MASK = 0xFF00,
	RFUCMD_READY_CLOSE_LINK = 0x5F00,
	RFUCMD_READY_EXIT_STANDBY = 0x6600,
	RFUCMD_SEND_PLAYER_IDS = 0x7700,
	RFUCMD_SEND_BLOCK_INIT = 0x8800,
	RFUCMD_SEND_BLOCK = 0x8900,
	RFUCMD_SEND_BLOCK_REQ = 0xA100,
	RFUCMD_SEND_HELD_KEYS = 0xBE00,
	RFUCMD_DISCONNECT = 0xED00,

	/* Cable commands (pokeruby link.h) */
	CA_LINKCMD_SEND_LINK_TYPE = 0x2222,
	CA_LINKCMD_READY_EXIT_STANDBY = 0x2FFE,
	CA_LINKCMD_READY_CLOSE_LINK = 0x5FFF,
	CA_LINKCMD_CONT_BLOCK = 0x8888,
	CA_LINKCMD_INIT_BLOCK = 0xBBBB,
	CA_LINKCMD_SEND_HELD_KEYS = 0xCAFE,
	CA_LINKCMD_SEND_BLOCK_REQ = 0xCCCC,

	/* librfu link-layer sub-frame states */
	LCOM_NULL = 0,
	LCOM_NI_START = 1,
	LCOM_NI = 2,
	LCOM_NI_END = 3,
	LCOM_UNI = 4,

	SLOT_BYTES = 14,
	FRAG_BYTES = 12,
	HOST_FRAME_BYTES = 3 + 5 * SLOT_BYTES,
	MAX_FRAGS = 24,
	MAX_BLOCK_BYTES = MAX_FRAGS * FRAG_BYTES,
	CMD_WORDS = 8,   /* RFU_WRAPPER_CMD_LENGTH */

	LINK_PLAYER_BLOCK_SIZE = 60,
	LP_LINK_TYPE_OFFSET = 16 + 0x14,
	LP_VERSION_OFFSET = 16,
	LP_TRAINER_ID_OFFSET = 16 + 4,
	LP_NAME_OFFSET = 16 + 8,
	LP_BUFFER_SIZE = 200,
	CARD_SIZE = 100,
	CARD_VERSION_OFFSET = 0x38,
	CARD_VERSION_LEAFGREEN = 5,
	/* bench: the Switch takes a Ruby/Sapphire partner down its own path (a 200-byte card swap, then a
	 * stall at the trade seat). Present the cart as an English Emerald that has cleared the game and has the
	 * National Dex, in the LinkPlayer, the trainer card and the NI game data (what GB-Link's Switch translator
	 * does for Ruby/Sapphire). FRLG reads its progress flags where Ruby/Sapphire's name field ends. */
	VERSION_EMERALD = 3,
	LP_PROGRESS_OFFSET = 16 + 0x10,
	PROGRESS_CLEARED = 0x11,
	NI_COMPAT_NATIONAL = (1 << 7) | (1 << 8) | (1 << 9),   /* canLinkNationally, hasNationalDex, gameClear */

	OWNER_FLAG = 0x80,
	CABLE_QUEUE = 256,        /* paced blocks ~2x the entries (idles interleaved); 200B held block = 34 */
	CA_BLOCK_INIT_GAP = 3,    /* idle packets after INIT_BLOCK — the decomp's 3-frame post-INIT delay */
	CA_BLOCK_CONT_GAP = 1,    /* idle packets between CONT frames — cb-3.65's proven pacing */
	CHILD_QUEUE = 8,
	HELD_BLOCKS = 8,
	ANNOUNCE_PACKETS = 30,
	ANNOUNCE_RETRY_PACKETS = 90,  /* cb-3.81: re-announce cadence until the game's LP is in */
	PULL_RETRY_PACKETS = 90,      /* cb-3.82: re-send an unanswered CCCC block request */
	/* After the LinkPlayer exchange the game still has to leave its link setup (pokeruby trade.c
	 * states 2-5). A block that arrives before it does is counted, then wiped by
	 * ResetBlockReceivedFlags, and the game ends up one exchange behind for good. */
	SESSION_SETTLE_PACKETS = 150,
	/* (c3rp bench, Ruby vs the Switch's LeafGreen): the game's standby is held until the leader pulls our
	 * trainer card (type 2), but the Switch's leader never pulls it there: it pulls type 0 (200 bytes), which the game
	 * answers with its gBlockSendBuffer = its card. Both sides then waited for each other's standby (black screens).
	 * Stop waiting for the card pull after this many packets (~2 s) and let the standby through. */
	CARD_PULL_WAIT_PACKETS = 180,   /* ~3 s: the Switch's wireless trade center never pulls type 2 here */
	BARRIER_EMITS = 6,
	INITIATE_TIMEOUT = 600,
	IDLE_TIMEOUT = 90,
	JOIN_HOLD_CAP = 3600,   /* max frames to hold a confirmation waiting on the Switch join (~60s) */
	HOST_SILENT_FRAMES = 600,
	EXIT_LINGER_FRAMES = 240,
};

enum { AIR_SEARCH, AIR_CONNECTING, AIR_NI, AIR_UNI };
enum { EXPECT_NONE, EXPECT_LINK_PLAYER, EXPECT_PULL, EXPECT_CARD };
enum { BAR_IDLE, BAR_STANDBY, BAR_CLOSE };
enum { PURPOSE_NONE, PURPOSE_ROUND0, PURPOSE_GAME_STANDBY, PURPOSE_GAME_CLOSE, PURPOSE_EXIT_CLOSE };

struct Recv {
	unsigned count;
	uint32_t flags;
	bool receiving;
	bool done;
	int lastIndex;
	uint8_t buf[MAX_BLOCK_BYTES];
};

enum { SEND_INIT, SEND_STREAM, SEND_HOLD, SEND_DONE };

struct Send {
	bool active;
	int state;
	unsigned count;
	unsigned index;
	unsigned initSends;
	unsigned holdSends;
	unsigned rr;
	uint8_t data[MAX_BLOCK_BYTES];
	bool isLinkPlayer;
	bool isCard;
};

struct ChildBlock {
	unsigned size;
	bool isLinkPlayer;
	bool isCard;
	uint8_t data[MAX_BLOCK_BYTES];
};

struct Barrier {
	int mode;
	bool initiated;
	int hostCount;      /* -1: none seen */
	unsigned localCount;
	unsigned sinceHost;
	unsigned sinceInitiate;
	int burstFor;
	unsigned burstN;
	unsigned rounds;
};

struct NI {
	int state;          /* 0 start, 1 sending, 2 last, 3 null, 4 done */
	unsigned phase;
	uint8_t n[4];
	unsigned now[4];
	int remain;
	uint8_t src[26];
	uint8_t header[7];
};

enum { KEY_QUEUE = 48, LINK_KEY_CODE_EMPTY = 0x11, LINK_KEY_CODE_EXIT_ROOM = 0x17 };

struct KeyQueue {
	uint8_t codes[KEY_QUEUE];
	unsigned head;
	unsigned count;
};

/* bench: the leader's held keys come at the Switch's 60 Hz but leave at the cable's rate (the RP2040 master
 * clocks ~42 Hz), so a plain FIFO overflowed (q=45/48 before the seat) and lost presses, the seat's READY among them.
 * Keys are kept as runs of one code and played back by time: each cable packet consumes `step` leader frames (the
 * measured in/out ratio, 1..3), faster when the backlog passes HOST_KEY_LAG frames; every run plays at least once,
 * so a one-frame press is never lost. Counts are fixed point, 1 frame = KEY_Q8. Same behaviour as GB-Link's
 * translator. */
enum { KEY_Q8 = 256, HOST_KEY_LAG = 12, KEY_RATE_WINDOW = 40 };
struct KeyRun { uint8_t code; int32_t count; };
struct HostKeyRuns {
	struct KeyRun runs[KEY_QUEUE];
	unsigned head;
	unsigned n;
	int32_t step;          /* leader frames per cable packet, KEY_Q8 units */
	unsigned pushes;
	unsigned pops;
};

/* Switch edge: the transport that carries LLSF child frames to/from the Switch (LDN/PIA). Provided by
 * the integration layer (slice 4: ldn_join/cport); all calls are made BY cable_air. Replaces
 * mgba_LDN's struct GBASIORFUBackend. */
struct CableAirSwitch {
	void* ctx;
	void (*search_start)(void* ctx);
	void (*search_stop)(void* ctx);
	void (*connect)(void* ctx, uint16_t device_id);
	void (*disconnect)(void* ctx, int reason);
	void (*send_frame)(void* ctx, const uint8_t* data, unsigned len);   /* one LLSF child frame out */
	void (*poll)(void* ctx);    /* optional, once per frame */
	void (*frame)(void* ctx);   /* optional, once per frame */
};

/* Optional trace sink (host tests print; device no-op). msg is a complete line without a newline. */
typedef void (*cable_air_trace_fn)(void* ctx, const char* msg);

/* The whole translator state. Mirrors mgba_LDN's struct Air, minus the mGBA emulator plumbing
 * (GBASIODriver/GBASIORFU/timing): on the C6 the cable edge is gba_cable and the Switch edge is
 * cport/ldn_join, wired in a later slice. */
struct CableAir {
	struct CableAirSwitch sw;
	cable_air_trace_fn trace;
	void* traceCtx;

	int link;
	uint16_t hostId;
	unsigned linkFrames;
	unsigned silentFrames;

	/* child link layer */
	struct NI ni;
	bool niStarted;
	bool hostUni;
	bool hostNiDone;
	uint8_t tag;
	uint8_t keyCount;
	struct Recv rx0;    /* the leader's own blocks */
	struct Recv rx1;    /* the leader's reflection of ours (our block's ack) */
	struct Send send;
	struct ChildBlock childQueue[CHILD_QUEUE];
	unsigned childHead;
	unsigned childCount;
	struct Barrier bar;
	unsigned lastHostOp;
	unsigned hostFrames;
	unsigned childFrames;

	/* translator state that outlives a cable link */
	bool haveRubyLP;
	bool haveHostLP;
	uint8_t rubyLP[LINK_PLAYER_BLOCK_SIZE];
	uint8_t hostLP[LINK_PLAYER_BLOCK_SIZE];
	bool lpSentToHost;
	bool lpPullWaiting;   /* leader pulled the LP before the game committed; answer on arrival */
	bool lpSendDone;
	bool round0Started;
	bool keysActive;
	struct KeyQueue rubyKeys;   /* game -> leader */
	struct HostKeyRuns hostKeys;   /* leader -> game, played back by time */
	int lastHostKeyCount;
	/* held-key bridge diagnostics: counters only, incremented on the hot path and read
	 * off it by cable_air_keystats_dump(). Never printf from the fsm/reply path (cb-2.16). */
	uint32_t statHostKeyRx;     /* 0xBE00 held-keys IN from the leader seen by _uniSlots */
	uint32_t statHostKeyPush;   /* ...actually pushed to hostKeys (fresh keyCount) */
	uint32_t statCafeReply;     /* 0xCAFE held-keys replies built toward the GBA */
	uint32_t statCafeReplyKey;  /* ...of which carried a non-EMPTY key */
	uint32_t statRubyKeyTx;     /* 0xBE00 held-keys OUT to the leader (GBA->Switch) */
	uint8_t  statLastHostKey;   /* last code pushed to hostKeys */
	uint8_t  statLastRubyKey;   /* last non-empty code sent up from rubyKeys */
	int purpose;
	unsigned closeRoundsLeft;
	unsigned closeCount;
	bool cancelPending;
	bool cancelClose;
	bool cancelReturn;
	bool cardArmed;
	unsigned cardAt;
	bool cardRequested;
	bool haveRubyCard;
	bool cardSynth;
	bool cardQueued;
	bool cardPullWaiting;
	bool cardSendDone;
	bool pendingGameStandby;
	uint32_t pendingStandbyAt;    /* cablePackets when the game's standby was held for the card pull */
	uint8_t rubyCard[CARD_SIZE];
	bool exitKeySeen;
	bool linkClosing;
	bool hostLPRepeat;
	bool returnRound;     /* our own standby for the Switch's trade menu after a trade (see _deliverLinkPlayerToGame) */    /* the leader's latest block was its LinkPlayer again: answer its pull with ours */     /* the game sent 5FFF: blocks wait for the reopened cable link */
	bool hostClosedSeen;
	unsigned exitLinger;
	bool exitKeyQueued;
	bool exitCloseDeferred;
	unsigned exitDeferFrames;
	bool roomClosed;
	bool pendingPurposeStandby;
	unsigned standbyRoundsSeen;

	/* join-confirmation gate (B). A cable "confirmation" is a standby round
	 * (game presses A -> READY_EXIT_STANDBY -> BAR_STANDBY). In option B the Switch's barrier answer
	 * arrives via the engine feed, but _barrierObserve releases the round on a 600-frame timeout even
	 * if the Switch never answered -> the GBA races past its confirmation before the Switch has joined,
	 * and the trade desyncs. switchReady is driven by the brain (host_status == JOIN_GROUP_OK); while
	 * it is false we HOLD the standby round (ignore the "Press A") so the game waits at the first
	 * confirmation until the Switch has accepted the join. Once accepted it stays true, so later
	 * confirmations pass normally. joinHoldFrames is a safety cap so a join that never lands can't
	 * hang the cart forever. */
	bool switchReady;
	unsigned joinHoldFrames;
	bool joinHoldLogged;

	struct ChildBlock held[HELD_BLOCKS];
	unsigned heldCount;
	int pendingPull;    /* -1: none */

	/* cable session state (reset whenever the game opens its link) */
	unsigned cablePackets;
	unsigned readyAt;
	bool announced;
	uint32_t announceRetryAt;     /* cb-3.81: next cablePackets count at which to re-announce */
	uint32_t pullRetryAt;         /* cb-3.82: next cablePackets count at which to re-request the pull */
	int expect;
	int pullType;
	bool sessionRubyLP;
	bool p0LPDelivered;
	unsigned rxSize;
	unsigned rxPos;
	uint8_t rxBuf[MAX_BLOCK_BYTES];
	uint16_t cq[CABLE_QUEUE][CMD_WORDS];
	unsigned cqHead;
	unsigned cqCount;
	unsigned sessions;
};

/* ---- slice 1 API (self-contained, host-tested) ---- */

/* little-endian 16-bit read from a byte buffer */
uint16_t cable_air_le16(const uint8_t* p);

/* fragment/size tables (link.c sBlockRequestLookupTable + the 20-byte link data) */
unsigned cable_air_frag_count(unsigned bytes);
unsigned cable_air_size_from_count(unsigned count);
unsigned cable_air_size_from_request(unsigned type);

/* key queue */
void    cable_air_key_push(struct KeyQueue* q, uint8_t code);
uint8_t cable_air_key_pop(struct KeyQueue* q);

/* cable-block framing: push {INIT_BLOCK,size,0x80} then CONT_BLOCK chunks of 7 words into cq[] */
void cable_air_push_block(struct CableAir* air, const uint8_t* data, unsigned size);
/* raw 3-word command push into cq[] */
void cable_air_push_cmd(struct CableAir* air, uint16_t c0, uint16_t c1, uint16_t c2);
/* pop the next queued cable command (8 words) into out; returns false if the queue is empty */
bool cable_air_cq_pop(struct CableAir* air, uint16_t out[CMD_WORDS]);

/* cross-gen transforms */
void cable_air_build_lp_for_host(struct CableAir* air, uint8_t out[LP_BUFFER_SIZE]);
void cable_air_lp_for_game(struct CableAir* air, uint8_t out[LINK_PLAYER_BLOCK_SIZE]);
void cable_air_build_host_card(struct CableAir* air, uint8_t out[CARD_SIZE]);
/* stamp an RS trainer card (0x38-byte RSE layout) as the FRLG-side card the leader expects */
void cable_air_stamp_card_for_host(const uint8_t* rs_card, uint8_t out[CARD_SIZE]);

/* ---- lifecycle / edges ---- */
void cable_air_init(struct CableAir* air);
void cable_air_set_switch(struct CableAir* air, const struct CableAirSwitch* sw);
void cable_air_set_trace(struct CableAir* air, cable_air_trace_fn fn, void* ctx);
void cable_air_keystats_dump(const struct CableAir* air);   /* [air-keys] held-key bridge counters */

/* ---- cable edge (the peer interface gba_cable drives; real cart = master/"the game") ---- */
void cable_air_reset(struct CableAir* air);                                        /* cable link (re)opened */
void cable_air_game_command(struct CableAir* air, const uint16_t cmd[CMD_WORDS]);  /* cart finished a packet */
bool cable_air_next_command(struct CableAir* air, uint16_t cmd[CMD_WORDS]);        /* our reply to the cart */
void cable_air_frame(struct CableAir* air);                                        /* once per emulated frame */

/* ---- switch edge events (the transport pushes these in; whole-child / option-A path) ---- */
void cable_air_on_broadcast(struct CableAir* air, uint16_t device_id, int slot, const uint32_t words[6]);
void cable_air_on_connect_result(struct CableAir* air, bool accepted, int slot);
void cable_air_on_host_frame(struct CableAir* air, const uint8_t* data, unsigned len);
void cable_air_on_disconnected(struct CableAir* air);

/* ---- option-B engine seam: cable_air as cport/sim's trade-slot engine ----
 * sim owns NI + reliable + Pia to the Switch; these give it the trade slots. */
void cable_air_engine_tick(struct CableAir* air, uint16_t words[CMD_WORDS - 1]);   /* next child UNI slot */
void cable_air_engine_feed_slots(struct CableAir* air, const uint8_t slot0[SLOT_BYTES],
                                 const uint8_t slot1[SLOT_BYTES]);                 /* leader cmd + our-ack */
/* the cart identity for sim's Switch NI game-data; false until the game's LinkPlayer arrives */
bool cable_air_get_identity(const struct CableAir* air, uint16_t* tid, uint8_t name8[8]);

/* Join-confirmation gate (option B). The mode drives this from the brain's join status: pass true
 * once the Switch has accepted the join (host_status == JOIN_GROUP_OK). Until then the cable child
 * holds its standby confirmation ("Press A") so the game waits for the Switch instead of racing past.
 * Returns true once the game is actually parked at a confirmation waiting on the join (so the mode can
 * surface it / trigger the association). */
void cable_air_set_switch_ready(struct CableAir* air, bool ready);
bool cable_air_join_pending(const struct CableAir* air);

#ifdef __cplusplus
}
#endif

#endif
