/* SPDX-License-Identifier: AGPL-3.0-only
 * Derived from GB-Link-Switch-LDN (https://github.com/GB-Link/GB-Link-Switch-LDN), Copyright (c) the GB-Link-Switch-LDN authors.
 * Changes Copyright (c) 2026 Kizuna. See NOTICE.
 */
/* gl_lead_shim.c -- the lead-role half of GB-Link's trade shim: the GBA leads, the Switch is its child.
 *
 * Copied from GB-Link-Switch-LDN firmware/common/trade_shim.c (commit c972db4, AGPL-3.0,
 * https://github.com/GB-Link/GB-Link-Switch-LDN); see NOTICE. Kizuna changes: only the lead-role functions, the state they
 * share and the frame helpers are kept; the reset-surviving event log (~7 KB of RTC memory) is dropped, so log_event is a
 * no-op; the public names carry a gls_ prefix. */
#include "gl_lead_shim.h"

#include <stdio.h>
#include <string.h>

static inline void log_event(int64_t now_ms, uint8_t type, uint8_t a, uint16_t b, uint16_t c)
{ (void)now_ms; (void)type; (void)a; (void)b; (void)c; }
enum { EV_BOOT = 1, EV_RESET, EV_CONFIRM, EV_PARENT_STANDBY, EV_CHILD_STANDBY, EV_REQUEST, EV_INJECT,
       EV_ACK, EV_REANSWER, EV_REREQUEST, EV_NOTE, EV_HOST_BLOCK, EV_GIVE_UP };

#define RFUCMD_READY_EXIT_STANDBY 0x6600
#define RFUCMD_SEND_BLOCK_INIT 0x8800
#define RFUCMD_SEND_BLOCK 0x8900
#define RFUCMD_SEND_BLOCK_REQ 0xa100
#define RFUCMD_SEND_HELD_KEYS 0xbe00
#define LINK_KEY_CODE_EMPTY 0x11
#define LINK_KEY_CODE_DPAD_RIGHT 0x15   /* 0x12-0x15: directions */
#define LINKCMD_CONFIRM_FINISH_TRADE 0xdcba

/* Post-trade standby rounds of a retail cartridge: trade_scene.c CB2_SaveAndEndTrade
   cases 1, 41, 5, 8, then trade.c CB2_CreateTradeMenu case 4. After the fifth answer
   the Switch parent waits in its sixth barrier. */
#define TRADE_SHIM_CHILD_ROUNDS 5
/* Quiet time after the fifth answer before injecting. Covers the parent processing its
   answer (1-2 frames) and a five-round parent sending its party request (~12 frames). */
#define TRADE_SHIM_SETTLE_MS 750
/* Fallback for other round counts: any post-trade answer followed by this much silence
   on both sides. Longer than the cartridge's save, the longest normal gap. */
#define TRADE_SHIM_FALLBACK_MS 8000
/* A waiting parent answers the extra round within 1-2 frames. */
#define TRADE_SHIM_ACK_MS 2000
#define TRADE_SHIM_MAX_ATTEMPTS 4
#define TRADE_SHIM_MAX_FAKES 16
/* A child that accepts a block request starts sending within 1-2 frames. Silence this
   long means it refused (Rfu_InitBlockSend returns FALSE while a block send or another
   command is pending). Measured as child silence so a slow accepted request is not
   repeated. */
#define TRADE_SHIM_REREQUEST_MS 700
#define TRADE_SHIM_MAX_REREQUESTS 3
/* Lead role: the Switch buffers its party and clears its block flags a few frames
   after its extra round is answered. Delay before releasing the GBA's held request and
   block. A hold with no extra round ends after the timeout. */
#define TRADE_SHIM_RELEASE_MS 150
#define TRADE_SHIM_HOLD_TIMEOUT_MS 5000
/* Lead role: repeat an unanswered request after this. Longer because the Switch's
   answering block can take a while to start. */
#define TRADE_SHIM_LEAD_REREQUEST_MS 1500
/* Fragments per block request type. sBlockRequests[] in link_rfu_2.c: 200, 200, 100,
   220, 40 bytes in 12-byte fragments. */
static const uint8_t kRequestFragments[] = {17, 17, 9, 19, 4};

/* ---- state ------------------------------------------------------------------- */

static struct
{
    uint16_t fakes[TRADE_SHIM_MAX_FAKES];   /* extra rounds, Switch numbering, ascending */
    int fake_count;
    int base;                 /* fakes evicted from the list, as an offset */
    uint8_t send_tag;         /* tag for the next frame sent to the parent */
    bool have_tag;            /* child has sent a command */
    bool post_trade;          /* from trade confirmation to the next block request */
    bool injected;            /* extra round supplied for this trade */
    int last_round;           /* newest parent standby answer, parent numbering, -1 none */
    int last_child_round;     /* newest forwarded child standby, parent numbering */
    int answers_since_confirm;
    int fake_round;           /* extra round, parent numbering, -1 none */
    bool fake_acked;
    int attempts;
    int64_t injected_at;
    int64_t quiet_since;
    int host_block_count;     /* fragment count of the parent's last BLOCK_INIT */
    int request_type;         /* parent's newest block request type, -1 none */
    bool request_served;      /* child has started answering it */
    int64_t request_at;       /* time of the request or its latest repeat */
    int request_attempts;
    int injections, reanswers, rerequests, trades, give_ups;
    uint16_t child_commands;  /* command frames sent to the parent, injected included */
    uint16_t echoes;          /* host frames echoing a child command */
    bool raw_tag_valid;
    uint8_t last_raw_tag;
    int tag_gaps;
    /* Child's block in flight. The parent echoes each accepted fragment once, in slot 1.
       A missing echo is a send failure for the child (its resends carry no tag). */
    struct { bool active; uint8_t count; uint32_t sent, echoed; uint8_t data[32][12]; } cb;
    /* Parent's block in flight. Joiner role: gap logging. Lead role: fragments kept for
       replay, since the Switch wipes a block that arrives early. */
    struct { bool active; uint8_t count, expect; uint32_t have; uint8_t data[32][12]; } pb;
    int replay_index, replay_count;   /* lead release: init, fragments, request */
    bool request_answering;           /* lead: Switch's answering block has begun */
    /* Lead hold: GBA block commands held from its last post-trade round until the
       Switch's extra round is answered. */
    bool hold;
    int64_t hold_since, release_at;
    int echoes_synthesized, resends_restamped;
    /* Child's newest command frame as received. */
    uint8_t last_frame[16];
    bool have_last_frame;
    int64_t last_command_ms;
    /* Recent command frames sent to the parent and their echoes, for the log. */
    struct { uint32_t ms; uint16_t command; uint8_t tag; } sent_ring[16];
    struct { uint32_t ms; uint16_t command; } echo_ring[16];
    int sent_head, echo_head, sent_count, echo_count;
    int64_t clock_ms, last_echo_ms;
    int sent_since_echo;
    bool echo_stall_reported;
    uint32_t request_frags;   /* fragment indices sent since the request */
    int duplicates;
} s;

static uint16_t rd16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static void wr16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }

/* Round-number mapping. The Switch runs the extra rounds, so its numbers are ahead.
   Fakes use Switch numbering. Cartridge round b maps to b plus every extra round at or
   below the result. Joiner role: Switch = parent. Lead role: Switch = child. */
static int to_ahead(int b)
{
    int a = b + s.base;
    for (int i = 0; i < s.fake_count; ++i) if (s.fakes[i] <= a) ++a;
    return a;
}
static int to_behind(int a)
{
    int b = a - s.base;
    for (int i = 0; i < s.fake_count; ++i) if (s.fakes[i] < a) --b;
    return b;
}
static bool is_fake(int p)
{
    for (int i = 0; i < s.fake_count; ++i) if (s.fakes[i] == p) return true;
    return false;
}
static void add_fake(int p)
{
    if (s.fake_count == TRADE_SHIM_MAX_FAKES)
    {
        /* Rounds only increase: fold the oldest fake into the offset. */
        memmove(s.fakes, s.fakes + 1, sizeof(s.fakes) - sizeof(s.fakes[0]));
        --s.fake_count;
        ++s.base;
    }
    s.fakes[s.fake_count++] = (uint16_t)p;
}

/* Parent UNI frame: 3-byte LLSF header (one child slot, UNI state, 70 data bytes) and
   five 14-byte slots. */
static size_t build_host_frame(uint8_t *out, uint16_t cmd0, uint16_t v0, uint16_t cmd1, uint16_t v1)
{
    memset(out, 0, 73);
    out[0] = 0x46; out[1] = 0x00; out[2] = 0x05;
    wr16(out + 3, cmd0); wr16(out + 5, v0);
    wr16(out + 17, cmd1); wr16(out + 19, v1);
    return 73;
}

/* ---- lead role: GBA is parent, Switch is child ------------------------------------ */

size_t gls_lead_child(uint8_t *payload, size_t length, int64_t now_ms, uint8_t *reply, size_t reply_capacity, bool *forward)
{
    if (forward) *forward = true;
    s.clock_ms = now_ms;
    if (length < 16) return 0;
    if (((rd16(payload) >> 10) & 15) != 4) return 0;  /* not a UNI frame */
    uint8_t *slot = payload + 2;
    if (slot[1] == 0) return 0;                        /* idle: no command */
    s.quiet_since = s.last_command_ms = now_ms;
    s.have_tag = true;
    uint16_t command = (uint16_t)((slot[1] << 8) | (slot[0] & 0x1f));
    uint8_t index = (uint8_t)(slot[0] & 0x1f);
    bool fragment = (command & 0xff00) == RFUCMD_SEND_BLOCK;
    if (s.request_type >= 0 && !s.request_served)
    {
        size_t type = (size_t)s.request_type;
        if (command == RFUCMD_SEND_BLOCK_INIT || fragment) s.request_answering = true;
        if (type >= sizeof(kRequestFragments)) s.request_served = true;
        else if (command == RFUCMD_SEND_BLOCK_INIT) s.request_served = rd16(slot + 2) == kRequestFragments[type];
        else if (fragment && index < 32)
        {
            uint32_t all = (1u << kRequestFragments[type]) - 1;
            s.request_frags |= 1u << index;
            s.request_served = (s.request_frags & all) == all;
        }
    }
    if (command != RFUCMD_READY_EXIT_STANDBY || reply_capacity < 146) return 0;

    /* Switch frames keep their own tags, which the GBA requires consecutive, so none is
       dropped. A frame the GBA must ignore gets round number 0xffff. */
    int c = rd16(slot + 2);
    if (is_fake(c))
    {
        /* Extra round retried: the answer has not reached the Switch yet. */
        wr16(slot + 2, 0xffff);
        log_event(now_ms, EV_CHILD_STANDBY, 1, (uint16_t)c, 0xffff);
        return build_host_frame(reply, RFUCMD_READY_EXIT_STANDBY, (uint16_t)c, RFUCMD_READY_EXIT_STANDBY, (uint16_t)c);
    }
    int p = to_behind(c);
    log_event(now_ms, EV_CHILD_STANDBY, 0, (uint16_t)c, (uint16_t)p);
    size_t n = 0;
    if (p > s.last_round)
    {
        /* Past the GBA's newest barrier. After the cartridge's post-trade rounds this is
           the Switch's extra round, which the GBA never answers. Answered here and
           forwarded as 0xffff, since the GBA's barrier of that number is still ahead. */
        wr16(slot + 2, (uint16_t)p);
        if (!s.post_trade || s.injected || s.answers_since_confirm < TRADE_SHIM_CHILD_ROUNDS) return 0;
        add_fake(c);
        s.fake_round = c;
        s.injected = true;
        s.post_trade = false;
        ++s.injections;
        if (s.hold) s.release_at = now_ms + TRADE_SHIM_RELEASE_MS;
        wr16(slot + 2, 0xffff);
        log_event(now_ms, EV_INJECT, 1, (uint16_t)c, (uint16_t)s.answers_since_confirm);
        printf("LDN_BRIDGE answered the Switch's extra standby round %d for the GBA after %d rounds\n", c, s.answers_since_confirm);
        n = build_host_frame(reply, RFUCMD_READY_EXIT_STANDBY, (uint16_t)c, RFUCMD_READY_EXIT_STANDBY, (uint16_t)c);
    }
    else
    {
        /* Retry of a barrier the GBA already left: its answer was lost. Answer for it. */
        wr16(slot + 2, (uint16_t)p);
        ++s.reanswers;
        log_event(now_ms, EV_REANSWER, 1, (uint16_t)c, (uint16_t)p);
        printf("LDN_BRIDGE Switch is retrying standby round %d that the GBA already answered (as %d), answering for it\n", c, p);
        n = build_host_frame(reply, RFUCMD_READY_EXIT_STANDBY, (uint16_t)c, RFUCMD_READY_EXIT_STANDBY, (uint16_t)c);
    }
    /* A party request refused during that barrier is repeated by
       gls_lead_inject. The GBA does not repeat it. */
    return n;
}

void gls_lead_parent(uint8_t *payload, size_t length, int64_t now_ms)
{
    s.clock_ms = now_ms;
    if (length < 3) return;
    uint32_t header = payload[0] | (payload[1] << 8) | ((uint32_t)payload[2] << 16);
    if (((header >> 14) & 15) != 4) return;             /* not a UNI frame */
    size_t size = header & 0x7f;
    if (size < 28 || 3 + size > length) return;
    for (int i = 0; i < 2; ++i)                       /* slot 0: GBA, slot 1: echo of the Switch */
    {
        uint8_t *slot = payload + 3 + 14 * i;
        uint16_t command = rd16(slot), value = rd16(slot + 2);
        if (command == 0) continue;
        s.quiet_since = now_ms;
        if (i == 0 && (command & 0xff00) == RFUCMD_SEND_BLOCK && s.pb.active)
        {
            uint8_t index = (uint8_t)(command & 0x1f);
            if (index < s.pb.count) { memcpy(s.pb.data[index], slot + 2, 12); s.pb.have |= 1u << index; }
        }
        if (i == 0)
        {
            if (command == RFUCMD_SEND_BLOCK_INIT)
            {
                if (value != s.host_block_count) log_event(now_ms, EV_HOST_BLOCK, 0, value, 0);
                s.host_block_count = value;
                s.pb.active = value >= 1 && value <= 32;
                s.pb.count = (uint8_t)value;
                s.pb.have = 0;
            }
            if (s.hold && (command == RFUCMD_SEND_BLOCK_REQ || command == RFUCMD_SEND_BLOCK_INIT || (command & 0xff00) == RFUCMD_SEND_BLOCK))
            {
                /* GBA past its last barrier, Switch still in its extra one. A request now
                   would arrive before the Switch buffers its party and the block would be
                   cleared. Held here, released by gls_lead_inject. */
                if (command == RFUCMD_SEND_BLOCK_REQ)
                {
                    s.request_type = value;
                    s.request_served = s.request_answering = false;
                    s.request_attempts = 0;
                    s.request_frags = 0;
                    log_event(now_ms, EV_REQUEST, 2, value, 0);
                }
                memset(slot, 0, 14);
                continue;
            }
            else if (command == RFUCMD_SEND_BLOCK && s.host_block_count == 2 && value == LINKCMD_CONFIRM_FINISH_TRADE)
            {
                s.post_trade = true;
                s.injected = false;
                s.answers_since_confirm = 0;
                s.fake_round = -1;
                s.hold = false;
                s.release_at = 0;
                ++s.trades;
                log_event(now_ms, EV_CONFIRM, 1, (uint16_t)s.trades, (uint16_t)(s.last_round + 1));
                printf("LDN_BRIDGE trade %d confirmed, watching the post-trade standby rounds\n", s.trades);
            }
            else if (command == RFUCMD_SEND_BLOCK_REQ)
            {
                s.request_type = value;
                s.request_served = s.request_answering = false;
                s.request_at = now_ms;
                s.request_attempts = 0;
                s.request_frags = 0;
                log_event(now_ms, EV_REQUEST, s.post_trade ? 1 : 0, value, 0);
            }
        }
        if (command != RFUCMD_READY_EXIT_STANDBY) continue;
        if (i == 0 && (int)value > s.last_round)
        {
            s.last_round = value;
            if (s.post_trade && ++s.answers_since_confirm == TRADE_SHIM_CHILD_ROUNDS && !s.hold)
            {
                s.hold = true;
                s.hold_since = now_ms;
                s.release_at = 0;
            }
        }
        uint16_t child_value = (uint16_t)to_ahead(value);
        wr16(slot + 2, child_value);
        log_event(now_ms, EV_PARENT_STANDBY, (uint8_t)(2 + i), value, child_value);
    }
}

/* Replay order: GBA's block first, request last. The Switch answers the request at once
   with its own block, whose echoes would otherwise crowd out the replayed fragments. */
static size_t lead_release_frame(uint8_t *out)
{
    int k = s.replay_index++;
    if (k == s.replay_count - 1) return build_host_frame(out, RFUCMD_SEND_BLOCK_REQ, (uint16_t)s.request_type, 0, 0);
    if (k == 0) return build_host_frame(out, RFUCMD_SEND_BLOCK_INIT, s.pb.count, 0x80, 0);
    uint8_t index = (uint8_t)(k - 1);
    memset(out, 0, 73);
    out[0] = 0x46; out[1] = 0x00; out[2] = 0x05;
    wr16(out + 3, (uint16_t)(RFUCMD_SEND_BLOCK | index));
    memcpy(out + 5, s.pb.data[index], 12);
    return 73;
}

static bool parent_block_whole(void)
{
    return s.pb.active && s.pb.count > 0 && s.pb.have == (s.pb.count == 32 ? 0xffffffffu : (1u << s.pb.count) - 1);
}

size_t gls_lead_inject(int64_t now_ms, uint8_t *out, size_t capacity)
{
    if (capacity < 73) return 0;
    if (s.replay_index < s.replay_count)
    {
        size_t n = lead_release_frame(out);
        if (s.replay_index == s.replay_count && s.hold) { s.hold = false; s.request_at = now_ms; }
        return n;
    }
    if (s.hold)
    {
        /* Release after release_at, or after the timeout if no extra round came. Block
           and request if the block is whole, else the request alone. */
        const bool timeout = now_ms - s.hold_since > TRADE_SHIM_HOLD_TIMEOUT_MS;
        if (s.request_type < 0) { if (timeout) s.hold = false; return 0; }
        const bool due = (s.release_at && now_ms >= s.release_at) || timeout;
        if (!due) return 0;
        if (!parent_block_whole() && !timeout && now_ms < s.release_at + 500) return 0;
        s.replay_count = parent_block_whole() ? 2 + s.pb.count : 1;
        s.replay_index = 0;
        s.release_at = 0;
        log_event(now_ms, EV_REREQUEST, 0x20, (uint16_t)s.request_type, (uint16_t)s.replay_count);
        printf("LDN_BRIDGE releasing the GBA's %sparty request %d to the Switch\n",
               s.replay_count > 1 ? "block and " : "", s.request_type);
        size_t n = lead_release_frame(out);
        if (s.replay_index == s.replay_count) { s.hold = false; s.request_at = now_ms; }
        return n;
    }
    if (s.request_type < 0 || s.request_served || s.request_answering) return 0;
    if (s.request_attempts >= TRADE_SHIM_MAX_REREQUESTS || now_ms - s.request_at < TRADE_SHIM_LEAD_REREQUEST_MS
        || now_ms - s.last_command_ms < TRADE_SHIM_REREQUEST_MS) return 0;
    /* Request refused by the Switch's link layer (barrier still open). The GBA asks once.
       Request only: the block already reached the Switch, and a second copy would cut
       into the answer. */
    ++s.request_attempts;
    ++s.rerequests;
    s.request_at = now_ms;
    s.replay_count = 1;
    s.replay_index = 0;
    log_event(now_ms, EV_REREQUEST, (uint8_t)(0x10 | s.request_attempts), (uint16_t)s.request_type, 1);
    printf("LDN_BRIDGE Switch has not answered the GBA's party request %d, repeating it (attempt %d)\n",
           s.request_type, s.request_attempts);
    return lead_release_frame(out);
}

/* Lead role: own slot and echo slot each empty, no key or a direction; no other slots
   in use. */
bool gls_parent_sheddable(const uint8_t *payload, size_t length)
{
    if (length < 3 + 14 * 2) return false;
    uint32_t header = payload[0] | (payload[1] << 8) | ((uint32_t)payload[2] << 16);
    if (((header >> 14) & 15) != 4) return false;
    for (size_t o = 3; o + 14 <= length; o += 14)
    {
        const uint8_t *slot = payload + o;
        if (slot[1] == 0 && slot[0] == 0) continue;
        if (o >= 3 + 14 * 2 || slot[1] != (RFUCMD_SEND_HELD_KEYS >> 8)) return false;
        const uint8_t key = slot[2];
        if (!(key == 0 || (key >= LINK_KEY_CODE_EMPTY && key <= LINK_KEY_CODE_DPAD_RIGHT))) return false;
    }
    return true;
}


void gls_reset(void)
{
    memset(&s, 0, sizeof(s));
    s.last_round = -1;
    s.fake_round = -1;
    s.request_type = -1;
}
