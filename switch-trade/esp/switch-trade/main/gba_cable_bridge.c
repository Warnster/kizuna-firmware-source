/* gba_cable_bridge — the GBA seam for Ruby/Sapphire (switch-trade built with -DKZ_GBA_LINK=cable).
 *
 * Implements rfu1_bridge.h, so switch_trade_main.c and trade_session.c run unchanged, but over GB-Link's
 * LINK-CABLE mode (0x01) instead of the wireless-adapter mode (0x07). Ruby and Sapphire have no wireless adapter
 * support: the cart talks the Gen-3 cable protocol, and cable_air (ported from mgba_LDN) translates that to the FRLG
 * wireless trade the Switch speaks. cable_air is also cport/sim's trade-slot engine (ldn_brain_set_engine).
 *
 * What the RP2040 does in mode 0x01 (GB-Link src/module/link.cpp, sections/usbSection, layers/packetLayer):
 *   - it is the other GBA on the cable: SetModeMaster (0x10) makes it the multiplayer parent that clocks the link, so
 *     the cart is the child (player 1), matching the Switch's view where the Switch leads (player 0) and we joined.
 *     The RP2040 sends one 8-halfword command every ~24 ms;
 *   - it runs the link handshake itself (0xB9A0 / 0x8FFF) and the per-packet CRC word: we only ever see commands;
 *   - statuses: AwaitMode 0xFF02 (send 0x10), HandshakeReceived 0xFF03 (the cart opened its link; we answer with
 *     StartHandshake 0x12, then ConnectLink 0x13 so the RP2040 sends 0x8FFF), LinkConnected 0xFF05, LinkReconnecting 0xFF06 (the cart closed the link, a new section
 *     waits for the next handshake), LinkClosed 0xFF07 (the cart left with EXIT_ROOM: the mode has ended);
 *   - cart -> us: received commands batched FOUR to a 64-byte data frame. An all-zero batch is never sent, and in a
 *     run of 0xCAFE/0x11 (held keys, nothing pressed) every packet after the first is zeroed. cable_air ignores zero
 *     commands, so nothing it needs is lost; but silence on the UART does NOT mean the cart stopped;
 *   - us -> cart: 64-byte data frames, split into four 16-byte commands on a 200-deep queue, one per cart transfer.
 *     An empty queue sends 0x0000 words, which the cart treats as "no command" (GB-Link's online mode relies on it).
 *
 * Reply pacing. cable_air_next_command() is the per-transfer reply: its packet counter drives the announce/pull
 * retries and the block gaps. Replies go four to a data frame, every frame (idle and key reports included), keeping at
 * most AHEAD commands queued in the RP2040 by wall time at an estimated 26 ms per command (the RP2040 master's real
 * period is ~24 ms, so the queue runs dry now and then instead of growing). Same pump as GB-Link's cable.js.
 *
 * Threading: the gblink RX task only queues statuses and commands (tap callbacks). Everything else, including every
 * cable_air call, runs in rfu1_bridge_poll() on the brain task, the same task that runs ldn_brain_tick() and so the
 * engine callbacks: cable_air is single-threaded with no lock. cable_air's trace goes through a ring to a low-priority
 * log task.
 *
 * Memory: the translator state is ~12 KB. It is allocated when a cart opens its link and freed once the cart has gone
 * and no trade runs, so the idle Bluetooth heap matches the FireRed/LeafGreen image the C3 budget was measured on
 *. By then Bluetooth is about to stop for the session anyway. */
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "nvs.h"

#include "rfu1_bridge.h"
#include "gblink.h"
#include "gba_bridge.h"
#include "ldn_brain.h"
#include "cable_air.h"
#include "cable_air_sim.h"

static const char *TAG = "cable";

/* GB-Link link-mode commands (src/module/link.hpp) and statuses (src/linkStatus.hpp) */
#define LINKCMD_SET_MODE_MASTER  0x10
#define LINKCMD_START_HANDSHAKE  0x12
#define LINKCMD_CONNECT_LINK     0x13
#define ST_HANDSHAKE_RECEIVED    0xFF03
#define ST_LINK_CONNECTED        0xFF05
#define ST_LINK_RECONNECTING     0xFF06

/* Bench: the cart must be the cable CHILD (player 1 after the leader). As the cable master it took
 * multiplayer id 0, the Switch (the wireless leader) is id 0 too, so both games put themselves on player 0's spot and
 * chair and the Switch never reached its trade menu. GB-Link's Switch translator has its adapter clock the cart
 * (master) for the same reason. The RP2040 master sends one command per ~24 ms (packetLayer: 8 x 1378 us word gaps +
 * 12953 us between commands), so replies are paced to that clock. */
#define FRAME_US          26000           /* estimated slightly long (as GB-Link's cable.js): the queue runs dry
                                            * now and then rather than growing */
#define AHEAD             6               /* commands queued in the RP2040 at most */
#define CONNECT_DELAY_US  300000          /* StartHandshake, let a few 0xB9A0 go out, then ConnectLink (0x8FFF) */
#define GROUP             4              /* commands per data frame, both directions */
#define REENTER_US        500000LL       /* LinkClosed -> SetMode 0x01 again, as GB-Link cable.js (5 s made no difference) */
#define GONE_LINGER_US    6000000LL      /* LinkClosed -> "GBA gone" (cable_air's exit linger is 4 s) */
#define OPEN_ACTIVE_US    180000000LL    /* an open link counts as activity for this long without any frame */
#define BROWSE_EVERY_US   1000000LL

typedef enum { CL_OFF, CL_AWAIT, CL_HANDSHAKE, CL_CONNECTED, CL_CLOSED } cl_state_t;
static const char *const CL_NAMES[] = { "off", "await_cart", "handshake", "connected", "closed" };

typedef struct { uint8_t kind; uint16_t w[8]; } cl_ev_t;   /* kind 0 = status in w[0], 1 = cart command */
static QueueHandle_t s_evq;

static bool          s_up, s_uart_up;
static cl_state_t    s_state = CL_OFF;
static bool          s_hs_pending;          /* the cart is handshaking; StartHandshake once the Switch room is up */
static int64_t       s_connect_at_us;       /* when to send ConnectLink after StartHandshake (0 = none due) */
static int64_t       s_last_activity_us;    /* any status or command from the RP2040 */
static int64_t       s_closed_at_us;
static int64_t       s_mode_lost_at_us;     /* link mode ended with no visit; AwaitMode expected */
/* Restart after every Cable Club visit . The C3's heap fragments across visits: after one
 * visit the decoder's 31 KB no longer had a block (bench), and idle memory was 5 KB lower. Each visit now starts from
 * the boot heap, as GB-Link's board does ("when you leave the room the board restarts"). Keys, Wi-Fi and the cart
 * identity are in NVS; the RP2040 keeps running and is put back in link mode at boot. */
#define VISIT_RESTART_US 1500000LL
static int64_t       s_restart_at_us;
static bool          s_reentered, s_gone_sent;
static int64_t       s_last_browse_us;
static bool          s_brain_was_active;    /* to see the Switch session end */
static int64_t       s_session_end_us;      /* when the Switch session ended with the cart still linked (0 = no) */
#define SESSION_END_GRACE_US 8000000LL      /* the cart's own room exit (EXIT_ROOM, the leader's close, our 5FFF) */
static bool          s_cancel_sent;         /* we dropped the cable link; LinkClosed follows */
static int64_t       s_cancel_at_us;

/* reply pacing */
static int64_t       s_t0_us;
static uint32_t      s_generated;           /* replies generated since LinkConnected */
static bool          s_last_sent_cafe_empty;
static uint32_t      s_frames_out, s_groups_dropped, s_cmds_in, s_sessions, s_ev_drops;

/* the translator and its sim engine */
static struct CableAir     *s_air;              /* NULL while no cart is linking (see Memory above) */
static struct cable_air_sim s_air_sim;
static bool                 s_engine_set;
static bool                 s_free_pending;
static bool                 s_identity_pushed, s_identity_real;
static bool                 s_join_wait_logged, s_join_done_logged;

/* ---- cart identity cache  ----
 * The cart's LinkPlayer (trainer id + name) only arrives once the game commits to the trade room, after the Cable
 * Club confirmations. The Switch join needs an identity long before that, so the first run caches it and later runs
 * join with the cached one the moment the cable handshake starts. A swapped cart re-caches on its first run. */
#define ID_NVS_NS  "lct"
#define ID_NVS_KEY "cartid"
struct cart_id { uint16_t tid; uint8_t name[8]; };
static bool           s_have_cached_id;
static struct cart_id s_cached_id;
static bool           s_save_pending;      /* the cart's real identity, saved once it has left (no flash write mid-trade) */
static struct cart_id s_seen_id;

static void id_cache_load(void)
{
    nvs_handle_t h;
    if (nvs_open(ID_NVS_NS, NVS_READONLY, &h) != ESP_OK) return;
    size_t sz = sizeof s_cached_id;
    if (nvs_get_blob(h, ID_NVS_KEY, &s_cached_id, &sz) == ESP_OK && sz == sizeof s_cached_id) s_have_cached_id = true;
    nvs_close(h);
}

static void id_cache_save(uint16_t tid, const uint8_t name[8])
{
    if (s_have_cached_id && s_cached_id.tid == tid && !memcmp(s_cached_id.name, name, 8)) return;
    struct cart_id id = { .tid = tid };
    memcpy(id.name, name, 8);
    nvs_handle_t h;
    if (nvs_open(ID_NVS_NS, NVS_READWRITE, &h) != ESP_OK) return;
    if (nvs_set_blob(h, ID_NVS_KEY, &id, sizeof id) == ESP_OK && nvs_commit(h) == ESP_OK) { s_cached_id = id; s_have_cached_id = true; }
    nvs_close(h);
}

/* ---- cable_air's step-by-step trace: a bench aid, OFF by default (build with -DKZ_CABLE_TRACE=1). A ring written on
 * the brain task and printed by a low-priority task, never printf on the brain task. ---- */
#ifndef KZ_CABLE_TRACE
#define KZ_CABLE_TRACE 0
#endif
#if KZ_CABLE_TRACE
#define TR_N   64
#define TR_LEN 192
static portMUX_TYPE s_tr_mux = portMUX_INITIALIZER_UNLOCKED;
static char         s_tr_ring[TR_N][TR_LEN];
static uint32_t     s_tr_wr, s_tr_rd, s_tr_drop;

static void air_trace(void *ctx, const char *msg)
{
    (void)ctx;
    portENTER_CRITICAL(&s_tr_mux);
    if (s_tr_wr - s_tr_rd >= TR_N) s_tr_drop++;
    else { strlcpy(s_tr_ring[s_tr_wr % TR_N], msg, TR_LEN); s_tr_wr++; }
    portEXIT_CRITICAL(&s_tr_mux);
}

static void log_task(void *arg)
{
    (void)arg;
    for (;;) {
        for (;;) {
            char line[TR_LEN];
            uint32_t drop = 0;
            bool have;
            portENTER_CRITICAL(&s_tr_mux);
            have = s_tr_rd != s_tr_wr;
            if (have) {
                memcpy(line, s_tr_ring[s_tr_rd % TR_N], TR_LEN);
                s_tr_rd++;
                if (s_tr_rd == s_tr_wr) { drop = s_tr_drop; s_tr_drop = 0; }
            }
            portEXIT_CRITICAL(&s_tr_mux);
            if (!have) break;
            printf("  [air] %s\n", line);
            if (drop) printf("  [air] (%u trace lines dropped)\n", (unsigned)drop);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
#define CABLE_SET_TRACE(a) cable_air_set_trace((a), air_trace, NULL)
#else
#define CABLE_SET_TRACE(a) ((void)(a))
#endif /* KZ_CABLE_TRACE */

/* ---- RX task side: queue only ---- */
static void ev_push(const cl_ev_t *e)
{
    if (xQueueSend(s_evq, e, 0) != pdTRUE) s_ev_drops++;
}

static void on_status(uint16_t st, void *ctx)
{
    (void)ctx;
    cl_ev_t e = { .kind = 0 };
    e.w[0] = st;
    ev_push(&e);
}

static void on_data(const uint8_t *p, size_t len, void *ctx)
{
    (void)ctx;
    if (len != GROUP * 16) return;                       /* link mode only sends whole 4-command frames */
    for (int i = 0; i < GROUP; i++) {
        cl_ev_t e = { .kind = 1 };
        bool any = false;
        for (int k = 0; k < 8; k++) {
            e.w[k] = (uint16_t)(p[i * 16 + 2 * k] | p[i * 16 + 2 * k + 1] << 8);
            any |= e.w[k] != 0;
        }
        if (any) ev_push(&e);                            /* zero = no command: cable_air ignores it anyway */
    }
}

/* ---- helpers on the brain task ---- */
static void link_cmd(uint8_t c) { gbl_send(GBL_CH_CMD, &c, 1); }

static void enter_link_mode(void)
{
    uint8_t cmd[2] = { 0x00, GBL_MODE_GBA_LINK };        /* SetMode 0x01; AwaitMode arrives through the tap */
    gbl_send(GBL_CH_CMD, cmd, sizeof cmd);
}


static void push_identity_for_join(void)
{
    /* trade_session joins only once gba_bridge has an identity (the wireless path gets it from the cart's 0x16
     * beacon, which a cable cart never sends). Cached real identity if we have one, else the Kizuna default. */
    uint16_t tid = 0x8822;
    uint8_t nm[8];
    memset(nm, 0xFF, sizeof nm);
    if (s_have_cached_id) { tid = s_cached_id.tid; memcpy(nm, s_cached_id.name, 8); cable_air_sim_seed_identity(&s_air_sim, tid, nm); }
    gba_bridge_set_gba_identity(tid, nm, 2 /* English */, 4 /* ACTIVITY_TRADE */);
    s_identity_pushed = true;
    printf("  [cable] join identity: %s (tid %04x)\n", s_have_cached_id ? "cached from this cart" : "Kizuna default", tid);
}

static bool air_alloc(void)
{
    s_free_pending = false;
    if (s_air) return true;
    s_air = heap_caps_malloc(sizeof *s_air, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!s_air) {
        ESP_LOGE(TAG, "no room for the cable translator (%u bytes, largest block %u)", (unsigned)sizeof *s_air,
                 (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
        return false;
    }
    cable_air_init(s_air);
    CABLE_SET_TRACE(s_air);
    cable_air_sim_set_air(&s_air_sim, s_air);
    s_identity_pushed = false; s_identity_real = false;
    return true;
}

static void air_free(void)
{
    if (!s_air) return;
    cable_air_sim_set_air(&s_air_sim, NULL);              /* the engine stops touching it first */
    heap_caps_free(s_air);
    s_air = NULL;
    s_free_pending = false;
}

static void handle_status(uint16_t st, int64_t now)
{
    switch (st) {
    case GBL_ST_AWAIT_MODE:
        s_reentered = true;                              /* link mode is up: nothing to re-enter */
        s_mode_lost_at_us = 0;
        link_cmd(LINKCMD_SET_MODE_MASTER);               /* the RP2040 clocks: the cart is the cable child */
        s_state = CL_AWAIT; s_hs_pending = false;
        printf("  [cable] link mode up, RP2040 is the cable master: waiting for the cart to open its link\n");
        break;
    case ST_HANDSHAKE_RECEIVED:
        if (s_closed_at_us) {
            /* A new Cable Club visit after the last one ended (bench: the old visit's translator state,
             * an exit linger among it, ended the new session). Start from a fresh translator. */
            s_closed_at_us = 0;
            if (!s_gone_sent) { s_identity_pushed = false; s_identity_real = false; }
            if (s_air) {
                cable_air_init(s_air);
                CABLE_SET_TRACE(s_air);
                cable_air_sim_set_air(&s_air_sim, s_air);
            }
        }
        s_state = CL_HANDSHAKE;
        s_hs_pending = true;                             /* memory is taken when we answer (see the poll) */
        printf("  [cable] the cart opened its link (handshake): waiting for the Switch room before answering\n");
        break;
    case ST_LINK_CONNECTED:
        if (!air_alloc()) { s_state = CL_AWAIT; break; }
        s_state = CL_CONNECTED; s_hs_pending = false;
        cable_air_reset(s_air);
        s_t0_us = now; s_generated = 0; s_last_sent_cafe_empty = false;
        s_sessions++;
        printf("  [cable] link connected (session %u): cable_air is the cart's partner\n", (unsigned)s_sessions);
        break;
    case ST_LINK_RECONNECTING:
        s_state = CL_AWAIT; s_hs_pending = false;
        printf("  [cable] the cart closed its link; waiting for it to reopen\n");
        break;
    case GBL_ST_LINK_CLOSED: {
        /* Only a real visit's end (or our own Cancel) starts the visit-end timers. A LinkClosed with no cart linking is
         * the previous mode ending because we entered link mode: AwaitMode follows by itself (re-entering on it
         * looped, bench). */
        bool visit = s_state == CL_HANDSHAKE || s_state == CL_CONNECTED || s_cancel_sent;
        if (visit && !s_cancel_sent) printf("  [cable] the cart left the room (link mode ended)\n");
        s_cancel_sent = false;
        s_state = CL_CLOSED; s_hs_pending = false;
        if (visit) { s_closed_at_us = now; s_reentered = false; s_gone_sent = false; }
        else s_mode_lost_at_us = now;               /* expect AwaitMode; enter link mode if none comes */
        break;
    }
    case GBL_ST_WRONG_CABLE:
        ESP_LOGW(TAG, "RP2040 reports the wrong cable type for GBA link mode");
        break;
    default:
        break;
    }
}

/* Replies to the cart, four commands to a data frame, keeping at most AHEAD queued in the RP2040 by wall time
 * (bench: the same pump as GB-Link's cable.js, proven on this firmware with real carts). Every frame goes,
 * idle and 0xCAFE/EMPTY included: the cart gets its partner's key report every packet; the RP2040 only fills in
 * zeros when the queue runs dry. (The earlier "drop idle groups" pump fed zeros instead of key reports.) */
static void pace_replies(int64_t now)
{
    uint32_t played = (uint32_t)((now - s_t0_us) / FRAME_US);
    if (s_generated < played) s_generated = played;          /* the RP2040 ran dry: catch up, do not burst */
    while (s_generated - played + GROUP <= AHEAD) {
        uint8_t frame[GROUP * 16];
        for (int i = 0; i < GROUP; i++) {
            uint16_t c[8] = { 0 };
            if (!cable_air_next_command(s_air, c)) memset(c, 0, sizeof c);
            for (int k = 0; k < 8; k++) { frame[i * 16 + 2 * k] = (uint8_t)c[k]; frame[i * 16 + 2 * k + 1] = (uint8_t)(c[k] >> 8); }
        }
        gbl_send(GBL_CH_DATA, frame, sizeof frame);
        s_frames_out++;
        s_generated += GROUP;
    }
}

/* ---- rfu1_bridge.h ---- */
bool rfu1_bridge_start(int tx_gpio, int rx_gpio)
{
    s_up = false;                                        /* poll stays out of the RX queue while we wait for replies */
    if (tx_gpio < 0) {
        if (!gbl_host_active() && !gbl_init_host()) return false;
    } else if (!s_uart_up) {
        if (!gbl_init(tx_gpio, rx_gpio)) return false;
        s_uart_up = true;
    }
    if (!s_evq) {
        s_evq = xQueueCreate(64, sizeof(cl_ev_t));
        id_cache_load();
#if KZ_CABLE_TRACE
        xTaskCreate(log_task, "cable_log", 3072, NULL, 1, NULL);
#endif
    }
    if (!s_engine_set) {
        ldn_brain_set_engine(cable_air_sim_bind(&s_air_sim, NULL));     /* cable_air IS the Switch-side engine */
        s_engine_set = true;
        /* Bench: after Bluetooth stops, the session heap was sometimes left with one large block only,
         * which the zstd decoder (31 KB) takes, and the sim state (25 KB) then failed ("out of memory for the sim
         * state"). Take the sim state now, before Bluetooth and Wi-Fi exist; the smaller pieces fit later. */
        if (!ldn_brain_prealloc_sim()) ESP_LOGE(TAG, "could not reserve the sim state at boot");
    }
    gbl_set_tap(NULL, NULL, NULL);
    uint8_t ver[3], landing, cable;
    if (!gbl_get_info(ver, &landing, &cable, 800) && !gbl_get_info(ver, &landing, &cable, 800)) {
        printf("  [cable] no RP2040 on the bridge UART\n");
        return false;
    }
    printf("  [cable] RP2040 GB-Link %u.%u.%u (cable %u); cart identity %s\n", ver[0], ver[1], ver[2], cable,
           s_have_cached_id ? "cached" : "not cached yet");
    xQueueReset(s_evq);
    gbl_set_tap(on_data, on_status, NULL);
    if (!gbl_set_mode(GBL_MODE_GBA_LINK, -1, GBL_ST_AWAIT_MODE, 1500)) {
        printf("  [cable] SetMode 01: no AwaitMode status\n");
        gbl_set_tap(NULL, NULL, NULL);
        return false;
    }
    s_up = true;
    s_last_activity_us = esp_timer_get_time();
    return true;
}

bool rfu1_bridge_up(void) { return s_up; }

void rfu1_bridge_stop(void)
{
    if (!s_up) return;
    gbl_set_tap(NULL, NULL, NULL);
    gbl_cancel();
    s_up = false;
    s_state = CL_OFF;
    s_hs_pending = false;
}

void rfu1_bridge_poll(void)
{
    if (!s_up) return;
    int64_t now = esp_timer_get_time();
    gbl_frame_t f;
    while (gbl_recv(&f, 0)) { }                          /* statuses are tapped AND queued: keep that queue empty */

    cl_ev_t e;
    while (xQueueReceive(s_evq, &e, 0) == pdTRUE) {
        s_last_activity_us = now;
        if (e.kind == 0) {
            handle_status(e.w[0], now);
            continue;
        }
        s_cmds_in++;
        if (s_state == CL_CONNECTED && s_air) cable_air_game_command(s_air, e.w);
    }

    /* The Switch session ended while the cart is linking or linked (bench: the Switch closed its room, the
     * RP2040 kept answering the cart's handshake, and the next Cable Club visit went past "Please wait" with no Switch
     * at all). Drop the cable link: the cart shows its link error and leaves; LinkClosed then frees the translator and
     * puts the RP2040 back to waiting with the handshake off. The same for a link left open with no frame for
     * OPEN_ACTIVE_US (a cart switched off mid-link). */
    bool cart_open = s_state == CL_HANDSHAKE || s_state == CL_CONNECTED;
    bool brain_active = ldn_brain_active();
    if (s_brain_was_active && !brain_active && cart_open) s_session_end_us = now;
    if (brain_active || !cart_open) s_session_end_us = 0;
    s_brain_was_active = brain_active;
    /* bench: the Switch ends its session the moment it has closed the room on an exit, while the cart still
     * waits for our close (cable_air answers its 5FFF). Dropping the link then crashed the GBA. Give the cart's own exit
     * SESSION_END_GRACE_US; only a cart still linked after that is cut off. */
    bool session_ended = s_session_end_us && now - s_session_end_us > SESSION_END_GRACE_US;
    bool stale = cart_open && now - s_last_activity_us > OPEN_ACTIVE_US;
    if (cart_open && !s_cancel_sent && (session_ended || stale)) {
        s_session_end_us = 0;
        printf("  [cable] %s: dropping the cable link so the cart does not carry on alone\n",
               session_ended ? "the Switch session ended" : "the cart has been silent too long");
        s_cancel_sent = true;
        s_cancel_at_us = now;
        s_hs_pending = false;
        gbl_cancel();                                    /* the RP2040 ends link mode and reports LinkClosed */
        cart_open = false;
    }

    if (s_cancel_sent && now - s_cancel_at_us > 3000000) {   /* no LinkClosed after our Cancel: treat it as closed */
        ESP_LOGW(TAG, "no LinkClosed after Cancel: closing locally");
        handle_status(GBL_ST_LINK_CLOSED, now);
    }

    /* The cart is in the Cable Club: ask for the session radio (Bluetooth off, Wi-Fi on, scan for the Switch room).
     * trade_session ignores repeats; without keys it does nothing and the cart keeps waiting at "Please wait". */
    if (cart_open && !s_cancel_sent && now - s_last_browse_us > BROWSE_EVERY_US) {
        s_last_browse_us = now; rfu1_bridge_on_gba_browsing();
    }

    /* Answer the cart's handshake only once the Switch's room is joinable, and fire the Switch join at the same
     * moment (the C6's "join at handshake": the player gets the whole Cable Club dialogue to accept on the Switch). */
    if (s_hs_pending && rfu1_bridge_room_visible()) {
        /* Memory order matters (bench: "[brain] out of memory for the sim state" with 63 KB free but a
         * 23 KB largest block): the translator, taken at the handshake while Bluetooth was still up, had split the
         * big block. Take the sim state first (the largest piece, kept for good once taken), then the translator.
         * Without both we do not answer and the cart keeps waiting at "Please wait". */
        if (!ldn_brain_prealloc() || !air_alloc()) {
            static int64_t last_warn;
            if (now - last_warn > 5000000) {
                last_warn = now;
                ESP_LOGE(TAG, "no room for the trade (largest block %u): not answering the cart",
                         (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
            }
            return;
        }
        s_hs_pending = false;
        link_cmd(LINKCMD_START_HANDSHAKE);
        s_connect_at_us = now + CONNECT_DELAY_US;
        if (!s_identity_pushed) push_identity_for_join();
        rfu1_bridge_on_gba_selected();
        printf("  [cable] Switch room joinable: answering the cart's handshake and joining the room\n");
    }

    if (s_connect_at_us && now >= s_connect_at_us) {   /* master: start the link (the RP2040 sends 0x8FFF) */
        s_connect_at_us = 0;
        if (s_state == CL_HANDSHAKE) link_cmd(LINKCMD_CONNECT_LINK);
    }

    if (s_state == CL_CONNECTED && s_air) {
        pace_replies(now);
        cable_air_frame(s_air);

        /* The cart's real identity arrives in its LinkPlayer: remember it, and cache it once the cart has left (a flash
         * write stalls the whole chip for milliseconds, not something to do mid-trade). */
        if (!s_identity_real) {
            uint16_t tid; uint8_t nm[8];
            if (cable_air_get_identity(s_air, &tid, nm)) {
                s_identity_real = true;
                s_seen_id.tid = tid; memcpy(s_seen_id.name, nm, 8); s_save_pending = true;
                if (!s_identity_pushed) { gba_bridge_set_gba_identity(tid, nm, 2, 4); s_identity_pushed = true; }
            }
        }
    }

    /* Hold the cart's first standby confirmation until the Switch has accepted the join (cable_air's gate). */
    if (ldn_brain_active() && s_air) {
        bool joined = ldn_brain_join_accepted();
        cable_air_set_switch_ready(s_air, joined);
        if (!joined && !s_join_wait_logged && cable_air_join_pending(s_air)) {
            s_join_wait_logged = true; s_join_done_logged = false;
            printf("  [cable] cart at its first confirmation: holding it until the Switch accepts the join\n");
        }
        if (joined && !s_join_done_logged) {
            s_join_done_logged = true; s_join_wait_logged = false;
            printf("  [cable] Switch accepted the join: confirmation released\n");
        }
    }

    /* Timed from the cart's LinkClosed, whatever state re-entering link mode moves us to (bench: keyed on
     * CL_CLOSED, which re-entry left after 0.5 s, "gone" never ran: the translator was never freed, 2 KB idle heap,
     * zstd could not reserve and the next join never got through). */
    if (s_mode_lost_at_us && now - s_mode_lost_at_us > 2000000) { s_mode_lost_at_us = 0; enter_link_mode(); }
    if (s_restart_at_us && now >= s_restart_at_us) {
        bool busy = ldn_brain_active() || s_state == CL_HANDSHAKE || s_state == CL_CONNECTED;
        if (busy) s_restart_at_us = 0;               /* a new visit began in the meantime: it restarts when it ends */
        else {
            printf("  [cable] visit over: restarting for a clean heap (free=%u largest=%u)\n",
                   (unsigned)heap_caps_get_free_size(MALLOC_CAP_8BIT),
                   (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
            fflush(stdout);
            /* LED: cyan fast blink for ~0.6 s = "visit over, restarting" (no Kizuna state is a solid colour: stock GB-Link
             * uses only solid ones). Written every 100 ms, so the LED task (lower priority) cannot paint over it. */
            for (int i = 0; i < 6; i++) { gbl_set_led(0, (i & 1) ? 0 : 32, (i & 1) ? 0 : 32, true); vTaskDelay(pdMS_TO_TICKS(100)); }
            esp_restart();
        }
    }
    if (s_closed_at_us && s_state != CL_HANDSHAKE && s_state != CL_CONNECTED) {
        if (!s_reentered && now - s_closed_at_us > REENTER_US) { s_reentered = true; enter_link_mode(); }
        if (!s_gone_sent && now - s_closed_at_us > GONE_LINGER_US) {
            s_closed_at_us = 0;
            s_gone_sent = true;
            s_identity_pushed = false; s_identity_real = false;
            s_free_pending = true;                       /* the next visit gets a fresh translator and join gate */
            if (s_save_pending) { s_save_pending = false; id_cache_save(s_seen_id.tid, s_seen_id.name); }
            rfu1_bridge_on_gba_gone();
            s_restart_at_us = now + VISIT_RESTART_US;    /* the visit is over: start the next one from a clean heap */
        }
    }
    /* Free the translator once the cart has gone and the trade (which ticks it as the engine) has stopped. */
    if (s_free_pending && s_state != CL_HANDSHAKE && s_state != CL_CONNECTED && !ldn_brain_active()) {
        air_free();
        ldn_brain_release();                             /* the sim's smaller buffers back to Bluetooth */
    }
}

int rfu1_bridge_login_state(void)
{
    if (!s_up) return 0;
    if (s_state == CL_HANDSHAKE || s_state == CL_CONNECTED) return 2;   /* "past login": the cart is linking */
    return s_closed_at_us && !s_gone_sent ? 1 : 0;
}

int64_t rfu1_bridge_gba_quiet_us(void)
{
    int64_t quiet = esp_timer_get_time() - s_last_activity_us;
    /* An open link is activity even when no frame arrives: GB-Link drops idle frames, so a cart sitting in the trade
     * room sends nothing. A cart switched off mid-link is never reported, so the cap brings the adapter back to
     * Bluetooth eventually. */
    if ((s_state == CL_HANDSHAKE || s_state == CL_CONNECTED) && quiet < OPEN_ACTIVE_US) return 0;
    return quiet;
}

void rfu1_bridge_isr_stats(void)
{
    printf("  [cable] state=%s in=%u out_frames=%u idle_groups_dropped=%u generated=%u sessions=%u ev_drops=%u\n",
           CL_NAMES[s_state], (unsigned)s_cmds_in, (unsigned)s_frames_out, (unsigned)s_groups_dropped,
           (unsigned)s_generated, (unsigned)s_sessions, (unsigned)s_ev_drops);
}

void rfu1_bridge_status(char *out, size_t cap)
{
    snprintf(out, cap, "\"cable\":{\"up\":%d,\"link\":\"%s\",\"in\":%u,\"out\":%u,\"sessions\":%u,\"lp\":%d,\"cached_id\":%d}",
             s_up, CL_NAMES[s_state], (unsigned)s_cmds_in, (unsigned)s_frames_out, (unsigned)s_sessions,
             s_identity_real, s_have_cached_id);
}
