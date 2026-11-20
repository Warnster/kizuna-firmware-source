/* trade_session (from an earlier trade_scan.c) — COEXISTENCE STRESS + TRADE-ROOM DISCOVERY (the C6 make-or-break).
 * Runs heavy Wi-Fi (continuous promiscuous channel-hop across 1/6/11 + decrypt) ALONGSIDE the PARLIO GBA
 * adapter, to prove the C6 can do heavy wireless work without disturbing the GBA link (where the S3's
 * CPU bit-bang failed). Also finds + decrypts the Switch's FRLG trade room advert so we know its channel
 * and ssid for the eventual join.
 */
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_timer.h"
#include "esp_mac.h"
#include "ldn_advert.h"
#include "esp_heap_caps.h"
#include "trade_session.h"
#include "ldn_join.h"
#include "ldn_udp.h"
#include "ldn_brain.h"
#include "gba_bridge.h"   /* join gate: don't start the sim/join until the GBA selects the Switch's room */
#include "rfu1_bridge.h"  /* M2: the GBA-side triggers arrive from the RP2040's RFU1 traffic */
#include "gblink.h"
#include "esp_system.h"
#ifdef KZ_GBA_LEAD
#include "gba_lead.h"     /* the GBA leads, the Switch joins */
#endif

/* Per-frame and heartbeat bench lines. Builds that define KZ_QUIET_TRACE (switch-online, the production online image)
 * compile them out; the arguments stay referenced so nothing becomes unused. */
#ifdef KZ_QUIET_TRACE
#define KZ_TRACE(...) do { if (0) printf(__VA_ARGS__); } while (0)
#else
#define KZ_TRACE(...) printf(__VA_ARGS__)
#endif

/* Online (host link): re-joining the Switch's NEXT room without a reboot fails -- auth is accepted but no data
 * frame from the Switch ever arrives, so the board never seats and stops beaconing (seen again on
 * the two-chip board). A reboot always seats at once. The PC bridge re-sends keys/rfu_host/start
 * when it sees the boot banner, so in online mode every re-arm is a clean reboot. Local mode keeps re-arming in
 * place (bench-proven there). */
static void online_reboot_instead(const char *why)
{
    if (!gbl_host_active()) return;
    printf("  [trade] online: %s -> reboot for a clean re-join (the PC bridge re-initialises us)\n", why);
    fflush(stdout);
    vTaskDelay(pdMS_TO_TICKS(100));
    esp_restart();
}

#define FRLG_HOST_COMM_ID 0x01006fa0233f8000ULL
/* tr-1.36: max age of an idle (brain-not-started) membership seat before we re-seat on the current room, so a
 * seat can't go stale while the human navigates the GBA menus (the Switch silently recreates its room). 20s
 * keeps staleness low; a re-seat is ~2s and a select landing in it only delays the brain a couple seconds. */
#define SEAT_STALE_US 20000000
/* Kizuna: how long STAGE B waits, after the GBA selects our room, for its own trainer-card
 * NI subframes to arrive (gba_bridge.c's ni_scan_child) before starting the sim anyway with the "EMU"
 * default. Bench-measured gap between 0x1f select and the child's name-send completing was ~1s; 1.5s
 * leaves margin without holding up the join noticeably (sim.c's ensure_ni() bakes identity on its first
 * tick and never rebuilds, so this is a one-shot window, not a retry budget). */
/* 5 s. Online (GBA's RP2040 across the relay, PC host link) the child's name-send landed 2.5 s after
 * select (bench: CONNECT 04:56:10.956 -> decoded identity 04:56:13.462), so 1.5 s timed out and the Switch saw
 * "EMU". The GBA holds at "please wait" for 20+ s without harm (see STAGE B), so the longer window costs nothing
 * when the identity is late and nothing at all when it arrives early (we start the moment it is decoded). */
#define GBA_IDENTITY_WAIT_US 5000000
static const uint8_t LDN_PREFIX[5] = { 0x7f, 0x00, 0x22, 0xaa, 0x04 };

static volatile uint32_t s_ldn_seen, s_frlg_seen;
static volatile uint32_t s_beacon_seen, s_beacon_from_host, s_probresp_from_host;
static volatile bool s_beacon_dumped;
static int8_t s_room_channel = -1;
static int64_t s_join_start_us;      /* when ldn_join_run was last called (join self-heal watchdog) */
/* hw (first trade): after the GBA left the room we re-associated to the Switch's
 * still-advertised room within ~1 s (the GBA was browsing again) and the Switch host errored and destroyed the
 * network. Hold the pre-connect association for a cooldown after any session end so the host finishes its own
 * leave handling first; the GBA can browse meanwhile (the room list stays live). */
#define LEAVE_COOLDOWN_US 15000000LL
static int64_t s_no_assoc_before_us;   /* pre-connect association deferred until this time (0 = no hold) */
static uint8_t s_room_ssid[16];
static volatile int64_t s_last_room_us;
static char s_room_name[9];          /* the Switch host's name (ASCII, from decrypted advert offset 52) */
static uint16_t s_room_tid;          /* the Switch host's trainer id (from decrypted advert) */
static uint8_t  s_room_name_raw[8];  /* the host's IN-GAME player name, FRLG charmap, from the same record */
static bool     s_room_raw_valid;
static uint8_t  s_room_activity;     /* the Switch room's activity (4 trade, 1 single, 2 double battle); 0 unknown */
#define BATTLE_CLOSE_GRACE_US 15000000          /* see rfu1_bridge_on_gba_gone */
static volatile int64_t s_battle_gone_us;     /* when the GBA left a battle room; 0 otherwise */

/* Kizuna: the FRLG host's RfuGameData rides in the advert's Pia application data as a
 * custom base85 string (alphabet 0x23..0x78 skipping 0x5c, first char = least-significant digit, 5 chars ->
 * 4 LE bytes) after a 0x5C-byte Pia header: record[0:2] = trainerId LE, [2:10] = player name (FRLG charmap),
 * [10:12] = RFU session id, [16:18] = search word, [20:24] = game-data word. Layout and alphabet from pokeldn
 * (ldn/beacon.py mutate_beacon, ldn/transport.py _b85_decode); verified on the bench: the record decoded to
 * 0xbcda, exactly the id the Switch's own trainer-card block carried in the trade. */
static int b85_decode(const uint8_t *s, int n, uint8_t *out, int cap) {
    int o = 0;
    for (int i = 0; i + 5 <= n && o + 4 <= cap; i += 5) {
        uint32_t v = 0;
        for (int k = 4; k >= 0; k--) {
            uint8_t c = s[i + k];
            if (c < 0x23 || c > 0x78 || c == 0x5c) return o;        /* not alphabet -> stop */
            v = v * 85u + (uint32_t)(c < 0x5c ? c - 0x23 : c - 0x24);
        }
        out[o++] = (uint8_t)v; out[o++] = (uint8_t)(v >> 8); out[o++] = (uint8_t)(v >> 16); out[o++] = (uint8_t)(v >> 24);
    }
    return o;
}
static void decode_host_game_record(const uint8_t *pt, long d) {
    /* app data = u16 LE size, then the Pia header whose first two bytes are 0x005C (its own size, BE) */
    for (long i = 2; i + 0x5c + 5 <= d; i++) {
        if (pt[i] != 0x00 || pt[i + 1] != 0x5c) continue;
        /* the u16 size before the app data is BIG-endian in the live advert (00 7a = 122), like scene_id;
         * accept either byte order, whichever is in range */
        int size = (pt[i - 2] << 8) | pt[i - 1];
        if (size < 0x5c + 5 || i + size > d) size = pt[i - 2] | (pt[i - 1] << 8);
        if (size < 0x5c + 5 || i + size > d) continue;
        uint8_t rec[24] = {0};
        int n = b85_decode(pt + i + 0x5c, size - 0x5c, rec, sizeof rec);
        if (n < 12) continue;
        s_room_tid = (uint16_t)(rec[0] | (rec[1] << 8));
        memcpy(s_room_name_raw, rec + 2, 8); s_room_raw_valid = true;
        /* word 16: activity in bits 0-6. Bit 7 is NOT "can link nationally" (that is rec[18] bits 0-1): GB-Link
         * 47e4de1 fixed exactly this misreading. Only the three Direct Corner rooms count. */
        uint8_t act = n >= 18 ? (uint8_t)(rec[16] & 0x7f) : 0;
        s_room_activity = (act == 4 || act == 1 || act == 2) ? act : 0;
        ldn_brain_set_room_activity(s_room_activity);
        printf("  [trade] host game record: tid=0x%04x (%u) name_raw=", s_room_tid, s_room_tid);
        for (int k = 0; k < 8; k++) printf("%02x", s_room_name_raw[k]);
        printf(" rfu_session=0x%04x search=0x%04x activity=%u (%s)\n", rec[10] | (rec[11] << 8), rec[16] | (rec[17] << 8),
               s_room_activity, s_room_activity == 4 ? "trade" : s_room_activity == 1 ? "single battle" : s_room_activity == 2 ? "double battle" : "unknown");
        return;
    }
    printf("  [trade] host game record NOT found in advert (tid stays 0)\n");
}

/* ---- LDN JOIN (tr-0.11): captured join params from the live decrypted advert + trigger state ---- */
static ldn_join_params_t s_jp;       /* filled from the decrypted advert (server_random, bssid, ...) */
static volatile bool s_jp_ready;     /* true once s_jp holds a valid decrypted FRLG room */
static void (*s_radio_cb)(bool session);   /* Wi-Fi/Bluetooth hand-over requests (mode-owned) */
static volatile bool s_scan_parked;         /* the scan task has torn down and is waiting for s_armed again */
/* M1: no GBA yet. The console arms discovery ("start" = the GBA is browsing), selects the room
 * ("select" = the GBA's 0x1f connect) and stops a session. Keys live in RAM, set by the console/app. */
static const ldn_keys  *s_keys;
static volatile bool    s_armed, s_browsing, s_stop_req;
static volatile bool    s_st_joined, s_st_brain, s_st_member;   /* snapshot of scan_task's state for status */
static void log_heap(const char *stage)
{
    printf("  [heap] %s: free=%u largest=%u min=%u\n", stage,
           (unsigned)heap_caps_get_free_size(MALLOC_CAP_8BIT),
           (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT),
           (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT));
}

/* ---- MEMBERSHIP (tr-brain): keep the latest decrypted host advert so scan_task can find our
 * assigned IP once the host re-advertises us as a participant (V2 list, port of S3 auto_get_ips). ---- */
static uint8_t          s_adv_pt[512];
static volatile int     s_adv_ptl;   /* length of s_adv_pt, or 0 if none captured yet */

static inline uint16_t le16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static inline uint64_t be64(const uint8_t *p) {
    uint64_t v = 0; for (int i = 0; i < 8; i++) v = (v << 8) | p[i]; return v;
}
static inline uint16_t be16(const uint8_t *p) { return (uint16_t)((p[0] << 8) | p[1]); }

/* Expose the live Switch identity so the adapter can present it to the GBA as the trade leader (no stub).
 * Returns true if we have a decrypted name. */
bool trade_room_identity_raw(uint8_t name_raw[8])
{
    if (!s_room_raw_valid) return false;
    memcpy(name_raw, s_room_name_raw, 8);
    return true;
}
bool trade_room_identity(char name_out[9], uint16_t *tid_out)
{
    if (!s_room_name[0]) return false;
    memcpy(name_out, s_room_name, 9);
    if (tid_out) *tid_out = s_room_tid;
    return true;
}

/* True while the Switch's FRLG trade room is actually being heard (within ~3 s). Gates the parent
 * candidate we present to the GBA so its trade "join" leader list only shows the Switch when it's here. */
bool trade_room_present(void)
{
    /* tr-0.23: latch on s_jp_ready — once we've decrypted the room (and are associating/associated to it),
     * keep presenting the leader to the GBA. The old 3s live-advert window went false once we associated
     * (we stop hearing the room's adverts on the locked channel), which made the leader vanish from the
     * GBA's join screen. Once we know the room, the leader should stay shown so the GBA can join. */
    if (s_jp_ready) return true;
    int64_t t = s_last_room_us;
    return t != 0 && (esp_timer_get_time() - t) < 3000000;
}

/* dedup: only log a distinct room advert occasionally */
static uint8_t s_last_ssid[16];

static void promisc_rx(void *buf, wifi_promiscuous_pkt_type_t type)
{
    if (!buf || type != WIFI_PKT_MGMT || !s_keys) return;   /* no keys -> nothing to decrypt */
    const wifi_promiscuous_pkt_t *p = buf;
    const uint8_t *f = p->payload;
    size_t len = p->rx_ctrl.sig_len;
    /* Feed EVERY mgmt frame to the join watcher first (it needs the assoc-response, subtype 0x10/0x30,
     * which the action-frame filter below would otherwise drop). */
    ldn_join_sniff(f, (uint32_t)len, p->rx_ctrl.channel);
    /* DIAGNOSTIC (tr-0.11e): does the Switch BEACON (subtype 0x80) or only send action-frame adverts?
     * esp_wifi_connect needs a beacon/probe-resp to find the AP; NO_AP_FOUND means the scan saw none. */
    if (len >= 16) {
        uint8_t st = f[0] & 0xf0U;               /* subtype nibble: 0x80=beacon, 0x50=probe-resp, 0xd0=action */
        if (st == 0x80) {
            s_beacon_seen++;
            if (s_jp_ready && memcmp(f + 10, s_jp.bssid, 6) == 0) {
                s_beacon_from_host++;
                if (!s_beacon_dumped) {   /* one-shot: dump the host beacon so we can see ssid/caps/RSN IE */
                    s_beacon_dumped = true;
                    printf("\n  [beacon] host beacon %u B:", (unsigned)len);
                    for (size_t i = 0; i < len && i < 180; i++) { if ((i & 31) == 0) printf("\n    %3u: ", (unsigned)i); printf("%02x", f[i]); }
                    printf("\n");
                }
            }
        }
        else if (st == 0x50) { if (s_jp_ready && memcmp(f + 10, s_jp.bssid, 6) == 0) s_probresp_from_host++; }
    }
    if (len < 29 || (f[0] & 0xfcU) != 0xd0U) return;
    const uint8_t *body = f + 24;
    size_t blen = (len > 28) ? len - 28 : 0;
    if (blen < sizeof LDN_PREFIX || memcmp(body, LDN_PREFIX, sizeof LDN_PREFIX) != 0) return;
    s_ldn_seen++;
    if (ldn_advert_local_comm_id(body, blen) != FRLG_HOST_COMM_ID) return;   /* not a FRLG room */
    s_frlg_seen++;
    s_room_channel = p->rx_ctrl.channel;
    s_last_room_us = esp_timer_get_time();
    uint8_t ssid[16];
    if (ldn_advert_ssid(body, blen, ssid) == 0) {
        memcpy(s_room_ssid, ssid, 16);
        /* MEMBERSHIP: once we're joining THIS host, keep the latest decrypted advert so scan_task can
         * find our assigned IP. The ssid dedup below stops re-decrypting after we join (the ssid no
         * longer changes), but the participant list — where OUR IP appears — does, so decrypt here on
         * every host advert. Cheap (~10/s). */
        if (s_jp_ready && memcmp(f + 10, s_jp.bssid, 6) == 0) {
            uint8_t pt[512];
            long d = ldn_advert_decrypt(s_keys, body, blen, pt);
            if (d >= 40 && d <= (long)sizeof(s_adv_pt)) {
                memcpy(s_adv_pt, pt, d);
                s_adv_ptl = (int)d;   /* published last; scan_task snapshots length then bytes */
            }
        }
        if (memcmp(ssid, s_last_ssid, 16) != 0) {           /* new/changed room -> log it (+ decrypt) */
            memcpy(s_last_ssid, ssid, 16);
            uint8_t pt[256];
            long d = ldn_advert_decrypt(s_keys, body, blen, pt);
            printf("  [trade] FRLG ROOM on ch%d  mac %02x:%02x:%02x:%02x:%02x:%02x  ssid ",
                   s_room_channel, f[10],f[11],f[12],f[13],f[14],f[15]);
            for (int i = 0; i < 16; i++) printf("%02x", ssid[i]);
            if (d > 60) {
                /* host name @52 (ASCII) -> store for presenting to the GBA as the trade leader */
                memset(s_room_name, 0, sizeof s_room_name);
                for (int i = 0; i < 8 && pt[52 + i] >= 32 && pt[52 + i] < 127; i++) s_room_name[i] = pt[52 + i];
                printf("  host='%s' (decrypt %ldB)", s_room_name, d);
                decode_host_game_record(pt, d);   /* real trainer id + in-game name for the GBA */

                /* tr-0.11: capture the LDN join params from this live decrypted advert. NetworkId is
                 * body[12:44] = { comm_id LE @0, pad2, scene_id LE @10, pad4, ssid @16 }; plaintext gives
                 * server_random @0, challenge (BE64) @16, app_version (BE16) @26 (host/core/Ldn.cs). */
                memcpy(s_jp.bssid, f + 10, 6);                 /* Switch host MAC */
                memcpy(s_jp.ssid, ssid, 16);
                s_jp.channel        = (uint8_t)s_room_channel;
                s_jp.comm_id        = ldn_advert_local_comm_id(body, blen);
                s_jp.scene_id       = be16(body + 12 + 10);   /* scene_id is BIG-endian in the advert (raw 57 0f = 0x570f); host reads B16 then writes LE into the auth NetworkId. Reading LE here byte-swapped it, corrupting the GCM AAD -> Switch silently dropped the auth request. */
                memcpy(s_jp.server_random, pt + 0, 16);
                s_jp.challenge_token= be64(pt + 16);
                s_jp.app_version    = be16(pt + 26);
                s_jp_ready = true;
                printf("  [join] params captured: ch%d bssid %02x:%02x:%02x:%02x:%02x:%02x scene=%u appver=%u"
                       " chal=%016llx srand=%02x%02x%02x%02x",
                       s_jp.channel, f[10],f[11],f[12],f[13],f[14],f[15], s_jp.scene_id, s_jp.app_version,
                       (unsigned long long)s_jp.challenge_token,
                       s_jp.server_random[0], s_jp.server_random[1], s_jp.server_random[2], s_jp.server_random[3]);
                /* DIAGNOSTIC: dump the full decrypted advert so we can locate the trainer-id field */
                printf("\n  [trade] plaintext:");
                for (long i = 0; i < d; i++) { if ((i & 31) == 0) printf("\n    %3ld: ", i); printf("%02x", pt[i]); }
            } else printf("  (decrypt %ld)", d);
            printf("\n");
        }
    }
}

/* Parse the V2 participant list of the latest host advert for our assigned IP + the host's IP/MAC.
 * Port of the S3's auto_get_ips (ldn_probe.c): 48-byte entries at offset 40, count at offset 39;
 * entry = ip(4) mac(6) index(1) plat(1) name(32) pad(4). index==0 => host. 0 when both resolved. */
static int membership_ips(const uint8_t our_mac[6], uint8_t our_ip[4], uint8_t host_ip[4],
                          uint8_t host_mac[6])
{
    int ptl = s_adv_ptl;                 /* snapshot length first (published last by promisc_rx) */
    if (ptl < 40) return -1;
    static uint8_t pt[512];
    memcpy(pt, s_adv_pt, ptl);           /* minor tearing risk vs the wifi-task writer; benign — a
                                          * torn snapshot just fails this parse and we retry next tick */
    int num = pt[39];
    int found_us = 0, found_host = 0;
    static int64_t last_dump = 0;
    bool dump = (esp_timer_get_time() - last_dump) > 2000000;   /* rate-limited participant dump */
    if (dump) { last_dump = esp_timer_get_time();
        printf("  [member?] participants=%d ptl=%d ourMAC=%02x:%02x:%02x:%02x:%02x:%02x\n", num, ptl,
               our_mac[0],our_mac[1],our_mac[2],our_mac[3],our_mac[4],our_mac[5]); }
    for (int i = 0; i < num; ++i) {
        size_t off = 40 + 48 * (size_t)i;
        if (off + 48 > (size_t)ptl) break;
        const uint8_t *ip = pt + off, *mac = pt + off + 4;
        uint8_t index = pt[off + 10];
        int zero_ip = !(ip[0] | ip[1] | ip[2] | ip[3]);
        if (dump) printf("    [p%d] idx=%u ip=%u.%u.%u.%u mac=%02x:%02x:%02x:%02x:%02x:%02x\n",
                         i, index, ip[0],ip[1],ip[2],ip[3], mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
        if (index == 0 && !zero_ip) { memcpy(host_ip, ip, 4); memcpy(host_mac, mac, 6); found_host = 1; }
        if (memcmp(mac, our_mac, 6) == 0 && !zero_ip) { memcpy(our_ip, ip, 4); found_us = 1; }
    }
    return (found_us && found_host) ? 0 : -1;
}

/* Single-core C6: drive the trade brain from a dedicated task at ~VBlank cadence. FreeRTOS runs at
 * 100 Hz here (10 ms tick), so we wake each tick and gate the actual sim advance on esp_timer at
 * 16.6 ms — that yields ~50 Hz (fires every other wake). Slightly slower than the S3's true 60 Hz,
 * which only makes the sim's fixed-VBlank timers marginally conservative; fine for the handshake. */
/* (brain_task removed — the SINGLE LOOP ticks the sim inline from the GBA command phase, gba_login_main.c.
 * No separate task, no priorities: ldn_udp_poll()+ldn_brain_tick() run in the GBA loop's idle gaps.) */

static void scan_task(void *arg)
{
    (void)arg;
    const uint8_t chans[3] = { 1, 6, 11 };
    int ci = 0; int64_t last_hb = 0;
    bool joined = false;
    bool brain_started = false;
    int64_t s_gba_identity_wait_start_us = 0;   /* 0 = not yet waiting; see STAGE B below */
    bool member_ready = false;                  /* seated + UDP transport up (SILENT — no join yet) */
    int64_t seat_us = 0;                         /* tr-1.36/1.63: time of the last CONFIRMED-live host traffic
                                                   * (seat-staleness clock) — see seat_last_ipf below. */
    uint32_t seat_last_ipf = 0;                  /* tr-1.63: ldn_join_ip_frames() snapshot for liveness tracking */
    uint8_t our_ip[4] = {0}, host_ip[4] = {0}, host_mac[6] = {0}, our_mac[6] = {0};

    /* DISCOVERY WINDOW (tr-0.56, user's model): discover rooms ONLY after login and before joining one.
     *  - BEFORE login: Wi-Fi FULLY QUIET (promiscuous never enabled) so the login-burst gba_parlio_flush()
     *    (PARLIO delete) can't race the Wi-Fi GDMA RX ISR -> the tr-0.53 vQueueDeleteWithCaps crash is gone
     *    and login runs in its proven Wi-Fi-idle environment. Discovery pre-login is pointless anyway.
     *  - AFTER login, BROWSING: promiscuous ON, discover continuously so the GBA's join-room list is LIVE
     *    (rooms populate/disappear as they come and go). scan_task is higher priority than the command loop,
     *    so the light per-hop set_channel is never starved.
     *  - AFTER join: the state machine locks the room + associates; discovery hopping naturally stops.
     * All in trade_scan.c — the frozen app_main login path is untouched. */
    while (!s_armed) vTaskDelay(pdMS_TO_TICKS(50));          /* console "start" (M1) / RP2040 mode 0x07 up (M2) */
    esp_wifi_set_promiscuous(true);
    printf("  [trade] armed -> discovering FRLG rooms on 1/6/11 until one is joined\n");
    log_heap("armed");

    for (;;) {
        static int64_t dwell_start = 0;
        if (!s_armed) {
            /* disarmed (the GBA went away, or the console said so). Tear down exactly like the session
             * re-arm branch below, switch the sniffer off so the mode may stop Wi-Fi, and park until armed again. */
            if (ldn_brain_active()) ldn_brain_stop();
            ldn_join_set_ip_forward(false);
            ldn_udp_stop();
            ldn_join_reset();
            gba_bridge_clear_join_selected();
            s_jp_ready = false;
            memset(s_last_ssid, 0, sizeof s_last_ssid);
            memset(&s_jp, 0, sizeof s_jp);
            joined = false; dwell_start = 0;
            brain_started = false; member_ready = false; s_gba_identity_wait_start_us = 0; s_browsing = false;
            esp_wifi_set_promiscuous(false);
            s_scan_parked = true;
            printf("  [trade] disarmed -> idle (scan parked)\n");
            while (!s_armed) vTaskDelay(pdMS_TO_TICKS(50));
            s_scan_parked = false;
            esp_wifi_set_promiscuous(true);
            printf("  [trade] armed -> discovering FRLG rooms on 1/6/11 until one is joined\n");
            log_heap("armed");
            continue;
        }
#ifdef KZ_GBA_LEAD
        /* the GBA LEADS a Trade Center / Colosseum group (it hosts, so the RP2040 relays its BROADCASTs; a GBA
         * joining only scans and never broadcasts). With no Switch session running, stop the join role and host a room
         * that mirrors the GBA's group for the Switch to join. The lead session runs on the brain task and restarts the
         * board when it is over; this task just parks. The Switch-leads path never sees a GBA beacon, so it is unchanged. */
        if (gba_lead_active()) { vTaskDelay(pdMS_TO_TICKS(100)); continue; }
#ifdef KZ_ONLINE_PACING
        static bool s_lead_off;   /* online: leading failed once this boot (no RAM beside Bluetooth) -> ignore GBA beacons */
#else
        const bool s_lead_off = false;
#endif
        if (!ldn_brain_active() && s_keys && !s_lead_off) {
            uint8_t bc[24]; uint16_t devid;
            if (rfu1_bridge_gba_hosting(bc, &devid) && gba_lead_wanted(bc)) {
                printf("  [trade] the GBA is LEADING a group (activity %u, devid %04x) -> hosting a room for the Switch (docs/101)\n",
                       (unsigned)(bc[12] & 0x7f), devid);
                esp_wifi_set_promiscuous(false);
                ldn_join_set_ip_forward(false);
                ldn_udp_stop();
                ldn_join_reset();
                ldn_join_set_passthrough(false);
                gba_bridge_clear_join_selected();
                s_jp_ready = false;
                memset(s_last_ssid, 0, sizeof s_last_ssid);
                memset(&s_jp, 0, sizeof s_jp);
                joined = false; brain_started = false; member_ready = false; s_gba_identity_wait_start_us = 0;
                ldn_brain_free_all();                         /* the join role's sim reserve: the Pia host needs ~30 KB in one block */
                log_heap("lead");
                if (!gba_lead_start(s_keys, bc, devid)) {
#ifdef KZ_ONLINE_PACING
                    {
                        /* Online the phone re-sets this board up after every boot and the GBA keeps leading, so a restart here
                         * would loop. Go back to the Switch-leads join role for the rest of this boot instead. */
                        printf("  [trade] could not lead -> staying in the Switch-leads role (GBA beacons ignored until restart)\n");
                        s_lead_off = true;
                        ldn_brain_prealloc_sim();
                        /* a failure after the calloc (ldn_ap_start / dist_udp) left the radio in AP mode: back to the station
                         * the join role scans with (as switch_online_main's Wi-Fi bring-up) */
                        esp_wifi_stop();
                        esp_wifi_set_mode(WIFI_MODE_STA);
                        esp_wifi_start();
                        esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G);
                        esp_wifi_set_ps(WIFI_PS_NONE);
                        esp_wifi_set_promiscuous(true);
                        continue;
                    }
#endif
                    printf("  [trade] could not lead -> restarting\n");
                    fflush(stdout);
                    esp_restart();
                }
                continue;
            }
        }
#endif
        /* A session whose association died before/while it ran (the Switch recreated or closed its room, deauth reason 7/8)
         * would otherwise sit forever with the brain "active" on a dead association: the re-arm branch below only fires when
         * the brain stops, and with no traffic the data phase never idle-exits. End it after 5 s without an association. */
        {
            static int64_t assoc_lost_us;
            if (joined && ldn_brain_active() && !ldn_join_associated()) {
                int64_t now = esp_timer_get_time();
                if (!assoc_lost_us) assoc_lost_us = now;
                else if (now - assoc_lost_us > 5000000) {
                    printf("  [trade] association lost during the session -> ending it and re-arming\n");
                    ldn_brain_note_stop("assoc_lost");
                    s_stop_req = true;
                    assoc_lost_us = 0;
                }
            } else {
                assoc_lost_us = 0;
            }
        }
        if (s_battle_gone_us && (!ldn_brain_active()
                                 || esp_timer_get_time() - s_battle_gone_us > BATTLE_CLOSE_GRACE_US)) {
            if (ldn_brain_active()) {
                printf("  [trade] the Switch did not close within %d s of the GBA leaving -> ending the session\n",
                       BATTLE_CLOSE_GRACE_US / 1000000);
                s_stop_req = true;
            }
            s_battle_gone_us = 0;
        }
        if (s_stop_req) {
            /* Console "stop": end the session and fall into the tr-1.31 full re-arm branch below (it tears down
             * UDP/join/latches and returns to scanning). brain_started=true is what that branch keys on. */
            s_stop_req = false;
            if (ldn_brain_active()) { ldn_brain_note_stop("stop_req"); ldn_brain_stop(); }
            if (joined) brain_started = true;
            s_browsing = false;
        }
        if (!joined) {
            /* tr-0.13: LDN join RE-ARMED on v6.1 (login+data-phase confirmed fixed at tr-0.12). Locking the
             * room channel also gives the GBA a STABLE leader on its join screen (no flicker from hopping).
             * On v6.1 esp_wifi_connect should associate to the Switch's hidden-SSID AP (the v5.3 wall). */
            /* Associate as soon as the GBA is BROWSING (it broadcast its 0x16 identity into the room -> past
             * login, in the room list). NOT on 0x1f connect: the 0x1f data phase is hard-real-time (~60Hz 0x26
             * pulls), and the heavy association burst (esp_wifi_connect + CCMP + auth) firing there preempts the
             * GBA service and freezes it at "Le" (tr-0.51: 1 wake, 0 x26). The browse phase uses the tolerant
             * SP_POLL timing (200ms), so the burst is absorbed there; by the time the GBA connects, Wi-Fi is a
             * settled member and the inline loop can sustain the 0x26 pulls + drain Pia. Still POST-login (0x16
             * only comes after login), so login stays Wi-Fi-idle; the delete-race is handled by soft_reset at the
             * re-arm sites. The sim JOIN (the "Emu wants to join" prompt) stays gated on 0x1f in STAGE B below. */
            bool gba_browsing = s_browsing;   /* M1: the console says so; M2: the RP2040's RFU1 traffic */
            bool cooling = esp_timer_get_time() < s_no_assoc_before_us;
            if (s_jp_ready && gba_browsing && cooling) {
                static int64_t last_note_us;
                if (esp_timer_get_time() - last_note_us > 3000000) {
                    last_note_us = esp_timer_get_time();
                    printf("  [join] GBA browsing but leave-cooldown active (%lld ms left) -> not associating yet\n",
                           (long long)((s_no_assoc_before_us - esp_timer_get_time()) / 1000));
                }
                esp_wifi_set_channel(s_jp.channel, WIFI_SECOND_CHAN_NONE);
                vTaskDelay(pdMS_TO_TICKS(200));
            } else if (s_jp_ready && gba_browsing) {
                /* NO dwell: associate the INSTANT the GBA starts browsing. tr-0.52 had a 2s dwell here and the
                 * player selected the room (0x1f -> connect -> data phase) INSIDE that window, so association
                 * still started after the connect (log: "dwell done" at line 997, AFTER the wake at 996) and
                 * starved the data phase again. Start now so it completes during the seconds of room-browsing,
                 * well before the connect. Channel is already locked by the else-branch below on prior passes. */
                (void)dwell_start;
                esp_wifi_set_channel(s_jp.channel, WIFI_SECOND_CHAN_NONE);
                printf("  [join] GBA browsing (identity seen) -> associating NOW (pre-connect, host_beacon=%u)\n",
                       (unsigned)s_beacon_from_host);
                ldn_join_run(&s_jp);
                log_heap("associating");
                joined = true;
                s_join_start_us = esp_timer_get_time();
            } else if (s_jp_ready) {
                /* Room found + presented to the GBA (0x1d from the sniffed advert), but the player hasn't
                 * selected it yet: lock the channel so the GBA sees a stable room, and DON'T associate —
                 * keep Wi-Fi idle so the GBA can log in / browse without the PARLIO-flush-vs-Wi-Fi crash. */
                esp_wifi_set_channel(s_jp.channel, WIFI_SECOND_CHAN_NONE);
                vTaskDelay(pdMS_TO_TICKS(200));
            } else {
                esp_wifi_set_channel(chans[ci], WIFI_SECOND_CHAN_NONE);   /* keep hopping until room found */
                ci = (ci + 1) % 3;
                vTaskDelay(pdMS_TO_TICKS(400));
            }
        } else if (brain_started && ldn_brain_active() && !ldn_join_member()) {
            /* Room gone under a running brain (hw the Switch went silent 8 s into the join,
             * then NETWORK_DESTROYED; the brain sat in FINALIZE for minutes and the GBA waited forever). Stop it
             * so the re-arm branch below runs on the next pass and scans for the Switch's new room. */
            printf("  [trade] room lost under the running brain (member=0) -> stop + re-arm\n");
            ldn_brain_stop();
        } else if (brain_started && !ldn_brain_active()) {
            /* tr-1.31 SESSION RE-ARM: the brain was started and has since STOPPED (the GBA died / restarted
             * login, or the data phase idle-exited -> ldn_brain_stop). Previously brain_started/member_ready/
             * joined were set-once latches, so a second attempt in the same boot was IMPOSSIBLE — and after
             * the Switch's post-death network teardown (STA_DISCONNECTED) the stale member state hid even the
             * self-heal path. The only recovery was a reflash/reboot . Re-arm
             * EVERYTHING back to the scanning state: the Switch recreates its room with a rotated per-session
             * challenge, so a fresh advert re-capture is REQUIRED anyway (a stale challenge gets the short
             * non-seating auth reply — the documented stale-room failure). GBA-independent: no PARLIO/login
             * touch; the GBA simply re-browses and re-selects when the player is ready. */
            printf("  [trade] session ended (brain stopped) -> full re-arm for a fresh room/attempt (assoc held %d s)\n",
                   (int)(LEAVE_COOLDOWN_US / 1000000));
            s_no_assoc_before_us = esp_timer_get_time() + LEAVE_COOLDOWN_US;
            online_reboot_instead("session ended");   /* online: the reboot replaces the in-place re-arm */
            ldn_join_set_ip_forward(false);
            ldn_udp_stop();
            ldn_join_reset();
            gba_bridge_clear_join_selected();             /* next join waits for a REAL new 0x1f (no premature prompt) */
            s_jp_ready = false;
            memset(s_last_ssid, 0, sizeof s_last_ssid);   /* force a fresh param re-capture on next sighting */
            memset(&s_jp, 0, sizeof s_jp);
            esp_wifi_set_promiscuous(true);               /* scan needs promisc ON (join_run turned it off) */
            joined = false; dwell_start = 0;
            brain_started = false; member_ready = false; s_gba_identity_wait_start_us = 0;
        } else if (member_ready && !brain_started &&
                   esp_timer_get_time() - seat_us > SEAT_STALE_US) {
            /* tr-1.36 STALE-SEAT RE-SEAT : a seat goes DEAD if the human takes minutes
             * between the re-arm and selecting the room. The Switch recreates its room (same host MAC, NEW
             * ssid + NEW subnet — hw 06:35 seat=169.254.102.x, 06:40 room=169.254.96.x) WITHOUT deauthing us
             * (no STA_DISCONNECTED), so our association silently points at a dead room. member_ready stays
             * latched, the !member self-heal below can't fire, and when the GBA finally selects, the brain
             * talks to a dead peer (hw: rx frozen at 2, acc=0, our_ni_sent=0 -> the Switch never sees the join
             * -> GBA errors). A FRESH FLASH hides it because it scans right when the player is ready. Fix:
             * bound seat age — if we've been seated + idle (brain not started) longer than SEAT_STALE_US,
             * tear down and re-seat on the CURRENT room in the background, so the seat is always fresh at
             * select time. A select landing mid-re-seat just delays the brain a couple seconds (build_recv
             * count=0-holds the GBA), it does not fail. GBA-independent (no PARLIO/login touch).
             * tr-1.63 FIX: this fired on a BLIND wall clock regardless of whether the room
             * was actually still alive, so simply sitting in the GBA's room-view menu for >20s caused a
             * disruptive re-seat (association drop + rejoin) every ~20s forever, even against a perfectly
             * live room — a real, always-present bug the user had been ignoring as non-priority. `seat_us`
             * is now advanced on every CONFIRMED live host packet (seat_last_ipf tracking below), not just
             * set once at seat time — so this branch only fires once host traffic has genuinely gone quiet
             * for SEAT_STALE_US, which is what "dead room" actually looks like (the live Pia broadcast the
             * Switch sends every ~0.5-1s stops arriving). A live room now never re-seats while just browsed. */
            printf("  [trade] seat idle-stale (%llds since last host packet) -> re-seat on the current room\n",
                   (long long)((esp_timer_get_time() - seat_us) / 1000000));
            online_reboot_instead("seat idle-stale");
            ldn_join_set_ip_forward(false);
            ldn_udp_stop();
            ldn_join_reset();
            gba_bridge_clear_join_selected();
            s_jp_ready = false;
            memset(s_last_ssid, 0, sizeof s_last_ssid);
            memset(&s_jp, 0, sizeof s_jp);
            esp_wifi_set_promiscuous(true);
            joined = false; dwell_start = 0;
            member_ready = false; s_gba_identity_wait_start_us = 0;
        } else if (!brain_started && !ldn_join_member() &&
                   (ldn_join_auth_exhausted() ||
                    esp_timer_get_time() - s_join_start_us > 20000000)) {
            /* SELF-HEAL: the latched room is stale (recreated: same ssid, rotated per-session challenge ->
             * the Switch's short non-seating auth reply) or GONE (esp_wifi_connect "not suitable AP"). Once
             * s_jp_ready latched, we'd otherwise retry the dead room forever and only a reboot recovers.
             * Tear down + drop the latch so promisc re-captures the CURRENT room's fresh params, then re-scan.
             * GBA-INDEPENDENT (no PARLIO/login touch). */
            printf("  [trade] join stalled (member=0, auth_exhausted=%d) -> re-arming scan for a fresh room\n",
                   ldn_join_auth_exhausted());
            online_reboot_instead("join stalled");
            ldn_join_reset();
            s_jp_ready = false;
            memset(s_last_ssid, 0, sizeof s_last_ssid);   /* force a fresh param re-capture on next sighting */
            memset(&s_jp, 0, sizeof s_jp);
            esp_wifi_set_promiscuous(true);               /* scan needs promisc ON (join_run turned it off) */
            joined = false;
            dwell_start = 0; s_gba_identity_wait_start_us = 0;
        } else {
            ldn_join_poll();                                   /* install key on assoc-resp, then send auth */
            /* tr-1.63: while seated and idle, advance the staleness clock on every CONFIRMED-live host packet
             * (the Switch's Pia broadcast, ~0.5-1s cadence) so SEAT_STALE_US only expires against genuine
             * silence, never against a human just sitting in the room-view menu. See the branch above. */
            if (member_ready && !brain_started) {
                uint32_t ipf = ldn_join_ip_frames();
                if (ipf != seat_last_ipf) { seat_last_ipf = ipf; seat_us = esp_timer_get_time(); }
            }
            /* MEMBERSHIP -> BRAIN: once associated + authorized, watch the host's re-broadcast advert
             * for OUR assigned IP. When it appears we're an IP member: bring up the static-IP UDP
             * transport and start the trade brain (GBA stubbed). Then tick it at ~60 Hz from brain_task. */
            /* STAGE A — become a MEMBER silently as soon as we're associated: resolve our IP + bring up the
             * UDP transport. This is the network-level seat only; it sends NO join to the Switch, so no
             * "Emu wants to join" prompt yet. Keeping this ungated (not waiting for 0x1f) also means we're a
             * real member, so the self-heal watchdog (which needs !member) won't tear us down while the human
             * navigates the GBA menus. */
            if (!member_ready && ldn_join_associated()) {
                esp_wifi_get_mac(WIFI_IF_STA, our_mac);
                bool have = (membership_ips(our_mac, our_ip, host_ip, host_mac) == 0);
                const char *ipsrc = "advert-list";
                if (!have && ldn_join_ip_frames() > 0 && ldn_join_host_ip(host_ip)) {
                    /* On a fresh room the Switch streams us Pia (host .X.1) before publishing us in the advert
                     * list; FRLG assigns the single joiner .2, so derive our IP = host+1 (observed .18.1/.18.2,
                     * .126.1/.126.2) and proceed — the participant list may never list us pre-join. */
                    memcpy(our_ip, host_ip, 4);
                    our_ip[3] = (uint8_t)(host_ip[3] + 1);
                    memcpy(host_mac, s_jp.bssid, 6);
                    have = true; ipsrc = "derived-from-Pia";
                }
                if (have) {
                    ldn_join_set_member(true);
                    printf("  [member:%s] our_ip=%u.%u.%u.%u host_ip=%u.%u.%u.%u host_mac=%02x:%02x:%02x:%02x:%02x:%02x\n",
                           ipsrc, our_ip[0], our_ip[1], our_ip[2], our_ip[3],
                           host_ip[0], host_ip[1], host_ip[2], host_ip[3],
                           host_mac[0], host_mac[1], host_mac[2], host_mac[3], host_mac[4], host_mac[5]);
                    esp_err_t r = ldn_udp_start(our_ip, host_ip, host_mac);
                    printf("  [member] ldn_udp_start = %d (seated, beacon-only; awaiting GBA room selection)\n", (int)r);
                    if (r == ESP_OK) { member_ready = true; seat_us = esp_timer_get_time();     /* tr-1.36 seat clock */
                                       seat_last_ipf = ldn_join_ip_frames(); }                   /* tr-1.63 liveness baseline */
                }
            }
            /* STAGE B — start the sim JOIN only once the GBA player selects the Switch's room (0x1f connect
             * to our peer). This is what actually sends the join to the Switch -> the Yes/No prompt. Gating it
             * here (not on membership) fixes the premature "Emu wants to join" + the GBA join-screen crash.
             *
             * Kizuna: bounded wait, up to GBA_IDENTITY_WAIT_US, for the GBA's own trainer-card
             * identity (gba_bridge.c's ni_scan_child -> gba_bridge_set_gba_identity) before starting the sim.
             * sim.c's ensure_ni() bakes the outgoing NI game-data block on its first tick and never rebuilds
             * it (s->ni_built latches) -- confirmed on the bench: the child's own name-send (0x24/
             * 0x25 LLSF subframes) only begins AFTER 0x1f select, roughly 0.5-1s later, so starting the sim
             * the instant select is seen (as before) always baked the sim's "EMU"/0x8822 stub identity into
             * that packet; real capture was `[ni-child] raw trainer-card bytes` arriving too late to matter.
             * Worst case here (identity never arrives) is unchanged from before: same timeout, same "EMU"
             * fallback, same start time as the old unconditional call. */
            if (member_ready && !brain_started && gba_bridge_gba_join_selected()) {
                if (s_gba_identity_wait_start_us == 0) s_gba_identity_wait_start_us = esp_timer_get_time();
                uint16_t id_tid; uint8_t id_name[8], id_lang, id_act;
                bool have_identity = gba_bridge_get_gba_identity(&id_tid, id_name, &id_lang, &id_act);
                bool waited_out = (esp_timer_get_time() - s_gba_identity_wait_start_us) > GBA_IDENTITY_WAIT_US;
                if (!have_identity && !waited_out) {
                    /* Not ready yet: fall through to the loop's own 50ms vTaskDelay below and re-check next
                     * pass. Deliberately NOT a `continue` here -- this whole block sits inside the per-
                     * iteration GBA command-phase service loop, and skipping the rest of an iteration for up
                     * to GBA_IDENTITY_WAIT_US risks starving whatever else that loop services each pass. */
                } else {
                printf("  [join] GBA selected the room -> starting the sim (identity %s, ticked INLINE by the GBA loop)\n",
                       have_identity ? "real" : "not captured in time -> EMU default");
                ldn_join_set_ip_forward(true);   /* NOW forward Pia into lwIP — the inline loop will drain it */
                ldn_brain_start(s_jp.ssid, our_ip, host_ip, our_mac, s_jp.bssid, "Kizuna");
                /* SINGLE LOOP: no separate brain task. ldn_brain_start() just inits the sim; the GBA command
                 * phase ticks it inline in its idle gaps (gba_login_main.c) — one cooperative loop, no
                 * priorities, so the GBA link and the Switch/Wi-Fi can't starve each other. */
                brain_started = true;
                log_heap("brain-started");
                }
            }
            vTaskDelay(pdMS_TO_TICKS(50));
        }
        s_st_joined = joined; s_st_brain = brain_started; s_st_member = member_ready;
        int64_t now = esp_timer_get_time();
        if (now - last_hb > 2000000) {
            last_hb = now;
#ifndef KZ_QUIET_TRACE
            log_heap("hb");                    /* the 2 s heartbeat; the stage reports stay */
#endif
            if (!joined) {
                bool room = s_room_channel >= 0 && (now - s_last_room_us) < 3000000;
                KZ_TRACE("  [trade-hb] ldn=%u frlg=%u beacons=%u host_beacon=%u host_probresp=%u | room=%s%d\n",
                       (unsigned)s_ldn_seen, (unsigned)s_frlg_seen, (unsigned)s_beacon_seen,
                       (unsigned)s_beacon_from_host, (unsigned)s_probresp_from_host,
                       room ? "ch" : "(none) ch", s_room_channel);
            } else {
                KZ_TRACE("  [join-hb] assoc(port authorized)=%d  member(has IP)=%d\n",
                       ldn_join_associated(), ldn_join_member());
            }
        }
    }
}

/* Bring up Wi-Fi promiscuous scanning across 1/6/11 (heavy load) + the LDN join. */
void trade_session_start(void)
{
    /* Configure the promiscuous cb + filter; scan_task ENABLES promiscuous only AFTER the GBA logs in (the
     * discovery window). Wi-Fi stays quiet through login so the login-burst flush() can't race Wi-Fi GDMA
     * (tr-0.53). Setting cb/filter before enabling is fine (proven tr-0.54: promisc_rx fired once enabled). */
    wifi_promiscuous_filter_t filt = { .filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT };
    esp_wifi_set_promiscuous_filter(&filt);
    esp_wifi_set_promiscuous_rx_cb(promisc_rx);
    ldn_join_init();                                    /* install the WPA hook + 0x88b7 data RX cb (once) */
    printf("  [trade] ready; send keys then \"start\" to discover FRLG rooms\n");
    xTaskCreate(scan_task, "scan", 8192, NULL, 4, NULL);
}

/* ---- console / app control (M1) ---- */
void trade_session_set_keys(const ldn_keys *k) { s_keys = k; ldn_join_set_keys(k); }
bool trade_session_arm(void)
{
    if (!s_keys) return false;
    s_armed = true;
    /* With the RP2040 up, "browsing" comes from the GBA's own RFU1 traffic (rfu1_bridge_on_gba_browsing);
     * without it (M1 bench) the console start still means "the GBA is browsing". */
    s_browsing = !rfu1_bridge_up() || rfu1_bridge_login_state() >= 2;
    return true;
}
void trade_session_select(void) { gba_bridge_note_gba_join_selected(); }

/* ---- rfu1_bridge hooks (M2): the GBA-side events, sourced from the RP2040's RFU1 packets ---- */
void rfu1_bridge_on_gba_browsing(void)
{
    if (s_armed) { s_browsing = true; return; }
    /* a GBA logged in while we sit idle (Bluetooth up, Wi-Fi down). With keys in hand, ask the mode to
     * hand the radio over; it arms us once Wi-Fi is up and the browsing flag is set by the next poll. */
    if (s_keys && s_radio_cb) s_radio_cb(true);
}
void rfu1_bridge_on_gba_selected(void) { gba_bridge_note_gba_join_selected(); }
/* in a Colosseum room the GBA leaving does not end the Switch session at once (GB-Link pia_bridge.c
 * RFU1_DISCONNECT: it only marks the child gone). Its last frames -- its own READY_CLOSE_LINK 0x5F00, which the
 * Switch waits for from every player -- can still be queued behind held keys; stopping the brain dropped them and
 * the Switch crashed in the close (hw run 9). Keep relaying until the Switch disconnects us (the brain stops on
 * host_d), the network goes, or BATTLE_CLOSE_GRACE_US passes. Trade rooms end at once, as before. */
void rfu1_bridge_on_gba_gone(void)
{
    if (ldn_brain_active() && (s_room_activity == 1 || s_room_activity == 2)) {
        if (!s_battle_gone_us) {
            s_battle_gone_us = esp_timer_get_time();
            printf("  [trade] GBA left the battle room -> keep relaying to the Switch until it closes (max %d s)\n",
                   BATTLE_CLOSE_GRACE_US / 1000000);
        }
        return;
    }
    if (ldn_brain_active()) { s_stop_req = true; return; }   /* session ends; stays armed for a re-browse */
    s_browsing = false;
    if (s_armed && s_radio_cb) s_radio_cb(false);           /* no session, no GBA: back to Bluetooth */
}
void trade_session_disarm(void)
{
    if (!s_armed) return;
    s_armed = false; s_browsing = false;
    for (int i = 0; i < 100 && !s_scan_parked; i++) vTaskDelay(pdMS_TO_TICKS(20));   /* let the scan task tear down */
}
bool trade_session_armed(void)    { return s_armed; }
bool trade_session_has_keys(void) { return s_keys != NULL; }
void trade_session_set_radio_cb(void (*cb)(bool session)) { s_radio_cb = cb; }
/* Beacon the Switch's room only once we are a joinable member (the C6's tr-0.57 gate) and the room is here. */
bool rfu1_bridge_room_visible(void)    { return s_armed && trade_room_present() && ldn_join_member(); }
bool rfu1_bridge_room_identity(char name[9], uint16_t *tid) { return trade_room_identity(name, tid); }
bool rfu1_bridge_room_identity_raw(uint8_t name_raw[8]) { return trade_room_identity_raw(name_raw); }
uint8_t rfu1_bridge_room_activity(void) { return s_room_activity; }
void trade_session_stop(void)   { s_stop_req = true; }
void trade_session_status(char *out, size_t cap)
{
    char rf[320]; rfu1_bridge_status(rf, sizeof rf);
    char ld[320] = "";
#ifdef KZ_GBA_LEAD
    ld[0] = ','; gba_lead_status(ld + 1, sizeof ld - 1);
#endif
    snprintf(out, cap, "{\"ev\":\"status\",%s%s,\"keys\":%d,\"armed\":%d,\"room\":%d,\"room_ch\":%d,\"host\":\"%s\",\"joined\":%d,\"assoc\":%d,\"member\":%d,\"seated\":%d,\"selected\":%d,\"brain\":%d,\"accepted\":%d,\"activity\":%u,\"free\":%u,\"largest\":%u}",
             rf, ld, s_keys != NULL, s_armed, s_jp_ready, s_room_channel, s_room_name, s_st_joined,
             ldn_join_associated(), ldn_join_member(), s_st_member, gba_bridge_gba_join_selected(),
             s_st_brain && ldn_brain_active(), ldn_brain_active() && ldn_brain_join_accepted(), s_room_activity,
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_8BIT), (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
}
