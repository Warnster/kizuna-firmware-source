/* trade_host_session.c — see trade_host_session.h. One task owns the whole host role: ldn_ap_poll (join/auth/advert),
 * dist_udp_poll (Switch datagrams -> pia_host_receive), pia_host_tick (every Pia/Reliable/RFU/trade timer). The
 * member callbacks run inside ldn_ap_poll, so they share that task's context with pia_host and need no locking;
 * start/stop from the console take the mutex and the task takes it around each loop pass. */
#include "trade_host_session.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "ldn_ap.h"
#include "ldn_host.h"
#include "dist_udp.h"
#include "pia_host.h"
#include "trade_host.h"
#include "proxy_activity.h"

static const char *TAG = "th_session";

/* ldn_sta's real hook (join role, switch-trade) is a strong definition that overrides this no-op when it is linked; the
 * hosted-only image has no join role and so no hook to stand down. */
extern void ldn_join_set_passthrough(bool on);
__attribute__((weak)) void ldn_join_set_passthrough(bool on) { (void)on; }

#define MEMBERS_MAX PIA_HOST_MAX_PARTICIPANTS

static th_emit_fn         s_emit;
static uint8_t            s_party[TH_PARTY_MAX][MON_PARTY_SIZE];
static bool               s_party_set[TH_PARTY_MAX];

static SemaphoreHandle_t  s_lock;
static TaskHandle_t       s_task;
static volatile bool      s_run, s_task_done;
static pia_host_t        *s_ph;
static trade_host        *s_th;
static const ldn_keys    *s_keys;

static ldn_frlg_room      s_room;
static uint8_t            s_app_inactive[LDN_FRLG_APP_DATA_LEN];
static uint8_t            s_app_active[LDN_FRLG_APP_DATA_LEN];
static long               s_app_active_len;
static ldn_ap_config      s_ap_cfg;
static char               s_name[8], s_sta_ssid[33], s_sta_pass[65];

static uint8_t            s_member_ip[MEMBERS_MAX][4];
static uint8_t            s_member_mac[MEMBERS_MAX][6];
static int                s_members;
static int                s_last_state = -1, s_last_commits, s_last_done;
static uint32_t           s_loops, s_slow_loops;

/* Frame-level remote trade (proxy): the activity is a px_activity that ships the Switch's child rows over BLE to the
 * phone and plays back the remote engine's parent rows; s_th stays NULL in this mode. */
static px_activity       *s_px;
static th_proxy_tx_fn     s_px_tx;
static uint32_t           s_px_seen_flags;
static uint32_t           s_px_flush_ms = PX_FLUSH_MS_DEFAULT;
static th_proxy_link_fn   s_px_link;
static bool px_tx(void *ctx, const uint8_t *m, size_t n) { (void)ctx; return s_px_tx ? s_px_tx(m, n) : false; }

void th_session_set_emit(th_emit_fn fn) { s_emit = fn; }
void th_session_set_proxy_tx(th_proxy_tx_fn fn) { s_px_tx = fn; }
void th_session_set_proxy_link(th_proxy_link_fn fn) { s_px_link = fn; }
void th_session_set_proxy_flush_ms(uint32_t ms)
{
    s_px_flush_ms = ms > 1000 ? 1000 : ms;
    px_activity *px = s_px;
    if (px) px_set_flush_ms(px, s_px_flush_ms);
}
bool th_session_proxy_data(const uint8_t *data, size_t len)
{
    px_activity *px = s_px;
    return px && px_on_data(px, data, len);
}

static void emit(const char *json) { if (s_emit) s_emit(json); }

bool th_session_set_mon(int index, const uint8_t mon[100])
{
    if (index < 0 || index >= TH_PARTY_MAX || th_session_active()) return false;
    memcpy(s_party[index], mon, MON_PARTY_SIZE);
    s_party_set[index] = true;
    return true;
}

void th_session_clear_party(void) { memset(s_party_set, 0, sizeof s_party_set); }

int th_session_party_count(void)
{
    int n = 0;
    while (n < TH_PARTY_MAX && s_party_set[n]) n++;
    return n;
}

bool th_session_active(void) { return s_ph != NULL; }

static uint32_t now_ms(void) { return (uint32_t)(esp_timer_get_time() / 1000); }

/* delay-injection test: hold every datagram to/from the Switch for a configurable one-way delay (+ jitter) to
 * emulate the internet in a frame-level proxy design. Items are malloc'd per packet (PSRAM when present), so a typical
 * 150 ms delay costs a few KB. Everything runs in the session task, so no locking beyond the session mutex. */
#define DQ_MAX 96
typedef struct { uint32_t due; uint16_t len; uint8_t ip[4]; uint8_t *data; } dq_item;
static dq_item s_dq_tx[DQ_MAX], s_dq_rx[DQ_MAX];
static int     s_dq_tx_n, s_dq_rx_n;
static volatile int s_delay_ms, s_jitter_ms;
static unsigned s_dq_peak, s_dq_drop;

static bool dq_push(dq_item *q, int *n, const uint8_t ip[4], const uint8_t *d, size_t len)
{
    if (*n >= DQ_MAX) { s_dq_drop++; return false; }
    uint8_t *copy = heap_caps_malloc(len, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!copy) copy = heap_caps_malloc(len, MALLOC_CAP_8BIT);
    if (!copy) { s_dq_drop++; return false; }
    memcpy(copy, d, len);
    uint32_t jit = s_jitter_ms > 0 ? esp_random() % (uint32_t)(s_jitter_ms + 1) : 0;
    q[*n].due = now_ms() + (uint32_t)s_delay_ms + jit;
    q[*n].len = (uint16_t)len;
    memcpy(q[*n].ip, ip, 4);
    q[*n].data = copy;
    (*n)++;
    if ((unsigned)*n > s_dq_peak) s_dq_peak = (unsigned)*n;
    return true;
}

/* Deliver items whose time has come (all of them when `all`), keeping the rest in order. */
static void dq_release(dq_item *q, int *n, bool all, void (*deliver)(const uint8_t ip[4], const uint8_t *d, size_t len))
{
    uint32_t now = now_ms();
    int keep = 0;
    for (int i = 0; i < *n; i++) {
        if (all || (int32_t)(now - q[i].due) >= 0) {
            deliver(q[i].ip, q[i].data, q[i].len);
            free(q[i].data);
        } else {
            q[keep++] = q[i];
        }
    }
    *n = keep;
}

static void dq_free_all(dq_item *q, int *n)
{
    for (int i = 0; i < *n; i++) free(q[i].data);
    *n = 0;
}

static void deliver_tx(const uint8_t ip[4], const uint8_t *d, size_t len) { dist_udp_send(ip, d, (int)len); }
static void deliver_rx(const uint8_t ip[4], const uint8_t *d, size_t len)
{
    if (s_ph) pia_host_receive(s_ph, d, len, ip, now_ms());
}

static void pia_send(void *ctx, const uint8_t *dg, size_t len, const uint8_t dst_ip[4])
{
    (void)ctx;
    if (s_delay_ms > 0) { dq_push(s_dq_tx, &s_dq_tx_n, dst_ip, dg, len); return; }
    dist_udp_send(dst_ip, dg, (int)len);
}

static void pia_rng(void *ctx, uint8_t *out, size_t n) { (void)ctx; esp_fill_random(out, n); }

static void pia_event(void *ctx, pia_host_event ev)
{
    (void)ctx;
    static const char *names[] = { "net_acked", "join", "finalized", "property_started", "leave" };
    char b[96];
    snprintf(b, sizeof b, "{\"ev\":\"th_pia\",\"e\":\"%s\"}", (unsigned)ev < 5 ? names[ev] : "?");
    emit(b);
    if (ev == PIA_HOST_EV_PROPERTY_STARTED && s_app_active_len > 0)
        ldn_ap_set_app_data(s_app_active, (size_t)s_app_active_len);
}

static void rx_datagram(const uint8_t src_ip[4], const uint8_t *data, int len)
{
    if (!s_ph) return;
    if (s_delay_ms > 0) { dq_push(s_dq_rx, &s_dq_rx_n, src_ip, data, (size_t)len); return; }
    pia_host_receive(s_ph, data, (size_t)len, src_ip, now_ms());
}

static void member_cb(const ldn_ap_member *m, bool joined)
{
    char b[160];
    snprintf(b, sizeof b, "{\"ev\":\"member\",\"joined\":%d,\"platform\":%d,\"name\":\"%s\",\"mac\":\"%02x:%02x:%02x:%02x:%02x:%02x\"}",
             joined, m->platform, m->name, m->mac[0], m->mac[1], m->mac[2], m->mac[3], m->mac[4], m->mac[5]);
    emit(b);
    if (!s_ph) return;
    int idx = -1;
    for (int i = 0; i < s_members; i++) if (!memcmp(s_member_mac[i], m->mac, 6)) idx = i;
    if (joined) {
        if (idx >= 0 || s_members >= MEMBERS_MAX) return;
        memcpy(s_member_ip[s_members], m->ip, 4);
        memcpy(s_member_mac[s_members], m->mac, 6);
        s_members++;
        dist_udp_peer(m->ip, m->mac);
        pia_host_set_participants(s_ph, (const uint8_t (*)[4])s_member_ip, s_members);
        if (s_members == 1) {
            pia_host_on_participant_joined(s_ph);
            if (s_px) px_ctrl(s_px, PX_CTRL_HELLO);
        }
    } else if (idx >= 0) {
        dist_udp_peer(s_member_ip[idx], NULL);
        for (int i = idx; i + 1 < s_members; i++) {
            memcpy(s_member_ip[i], s_member_ip[i + 1], 4);
            memcpy(s_member_mac[i], s_member_mac[i + 1], 6);
        }
        s_members--;
        pia_host_set_participants(s_ph, (const uint8_t (*)[4])s_member_ip, s_members);
        if (s_members == 0) {
            pia_host_on_ldn_leave(s_ph);
            if (s_px) px_ctrl(s_px, PX_CTRL_CLOSE);
        }
    }
}

static void report_progress(void)
{
    int st = (int)trade_host_state(s_th);
    if (st != s_last_state) {
        s_last_state = st;
        char b[96];
        snprintf(b, sizeof b, "{\"ev\":\"th_state\",\"state\":\"%s\"}", trade_host_state_name((th_state)st));
        emit(b);
    }
    int c = trade_host_commits(s_th);
    while (s_last_commits < c) {
        const uint8_t *m = trade_host_received_mon(s_th, s_last_commits);
        if (m) {
            static char b[300];
            int n = snprintf(b, sizeof b, "{\"ev\":\"th_received\",\"i\":%d,\"hex\":\"", s_last_commits);
            for (int i = 0; i < MON_PARTY_SIZE && n < (int)sizeof b - 8; i++) n += snprintf(b + n, sizeof b - (size_t)n, "%02x", m[i]);
            snprintf(b + n, sizeof b - (size_t)n, "\"}");
            emit(b);
        }
        s_last_commits++;
    }
    if (trade_host_done(s_th) && !s_last_done) {
        s_last_done = 1;
        emit("{\"ev\":\"th_done\"}");
    }
}

/* Proxy mode: surface the remote's control messages once each, as {"ev":"th_proxy","e":...}. */
static void report_proxy(void)
{
    static const struct { uint8_t code; const char *name; } ev[] = {
        { PX_CTRL_ESTABLISHED, "established" }, { PX_CTRL_DISCONNECT_REQUESTED, "disconnect_requested" },
        { PX_CTRL_PEER_DOWN, "peer_down" }, { PX_CTRL_CLOSE, "close" }, { PX_CTRL_DONE, "done" },
    };
    uint32_t f = __atomic_load_n(&s_px->flags, __ATOMIC_ACQUIRE);
    uint32_t fresh = f & ~s_px_seen_flags;
    if (!fresh) return;
    s_px_seen_flags |= fresh;
    for (unsigned i = 0; i < sizeof ev / sizeof ev[0]; i++) {
        if (!(fresh & (1u << ev[i].code))) continue;
        char b[80];
        snprintf(b, sizeof b, "{\"ev\":\"th_proxy\",\"e\":\"%s\"}", ev[i].name);
        emit(b);
        if (ev[i].code == PX_CTRL_DONE) emit("{\"ev\":\"th_done\"}");
    }
}

/* Once a second while a session runs: the compact feed the phone's trade display draws from (over BLE and the console). */
static void report_tick(void)
{
    unsigned dtx, drx, drej, nomem, other; int err;
    dist_udp_counters(&dtx, &drx, &drej);
    dist_udp_send_errors(&nomem, &other, &err);
    if (s_px) {
        uint16_t itvl = 0, mtu = 0;
        if (s_px_link) s_px_link(&itvl, &mtu);
        char b[700];
        snprintf(b, sizeof b,
                 "{\"ev\":\"th_tick\",\"state\":\"PROXY\",\"commits\":0,\"members\":%d,\"rx\":%u,\"tx\":%u,\"fail\":%u,\"nomem\":%u,\"errno\":%d,\"slow\":%u,"
                 "\"delay\":%d,\"dqpeak\":%u,\"dqdrop\":%u,"
                 "\"px\":{\"up\":%u,\"down\":%u,\"under\":%u,\"pre\":%u,\"gaps\":%u,\"dups\":%u,\"bledrop\":%u,\"retry\":%u,\"bndl\":%u,\"bytes\":%u,"
                 "\"upovf\":%u,\"dnovf\":%u,\"depth\":%d,\"est\":%d,\"ticks\":%u,\"tmsg\":%u,\"shed\":%u,\"resets\":%u,\"tovf\":%u,\"bad\":%u,"
                 "\"fl\":%u,\"itvl\":%.2f,\"mtu\":%u},"
                 "\"int\":%u,\"intmin\":%u}",
                 s_members, drx, dtx, drej, nomem, err, (unsigned)s_slow_loops, (int)s_delay_ms, s_dq_peak, s_dq_drop,
                 (unsigned)s_px->up_rows, (unsigned)s_px->down_rows, (unsigned)s_px->underruns, (unsigned)s_px->prefill_idle,
                 (unsigned)s_px->gaps, (unsigned)s_px->dups, (unsigned)s_px->ble_drops, (unsigned)s_px->ble_retries,
                 (unsigned)s_px->bundles_tx, (unsigned)s_px->bundle_bytes_tx, (unsigned)s_px->up_overflow, (unsigned)s_px->down_overflow,
                 px_down_depth(s_px), px_flag(s_px, PX_CTRL_ESTABLISHED), (unsigned)s_px->ticks, (unsigned)s_px->tick_msgs,
                 (unsigned)s_px->idle_shed, (unsigned)s_px->credit_resets, (unsigned)s_px->tick_overflow, (unsigned)s_px->bad_msgs,
                 (unsigned)px_effective_flush_ms(s_px), itvl * 1.25, (unsigned)mtu,
                 (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
                 (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
        emit(b);
        return;
    }
    char b[300];
    snprintf(b, sizeof b,
             "{\"ev\":\"th_tick\",\"state\":\"%s\",\"commits\":%d,\"members\":%d,\"rx\":%u,\"tx\":%u,\"fail\":%u,\"nomem\":%u,\"errno\":%d,\"slow\":%u,\"delay\":%d,\"dqpeak\":%u,\"dqdrop\":%u,\"int\":%u,\"intmin\":%u}",
             trade_host_state_name(trade_host_state(s_th)), trade_host_commits(s_th), s_members, drx, dtx, drej, nomem, err,
             (unsigned)s_slow_loops, (int)s_delay_ms, s_dq_peak, s_dq_drop, (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
             (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    emit(b);
}

static void session_task(void *arg)
{
    (void)arg;
    int64_t next_tick = esp_timer_get_time() + 1000000;
    while (s_run) {
        int64_t t0 = esp_timer_get_time();
        xSemaphoreTake(s_lock, portMAX_DELAY);
        if (s_ph) {
            ldn_ap_poll();
            dist_udp_poll();
            if (s_dq_rx_n) dq_release(s_dq_rx, &s_dq_rx_n, false, deliver_rx);
            pia_host_tick(s_ph, now_ms());
            if (s_dq_tx_n) dq_release(s_dq_tx, &s_dq_tx_n, false, deliver_tx);
            if (s_px) {
                if (s_px_link) {                                   /* pace bundles to the negotiated connection interval */
                    uint16_t it = 0, mt = 0;
                    s_px_link(&it, &mt);
                    px_set_link_itvl_ms(s_px, ((uint32_t)it * 5 + 3) / 4);
                }
                px_flush(s_px, now_ms()); report_proxy();
            }
            else report_progress();
            if (esp_timer_get_time() >= next_tick) { next_tick += 1000000; report_tick(); }
        }
        xSemaphoreGive(s_lock);
        s_loops++;
        if (esp_timer_get_time() - t0 > 12000) s_slow_loops++;   /* a pass longer than ~a VBlank */
        vTaskDelay(1);
    }
    s_task_done = true;
    vTaskDelete(NULL);
}

static bool session_start(const ldn_keys *keys, const th_session_cfg *cfg, bool proxy)
{
    if (!s_lock) s_lock = xSemaphoreCreateMutex();
    if (s_ph) { emit("{\"ev\":\"error\",\"message\":\"th: already running.\"}"); return false; }
    int n = proxy ? 0 : th_session_party_count();
    if (proxy && !s_px_tx) { emit("{\"ev\":\"error\",\"message\":\"th: no Bluetooth data path registered for the proxy.\"}"); return false; }
    if (!proxy && n < 1) { emit("{\"ev\":\"error\",\"message\":\"th: stage a party first (th_mon).\"}"); return false; }
    if (!proxy && (cfg->trades < 1 || cfg->trades > n || cfg->slot < 0 || cfg->slot + cfg->trades > n)) {
        emit("{\"ev\":\"error\",\"message\":\"th: trades/slot do not fit the party.\"}");
        return false;
    }

    /* ~27 KB of session state: PSRAM when the board has it (internal heap is the scarce kind), else internal. */
    s_ph = heap_caps_calloc(1, sizeof *s_ph, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_ph) s_ph = heap_caps_calloc(1, sizeof *s_ph, MALLOC_CAP_8BIT);
    if (proxy) {
        s_px = heap_caps_calloc(1, sizeof *s_px, MALLOC_CAP_8BIT);     /* ~2 KB: internal is fine, it is touched from the BLE task */
        if (!s_ph || !s_px) goto fail_mem;
        px_init(s_px, px_tx, NULL);
        px_set_flush_ms(s_px, s_px_flush_ms);
        s_px_seen_flags = 0;
    } else {
        s_th = heap_caps_calloc(1, sizeof *s_th, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!s_th) s_th = heap_caps_calloc(1, sizeof *s_th, MALLOC_CAP_8BIT);
        if (!s_ph || !s_th) goto fail_mem;
    }

    snprintf(s_name, sizeof s_name, "%s", cfg->name && cfg->name[0] ? cfg->name : "Kizuna");
    if (!proxy) {
        static th_cfg tc;   /* ~1 KB: off the console task's stack */
        th_cfg_default(&tc);
        snprintf(tc.lp.name, sizeof tc.lp.name, "%s", s_name);
        tc.lp.trainer_id = ((uint32_t)cfg->sid << 16) | cfg->tid;
        tc.lp.version = LP_VERSION_FIRE_RED;
        for (int i = 0; i < n; i++) memcpy(tc.party[i], s_party[i], MON_PARTY_SIZE);
        tc.party_count = n;
        tc.trades = cfg->trades;
        tc.trade_slot = cfg->slot;
        if (trade_host_init(s_th, &tc) != 0) { emit("{\"ev\":\"error\",\"message\":\"th: trade engine rejected the config.\"}"); goto fail; }
    }

    uint8_t hsid[2] = { (uint8_t)esp_random(), 0xf1 };
    ldn_frlg_room_trade(&s_room, s_name, cfg->tid, hsid, LDN_FRLG_VERSION_FIRE_RED, LDN_FRLG_LANGUAGE_ENGLISH);
    long len = ldn_frlg_app_data_build(&s_room, s_app_inactive, sizeof s_app_inactive);
    ldn_frlg_room active = s_room;
    ldn_frlg_room_trade_activate(&active);
    s_app_active_len = ldn_frlg_app_data_build(&active, s_app_active, sizeof s_app_active);
    if (len < 0 || s_app_active_len < 0) { emit("{\"ev\":\"error\",\"message\":\"th: room app data failed.\"}"); goto fail; }

    s_sta_ssid[0] = s_sta_pass[0] = 0;
    if (cfg->sta_ssid && cfg->sta_ssid[0]) {
        snprintf(s_sta_ssid, sizeof s_sta_ssid, "%s", cfg->sta_ssid);
        snprintf(s_sta_pass, sizeof s_sta_pass, "%s", cfg->sta_pass ? cfg->sta_pass : "");
    }
    ldn_join_set_passthrough(s_sta_ssid[0] != 0);
    memset(&s_ap_cfg, 0, sizeof s_ap_cfg);
    s_ap_cfg.channel = 6;
    s_ap_cfg.host_name = s_name;
    s_ap_cfg.app_data = s_app_inactive;
    s_ap_cfg.app_data_len = (size_t)len;
    s_ap_cfg.sta_ssid = s_sta_ssid[0] ? s_sta_ssid : NULL;
    s_ap_cfg.sta_pass = s_sta_pass;
    s_keys = keys;
    s_members = 0;
    s_last_state = -1; s_last_commits = 0; s_last_done = 0; s_loops = s_slow_loops = 0;

    if (ldn_ap_start(keys, &s_ap_cfg, member_cb) != ESP_OK) { emit("{\"ev\":\"error\",\"message\":\"th: ldn_ap_start failed.\"}"); goto fail; }

    pia_host_cfg pc;
    memset(&pc, 0, sizeof pc);
    pia_host_cfg_trade_defaults(&pc);
    memcpy(pc.ssid, ldn_ap_ssid(), sizeof pc.ssid);
    ldn_ap_identity(pc.our_mac, pc.our_ip);
    memcpy(pc.broadcast_ip, pc.our_ip, 3);
    pc.broadcast_ip[3] = 255;
    pc.session_name = s_name;
    pc.active_app_data = s_app_active;
    pc.active_app_data_len = (int)s_app_active_len;
    memcpy(pc.host_session_id, hsid, 2);
    pc.send = pia_send;
    pc.rng = pia_rng;
    pc.event = pia_event;
    pia_activity act = proxy ? px_as_activity(s_px) : trade_host_activity(s_th);
    pia_host_init(s_ph, &pc, &act);

    if (dist_udp_start(rx_datagram) != ESP_OK) { emit("{\"ev\":\"error\",\"message\":\"th: dist_udp_start failed.\"}"); ldn_ap_stop(); goto fail; }

    s_run = true; s_task_done = false;
    if (xTaskCreate(session_task, "th_session", 8192, NULL, 6, &s_task) != pdPASS) {
        s_run = false; dist_udp_stop(); ldn_ap_stop();
        emit("{\"ev\":\"error\",\"message\":\"th: task create failed.\"}");
        goto fail;
    }
    if (proxy) ESP_LOGI(TAG, "hosting trade room, proxying the Switch's rows to a remote engine over Bluetooth");
    else ESP_LOGI(TAG, "hosting trade room, party %d, trades %d, slot %d", n, cfg->trades, cfg->slot);
    emit(proxy ? "{\"ev\":\"th_host\",\"on\":true,\"mode\":\"proxy\"}" : "{\"ev\":\"th_host\",\"on\":true}");
    return true;

fail_mem:
    emit("{\"ev\":\"error\",\"message\":\"th: out of memory.\"}");
fail:
    ldn_join_set_passthrough(false);
    free(s_ph); free(s_th); free(s_px); s_ph = NULL; s_th = NULL; s_px = NULL;
    return false;
}

bool th_session_start(const ldn_keys *keys, const th_session_cfg *cfg) { return session_start(keys, cfg, false); }
bool th_session_start_proxy(const ldn_keys *keys, const th_session_cfg *cfg) { return session_start(keys, cfg, true); }

/* Runtime one-way delay (+ random 0..jitter) on both directions; 0 flushes and disables. Safe while a session runs. */
void th_session_set_delay(int delay_ms, int jitter_ms)
{
    if (delay_ms < 0) delay_ms = 0;
    if (delay_ms > 2000) delay_ms = 2000;
    if (jitter_ms < 0) jitter_ms = 0;
    if (s_lock) xSemaphoreTake(s_lock, portMAX_DELAY);
    bool was = s_delay_ms > 0;
    s_delay_ms = delay_ms; s_jitter_ms = jitter_ms;
    if (was && delay_ms == 0 && s_ph) {
        dq_release(s_dq_rx, &s_dq_rx_n, true, deliver_rx);
        dq_release(s_dq_tx, &s_dq_tx_n, true, deliver_tx);
    }
    s_dq_peak = s_dq_drop = 0;
    if (s_lock) xSemaphoreGive(s_lock);
}

void th_session_stop(void)
{
    if (!s_ph) return;
    s_run = false;
    for (int i = 0; i < 100 && !s_task_done; i++) vTaskDelay(pdMS_TO_TICKS(10));
    xSemaphoreTake(s_lock, portMAX_DELAY);
    px_activity *px = s_px;
    if (px) { px_ctrl(px, PX_CTRL_CLOSE); px_flush_now(px); s_px = NULL; vTaskDelay(pdMS_TO_TICKS(50)); }   /* let a BLE-task write finish */
    dq_free_all(s_dq_rx, &s_dq_rx_n); dq_free_all(s_dq_tx, &s_dq_tx_n);
    dist_udp_stop();
    ldn_ap_stop();
    ldn_join_set_passthrough(false);
    free(s_ph); free(s_th); free(px);
    s_ph = NULL; s_th = NULL;
    s_task = NULL;
    xSemaphoreGive(s_lock);
    emit("{\"ev\":\"th_host\",\"on\":false}");
}

void th_session_status(char *buf, size_t n)
{
    if (!s_ph) { snprintf(buf, n, "{\"ev\":\"th_status\",\"active\":0,\"party\":%d}", th_session_party_count()); return; }
    unsigned dtx, drx, drej, dnomem, dother; int derr;
    dist_udp_counters(&dtx, &drx, &drej);
    dist_udp_send_errors(&dnomem, &dother, &derr);
    if (s_px) {
        snprintf(buf, n,
                 "{\"ev\":\"th_status\",\"active\":1,\"mode\":\"proxy\",\"members\":%d,\"est\":%d,"
                 "\"pia\":{\"joined\":%d,\"net_acked\":%d,\"finalized\":%d,\"prop\":%d,\"rx\":%d,\"rej\":%d,\"tx\":%d},"
                 "\"udp\":{\"tx\":%u,\"rx\":%u,\"rej\":%u,\"nomem\":%u,\"errno\":%d},"
                 "\"px\":{\"up\":%u,\"down\":%u,\"under\":%u,\"gaps\":%u,\"dups\":%u,\"bledrop\":%u,\"retry\":%u,\"bndl\":%u,\"depth\":%d,\"flags\":%u},"
                 "\"loops\":%u,\"slow\":%u}",
                 s_members, px_flag(s_px, PX_CTRL_ESTABLISHED), s_ph->joined, s_ph->net_acked, s_ph->session_finalized, s_ph->property_started,
                 s_ph->rx_datagrams, s_ph->rx_rejected, s_ph->tx_datagrams, dtx, drx, drej, dnomem, derr,
                 (unsigned)s_px->up_rows, (unsigned)s_px->down_rows, (unsigned)s_px->underruns, (unsigned)s_px->gaps, (unsigned)s_px->dups,
                 (unsigned)s_px->ble_drops, (unsigned)s_px->ble_retries, (unsigned)s_px->bundles_tx, px_down_depth(s_px), (unsigned)s_px->flags,
                 (unsigned)s_loops, (unsigned)s_slow_loops);
        return;
    }
    snprintf(buf, n,
             "{\"ev\":\"th_status\",\"active\":1,\"state\":\"%s\",\"commits\":%d,\"members\":%d,\"est\":%d,"
             "\"pia\":{\"joined\":%d,\"net_acked\":%d,\"finalized\":%d,\"prop\":%d,\"rx\":%d,\"rej\":%d,\"tx\":%d},"
             "\"udp\":{\"tx\":%u,\"rx\":%u,\"rej\":%u,\"nomem\":%u,\"errno\":%d},\"loops\":%u,\"slow\":%u}",
             trade_host_state_name(trade_host_state(s_th)), trade_host_commits(s_th), s_members,
             trade_host_established(s_th), s_ph->joined, s_ph->net_acked, s_ph->session_finalized, s_ph->property_started,
             s_ph->rx_datagrams, s_ph->rx_rejected, s_ph->tx_datagrams, dtx, drx, drej, dnomem, derr,
             (unsigned)s_loops, (unsigned)s_slow_loops);
}
