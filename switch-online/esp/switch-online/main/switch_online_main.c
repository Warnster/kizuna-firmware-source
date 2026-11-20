/* switch-online: the Switch side of the online GBA<->Switch trade, as its own mode.
 *
 *   Switch <-LDN-> C3 (trade brain: Pia <-> GBA slots, rfu1_bridge: RFU1 parent) <-BLE-> phone <-WSS-> relay
 *     (LINK stratum, family gba-rfu) <-> GBA side (another adapter + GBA)
 *
 * The RP2040 the bridge talks to is the REMOTE one, on the GBA side; its frames arrive here as GB-Link frames
 * (channel + <=64 bytes). This board's own RP2040 is never opened (its telemetry once mixed in).
 *
 * Transports for those frames (gblink host link, gbl_set_host_tx):
 *   - Bluetooth (the product): kz_mode_ble data characteristic ...-0004. Board -> phone notifications and phone ->
 *     board writes carry BUNDLES of messages [ch u8][len u8][payload len], at most SO_BUNDLE_MAX bytes, a message is
 *     never split. Board -> phone: trailing zero bytes of a channel-1 frame are trimmed; the phone pads every
 *     channel-1 frame back to 64 bytes for the relay. Phone -> board: frames exactly as the relay delivered them
 *     (channel-1 data = 64 bytes; a shorter channel-1 frame is telemetry, e.g. the 0x0E "GBA past login" report).
 *   - USB console (the bench): "@GB <ch> <hex>" lines out, {"op":"gb","ch":N,"hex":"..."} in, used whenever no
 *     phone is connected. A PC script can play the phone's role.
 *
 * Control: the same JSON ops on the USB console and on the BLE control characteristic (...-0003): keys_set /
 * keys_clear (RAM only), rfu_host (start the host link once the far end is there), start / select / stop, status,
 * heap. Status, OTA trigger and the signed image push are kz_mode_ble's (category "switch-online"). */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#include "esp_event.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_coexist.h"
#include "esp_wifi.h"
#include "esp_wifi_default.h"
#include "nvs_flash.h"
#include "ldn_join.h"
#include "ldn_udp.h"
#include "ldn_brain.h"
#include "gba_bridge.h"
#include "trade_session.h"
#include "gba_lead.h"       /* the GBA leads, the Switch joins */
#include "ldn_ap.h"
#include "rfu1_bridge.h"
#include "gblink.h"
#include "kz_mode_ble.h"
#include "ota_update.h"

#define SO_MODE        "switch-online"     /* category: the app's mode id, the manifest category, the push check */
#define SO_BLE_NAME    "KIZUNA_SO"
#define SO_BUNDLE_MAX  240                 /* one BLE notification (MTU 256 - 3 ATT header, rounded down) */
#define SO_FLUSH_AT    200                 /* send a bundle once it holds this much */
#define SO_FLUSH_US    10000               /* ... or this long after its first message */
#define SO_TXQ_BYTES   4096                /* board -> phone backlog (messages), drop-oldest never: refuse new */

static const char *TAG = "switch_online";
static ldn_keys s_keys;
static bool     s_have_keys;

/* ---------------------------------------------------------------- output */
static void emit(const char *json) { printf("%s\n", json); kz_mode_ble_notify(json); }
static void emit_error(const char *msg)
{
    char b[160];
    snprintf(b, sizeof b, "{\"ev\":\"error\",\"message\":\"%s\"}", msg);
    emit(b);
}

/* ---------------------------------------------------------------- frames to the phone (or the bench console) */
static SemaphoreHandle_t s_txq_lock;
static uint8_t  s_txq[SO_TXQ_BYTES];
static volatile bool     s_fast_reply = true;   /* {"op":"fast","on":0|1}, reply between ticks */
static volatile uint32_t s_fast_wakes;
static size_t   s_txq_len;                 /* bytes of whole messages waiting */
static int64_t  s_txq_first_us;            /* when the oldest waiting byte arrived */
static uint32_t s_ble_msgs, s_ble_bundles, s_ble_refused, s_ble_overflow, s_txq_hw;
static TaskHandle_t s_tx_task;
static bool     s_ble_ok;                  /* Bluetooth stack up (started at boot; {"op":"ble"} stops/restarts it) */

static void console_frame(uint8_t ch, const uint8_t *p, size_t n)
{
    /* byte-for-byte the gblink console format  */
    char line[16 + 2 * GBL_MAX_PAYLOAD];
    int k = snprintf(line, sizeof line, "@GB %u ", ch);
    for (size_t i = 0; i < n; i++) k += snprintf(line + k, sizeof line - k, "%02x", p[i]);
    line[k++] = '\n';
    fwrite(line, 1, (size_t)k, stdout); fflush(stdout);
}

static void host_tx(uint8_t ch, const uint8_t *p, size_t n)
{
    if (!kz_mode_ble_connected()) { console_frame(ch, p, n); return; }
    if (ch == GBL_CH_DATA) while (n && p[n - 1] == 0) n--;   /* the phone pads channel 1 back to 64 */
    if (n > GBL_MAX_PAYLOAD) n = GBL_MAX_PAYLOAD;
    xSemaphoreTake(s_txq_lock, portMAX_DELAY);
    if (s_txq_len + 2 + n > sizeof s_txq) { s_ble_overflow++; xSemaphoreGive(s_txq_lock); return; }
    if (!s_txq_len) s_txq_first_us = esp_timer_get_time();
    s_txq[s_txq_len++] = ch; s_txq[s_txq_len++] = (uint8_t)n;
    memcpy(s_txq + s_txq_len, p, n); s_txq_len += n;
    if (s_txq_len > s_txq_hw) s_txq_hw = (uint32_t)s_txq_len;
    s_ble_msgs++;
    bool kick = s_txq_len >= SO_FLUSH_AT / 2;   /* the sender caps each bundle at the link's MTU */
    xSemaphoreGive(s_txq_lock);
    if (kick && s_tx_task) xTaskNotifyGive(s_tx_task);
}

/* Bundles whole messages from the front of the queue into one notification. A refused notification (mbuf pool
 * empty) is retried with a growing back-off; nothing is dropped (the relay link above it has no loss recovery for
 * the board -> phone hop). */
static void tx_task(void *arg)
{
    (void)arg;
    static uint8_t bundle[SO_BUNDLE_MAX];
    uint32_t backoff_ms = 0;
    for (;;) {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(backoff_ms ? backoff_ms : SO_FLUSH_US / 1000));
        /* One notification carries at most ATT MTU - 3 bytes: iOS often negotiates 185 (182 usable), not our 256. */
        uint16_t itvl = 0, mtu = 0;
        kz_mode_ble_link_info(&itvl, &mtu);
        size_t cap = (mtu >= 23 && mtu - 3u < sizeof bundle) ? (size_t)(mtu - 3) : sizeof bundle;
        size_t flush_at = cap < SO_FLUSH_AT ? cap : SO_FLUSH_AT;
        for (;;) {
            size_t blen = 0;
            xSemaphoreTake(s_txq_lock, portMAX_DELAY);
            bool due = s_txq_len && (s_txq_len >= flush_at || esp_timer_get_time() - s_txq_first_us >= SO_FLUSH_US);
            if (due) {
                size_t off = 0;
                while (off + 2 <= s_txq_len) {
                    size_t m = 2 + s_txq[off + 1];
                    if (blen + m > cap) break;
                    memcpy(bundle + blen, s_txq + off, m); blen += m; off += m;
                }
            }
            xSemaphoreGive(s_txq_lock);
            if (!blen) break;
            if (!kz_mode_ble_connected()) {            /* the phone went away: the bench console takes over */
                xSemaphoreTake(s_txq_lock, portMAX_DELAY); s_txq_len = 0; xSemaphoreGive(s_txq_lock);
                break;
            }
            if (!kz_mode_ble_notify_data(bundle, blen)) {
                s_ble_refused++;
                backoff_ms = backoff_ms ? (backoff_ms + 8 > 60 ? 60 : backoff_ms + 8) : 8;
                break;
            }
            backoff_ms = 0; s_ble_bundles++;
            xSemaphoreTake(s_txq_lock, portMAX_DELAY);  /* consume what was sent */
            memmove(s_txq, s_txq + blen, s_txq_len - blen); s_txq_len -= blen;
            if (s_txq_len) s_txq_first_us = esp_timer_get_time() - SO_FLUSH_US;   /* rest is overdue: send now */
            xSemaphoreGive(s_txq_lock);
        }
    }
}

/* Phone -> board: a bundle of [ch][len][payload] messages, each injected as if the remote RP2040 sent it. */
static uint32_t s_rx_msgs, s_rx_bad;
static void ble_data(const uint8_t *d, size_t len)
{
    size_t off = 0;
    while (off + 2 <= len) {
        uint8_t ch = d[off], n = d[off + 1];
        if (ch > 2 || n > GBL_MAX_PAYLOAD || off + 2 + n > len) { s_rx_bad++; return; }
        if (gba_lead_standin_active()) gba_lead_standin_rx(ch, d + off + 2, n);   /* the relay's cable stream */
        else gbl_host_inject(ch, d + off + 2, n);
        s_rx_msgs++;
        off += 2 + (size_t)n;
    }
}

/* ---------------------------------------------------------------- ops */
static bool hex_field(const char *line, const char *name, uint8_t out[16])
{
    char key[64];
    snprintf(key, sizeof key, "\"%s\":\"", name);
    const char *p = strstr(line, key);
    if (!p) return false;
    p += strlen(key);
    for (int i = 0; i < 16; i++) {
        unsigned b;
        if (!p[2 * i] || !p[2 * i + 1] || sscanf(p + 2 * i, "%2x", &b) != 1) return false;
        out[i] = (uint8_t)b;
    }
    return p[32] == '"';
}
static bool is_op(const char *line, const char *op)
{
    if (!strcmp(line, op)) return true;
    char key[48];
    snprintf(key, sizeof key, "\"op\":\"%s\"", op);
    return strstr(line, key) != NULL;
}
/* how close each task came to its stack limit (bytes never used), to size the stacks from a real run */
static void stack_event(void)
{
    static const char *const names[] = { "brain", "usb", "led", "ble_tx", "rfu_host", "scan", "nimble_host", "tiT", "wifi",
                                         "sys_evt", "esp_timer", "Tmr Svc", "main" };
    char b[260];
    int n = snprintf(b, sizeof b, "{\"ev\":\"stack\"");
    for (size_t i = 0; i < sizeof names / sizeof names[0] && n < (int)sizeof b - 40; i++) {
        TaskHandle_t h = xTaskGetHandle(names[i]);
        if (h) n += snprintf(b + n, sizeof b - n, ",\"%s\":%u", names[i], (unsigned)uxTaskGetStackHighWaterMark(h));
    }
    snprintf(b + n, sizeof b - n, "}");
    emit(b);
}
static void heap_event(const char *tag)
{
    char b[260];
    snprintf(b, sizeof b, "{\"ev\":\"heap\",\"at\":\"%s\",\"free\":%u,\"largest\":%u,\"min\":%u,\"ble\":%d}", tag,
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_8BIT), (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT),
             (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT), kz_mode_ble_connected());
    emit(b);
    if (!strcmp(tag, "op") || !strcmp(tag, "session") || !strcmp(tag, "ready")) stack_event();
}
static void link_event(void)
{
    uint16_t itvl = 0, mtu = 0;
    kz_mode_ble_link_info(&itvl, &mtu);
    char b[260];
    snprintf(b, sizeof b, "{\"ev\":\"blelink\",\"conn\":%d,\"itvl_ms\":%.1f,\"mtu\":%u,\"tx_msgs\":%u,\"bundles\":%u,\"refused\":%u,"
             "\"overflow\":%u,\"txq_hw\":%u,\"rx_msgs\":%u,\"rx_bad\":%u}", kz_mode_ble_connected(), itvl * 1.25, (unsigned)mtu,
             (unsigned)s_ble_msgs, (unsigned)s_ble_bundles, (unsigned)s_ble_refused, (unsigned)s_ble_overflow, (unsigned)s_txq_hw,
             (unsigned)s_rx_msgs, (unsigned)s_rx_bad);
    emit(b);
}

static void rfu_host_task(void *arg)
{
    (void)arg;
    bool ok = rfu1_bridge_start(-1, -1);     /* waits for the far end's GetFirmwareInfo / SetMode replies */
    emit(ok ? "{\"ev\":\"rfu\",\"up\":true,\"host\":true}" : "{\"ev\":\"rfu\",\"up\":false,\"host\":true}");
    vTaskDelete(NULL);
}

static bool handle_op(const char *line)
{
    if (is_op(line, "keys_set")) {
        ldn_keys k;
        if (hex_field(line, "master_key_12", k.master_key_12)
            && hex_field(line, "aes_kek_generation_source", k.aes_kek_generation_source)
            && hex_field(line, "aes_key_generation_source", k.aes_key_generation_source)) {
            s_keys = k; s_have_keys = true;
            trade_session_set_keys(&s_keys);
            emit("{\"ev\":\"keys\",\"loaded\":true}");
        } else emit_error("keys_set needs three 32-character hex keys.");
        return true;
    }
    if (is_op(line, "keys_clear")) {
        trade_session_set_keys(NULL);
        memset(&s_keys, 0, sizeof s_keys); s_have_keys = false;
        emit("{\"ev\":\"keys\",\"loaded\":false}");
        return true;
    }
    if (is_op(line, "rfu_host")) {
        /* online settings: drain at the GBA's 60/s, the 6/1 reliable window */
        ldn_brain_set_up_floor_us(12000);
        ldn_brain_set_window(6, 1);
        xTaskCreate(rfu_host_task, "rfu_host", 4096, NULL, 5, NULL);
        return true;
    }
    if (is_op(line, "standin")) {          /* {"op":"standin"} the far GBA is in a Cable Club; lead the Switch's group for it */
        if (!s_have_keys) { emit_error("Load keys first (keys_set)."); return true; }
        if (!gba_lead_standin_active()) {
            /* The join role's ~34 KB sim reserve goes first, as when a real GBA leads (trade_session.c): the Pia host, the
             * stand-in and the soft-AP's beacons need it (bench: without it the beacon alloc failed and the Wi-Fi
             * driver crashed). */
            ldn_brain_free_all();
            heap_event("standin");
            if (!gba_lead_start_standin(&s_keys, host_tx)) { emit_error("Could not start the stand-in (memory or radio)."); return true; }
        } else gba_lead_standin_resend();
        emit("{\"ev\":\"session\",\"state\":\"standin\"}");
        return true;
    }
    if (is_op(line, "start")) {
        if (!trade_session_arm()) emit_error("Load keys first (keys_set).");
        else emit("{\"ev\":\"session\",\"state\":\"scanning\"}");
        return true;
    }
    if (is_op(line, "select")) { trade_session_select(); emit("{\"ev\":\"session\",\"state\":\"selected\"}"); return true; }
    if (is_op(line, "stop"))   { trade_session_stop();   emit("{\"ev\":\"session\",\"state\":\"stopping\"}"); return true; }
    if (is_op(line, "status")) { char b[700]; trade_session_status(b, sizeof b); emit(b); return true; }
    if (is_op(line, "heap"))   { heap_event("op"); return true; }
    if (is_op(line, "window")) {           /* A/B: {"op":"window","inflight":24,"ceil":2}, send after rfu_host */
        const char *a = strstr(line, "\"inflight\":"), *c = strstr(line, "\"ceil\":");
        int inflight = a ? atoi(a + 11) : 0, ceil = c ? atoi(c + 7) : 1;
        if (inflight <= 0) { emit_error("window needs inflight >= 1"); return true; }
        ldn_brain_set_window(inflight, ceil);
        char o[96]; snprintf(o, sizeof o, "{\"ev\":\"window\",\"inflight\":%d,\"ceil\":%d}", inflight, ceil); emit(o);
        return true;
    }
    if (is_op(line, "floor")) {            /* A/B: {"op":"floor","us":15000} minimum gap between GBA slot takes toward the Switch */
        const char *a = strstr(line, "\"us\":");
        int us = a ? atoi(a + 5) : -1;
        if (us < 0) { emit_error("floor needs us >= 0"); return true; }
        ldn_brain_set_up_floor_us(us);
        char o[64]; snprintf(o, sizeof o, "{\"ev\":\"floor\",\"us\":%d}", us); emit(o);
        return true;
    }
    if (is_op(line, "fast"))   {           /* A/B switch for the between-ticks reply */
        s_fast_reply = strstr(line, "\"on\":0") == NULL;
        char b[80]; snprintf(b, sizeof b, "{\"ev\":\"fast\",\"on\":%d}", s_fast_reply ? 1 : 0); emit(b); return true;
    }
    if (is_op(line, "blelink")) { link_event(); return true; }
    if (is_op(line, "ble")) {              /* A/B: {"op":"ble","on":0} stops Bluetooth for the session (as switch-trade's local
                                            * mode does), {"on":1} brings it back. With Bluetooth up beside the LDN link, the C3's Wi-Fi/BLE
                                            * coexistence anchors a >100 ms period to the Switch's beacon and gives Wi-Fi ~50% of it: the
                                            * Switch's polls then arrive in 100 ms bursts at ~35/s instead of 60/s (every online run so far). */
        bool on = strstr(line, "\"on\":0") == NULL;
        esp_err_t e = ESP_OK;
        if (!on && kz_mode_ble_running()) e = kz_mode_ble_stop();
        else if (on && !kz_mode_ble_running()) e = kz_mode_ble_start(SO_MODE, SO_BLE_NAME);
        s_ble_ok = kz_mode_ble_running();
        char b[128]; snprintf(b, sizeof b, "{\"ev\":\"ble\",\"on\":%d,\"rc\":%d,\"free\":%u,\"largest\":%u}", s_ble_ok ? 1 : 0, (int)e,
                 (unsigned)heap_caps_get_free_size(MALLOC_CAP_8BIT), (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
        emit(b); return true;
    }
    if (is_op(line, "jitter")) {           /* {"op":"jitter","frames":N} replay buffer for the GBA's held keys (default 20 = 333 ms) */
        const char *a = strstr(line, "\"frames\":"); int n = a ? atoi(a + 9) : 0;
        if (n > 0) gba_bridge_set_replay_delay(n);
        int d; gba_bridge_replay_info(&d, NULL, NULL, NULL);
        char b[64]; snprintf(b, sizeof b, "{\"ev\":\"jitter\",\"frames\":%d}", d); emit(b); return true;
    }
    if (is_op(line, "ble_itvl")) {         /* {"op":"ble_itvl","min":48,"max":80,"latency":2} in 1.25 ms units -- ask the phone for a longer
                                            * connection interval so the shared radio spends less time on Bluetooth while LDN runs */
        const char *a = strstr(line, "\"min\":"), *b = strstr(line, "\"max\":"), *c = strstr(line, "\"latency\":");
        int mn = a ? atoi(a + 6) : 0, mx = b ? atoi(b + 6) : 0, lat = c ? atoi(c + 10) : 0;
        if (mn < 6 || mx < mn || mx > 3200) { emit_error("ble_itvl needs 6 <= min <= max <= 3200 (x1.25 ms)."); return true; }
        kz_mode_ble_set_conn_pref((uint16_t)mn, (uint16_t)mx, (uint16_t)lat, 400);
        uint16_t itvl = 0, mtu = 0; kz_mode_ble_link_info(&itvl, &mtu);
        char bb[128]; snprintf(bb, sizeof bb, "{\"ev\":\"ble_itvl\",\"min\":%d,\"max\":%d,\"latency\":%d,\"now_ms\":%.1f}", mn, mx, lat, itvl * 1.25); emit(bb); return true;
    }
    if (is_op(line, "coex")) {             /* A/B: {"op":"coex","prefer":"wifi"|"bt"|"balance"}. kz_mode_ble sets BT while advertising
                                            * and BALANCE on connect, overriding app_main's PREFER_WIFI; this re-asserts a preference mid-session. */
        esp_coex_prefer_t pref = strstr(line, "\"bt\"") ? ESP_COEX_PREFER_BT : strstr(line, "\"balance\"") ? ESP_COEX_PREFER_BALANCE : ESP_COEX_PREFER_WIFI;
        esp_err_t e = esp_coex_preference_set(pref);
        char b[96]; snprintf(b, sizeof b, "{\"ev\":\"coex\",\"prefer\":\"%s\",\"rc\":%d}",
                 pref == ESP_COEX_PREFER_BT ? "bt" : pref == ESP_COEX_PREFER_BALANCE ? "balance" : "wifi", (int)e);
        emit(b); return true;
    }
    if (is_op(line, "gb")) {                         /* bench: a frame from the PC bridge, as if from the remote RP2040 */
        int ch = -1; const char *h = strstr(line, "\"hex\":\""); const char *c = strstr(line, "\"ch\":");
        if (c) ch = atoi(c + 5);
        uint8_t buf[GBL_MAX_PAYLOAD]; size_t n = 0;
        if (h) for (h += 7; h[0] && h[1] && h[0] != '"' && n < sizeof buf; h += 2) {
            unsigned b; if (sscanf(h, "%2x", &b) != 1) break; buf[n++] = (uint8_t)b;
        }
        if (ch >= 0 && ch <= 2) gbl_host_inject((uint8_t)ch, buf, n);
        return true;
    }
    return false;
}
static bool ble_op(const char *json, size_t len)
{
    char line[400];
    if (len >= sizeof line) return false;
    memcpy(line, json, len); line[len] = 0;
    return handle_op(line);
}
/* A live session refuses OTA installs/pushes (kz_mode_ble asks before restarting the adapter). */
static bool session_busy(void) { return ldn_brain_active() || gba_lead_active(); }

/* ---------------------------------------------------------------- USB console (bench) */
static void usb_task(void *arg)
{
    (void)arg;
    usb_serial_jtag_driver_config_t cfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    cfg.rx_buffer_size = 4096;          /* the PC bridge streams RP2040 frames in */
    cfg.tx_buffer_size = 4096;
    if (usb_serial_jtag_driver_install(&cfg) != ESP_OK) { vTaskDelete(NULL); return; }
    usb_serial_jtag_vfs_use_driver();
    static char line[400];
    static uint8_t chunk[128];
    size_t n = 0;
    for (;;) {
        int got = usb_serial_jtag_read_bytes(chunk, sizeof chunk, portMAX_DELAY);
        for (int i = 0; i < got; i++) {
            char c = (char)chunk[i];
            if (c == '\r') continue;
            if (c != '\n') { if (n < sizeof line - 1) line[n++] = c; continue; }
            line[n] = 0;
            if (n && !handle_op(line)) emit_error("Unknown op.");
            n = 0;
        }
    }
}

/* ---------------------------------------------------------------- status LED (this board's own RP2040, GP16) */
/* Blue = switch-online, so the two boards of an online trade can be told apart (the GBA side keeps GB-Link's own
 * colours). Never a solid colour (stock GB-Link uses only solid ones, see switch-trade's LED notes):
 *   blue breathing      Bluetooth advertising, no phone
 *   blue double blink   phone connected
 *   green, short off    a Switch session is running
 *   red fast blink      Bluetooth did not start
 * The local RP2040 has no GBA in this mode, so LED writes never compete with a timing-critical link. */
#define PIN_LOCAL_RP_TX 5    /* C3 GPIO5 -> RP2040 GP9 (pin map) */
#define PIN_LOCAL_RP_RX 4    /* C3 GPIO4 <- RP2040 GP8 */
static void led_task(void *arg)
{
    (void)arg;
    if (!gbl_init_local_tx(PIN_LOCAL_RP_TX, PIN_LOCAL_RP_RX)) { ESP_LOGW(TAG, "local RP2040 UART: no LED"); vTaskDelete(NULL); return; }
    uint8_t lr = 1, lg = 1, lb = 1;
    int64_t last_refresh = 0;
    for (;;) {
        int64_t now = esp_timer_get_time();
        uint8_t r = 0, g = 0, b = 0;
        if (!s_ble_ok) { if (((now / 125000) & 1) == 0) r = 40; }
        else if (ldn_brain_active() || gba_lead_active()) { if (now % 2000000 >= 150000) g = 40; }
        else if (kz_mode_ble_connected()) { int64_t ph = now % 2000000; if (!(ph < 120000 || (ph >= 240000 && ph < 360000))) b = 40; }
        else { float x = 0.5f - 0.5f * cosf(2.0f * 3.14159f * (float)(now % 2000000) / 2000000.0f); b = (uint8_t)(3 + 32 * x); }
        bool refresh = now - last_refresh > 2000000;     /* GB-Link re-applies its own colour on some events */
        if (refresh || r != lr || g != lg || b != lb) {
            gbl_local_set_led(r, g, b, true);
            lr = r; lg = g; lb = b;
            if (refresh) last_refresh = now;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/* ---------------------------------------------------------------- the brain's clock (switch-trade's, unchanged) */
#define TICK_PERIOD_US  16667
#define FAST_POLL_US    3000          /* socket look-in interval between ticks */
#define TICK_MIN_GAP_US 15000
static SemaphoreHandle_t   s_tick_sem;
static esp_timer_handle_t  s_tick_timer;
static void tick_timer_cb(void *arg) { (void)arg; xSemaphoreGive(s_tick_sem); }

static void brain_task(void *arg)
{
    (void)arg;
    s_tick_sem = xSemaphoreCreateBinary();
    const esp_timer_create_args_t ta = { .callback = tick_timer_cb, .name = "vblank", .dispatch_method = ESP_TIMER_TASK };
    ESP_ERROR_CHECK(esp_timer_create(&ta, &s_tick_timer));
    int64_t  sched = esp_timer_get_time(), last_tick = 0, last_report = esp_timer_get_time();
    uint32_t missed_total = 0, missed_5s = 0, iter_max_us = 0, iters = 0, late_5s = 0;
    bool was_active = false;
    for (;;) {
        int64_t now = esp_timer_get_time();
        int64_t next = sched;
        if (last_tick && next < last_tick + TICK_MIN_GAP_US) next = last_tick + TICK_MIN_GAP_US;
        while (next > now) {
            /* while a session runs, wake every FAST_POLL_US to read the socket and answer at once, instead of
             * sleeping to the next 60 Hz tick (which put up to 17 ms on each leg of every Switch poll/ack round trip). */
            int64_t wait = next - now;
            bool fast = s_fast_reply && ldn_brain_active();
            if (fast && wait > FAST_POLL_US) wait = FAST_POLL_US;
            ESP_ERROR_CHECK(esp_timer_start_once(s_tick_timer, (uint64_t)wait));
            xSemaphoreTake(s_tick_sem, portMAX_DELAY);
            now = esp_timer_get_time();
            if (fast && now < next) { ldn_udp_poll(); ldn_brain_reply_now(); s_fast_wakes++; now = esp_timer_get_time(); }
        }
        int missed = 0;
        while (sched + TICK_PERIOD_US <= now) { sched += TICK_PERIOD_US; missed++; }
        sched += TICK_PERIOD_US;
        if (missed > 0) {
            if (missed > 60) missed = 60;
            missed_total += (uint32_t)missed; missed_5s += (uint32_t)missed;
            if (ldn_brain_active()) ldn_brain_skip_ticks(missed);
        }
        if (last_tick && now - last_tick < TICK_MIN_GAP_US) late_5s++;
        last_tick = now;
        rfu1_bridge_poll();
        if (ldn_ap_running()) ldn_ap_poll();         /* the hosted room -- its LDN advert (what the Switch lists), joins */
        gba_lead_poll();                             /* the GBA leads, the Switch joins (no-op otherwise) */
        /* online: while the GBA leads, Wi-Fi gets the radio. The Pia host sends one GBA frame
         * per Switch datagram, and with Bluetooth sharing the radio evenly the Switch's rate fell to 18-33/s against the
         * GBA's ~46/s: the queue filled with data frames and the session failed. kz_mode_ble sets BALANCE when the phone
         * connects, so this is applied when the session starts; the phone's link (a few KB/s) moves to 60-100 ms. The board
         * restarts after the session, which restores both. */
        /* Bench: OFF. Online the GBA's own frames reach this board over that Bluetooth link (~4 KB/s), and with
         * Wi-Fi preferred at 60-100 ms they arrived with 64-byte chunks missing (the GBA side traced them in order; here block
         * fragments came out of order, spliced). The Switch never got the GBA's block. Radio stays BALANCE, interval as is. */
#ifdef KZ_LEAD_RADIO_WIFI
        static bool s_lead_radio;
        if (gba_lead_active() && !s_lead_radio) {
            s_lead_radio = true;
            esp_err_t ce = esp_coex_preference_set(ESP_COEX_PREFER_WIFI);
            kz_mode_ble_set_conn_pref(48, 80, 0, 400);   /* 1.25 ms units: 60-100 ms, no slave latency, 4 s supervision */
            printf("  [lead] radio: Wi-Fi preferred (rc=%d), Bluetooth interval 60-100 ms requested\n", (int)ce);
        }
#endif
        bool active = ldn_brain_active();
        if (active) { ldn_udp_poll(); ldn_brain_tick(); }
        if (active != was_active) { was_active = active; heap_event(active ? "session-start" : "session-end"); }
        uint32_t dt = (uint32_t)(esp_timer_get_time() - now);
        if (dt > iter_max_us) iter_max_us = dt;
        iters++;
        if (now - last_report > 5000000) {
            last_report = now;
            if (active) {
                printf("  [loop] iters=%u max_iter=%uus missed_ticks=%u (5s) / %u (total) gap_violations=%u fast=%d wakes=%u\n",
                       (unsigned)iters, (unsigned)iter_max_us, (unsigned)missed_5s, (unsigned)missed_total, (unsigned)late_5s,
                       s_fast_reply ? 1 : 0, (unsigned)s_fast_wakes);
                heap_event("session");                 /* the number that decides the single-C3 question */
                if (kz_mode_ble_connected()) link_event();
            }
            iters = 0; iter_max_us = 0; missed_5s = 0; late_5s = 0;
        }
    }
}

void app_main(void)
{
    heap_event("boot");
    esp_err_t nv = nvs_flash_init();
    if (nv == ESP_ERR_NVS_NO_FREE_PAGES || nv == ESP_ERR_NVS_NEW_VERSION_FOUND) { nvs_flash_erase(); nvs_flash_init(); }
    /* the banner a host script keys its re-init on (online re-arm = reboot) */
    printf("\n==== kizuna switch-trade (%s): switch-online, Switch LDN + remote GBA over BLE/USB ====\n", CONFIG_IDF_TARGET);
    ota_update_boot();                      /* a queued install / boot action runs first (no Wi-Fi state yet) */

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    heap_event("netif");
    esp_netif_inherent_config_t netif_base = ESP_NETIF_INHERENT_DEFAULT_WIFI_STA();
    netif_base.flags &= ~ESP_NETIF_DHCP_CLIENT;              /* the Switch assigns our IP through the LDN membership */
    esp_netif_config_t netif_cfg = ESP_NETIF_DEFAULT_WIFI_STA();
    netif_cfg.base = &netif_base;
    esp_netif_t *sta_netif = esp_netif_new(&netif_cfg);
    ESP_ERROR_CHECK(esp_netif_attach_wifi_station(sta_netif));
    ESP_ERROR_CHECK(esp_wifi_set_default_wifi_sta_handlers());
    ldn_udp_init(sta_netif);
    ldn_udp_set_rx(ldn_brain_on_datagram);
    ldn_join_set_netif(sta_netif);

    wifi_init_config_t wic = WIFI_INIT_CONFIG_DEFAULT();
    heap_event("pre-wifi");
    ESP_ERROR_CHECK(esp_wifi_init(&wic));
    heap_event("wifi-init");
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G);
    esp_wifi_set_ps(WIFI_PS_NONE);
    heap_event("wifi-up");

    trade_session_start();
    /* the ~25 KB sim state, taken now while the heap is whole: Bluetooth stays up for the whole session here,
     * so the session must not have to find that block later between NimBLE's allocations */
    if (!ldn_brain_prealloc_sim()) ESP_LOGE(TAG, "could not reserve the sim state at boot");
    heap_event("sim-reserved");
    gbl_init_host();                        /* the remote RP2040's frames: in via gbl_host_inject, out via host_tx */
    s_txq_lock = xSemaphoreCreateMutex();
    gbl_set_host_tx(host_tx);
    xTaskCreate(tx_task, "ble_tx", 3072, NULL, 6, &s_tx_task);

    kz_mode_ble_set_handlers(ble_op, ble_data);
    kz_mode_ble_set_busy_fn(session_busy);
    s_ble_ok = kz_mode_ble_start(SO_MODE, SO_BLE_NAME) == ESP_OK;
    if (!s_ble_ok) ESP_LOGW(TAG, "Bluetooth did not start");
    /* one radio. The Switch link is a lockstep loop (our child frame needs the Switch's poll, its next poll needs
     * our frame): every ms Bluetooth takes from Wi-Fi stretches that loop. On the bench it ran at ~36 polls/s
     * instead of 60, so two thirds of the GBA's walking frames never reached the Switch. Bluetooth retries cheaply, the
     * Switch link does not: give Wi-Fi the radio. */
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_coex_preference_set(ESP_COEX_PREFER_WIFI));
    heap_event("ble-up");

    xTaskCreate(brain_task, "brain", 16384, NULL, 5, NULL);
    xTaskCreate(usb_task, "usb", 4096, NULL, 4, NULL);
    xTaskCreate(led_task, "led", 2560, NULL, 2, NULL);       /* below the radio and the brain */
    heap_event("ready");
    ESP_LOGI(TAG, "ready: keys_set, rfu_host, start");
    vTaskDelay(pdMS_TO_TICKS(5000));
    ota_update_confirm_boot();              /* booted and running: keep this image (rollback otherwise) */
    for (;;) vTaskDelay(pdMS_TO_TICKS(1000));
}
