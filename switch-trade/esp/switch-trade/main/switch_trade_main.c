/* switch_trade_main.c — Switch Trade mode on the two-chip board.
 *
 * The ESP (C3 in the product, C6 on the bench) joins the Switch's FireRed/LeafGreen LDN room and runs the
 * virtual RFU child (cport sim.c) toward it. The GBA's side of the trade arrives over the RP2040 (GB-Link
 * wireless-adapter mode, RFU1 frames) in M2/M3. M1 = Switch-only: the sim's stub engine, driven from the
 * USB Serial/JTAG console, one JSON object (or a bare word) per line:
 *
 *   {"op":"keys_set","master_key_12":hex32,"aes_kek_generation_source":hex32,"aes_key_generation_source":hex32}
 *   {"op":"keys_clear"}     also erases the stored copy;  {"op":"keys_status"} -> {"ev":"keys","loaded","stored"} (presence only)
 *   keys_set stores the keys on the adapter (NVS, write-only) and they are loaded again at every boot.
 *   {"op":"start"}     discover FRLG rooms on 1/6/11, associate + seat silently (= the GBA is browsing)
 *   {"op":"select"}    send the join to the Switch (= the GBA picked the room) -> "Kizuna wants to join"
 *   {"op":"stop"}      end the session, back to scanning
 *   {"op":"status"}    one {"ev":"status",...} line
 *   {"op":"heap"}
 *   {"op":"rfu"}       re-enter GB-Link mode 0x07 on the RP2040 (after an RP2040 reset); {"op":"rfu_stop"}
 *   {"op":"linkv","v":3|5}          c3rp R2: select the link-port rail (GB-Link cmd 0x40/0x41); boots at 3.3 V
 *   {"op":"led","r":..,"g":..,"b":..} set the RP2040 status LED (GB-Link cmd 0x42); {"op":"led","off":1}
 * Events are printed as {"ev":...} lines . Keys are never printed or logged; they are
 * stored on the adapter (kz_keys, write-only) so it works after a power cycle without the phone. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#if CONFIG_ESP_CONSOLE_UART
#include "driver/uart.h"
#include "driver/uart_vfs.h"
#else
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#endif
#include "esp_event.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include <sys/socket.h>
#include <netdb.h>
#include <unistd.h>
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_wifi_default.h"
#include "nvs_flash.h"
#include "ldn_join.h"
#include "ldn_udp.h"
#include "ldn_brain.h"
#include "gba_bridge.h"
#include "trade_session.h"
#include "relay_engine.h"   /* T1: the GBA side as a USB byte pipe */
#include "kz_mode_ble.h"     /* T3: the same ops and the slot stream over Bluetooth */
#include "ldn_ap.h"          /* the AP+STA LDN-host feasibility test (S3) */
#include "trade_host_session.h" /* host the Direct Corner trade and trade with the Switch */
#include "ldn_host.h"
#include "rfu1_bridge.h"
#include "gblink.h"
#include "ota_update.h"   /* Bluetooth firmware push + status "hw" */
#include "ota_push.h"
#include <math.h>
#include "pia_zstd.h"
#include "kz_keys.h"      /* the user's keys survive power cycles and updates (write-only) */
#include "freertos/queue.h"
#ifdef KZ_GBA_LEAD
#include "gba_lead.h"     /* the GBA leads, the Switch joins */
#endif

/* GB UART to the RP2040 (GB-Link bridge GP9 = its RX, GP8 = its TX). Bench XIAO C6: GPIO16/17;
 * c3rp board: C3 GPIO5 TX / GPIO4 RX. */
#if CONFIG_IDF_TARGET_ESP32C3
#define PIN_GBL_TX 5
#define PIN_GBL_RX 4
#elif CONFIG_IDF_TARGET_ESP32S3
/* No RP2040 on the S3 bench board for this test (AP+STA LDN-host feasibility, Switch-side
 * only). Any free, non-strapping GPIO pair is fine -- rfu1_bridge_start() just reports "not answering"
 * and the console-driven start/select path drives the session, exactly as the C3 bench runs do when
 * no Pico is attached. Revisit once an S3-based board design exists. */
#define PIN_GBL_TX 17
#define PIN_GBL_RX 18
#else
#define PIN_GBL_TX 16
#define PIN_GBL_RX 17
#endif

static const char *TAG = "switch_trade";
static ldn_keys s_keys;
static bool     s_have_keys;
static bool     s_ble_started;

/* the adapter hardware this image is for, reported as "hw" in the Bluetooth status so the app never offers a
 * single-chip image to a two-chip board. The C3 build is the c3rp product board. */
#if CONFIG_IDF_TARGET_ESP32C3
#define KZ_HW_TAG "c3rp"
#else
#define KZ_HW_TAG ""
#endif

/* the same app built for Ruby/Sapphire (-DKZ_GBA_LINK=cable) is its own mode for the app and the firmware
 * manifest: a different mode id (the manifest category) and advertised name, and the GBA side is the link cable. */
#if KZ_GBA_LINK_CABLE
#define KZ_MODE_ID  "switch-trade-rs"
#define KZ_BLE_NAME "KIZUNA_RS"
#define KZ_GBA_SIDE "GBA over the RP2040 link cable (Ruby/Sapphire)"
#else
#define KZ_MODE_ID  "switch-trade"
#define KZ_BLE_NAME "KIZUNA_ST"
#define KZ_GBA_SIDE "GBA over RP2040 RFU1"
#endif

/* Radio hand-over for the LOCAL GBA<->Switch trade. The C3 cannot hold NimBLE (~52 KB heap) and a live local
 * session at once, so:
 *   IDLE     Bluetooth up (app: status, keys, firmware push, Wi-Fi setup), no session; Wi-Fi stays started (the host/relay/
 *            online ops expect it) but the sniffer is off and the zstd decoder workspace is released.
 *   SESSION  Bluetooth fully stopped (kz_mode_ble_stop), decoder workspace reserved, trade_session armed.
 * The online (gbl_host_active) and relay paths keep Bluetooth up and are not handed over. */
typedef enum { RADIO_IDLE = 0, RADIO_SESSION = 1 } radio_state_t;
static QueueHandle_t s_radio_q;
static volatile radio_state_t s_radio_state = RADIO_IDLE;
static volatile bool s_console_hold;      /* console/app "start": keep the session even with no GBA (bench) */
static volatile bool s_led_ble_failed;    /* status LED: Bluetooth did not start */
static volatile bool s_op_from_ble;       /* the op being handled arrived over Bluetooth (the app), not the USB bench console */
static volatile int64_t s_session_since_us;
/* A Switch session is running: the join role's brain, or the GBA leading and the board hosting the room. */
static bool session_running(void)
{
#ifdef KZ_GBA_LEAD
    if (gba_lead_active()) return true;
#endif
    return ldn_brain_active();
}
static bool local_trade_path(void) { return !gbl_host_active() && !relay_engine_installed(); }
static void radio_request(radio_state_t want) { if (s_radio_q) xQueueSend(s_radio_q, &want, 0); }
static void radio_cb(bool session) { if (local_trade_path()) radio_request(session ? RADIO_SESSION : RADIO_IDLE); }
static void heap_map(const char *tag);

/* Wi-Fi up/down for the radio hand-over. Up = started, legacy 11b/g, power save off (the trade's proven settings). */
static void wifi_up(void)
{
    esp_err_t e = esp_wifi_start();
    if (e != ESP_OK) ESP_LOGW(TAG, "esp_wifi_start: %s", esp_err_to_name(e));
    /* Legacy 11b/g only: the Switch's LDN AP is a legacy-rate network (tr-0.14). */
    esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G);
    esp_wifi_set_ps(WIFI_PS_NONE);
}
static void wifi_down(void)
{
    esp_err_t e = esp_wifi_stop();
    if (e != ESP_OK && e != ESP_ERR_WIFI_NOT_INIT) ESP_LOGW(TAG, "esp_wifi_stop: %s", esp_err_to_name(e));
}
/* BLE relay round-trip test (see ping_task below) */
static volatile bool     s_ping_on;
static volatile int      s_ping_hz = 10;
static TaskHandle_t      s_ping_task;
static portMUX_TYPE      s_ping_lock = portMUX_INITIALIZER_UNLOCKED;
static uint32_t          s_pw_sent, s_pw_n, s_pw_drop, s_pw_min = UINT32_MAX, s_pw_max;
static uint64_t          s_pw_sum;
static uint32_t          s_ping_total_sent, s_ping_total_n;
static void ping_task(void *arg);

static void emit(const char *json) { printf("%s\n", json); kz_mode_ble_notify(json); }
static void host_member_cb(const ldn_ap_member *m, bool joined)
{
    char b[160];
    snprintf(b, sizeof b, "{\"ev\":\"member\",\"joined\":%d,\"platform\":%d,\"name\":\"%s\",\"mac\":\"%02x:%02x:%02x:%02x:%02x:%02x\"}",
             joined, m->platform, m->name, m->mac[0],m->mac[1],m->mac[2],m->mac[3],m->mac[4],m->mac[5]);
    emit(b);
}
static void emit_error(const char *msg)
{
    char b[160];
    snprintf(b, sizeof b, "{\"ev\":\"error\",\"message\":\"%s\"}", msg);
    emit(b);
}

/* "name":"<32 hex>" out of a JSON line, without pulling in a JSON library (the app path uses kz_mode_ble
 * + cJSON in M4; the bench format is identical to the distributor's). */
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

/* "name":"<text>" (no escapes) out of a JSON line; false if absent or too long. */
static bool str_field(const char *line, const char *name, char *out, size_t cap)
{
    char key[64];
    snprintf(key, sizeof key, "\"%s\":\"", name);
    const char *p = strstr(line, key);
    if (!p) return false;
    p += strlen(key);
    const char *e = strchr(p, '"');
    if (!e || (size_t)(e - p) >= cap) return false;
    memcpy(out, p, (size_t)(e - p)); out[e - p] = 0;
    return true;
}

/* prove the uplink carries traffic while the room is up. DNS + TCP + HTTP/1.0 GET. */
static void uplink_probe(void)
{
    char b[200];
    int64_t t0 = esp_timer_get_time();
    struct addrinfo hints = { .ai_family = AF_INET, .ai_socktype = SOCK_STREAM }, *res = NULL;
    if (getaddrinfo("example.com", "80", &hints, &res) != 0 || !res) { emit("{\"ev\":\"probe\",\"ok\":0,\"stage\":\"dns\"}"); return; }
    int64_t t1 = esp_timer_get_time();
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    struct timeval tv = { .tv_sec = 5 };
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof tv);
    if (fd < 0 || connect(fd, res->ai_addr, res->ai_addrlen) != 0) {
        if (fd >= 0) close(fd);
        freeaddrinfo(res); emit("{\"ev\":\"probe\",\"ok\":0,\"stage\":\"connect\"}"); return;
    }
    freeaddrinfo(res);
    int64_t t2 = esp_timer_get_time();
    static const char req[] = "GET / HTTP/1.0\r\nHost: example.com\r\nConnection: close\r\n\r\n";
    char rx[64] = {0};
    int n = (send(fd, req, sizeof req - 1, 0) > 0) ? recv(fd, rx, sizeof rx - 1, 0) : -1;
    close(fd);
    char *nl = strpbrk(rx, "\r\n"); if (nl) *nl = 0;
    snprintf(b, sizeof b, "{\"ev\":\"probe\",\"ok\":%d,\"dns_ms\":%d,\"connect_ms\":%d,\"first_ms\":%d,\"status\":\"%s\"}",
             n > 0, (int)((t1 - t0) / 1000), (int)((t2 - t1) / 1000), (int)((esp_timer_get_time() - t2) / 1000), rx);
    emit(b);
}

static bool int_field(const char *line, const char *name, long *out)
{
    char key[64];
    snprintf(key, sizeof key, "\"%s\":", name);
    const char *p = strstr(line, key);
    if (!p) return false;
    char *end;
    long v = strtol(p + strlen(key), &end, 0);
    if (end == p + strlen(key)) return false;
    *out = v;
    return true;
}

/* "hex":"<2n hex>" into out[n]. */
static bool hex_bytes(const char *line, const char *name, uint8_t *out, int n)
{
    char key[64];
    snprintf(key, sizeof key, "\"%s\":\"", name);
    const char *p = strstr(line, key);
    if (!p) return false;
    p += strlen(key);
    for (int i = 0; i < n; i++) {
        unsigned b;
        if (!p[2 * i] || !p[2 * i + 1] || sscanf(p + 2 * i, "%2x", &b) != 1) return false;
        out[i] = (uint8_t)b;
    }
    return p[2 * n] == '"';
}

static bool is_op(const char *line, const char *op)
{
    if (!strcmp(line, op)) return true;            /* bare word for a human on a terminal */
    char key[48];
    snprintf(key, sizeof key, "\"op\":\"%s\"", op);
    return strstr(line, key) != NULL;
}

static void heap_event(void)
{
    char b[220];
    const uint32_t in = MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT;
    snprintf(b, sizeof b, "{\"ev\":\"heap\",\"free\":%u,\"largest\":%u,\"min\":%u,\"int_free\":%u,\"int_largest\":%u,\"int_min\":%u}",
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_8BIT),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT),
             (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT),
             (unsigned)heap_caps_get_free_size(in), (unsigned)heap_caps_get_largest_free_block(in),
             (unsigned)heap_caps_get_minimum_free_size(in));
    emit(b);
}

static void rfu_host_task(void *arg)
{
    (void)arg;
    bool ok = rfu1_bridge_start(-1, -1);
    emit(ok ? "{\"ev\":\"rfu\",\"up\":true,\"host\":true}" : "{\"ev\":\"rfu\",\"up\":false,\"host\":true}");
    vTaskDelete(NULL);
}

/* Give the BLE write response time to actually reach the app before the restart drops the link
 * (save_ble.c's restart_soon_task does the same, 600 ms, for the same reason). */
static void restart_soon_task(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(600));
    esp_restart();
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
            bool stored = kz_keys_save((const uint8_t *)&k) == ESP_OK;   /* kept for the next power-on */
            memset(&k, 0, sizeof k);
            emit(stored ? "{\"ev\":\"keys\",\"loaded\":true,\"stored\":true}" : "{\"ev\":\"keys\",\"loaded\":true,\"stored\":false}");
        } else emit_error("keys_set needs three 32-character hex keys.");
        return true;
    }
    if (is_op(line, "keys_clear")) {
        trade_session_set_keys(NULL);
        memset(&s_keys, 0, sizeof s_keys); s_have_keys = false;
        kz_keys_clear();
        emit("{\"ev\":\"keys\",\"loaded\":false,\"stored\":false}");
        return true;
    }
    if (is_op(line, "keys_status")) {               /* presence only: the values never leave the adapter */
        char o[64]; snprintf(o, sizeof o, "{\"ev\":\"keys\",\"loaded\":%s,\"stored\":%s}",
                             s_have_keys ? "true" : "false", kz_keys_present() ? "true" : "false");
        emit(o);
        return true;
    }
    if (is_op(line, "start")) {
        if (!s_have_keys) { emit_error("Load keys first (keys_set)."); return true; }
        /* Only a USB bench start holds the session open with no GBA; the app's start follows the GBA like a login does. */
        if (local_trade_path() && s_radio_q) { s_console_hold = !s_op_from_ble; radio_request(RADIO_SESSION); }   /* Bluetooth off, then arm */
        else { wifi_up(); if (!trade_session_arm()) { emit_error("Load keys first (keys_set)."); return true; } }
        emit("{\"ev\":\"session\",\"state\":\"scanning\"}");
        return true;
    }
    if (is_op(line, "idle")) {                       /* end the local session, Bluetooth back up */
        s_console_hold = false;
        radio_request(RADIO_IDLE);
        emit("{\"ev\":\"session\",\"state\":\"idle\"}");
        return true;
    }
    if (is_op(line, "radio")) {
        char o[112]; snprintf(o, sizeof o, "{\"ev\":\"radio\",\"state\":\"%s\",\"ble\":%d,\"armed\":%d,\"hold\":%d,\"local\":%d}",
                 s_radio_state == RADIO_SESSION ? "session" : "idle", kz_mode_ble_running(), trade_session_armed(), s_console_hold,
                 local_trade_path());
        emit(o);
        return true;
    }
    if (is_op(line, "heapmap")) { heap_map("op"); return true; }
    if (is_op(line, "wifi")) {                       /* bench: {"op":"wifi","on":0|1} -- Wi-Fi coexistence diagnosis */
        bool on = strstr(line, "\"on\":1") != NULL;
        if (on) wifi_up(); else wifi_down();
        char o[48]; snprintf(o, sizeof o, "{\"ev\":\"wifi\",\"on\":%d}", on); emit(o);
        return true;
    }
    if (is_op(line, "select")) { trade_session_select(); emit("{\"ev\":\"session\",\"state\":\"selected\"}"); return true; }
    if (is_op(line, "stop"))   { trade_session_stop();   emit("{\"ev\":\"session\",\"state\":\"stopping\"}"); return true; }
    if (is_op(line, "status")) { char b[1000]; trade_session_status(b, sizeof b); emit(b); return true; }
    if (is_op(line, "heap"))   { heap_event(); return true; }
    if (is_op(line, "window")) {                     /* {"op":"window","inflight":6,"ceil":1}; local default 24/2 */
        const char *a = strstr(line, "\"inflight\":"), *b = strstr(line, "\"ceil\":");
        int inflight = a ? atoi(a + 11) : 0, ceil = b ? atoi(b + 7) : 2;
        if (inflight <= 0) { emit_error("window needs inflight >= 1"); return true; }
        ldn_brain_set_window(inflight, ceil);
        char o[96]; snprintf(o, sizeof o, "{\"ev\":\"window\",\"inflight\":%d,\"ceil\":%d}", inflight, ceil); emit(o);
        return true;
    }
    if (is_op(line, "host_trade")) {         /* AP-only, advertise a TRADE room, watch for a join */
        if (!s_have_keys) { emit_error("Load keys first."); return true; }
        if (ldn_ap_running()) { emit_error("Already hosting."); return true; }
        static ldn_frlg_room room; static uint8_t app_data[LDN_FRLG_APP_DATA_LEN];
        ldn_frlg_room_gift(&room, LDN_FRLG_ACTIVITY_TRADE, "KIZUNA", 0x4b5a, 0x1234);
        long n = ldn_frlg_app_data_build(&room, app_data, sizeof app_data);
        if (n < 0) { emit_error("app_data build failed."); return true; }
        static ldn_ap_config cfg;
        static char sta_ssid[33], sta_pass[65];
        sta_ssid[0] = sta_pass[0] = 0;
        str_field(line, "sta_ssid", sta_ssid, sizeof sta_ssid);
        str_field(line, "sta_pass", sta_pass, sizeof sta_pass);
        cfg.sta_ssid = sta_ssid[0] ? sta_ssid : NULL; cfg.sta_pass = sta_pass;
        ldn_join_set_passthrough(cfg.sta_ssid != NULL);   /* the join role's EAPOL-drop would break the uplink's WPA2 */
        cfg.channel = 6; cfg.host_name = "KIZUNA"; cfg.app_data = app_data; cfg.app_data_len = (size_t)n;
        esp_err_t e = ldn_ap_start(&s_keys, &cfg, host_member_cb);
        emit(e == ESP_OK ? "{\"ev\":\"host\",\"on\":true,\"activity\":\"trade\"}"
                          : "{\"ev\":\"error\",\"message\":\"ldn_ap_start failed\"}");
        return true;
    }
    if (is_op(line, "host_status")) { char b[640]; ldn_ap_uplink_status(b, sizeof b); emit(b); heap_event(); return true; }
    if (is_op(line, "uplink_probe")) { uplink_probe(); return true; }
    if (is_op(line, "wifi_test")) {   /* + restart, check the Kizuna server is reachable, restart back */
        if (s_radio_state == RADIO_SESSION) { emit_error("The adapter is busy with a trade: stop it first."); return true; }
        if (!ota_update_request_test()) { emit_error("Could not start the Wi-Fi test."); return true; }
        emit("{\"ev\":\"wifi\",\"testing\":true}");
        xTaskCreate(restart_soon_task, "wt_rst", 3072, NULL, 4, NULL);
        return true;
    }
    if (is_op(line, "th_mon")) {             /* stage one party member: {"op":"th_mon","i":0,"hex":"<200 hex>"} */
        long i; uint8_t mon[100];
        if (!int_field(line, "i", &i) || !hex_bytes(line, "hex", mon, 100) || !th_session_set_mon((int)i, mon)) {
            emit_error("th_mon needs i (0-5) and 200 hex chars, before th_start.");
            return true;
        }
        char b[64]; snprintf(b, sizeof b, "{\"ev\":\"th_mon\",\"i\":%ld,\"party\":%d}", i, th_session_party_count());
        emit(b);
        return true;
    }
    if (is_op(line, "th_clear")) { th_session_clear_party(); emit("{\"ev\":\"th_mon\",\"party\":0}"); return true; }
    if (is_op(line, "th_start")) {           /* host the trade room and trade (needs keys + staged party) */
        if (!s_have_keys) { emit_error("Load keys first."); return true; }
        if (ldn_ap_running()) { emit_error("Already hosting (host_stop first)."); return true; }
        static char name[16], sta_ssid[33], sta_pass[65];
        long tid = 0x1234, sid = 0x5678, trades = 1, slot = 0;
        name[0] = sta_ssid[0] = sta_pass[0] = 0;
        str_field(line, "name", name, sizeof name);
        str_field(line, "sta_ssid", sta_ssid, sizeof sta_ssid);
        str_field(line, "sta_pass", sta_pass, sizeof sta_pass);
        int_field(line, "tid", &tid); int_field(line, "sid", &sid);
        int_field(line, "trades", &trades); int_field(line, "slot", &slot);
        th_session_cfg c = { .name = name, .tid = (uint16_t)tid, .sid = (uint16_t)sid, .trades = (int)trades, .slot = (int)slot,
                             .sta_ssid = sta_ssid[0] ? sta_ssid : NULL, .sta_pass = sta_pass };
        th_session_start(&s_keys, &c);
        return true;
    }
    if (is_op(line, "th_delay")) {           /* {"op":"th_delay","ms":150,"jitter":20}: one-way delay on all Switch traffic */
        long ms = 0, jit = 0;
        int_field(line, "ms", &ms); int_field(line, "jitter", &jit);
        th_session_set_delay((int)ms, (int)jit);
        char b[96]; snprintf(b, sizeof b, "{\"ev\":\"th_delay\",\"ms\":%ld,\"jitter\":%ld}", ms, jit);
        emit(b);
        return true;
    }
    if (is_op(line, "th_stop"))   { th_session_stop(); return true; }
    if (is_op(line, "th_status")) { char b[400]; th_session_status(b, sizeof b); emit(b); heap_event(); return true; }
    if (is_op(line, "host_stop")) { th_session_stop(); ldn_ap_stop(); ldn_join_set_passthrough(false); emit("{\"ev\":\"host\",\"on\":false}"); return true; }
    if (is_op(line, "relay"))  {                        /* T1: GBA side = USB pipe; install before 'select' */
        if (ldn_brain_active()) { emit_error("relay: stop the session first."); return true; }
        wifi_up();                                   /* idle stops Wi-Fi: the relay session needs it */
        relay_via_t via = strstr(line, "\"via\":\"ble\"") ? RELAY_VIA_BLE : RELAY_VIA_USB;
        relay_engine_install(via);
        emit(via == RELAY_VIA_BLE ? "{\"ev\":\"relay\",\"on\":true,\"via\":\"ble\"}" : "{\"ev\":\"relay\",\"on\":true,\"via\":\"usb\"}");
        return true;
    }
    if (is_op(line, "relay_ping")) {         /* {"op":"relay_ping","on":1,"hz":10} / {"op":"relay_ping","on":0} */
        long on = 1, hz = 10;
        int_field(line, "on", &on); int_field(line, "hz", &hz);
        if (on && !s_ble_started) { emit_error("relay_ping needs Bluetooth: send ble_on first."); return true; }
        if (hz < 1) hz = 1;
        if (hz > 50) hz = 50;
        if (on) {
            portENTER_CRITICAL(&s_ping_lock);
            s_pw_sent = s_pw_n = s_pw_drop = s_pw_max = 0; s_pw_min = UINT32_MAX; s_pw_sum = 0;
            s_ping_total_sent = s_ping_total_n = 0;
            portEXIT_CRITICAL(&s_ping_lock);
            s_ping_hz = (int)hz;
            if (!s_ping_task) xTaskCreate(ping_task, "relay_ping", 3072, NULL, 3, &s_ping_task);
        }
        s_ping_on = on != 0;
        char b[80]; snprintf(b, sizeof b, "{\"ev\":\"relay_ping\",\"on\":%d,\"hz\":%d}", s_ping_on, s_ping_hz);
        emit(b);
        return true;
    }
    if (is_op(line, "ble_status")) {
        uint16_t itvl = 0, mtu = 0;
        if (!s_ble_started) { emit("{\"ev\":\"ble\",\"started\":0}"); return true; }
        kz_mode_ble_link_info(&itvl, &mtu);
        char b[160]; snprintf(b, sizeof b, "{\"ev\":\"ble\",\"started\":%d,\"rc\":%d,\"active\":%d,\"conn\":%d,\"itvl_ms\":%.1f,\"mtu\":%u}", s_ble_started, kz_mode_ble_adv_rc(), kz_mode_ble_adv_active(), kz_mode_ble_connected(), itvl * 1.25, (unsigned)mtu);
        emit(b); return true;
    }
    if (is_op(line, "ble_off")) { kz_mode_ble_set_advertising(false); emit("{\"ev\":\"ble\",\"adv\":false}"); return true; }
    if (is_op(line, "ble_on"))  {
        /* First call brings the whole Bluetooth stack up (measures its cost against the same boot). */
        if (!s_ble_started) { s_ble_started = kz_mode_ble_start(KZ_MODE_ID, KZ_BLE_NAME) == ESP_OK; }
        else kz_mode_ble_set_advertising(true);
        if (!s_ble_started) { emit("{\"ev\":\"error\",\"message\":\"BLE did not start\"}"); return true; }
        vTaskDelay(pdMS_TO_TICKS(300));   /* let NimBLE sync and start advertising */
        { char b[96]; snprintf(b, sizeof b, "{\"ev\":\"ble\",\"adv\":true,\"rc\":%d,\"active\":%d,\"conn\":%d}", kz_mode_ble_adv_rc(), kz_mode_ble_adv_active(), kz_mode_ble_connected()); emit(b); }
        return true;
    }
    if (is_op(line, "slot"))   {                        /* partner -> Switch: {"op":"slot","hex":"<28 hex>"} */
        const char *p = strstr(line, "\"hex\":\"");
        uint8_t sl[14]; unsigned b;
        if (!p) { emit_error("slot needs hex."); return true; }
        p += 7;
        for (int i = 0; i < 14; i++) { if (sscanf(p + 2 * i, "%2x", &b) != 1) { emit_error("slot: bad hex."); return true; } sl[i] = (uint8_t)b; }
        if (!relay_engine_push(sl)) emit_error("slot queue full.");
        return true;
    }
    if (is_op(line, "relay_status")) { char b[200]; relay_engine_status(b, sizeof b); printf("{%s}\n", b); return true; }
    if (is_op(line, "rfu"))    {                        /* (re)enter mode 07 on the RP2040 */
        bool ok = rfu1_bridge_start(PIN_GBL_TX, PIN_GBL_RX);
        emit(ok ? "{\"ev\":\"rfu\",\"up\":true}" : "{\"ev\":\"rfu\",\"up\":false}");
        return true;
    }
    if (is_op(line, "linkv")) {                      /* c3rp R2 link rail: {"op":"linkv","v":5} or 3 (GB-Link cmd 0x41/0x40; boots at 3.3 V) */
        const char *a = strstr(line, "\"v\":");
        int v = a ? atoi(a + 4) : 0;
        if (v != 3 && v != 5) { emit_error("linkv needs v:3 or v:5"); return true; }
        bool ok = gbl_set_link_voltage_5v(v == 5);
        char o[64]; snprintf(o, sizeof o, "{\"ev\":\"linkv\",\"v\":%d,\"sent\":%s}", v, ok ? "true" : "false"); emit(o);
        return true;
    }
    if (is_op(line, "led")) {                        /* status LED on the RP2040: {"op":"led","r":0,"g":40,"b":0} ("off":1 turns it off) */
        const char *r = strstr(line, "\"r\":"), *g = strstr(line, "\"g\":"), *b = strstr(line, "\"b\":");
        bool off = strstr(line, "\"off\":1") != NULL;
        bool ok = gbl_set_led(r ? atoi(r + 4) : 0, g ? atoi(g + 4) : 0, b ? atoi(b + 4) : 0, !off);
        emit(ok ? "{\"ev\":\"led\",\"sent\":true}" : "{\"ev\":\"led\",\"sent\":false}");
        return true;
    }
    if (is_op(line, "rfu_host")) {                   /* online: the RP2040 is remote, the PC bridge plays it */
        /* Its own task: start() waits for the PC's replies, which arrive on THIS console task. */
        wifi_up();                                   /* idle stops Wi-Fi on the local path: the online session needs it */
        ldn_brain_set_up_floor_us(12000);            /* drain at the GBA's 60/s */
        ldn_brain_set_window(6, 1);                  /* online window (the trades ran 6/1) */
        gba_bridge_set_coalesce_up_keys(true);       /* merge back-to-back GBA held keys (no standing backlog) */
        xTaskCreate(rfu_host_task, "rfu_host", 4096, NULL, 5, NULL);
        return true;
    }
    if (is_op(line, "gb")) {                         /* a frame from the PC bridge, as if from the RP2040 */
        int ch = -1; const char *h = strstr(line, "\"hex\":\""); const char *c = strstr(line, "\"ch\":");
        if (c) ch = atoi(c + 5);
        uint8_t buf[GBL_MAX_PAYLOAD]; size_t n = 0;
        if (h) for (h += 7; h[0] && h[1] && h[0] != '"' && n < sizeof buf; h += 2) {
            unsigned b; if (sscanf(h, "%2x", &b) != 1) break; buf[n++] = (uint8_t)b;
        }
        if (ch >= 0 && ch <= 2) gbl_host_inject((uint8_t)ch, buf, n);
        return true;
    }
    if (is_op(line, "rfu_stats")) { rfu1_bridge_isr_stats(); return true; }
    if (is_op(line, "rfu_stop")) { rfu1_bridge_stop(); emit("{\"ev\":\"rfu\",\"up\":false}"); return true; }
    return false;
}

/* BLE: the same ops arrive as control-characteristic writes (NimBLE host task; the ops only set flags / copy
 * keys, so handling them inline is fine) and partner slots as data-characteristic writes, 14 bytes each. */
/* BLE relay round-trip test. The device notifies a 4-byte u32 ms timestamp on the data characteristic; the
 * phone app wraps it in a relay PING, the relay PONGs it back, the app writes the 4 bytes back, and the full
 * ESP -> BLE -> phone -> internet -> relay -> internet -> phone -> BLE -> ESP time is measured here. */

static uint32_t ping_now_ms(void) { return (uint32_t)(esp_timer_get_time() / 1000); }

static void ping_rx(const uint8_t *d)
{
    uint32_t t = (uint32_t)d[0] | ((uint32_t)d[1] << 8) | ((uint32_t)d[2] << 16) | ((uint32_t)d[3] << 24);
    uint32_t rtt = ping_now_ms() - t;
    if (rtt > 60000) return;   /* a stale echo from an earlier run */
    portENTER_CRITICAL(&s_ping_lock);
    s_pw_n++; s_ping_total_n++; s_pw_sum += rtt;
    if (rtt < s_pw_min) s_pw_min = rtt;
    if (rtt > s_pw_max) s_pw_max = rtt;
    portEXIT_CRITICAL(&s_ping_lock);
}

static void ping_task(void *arg)
{
    (void)arg;
    TickType_t last_report = xTaskGetTickCount();
    for (;;) {
        int hz = s_ping_hz > 0 ? s_ping_hz : 1;
        if (s_ping_on) {
            uint32_t t = ping_now_ms();
            uint8_t b[4] = { (uint8_t)t, (uint8_t)(t >> 8), (uint8_t)(t >> 16), (uint8_t)(t >> 24) };
            bool ok = kz_mode_ble_notify_data(b, sizeof b);
            portENTER_CRITICAL(&s_ping_lock);
            if (ok) { s_pw_sent++; s_ping_total_sent++; } else s_pw_drop++;
            portEXIT_CRITICAL(&s_ping_lock);
        }
        if (xTaskGetTickCount() - last_report >= pdMS_TO_TICKS(1000)) {
            last_report = xTaskGetTickCount();
            if (s_ping_on) {
                uint32_t sent, n, drop, mn, mx; uint64_t sum;
                portENTER_CRITICAL(&s_ping_lock);
                sent = s_pw_sent; n = s_pw_n; drop = s_pw_drop; mn = s_pw_min; mx = s_pw_max; sum = s_pw_sum;
                s_pw_sent = s_pw_n = s_pw_drop = s_pw_max = 0; s_pw_min = UINT32_MAX; s_pw_sum = 0;
                portEXIT_CRITICAL(&s_ping_lock);
                char b[200];
                snprintf(b, sizeof b, "{\"ev\":\"relay_rtt\",\"sent\":%u,\"n\":%u,\"lost\":%d,\"min\":%d,\"avg\":%d,\"max\":%d,\"ble_drop\":%u,\"conn\":%d,\"total_sent\":%u,\"total_n\":%u}",
                         (unsigned)sent, (unsigned)n, (int)sent - (int)n, n ? (int)mn : -1, n ? (int)(sum / n) : -1, n ? (int)mx : -1,
                         (unsigned)drop, kz_mode_ble_connected(), (unsigned)s_ping_total_sent, (unsigned)s_ping_total_n);
                emit(b);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000 / hz) ? pdMS_TO_TICKS(1000 / hz) : 1);
    }
}

static bool ble_op(const char *json, size_t len)
{
    char line[400];
    if (len >= sizeof line) return false;
    memcpy(line, json, len); line[len] = 0;
    s_op_from_ble = true;
    bool ok = handle_op(line);
    s_op_from_ble = false;
    return ok;
}
static void ble_data(const uint8_t *data, size_t len)
{
    if (len == 4 && s_ping_on) { ping_rx(data); return; }
    for (size_t off = 0; off + 14 <= len; off += 14) relay_engine_push(data + off);
}

/* Free-block map (bench): the walk holds the heap lock, so the callback only records sizes; printing happens after. */
typedef struct { int n; uint32_t sz[12]; } free_walk_t;
static bool free_walk(walker_heap_into_t h, walker_block_info_t b, void *u)
{
    (void)h;
    free_walk_t *w = u;
    if (!b.used && b.size >= 2048) { if (w->n < 12) w->sz[w->n] = (uint32_t)b.size; w->n++; }
    return true;
}
static void heap_map(const char *tag)
{
    free_walk_t w = { 0 };
    size_t fr = heap_caps_get_free_size(MALLOC_CAP_8BIT), lg = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
    heap_caps_walk(MALLOC_CAP_8BIT, free_walk, &w);
    char b[160]; int o = snprintf(b, sizeof b, "  [heapmap] %s free=%u largest=%u blocks>=2K:", tag, (unsigned)fr, (unsigned)lg);
    for (int i = 0; i < w.n && i < 12 && o < (int)sizeof b - 8; i++) o += snprintf(b + o, sizeof b - o, " %u", (unsigned)w.sz[i]);
    printf("%s%s\n", b, w.n > 12 ? " ..." : "");
}

/* Restart instead of going back to Bluetooth once a trade has run since boot (extended to the
 * FRLG/Emerald image ). The C3 heap fragments across a session: on the bench, after one Emerald trade the next
 * session could not reserve the zstd decoder (largest block 30720), so the adapter could not read the Switch room and
 * flipped between Bluetooth and Wi-Fi every 20 s. Keys, Wi-Fi and settings are in NVS; the RP2040 keeps running. */
static volatile bool s_trade_ran;
static void restart_clean(void)
{
    printf("  [radio] a trade ran this boot: restarting for a clean heap (free=%u largest=%u)\n",
           (unsigned)heap_caps_get_free_size(MALLOC_CAP_8BIT), (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
    fflush(stdout);
    for (int i = 0; i < 6; i++) { gbl_set_led(0, (i & 1) ? 0 : 32, (i & 1) ? 0 : 32, true); vTaskDelay(pdMS_TO_TICKS(100)); }   /* cyan fast blink */
    esp_restart();
}

/* Bluetooth image push (kz_mode_ble_set_push_fns). The push needs one ~17 KB block plus ~13 KB more while Bluetooth
 * is up; on the RS image the boot-time sim reserve (25 KB) left it none (bench: largest block 9.7 KB, "the
 * adapter couldn't write the new firmware"). Lend everything the brain holds; a good push restarts into the new image,
 * a failed one restarts here to take the reserve back on a clean heap, after the LED has shown the failure. */
static void push_lend(void)
{
    ldn_brain_free_all();
    printf("  [ota] push: trade memory lent (free=%u largest=%u)\n",
           (unsigned)heap_caps_get_free_size(MALLOC_CAP_8BIT), (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
}
static void push_failed(void)
{
    printf("  [ota] push failed: restarting for a clean heap\n");
    fflush(stdout);
    vTaskDelay(pdMS_TO_TICKS(3000));   /* the app has the fw_error already; the LED shows the failure meanwhile */
    esp_restart();
}

static void radio_apply(radio_state_t want)
{
    if (want != RADIO_SESSION && s_trade_ran && local_trade_path()) restart_clean();
    if (want == RADIO_SESSION) {
        kz_mode_ble_stop(); s_ble_started = false;                       /* ~52 KB back for the LDN session */
        heap_map("ble-off");
        if (!pia_zstd_reserve()) ESP_LOGE(TAG, "zstd decoder workspace: no room (largest block %u)",
                                          (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
        wifi_up();
        s_radio_state = RADIO_SESSION;
        s_session_since_us = esp_timer_get_time();
        if (!trade_session_arm()) ESP_LOGW(TAG, "session requested but no keys: nothing to scan for");
        printf("  [radio] SESSION: Bluetooth off, Wi-Fi on, armed=%d\n", trade_session_armed());
        heap_map("session");
    } else {
        trade_session_disarm();                                          /* ends any session, sniffer off, scan parked */
        pia_zstd_release();
        if (local_trade_path() && !ldn_ap_running()) wifi_down();       /* nothing needs Wi-Fi while idle; Bluetooth gets the radio */
        s_radio_state = RADIO_IDLE;
        esp_err_t e = kz_mode_ble_start(KZ_MODE_ID, KZ_BLE_NAME);
        s_ble_started = e == ESP_OK;
        s_led_ble_failed = !s_ble_started;
        static bool confirmed;
        if (s_ble_started && !confirmed) { confirmed = true; ota_update_confirm_boot(); }   /* an OTA'd image that got here is healthy */
        printf("  [radio] IDLE: Wi-Fi off, Bluetooth %s\n", s_ble_started ? "advertising " KZ_BLE_NAME : esp_err_to_name(e));
        heap_map("idle");
    }
}

/* Back to IDLE (Bluetooth) when the local session has nothing to do: no trade running and the GBA side silent for
 * RADIO_GBA_QUIET_US. A GBA switched off never logs out -- it just goes quiet -- so silence is the signal, not the login
 * state (bench: the LED stayed amber with the GBA off). A USB bench "start" holds the session; the app's does not. */
#define RADIO_GBA_QUIET_US 20000000LL
static void radio_task(void *arg)
{
    (void)arg;
    radio_apply(RADIO_IDLE);
    for (;;) {
        radio_state_t want;
        if (session_running()) s_trade_ran = true;
        if (xQueueReceive(s_radio_q, &want, pdMS_TO_TICKS(1000)) == pdTRUE) {
            if (want != s_radio_state) radio_apply(want);
            continue;
        }
        if (s_radio_state == RADIO_SESSION && local_trade_path() && !s_console_hold && !session_running()) {
            int64_t now = esp_timer_get_time();
            if (now - s_session_since_us > RADIO_GBA_QUIET_US && rfu1_bridge_gba_quiet_us() > RADIO_GBA_QUIET_US) {
                printf("  [radio] GBA silent for %d s and no trade -> idle\n", (int)(RADIO_GBA_QUIET_US / 1000000));
                radio_apply(RADIO_IDLE);
            }
        }
    }
}

/* ---- Status LED (WS2812 on the RP2040, set over the UART with GB-Link cmd 0x42) --------------------------------------
 * The C3 owns the LED so the user can see what the adapter is doing without a phone or a serial console:
 *   white, 2 Hz blink        starting                     blue, slow breathe   Bluetooth advertising, no phone yet
 *   blue, 2 gaps / 2 s       a phone is connected         magenta, fast blink  receiving firmware over Bluetooth
 *   red double-blink         a firmware push failed (5 s) red, fast blink      Bluetooth could not start
 *   amber, 1 Hz              session: waiting for the Switch room / the GBA
 *   green, 1 gap / 2 s       session: trading (the brain is running)
 *   cyan, fast blink ~0.6 s  a trade session is over: restarting for a clean heap (both images)
 *   purple, fast blink       Wi-Fi OTA update downloading/flashing (set directly by ota_update.c, not this
 *                            task -- led_task doesn't exist yet at that point in boot
 *                            no LED feedback during a Wi-Fi update read as a hang)
 * No Kizuna state is a solid colour: stock GB-Link firmware shows only solid colours (green idle,
 * yellow GBA, blue GB/GBC, purple printer, white Advance Wars, light blue e-Reader, deep blue Battle Chip Gate, dim red at
 * power-up), so an adapter running Kizuna can never be mistaken for one running GB-Link. The patterns are on/off steps,
 * at most a few LED writes per 2 s, so they stay legal during a GBA login or a trade (see below).
 * Animations only while idle and no GBA is logging in; otherwise the colour is written on change plus a 2 s refresh, so
 * the GB-Link command traffic never competes with the GBA's timing-critical login or the trade. GB-Link re-applies its own
 * colours on some events (mode entry, USB host); the refresh overrides them. */
typedef enum { LED_BOOT, LED_ADV, LED_PHONE, LED_PUSH, LED_PUSH_FAIL, LED_BLE_FAIL, LED_WAIT, LED_TRADE } led_state_t;
static const char *const LED_NAMES[] = { "boot", "advertising", "phone", "push", "push_failed", "ble_failed", "waiting", "trading" };
static void led_set(uint8_t r, uint8_t g, uint8_t b)
{
    static uint8_t lr = 1, lg = 1, lb = 1;
    if (r == lr && g == lg && b == lb) return;
    lr = r; lg = g; lb = b;
    gbl_set_led(r, g, b, true);
}
static void led_task(void *arg)
{
    (void)arg;
    led_state_t last = LED_BOOT;
    bool was_pushing = false;
    int64_t fail_until = 0, last_refresh = 0;
    for (int tick = 0;; tick++) {
        int64_t now = esp_timer_get_time();
        bool pushing = ota_push_active();
        if (was_pushing && !pushing) fail_until = now + 5000000;   /* a successful push restarts the board, so ending = failed */
        was_pushing = pushing;
        led_state_t st;
        if (pushing) st = LED_PUSH;
        else if (now < fail_until) st = LED_PUSH_FAIL;
        else if (s_radio_state == RADIO_SESSION) st = session_running() ? LED_TRADE : LED_WAIT;
        else if (s_led_ble_failed) st = LED_BLE_FAIL;
        else if (!kz_mode_ble_running()) st = LED_BOOT;
        else st = kz_mode_ble_connected() ? LED_PHONE : LED_ADV;
        if (st != last) { printf("  [led] %s\n", LED_NAMES[st]); last = st; }
        bool quiet = rfu1_bridge_login_state() >= 1 || s_radio_state == RADIO_SESSION;   /* GBA logging in / session */
        bool refresh = now - last_refresh > 2000000;
        if (refresh) last_refresh = now;
        uint8_t r = 0, g = 0, b = 0;
        switch (st) {
            case LED_BOOT:      if (((now / 250000) & 1) == 0) r = g = b = 10; break;
            case LED_ADV:       if (quiet) { if (((now / 500000) & 1) == 0) b = 24; }   /* GBA logging in: few writes, never solid */
                                else { float x = 0.5f - 0.5f * cosf(2.0f * 3.14159f * (float)(now % 2000000) / 2000000.0f); b = (uint8_t)(3 + 32 * x); }
                                break;
            case LED_PHONE:     { int64_t ph = now % 2000000; if (!(ph < 120000 || (ph >= 240000 && ph < 360000))) b = 40; } break;
            case LED_PUSH:      if (((now / 125000) & 1) == 0) { r = 40; b = 40; } break;
            case LED_PUSH_FAIL: { int ph = (int)((now / 125000) % 8); if (ph == 0 || ph == 2) r = 48; } break;
            case LED_BLE_FAIL:  if (((now / 125000) & 1) == 0) r = 40; break;
            case LED_WAIT:      if (((now / 500000) & 1) == 0) { r = 40; g = 16; } break;
            case LED_TRADE:     if (now % 2000000 >= 150000) g = 40; break;
        }
        led_set(r, g, b);                           /* writes only when the colour changes (breathe/blink = a change) */
        if (refresh) gbl_set_led(r, g, b, true);    /* every 2 s: override GB-Link's own colour changes */
        bool animate = !quiet && st == LED_ADV;
        vTaskDelay(pdMS_TO_TICKS(animate ? 100 : 50));
    }
}

/* JSON ops one per line on the USB Serial/JTAG port (same shape as the distributor's bench path). */
static void usb_task(void *arg)
{
    (void)arg;
#if CONFIG_ESP_CONSOLE_UART
    if (uart_driver_install(CONFIG_ESP_CONSOLE_UART_NUM, 2048, 0, 0, NULL, 0) != ESP_OK) { vTaskDelete(NULL); return; }
    uart_vfs_dev_use_driver(CONFIG_ESP_CONSOLE_UART_NUM);
#else
    usb_serial_jtag_driver_config_t cfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    cfg.rx_buffer_size = 4096;          /* host link: the PC streams RP2040 frames in at up to ~40 KB/s */
    cfg.tx_buffer_size = 4096;
    if (usb_serial_jtag_driver_install(&cfg) != ESP_OK) { vTaskDelete(NULL); return; }
    usb_serial_jtag_vfs_use_driver();   /* printf and our reads share the driver, not the raw FIFO */
#endif
    static char line[400];
    static uint8_t chunk[128];
    size_t n = 0;
    for (;;) {
#if CONFIG_ESP_CONSOLE_UART
        int got = uart_read_bytes(CONFIG_ESP_CONSOLE_UART_NUM, chunk, 1, portMAX_DELAY);   /* UART console (S3 bench) */
#else
        int got = usb_serial_jtag_read_bytes(chunk, sizeof chunk, portMAX_DELAY);
#endif
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

/* The brain's clock. On the C6 the GBA's 0x26 poll loop ticked the sim inline; here nothing paces us, so run
 * sim_tick at ~60 Hz (the VBlank cadence the sim's timers assume) and drain Pia UDP on the same task. In M2
 * this task also drives the RFU1 bridge. 24 KB stack: the sim's decrypt/decompress path overflowed smaller
 * ones (tr-0.34). */
/* Kizuna: the tick is paced by an esp_timer at exactly one VBlank (16667 us). The old loop
 * slept with vTaskDelay (CONFIG_FREERTOS_HZ=100: 10 ms quanta, so a 16.7 ms period could not be hit and the
 * loop drifted), then "caught up" by running up to 8 sim ticks back-to-back. Each catch-up tick could emit an
 * UP slot, so two child slots regularly left inside one console poll (microseconds apart) -- which the FRLG
 * parent cannot take: RfuMain2_Parent keeps one child slot per poll and a rolling tag that is not +1 mod 8 is
 * a receive error, fatal on the fifth (link_rfu_2.c:876-892; pokeldn docs/frlg_link.md). Now: one sim tick
 * per timer period; if periods were missed, the sim's CLOCK is advanced for them (ldn_brain_skip_ticks) so its
 * RTO/ack timers still track wall time, but nothing is emitted for them. Stalls are measured and printed. */
/* Bench (first run): the periodic-timer version still produced 51 UP emissions closer than
 * 15 ms. When the esp_timer task is held off (Wi-Fi), its queued periodic callbacks fire late and only
 * microseconds apart; each one looked like a legitimate next period to the loop. The rule has to be enforced
 * on the wall clock: two emitting ticks are never closer than TICK_MIN_GAP_US, whatever the timer did. One-shot
 * timer re-armed per tick at max(schedule, last tick + min gap); an overrun delays the next tick, never
 * bunches it; periods lost to an overrun advance the sim clock (skip) without emitting. */
#define TICK_PERIOD_US  16667
#define TICK_MIN_GAP_US 15000
static SemaphoreHandle_t   s_tick_sem;
static esp_timer_handle_t  s_tick_timer;
static void tick_timer_cb(void *arg) { (void)arg; xSemaphoreGive(s_tick_sem); }   /* ESP_TIMER_TASK: task context */

static void brain_task(void *arg)
{
    (void)arg;
    s_tick_sem = xSemaphoreCreateBinary();
    const esp_timer_create_args_t ta = { .callback = tick_timer_cb, .name = "vblank", .dispatch_method = ESP_TIMER_TASK };
    ESP_ERROR_CHECK(esp_timer_create(&ta, &s_tick_timer));
    int64_t  sched = esp_timer_get_time();     /* nominal next-tick time, advances by exactly one period per tick */
    int64_t  last_tick = 0;
    uint32_t missed_total = 0, missed_5s = 0, iter_max_us = 0, iters = 0, late_5s = 0;
    int64_t  last_report = esp_timer_get_time();
    for (;;) {
        int64_t now = esp_timer_get_time();
        int64_t next = sched;
        if (last_tick && next < last_tick + TICK_MIN_GAP_US) next = last_tick + TICK_MIN_GAP_US;
        if (next > now) {
            ESP_ERROR_CHECK(esp_timer_start_once(s_tick_timer, (uint64_t)(next - now)));
            xSemaphoreTake(s_tick_sem, portMAX_DELAY);
            now = esp_timer_get_time();
        }
        /* periods the schedule fell behind by (an overrun): advance the sim clock for them, emit nothing */
        int missed = 0;
        while (sched + TICK_PERIOD_US <= now) { sched += TICK_PERIOD_US; missed++; }
        sched += TICK_PERIOD_US;                   /* this tick consumes one period */
        if (missed > 0) {
            if (missed > 60) missed = 60;
            missed_total += (uint32_t)missed; missed_5s += (uint32_t)missed;
            if (ldn_brain_active()) ldn_brain_skip_ticks(missed);
        }
        if (last_tick && now - last_tick < TICK_MIN_GAP_US) late_5s++;   /* must stay 0: the invariant */
        last_tick = now;
        if (ldn_ap_running() && !th_session_active()) ldn_ap_poll();   /* the plain host room; th_session polls its own */
        rfu1_bridge_poll();                          /* GBA side: RFU1 in/out (runs before and during the session) */
#ifdef KZ_GBA_LEAD
        gba_lead_poll();                             /* the GBA leads, the Switch joins (no-op otherwise) */
#endif
        if (ldn_brain_active()) { ldn_udp_poll(); ldn_brain_tick(); }
        uint32_t dt = (uint32_t)(esp_timer_get_time() - now);
        if (dt > iter_max_us) iter_max_us = dt;
        iters++;
        if (now - last_report > 5000000) {
            last_report = now;
            if (ldn_brain_active())
                printf("  [loop] iters=%u max_iter=%uus missed_ticks=%u (5s) / %u (total) gap_violations=%u\n",
                       (unsigned)iters, (unsigned)iter_max_us, (unsigned)missed_5s, (unsigned)missed_total, (unsigned)late_5s);
            iters = 0; iter_max_us = 0; missed_5s = 0; late_5s = 0;
        }
    }
}

void app_main(void)
{
    esp_err_t nv = nvs_flash_init();
    if (nv == ESP_ERR_NVS_NO_FREE_PAGES || nv == ESP_ERR_NVS_NEW_VERSION_FOUND) { nvs_flash_erase(); nvs_flash_init(); }
    ota_update_boot();            /* one-shot boot actions (Wi-Fi setup session / Wi-Fi install): never return */
    ota_update_set_hw(KZ_HW_TAG);
    printf("\n==== kizuna %s (%s, hw %s): Switch LDN + " KZ_GBA_SIDE " ====\n", KZ_MODE_ID, CONFIG_IDF_TARGET, KZ_HW_TAG);
    th_session_set_emit(emit);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    /* Static-IP STA netif for the LDN Pia UDP path: DHCP off (the Switch assigns our IP through the LDN
     * membership, not DHCP). ldn_udp owns the socket; ldn_join's STA rxcb forwards the Switch's Pia IP frames
     * into lwIP on this netif.  */
    esp_netif_inherent_config_t netif_base = ESP_NETIF_INHERENT_DEFAULT_WIFI_STA();
    netif_base.flags &= ~ESP_NETIF_DHCP_CLIENT;
    esp_netif_config_t netif_cfg = ESP_NETIF_DEFAULT_WIFI_STA();
    netif_cfg.base = &netif_base;
    esp_netif_t *sta_netif = esp_netif_new(&netif_cfg);
    ESP_ERROR_CHECK(esp_netif_attach_wifi_station(sta_netif));
    ESP_ERROR_CHECK(esp_wifi_set_default_wifi_sta_handlers());
    ldn_udp_init(sta_netif);
    ldn_udp_set_rx(ldn_brain_on_datagram);
    ldn_join_set_netif(sta_netif);

    wifi_init_config_t wic = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wic));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    /* Legacy 11b/g only: the Switch's LDN AP is a legacy-rate network; on the C6 the STA otherwise came up in
     * 11ax and esp_wifi_connect rejected the AP as "not suitable" (tr-0.14). Harmless on the C3. */
    esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G);
    esp_wifi_set_ps(WIFI_PS_NONE);

    trade_session_start();
    /* Bluetooth face (T3): ops + slot stream over the shared radio while LDN runs. 'ble_off' drops it. */
    kz_mode_ble_set_handlers(ble_op, ble_data);
    kz_mode_ble_set_push_fns(push_lend, push_failed);
    trade_session_set_radio_cb(radio_cb);       /* a GBA logging in requests the session radio */
    _Static_assert(sizeof(ldn_keys) == KZ_KEYS_LEN, "kz_keys stores exactly cport's ldn_keys");
    if (kz_keys_load((uint8_t *)&s_keys)) {      /* keys set once from the app survive power-off and updates */
        s_have_keys = true;
        trade_session_set_keys(&s_keys);
        printf("  [keys] loaded from adapter storage (values not shown)\n");
    } else printf("  [keys] none stored: the app sends them once (keys_set)\n");
    /* M2: bring up the RP2040 link (GB-Link mode 0x07). If it is absent the console triggers (M1) still work. */
    if (!rfu1_bridge_start(PIN_GBL_TX, PIN_GBL_RX))
        ESP_LOGW(TAG, "RP2040 not answering: GBA side disabled, console 'start'/'select' drive the session");
#if CONFIG_IDF_TARGET_ESP32S3
    /* S3 test build is the hosted-trade role: the join-role sim (which needs the 24 KB stack) is not used, and internal RAM
     * is the scarce resource. */
    xTaskCreate(brain_task, "brain", 8192, NULL, 5, NULL);
    xTaskCreate(usb_task, "usb", 8192, NULL, 4, NULL);
#else
    xTaskCreate(brain_task, "brain", 24576, NULL, 5, NULL);
    xTaskCreate(usb_task, "usb", 12288, NULL, 4, NULL);   /* below the brain (5): keep its proven tick timing; 12 KB for the host/relay ops' buffers */
#endif
    s_radio_q = xQueueCreate(4, sizeof(radio_state_t));
    xTaskCreate(radio_task, "radio", 4096, NULL, 3, NULL);   /* brings Bluetooth up (IDLE) */
    xTaskCreate(led_task, "led", 3072, NULL, 2, NULL);        /* status LED (below the radio and the brain) */
    heap_event();
    ESP_LOGI(TAG, "ready: Bluetooth " KZ_BLE_NAME "; keys_set, then start / select / stop / idle / status");
    for (;;) vTaskDelay(pdMS_TO_TICKS(1000));
}
