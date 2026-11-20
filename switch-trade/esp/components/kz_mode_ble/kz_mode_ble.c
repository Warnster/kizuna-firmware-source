#include "kz_mode_ble.h"
#include "kz_wifi.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "esp_coexist.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "esp_heap_caps.h"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "ota_update.h"
#include "ota_push.h"

static const char *TAG = "kz_ble";

/* 4b494e4a-000X-4b5a-9a6c-1d7a2f00c6a0 (BLE_UUID128_INIT bytes are the human form reversed) */
#define KZ_UUID(x) BLE_UUID128_INIT(0xa0,0xc6,0x00,0x2f,0x7a,0x1d,0x6c,0x9a,0x5a,0x4b,x,0x00,0x4a,0x4e,0x49,0x4b)
static const ble_uuid128_t SVC_UUID    = KZ_UUID(0x00);
static const ble_uuid128_t STATUS_UUID = KZ_UUID(0x01);
static const ble_uuid128_t OTA_UUID    = KZ_UUID(0x02);
static const ble_uuid128_t CTRL_UUID   = KZ_UUID(0x03);
static const ble_uuid128_t DATA_UUID   = KZ_UUID(0x04);
static const ble_uuid128_t IMAGE_UUID  = KZ_UUID(0x05);

static const char *s_mode = "";
static const char *s_name = "KIZUNA";
static uint16_t s_conn = BLE_HS_CONN_HANDLE_NONE;
static uint16_t s_status_h, s_ctrl_h, s_data_h;
static uint16_t s_itvl, s_mtu;   /* live connection interval (1.25 ms units) and ATT MTU, for the bench log */
/* Connection parameters we ask the central for after connecting. 12..24 x 1.25 ms = 15..30 ms satisfies Apple's accessory rules (interval
 * min >= 15 ms, max >= min + 15 ms, timeout > 3 x max); iOS ignores requests below 15 ms, which the old 6..12 was. */
static struct ble_gap_upd_params s_pref = { .itvl_min = 12, .itvl_max = 24, .latency = 0, .supervision_timeout = 400,
                                            .min_ce_len = 0, .max_ce_len = 0 };
static bool s_pref_retried;      /* the one re-request per connection has been used */
static uint8_t s_addr_type;
static kz_mode_ble_op_fn s_op_fn;
static kz_mode_ble_data_fn s_data_fn;
static bool s_adv_on = true;
static volatile bool s_running;   /* stack up (start ... stop); see kz_mode_ble_stop() */

static void start_advertising(void);

static void notify(uint16_t handle, const void *data, uint16_t len)
{
    for (int i = 0; i < 1000; i++) {   /* ~2 s: ride out a momentarily exhausted mbuf pool */
        if (s_conn == BLE_HS_CONN_HANDLE_NONE) return;
        struct os_mbuf *om = ble_hs_mbuf_from_flat(data, len);
        if (om && ble_gatts_notify_custom(s_conn, handle, om) == 0) return;
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}
static void notify_ctrl(const char *json) { notify(s_ctrl_h, json, (uint16_t)strlen(json)); }

/* One attempt, never blocks: for the relay engine's per-frame slot stream on the brain task. */
bool kz_mode_ble_notify_data(const void *data, size_t len)
{
    if (s_conn == BLE_HS_CONN_HANDLE_NONE) return false;
    struct os_mbuf *om = ble_hs_mbuf_from_flat(data, (uint16_t)len);
    return om && ble_gatts_notify_custom(s_conn, s_data_h, om) == 0;
}
bool kz_mode_ble_connected(void) { return s_conn != BLE_HS_CONN_HANDLE_NONE; }
void kz_mode_ble_set_conn_pref(uint16_t itvl_min, uint16_t itvl_max, uint16_t latency, uint16_t supervision_timeout)
{
    s_pref.itvl_min = itvl_min; s_pref.itvl_max = itvl_max; s_pref.latency = latency; s_pref.supervision_timeout = supervision_timeout;
    if (s_conn != BLE_HS_CONN_HANDLE_NONE) { s_pref_retried = false; ble_gap_update_params(s_conn, &s_pref); }
}
void kz_mode_ble_link_info(uint16_t *itvl_1p25ms, uint16_t *mtu) { if (itvl_1p25ms) *itvl_1p25ms = s_itvl; if (mtu) *mtu = s_mtu; }

static bool (*s_busy_fn)(void);
void kz_mode_ble_set_busy_fn(bool (*busy)(void)) { s_busy_fn = busy; }
static bool refuse_if_busy(void)
{
    if (!s_busy_fn || !s_busy_fn()) return false;
    notify_ctrl("{\"ev\":\"error\",\"message\":\"The adapter is busy with a trade: stop it first.\"}");
    return true;
}

static void restart_soon_task(void *arg)
{
    /* Let the peer receive the write response, tell it what is about to happen, then restart into the queued action. */
    int kind = (int)(intptr_t)arg;
    vTaskDelay(pdMS_TO_TICKS(600));
    if (kind == 1) notify_ctrl("{\"ev\":\"done\"}");
    else {
        char st[320]; size_t n = ota_update_status_json(st, sizeof st, s_mode);
        if (n) notify(s_status_h, st, (uint16_t)n);
    }
    vTaskDelay(pdMS_TO_TICKS(900));
    ESP_LOGI(TAG, "restarting for the queued OTA action");
    esp_restart();
}

/* ---- signed image push (ota_push.h) ------------------------------------------- */
static void (*s_push_lend_fn)(void), (*s_push_failed_fn)(void);
static volatile bool s_push_lent;   /* lend() ran for the current push */
void kz_mode_ble_set_push_fns(void (*lend)(void), void (*failed)(void)) { s_push_lend_fn = lend; s_push_failed_fn = failed; }
static void push_emit(const char *json) { notify_ctrl(json); }
static void push_prepare(void) { s_push_lent = true; if (s_push_lend_fn) s_push_lend_fn(); }
static void push_finished(bool restarting)
{
    if (!restarting && s_push_lent && s_push_failed_fn) s_push_failed_fn();
    s_push_lent = false;
}
static const ota_push_ops_t PUSH_OPS = { .emit = push_emit, .finished = push_finished, .prepare = push_prepare };

typedef struct { char *json; size_t len; } fw_begin_arg_t;
static void fw_begin_task(void *p)
{
    fw_begin_arg_t *a = p;   /* erasing the spare slot takes seconds: never on the BLE host task */
    ota_push_begin(a->json, a->len, s_mode, &PUSH_OPS);   /* a failure after prepare() comes back as finished(false) */
    free(a->json); free(a);
    vTaskDelete(NULL);
}

/* ---- GATT ---------------------------------------------------------------------- */
static bool json_has_op(const char *s, const char *op)
{
    char pat[48]; snprintf(pat, sizeof pat, "\"op\":\"%s\"", op);
    if (strstr(s, pat)) return true;
    snprintf(pat, sizeof pat, "\"op\": \"%s\"", op);
    return strstr(s, pat) != NULL;
}

/* "key":"value" out of a JSON line (\" and \\ unescaped; no other escapes in a network name or password from the app).
 * false if absent or too long. */
static bool json_str(const char *s, const char *key, char *out, size_t cap)
{
    char pat[40]; snprintf(pat, sizeof pat, "\"%s\":\"", key);
    const char *p = strstr(s, pat);
    if (!p) return false;
    p += strlen(pat);
    size_t n = 0;
    for (; *p && *p != '"'; p++) {
        if (*p == '\\' && (p[1] == '"' || p[1] == '\\')) p++;
        if (n + 1 >= cap) return false;
        out[n++] = *p;
    }
    out[n] = 0;
    return *p == '"';
}

/* {"op":"wifi_set","ssid":"...","pass":"..."}: store the home network in the shared Kizuna store (kz_wifi), the one
 * every mode and the OTA install read. The app's "Change Wi-Fi" form uses it instead of restarting into the setup
 * session. On its own task: NVS writes can take a while and the host task must keep the link alive. */
static void wifi_set_task(void *p)
{
    char *buf = p;
    char ssid[KZ_WIFI_SSID_MAX + 1], pass[KZ_WIFI_PASS_MAX + 1] = { 0 };
    if (!json_str(buf, "ssid", ssid, sizeof ssid) || !ssid[0]) {
        notify_ctrl("{\"ev\":\"error\",\"message\":\"wifi_set needs a network name (ssid).\"}");
    } else {
        json_str(buf, "pass", pass, sizeof pass);
        esp_err_t e = kz_wifi_set(ssid, pass);
        notify_ctrl(e == ESP_OK ? "{\"ev\":\"wifi\",\"stored\":true}" : "{\"ev\":\"wifi\",\"stored\":false}");
    }
    memset(pass, 0, sizeof pass);
    memset(buf, 0, strlen(buf));
    free(buf);
    vTaskDelete(NULL);
}

static int ctrl_access(uint16_t ch, uint16_t attr, struct ble_gatt_access_ctxt *c, void *arg)
{
    (void)ch; (void)attr; (void)arg;
    if (c->op != BLE_GATT_ACCESS_OP_WRITE_CHR) return 0;
    uint16_t len = OS_MBUF_PKTLEN(c->om);
    if (len == 0 || len > 512) return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    char *buf = malloc((size_t)len + 1);
    if (!buf) return BLE_ATT_ERR_INSUFFICIENT_RES;
    ble_hs_mbuf_to_flat(c->om, buf, len, NULL);
    buf[len] = 0;
    if (json_has_op(buf, "wifi_setup")) {
        if (refuse_if_busy()) { free(buf); return 0; }
        if (!ota_update_request_provision()) notify_ctrl("{\"ev\":\"error\",\"message\":\"Could not start Wi-Fi setup.\"}");
        else xTaskCreate(restart_soon_task, "kz_rst", 3072, (void *)(intptr_t)1, 4, NULL);
        free(buf);
    } else if (json_has_op(buf, "fw_begin")) {
        if (refuse_if_busy()) { free(buf); return 0; }
        fw_begin_arg_t *a = malloc(sizeof *a);
        if (a) { a->json = buf; a->len = len; }   /* fill in BEFORE the task can run */
        if (!a || xTaskCreate(fw_begin_task, "kz_fwbegin", 6144, a, 4, NULL) != pdPASS) {
            free(a); free(buf);
            notify_ctrl("{\"ev\":\"fw_error\",\"reason\":\"busy\"}");
        }   /* on success the task owns buf and a */
    } else if (json_has_op(buf, "boot_spare")) {
        /* Instant mode switch: boot the image already in the other slot; {"ev":"done"}, then the restart. */
        if (refuse_if_busy()) { free(buf); return 0; }
        if (ota_push_active()) notify_ctrl("{\"ev\":\"error\",\"message\":\"An update is being sent: wait for it to finish.\"}");
        else if (!ota_update_boot_spare()) notify_ctrl("{\"ev\":\"error\",\"message\":\"That mode is no longer on the adapter.\"}");
        else xTaskCreate(restart_soon_task, "kz_rst", 3072, (void *)(intptr_t)1, 4, NULL);
        free(buf);
    } else if (json_has_op(buf, "wifi_set")) {
        if (xTaskCreate(wifi_set_task, "kz_wifiset", 4096, buf, 4, NULL) != pdPASS) {   /* the task owns buf */
            free(buf);
            notify_ctrl("{\"ev\":\"error\",\"message\":\"Busy: try again.\"}");
        }
    } else if (json_has_op(buf, "fw_abort")) {
        ota_push_abort(); free(buf);
    } else if (s_op_fn && s_op_fn(buf, len)) {
        free(buf);
    } else {
        notify_ctrl("{\"ev\":\"error\",\"message\":\"Unsupported command in this mode.\"}");
        free(buf);
    }
    return 0;
}

static int status_access(uint16_t ch, uint16_t attr, struct ble_gatt_access_ctxt *c, void *arg)
{
    (void)ch; (void)attr; (void)arg;
    if (c->op != BLE_GATT_ACCESS_OP_READ_CHR) return BLE_ATT_ERR_UNLIKELY;
    char st[320]; size_t n = ota_update_status_json(st, sizeof st, s_mode);
    return os_mbuf_append(c->om, st, (uint16_t)n) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

static int ota_access(uint16_t ch, uint16_t attr, struct ble_gatt_access_ctxt *c, void *arg)
{
    (void)ch; (void)attr; (void)arg;
    if (c->op != BLE_GATT_ACCESS_OP_WRITE_CHR) return BLE_ATT_ERR_UNLIKELY;
    uint16_t len = OS_MBUF_PKTLEN(c->om);
    if (len == 0 || len > 1024) return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    char *tmp = malloc((size_t)len + 1);
    if (!tmp) return BLE_ATT_ERR_INSUFFICIENT_RES;
    ble_hs_mbuf_to_flat(c->om, tmp, len, NULL);
    tmp[len] = 0;
    if (refuse_if_busy()) { free(tmp); return BLE_ATT_ERR_UNLIKELY; }
    bool ok = ota_update_request_install(tmp, len);   /* validates the URL/version/sha; the reason lands in the status */
    free(tmp);
    if (!ok) return BLE_ATT_ERR_UNLIKELY;
    xTaskCreate(restart_soon_task, "kz_rst", 3072, (void *)(intptr_t)0, 4, NULL);
    return 0;
}

static int image_access(uint16_t ch, uint16_t attr, struct ble_gatt_access_ctxt *c, void *arg)
{
    (void)ch; (void)attr; (void)arg;
    if (c->op != BLE_GATT_ACCESS_OP_WRITE_CHR) return BLE_ATT_ERR_UNLIKELY;
    uint16_t len = OS_MBUF_PKTLEN(c->om);
    if (len == 0 || len > 512) return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    uint8_t tmp[512];
    ble_hs_mbuf_to_flat(c->om, tmp, len, NULL);
    ota_push_feed(tmp, len);   /* copies into a stream buffer; never touches flash */
    return 0;
}

static int data_access(uint16_t ch, uint16_t attr, struct ble_gatt_access_ctxt *c, void *arg)
{
    (void)ch; (void)attr; (void)arg;
    if (c->op != BLE_GATT_ACCESS_OP_WRITE_CHR) return BLE_ATT_ERR_UNLIKELY;
    uint16_t len = OS_MBUF_PKTLEN(c->om);
    if (len == 0 || len > 512) return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    if (!s_data_fn) return BLE_ATT_ERR_UNLIKELY;
    uint8_t tmp[512];
    ble_hs_mbuf_to_flat(c->om, tmp, len, NULL);
    s_data_fn(tmp, len);
    return 0;
}

static const struct ble_gatt_svc_def GATT_SVCS[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &SVC_UUID.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            { .uuid = &STATUS_UUID.u, .access_cb = status_access, .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY, .val_handle = &s_status_h },
            { .uuid = &OTA_UUID.u, .access_cb = ota_access, .flags = BLE_GATT_CHR_F_WRITE },
            { .uuid = &CTRL_UUID.u, .access_cb = ctrl_access, .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_NOTIFY, .val_handle = &s_ctrl_h },
            { .uuid = &DATA_UUID.u, .access_cb = data_access, .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP | BLE_GATT_CHR_F_NOTIFY, .val_handle = &s_data_h },
            { .uuid = &IMAGE_UUID.u, .access_cb = image_access, .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP },
            { 0 },
        },
    },
    { 0 },
};

/* ---- GAP ----------------------------------------------------------------------- */
static int gap_event(struct ble_gap_event *e, void *arg)
{
    (void)arg;
    switch (e->type) {
        case BLE_GAP_EVENT_CONNECT:
            s_conn = e->connect.status == 0 ? e->connect.conn_handle : BLE_HS_CONN_HANDLE_NONE;
            if (e->connect.status != 0 && s_adv_on) start_advertising();
            if (e->connect.status == 0) {
                struct ble_gap_conn_desc d;
                if (ble_gap_conn_find(s_conn, &d) == 0) s_itvl = d.conn_itvl;
                /* Ask for a short interval: the relay carries one slot per 16.7 ms game frame each way. The central
                 * (phone / Mac) decides; whatever it grants is logged on CONN_UPDATE. */
                s_pref_retried = false;
                esp_coex_preference_set(ESP_COEX_PREFER_BALANCE);
                ble_gap_update_params(s_conn, &s_pref);
                ESP_LOGI(TAG, "connected, interval %u x1.25ms", s_itvl);
            }
            break;
        case BLE_GAP_EVENT_CONN_UPDATE: {
            struct ble_gap_conn_desc d;
            if (ble_gap_conn_find(s_conn, &d) == 0) {
                s_itvl = d.conn_itvl;
                ESP_LOGI(TAG, "conn update: interval %u x1.25ms latency %u", d.conn_itvl, d.conn_latency);
                /* The central kept a longer interval than we asked for: ask once more (some phones only honour a request made after
                 * their own parameter update has settled). One retry per connection, never a loop. */
                if (d.conn_itvl > s_pref.itvl_max && !s_pref_retried) {
                    s_pref_retried = true;
                    ble_gap_update_params(s_conn, &s_pref);
                    ESP_LOGI(TAG, "interval %u > %u: asked again", d.conn_itvl, s_pref.itvl_max);
                }
            }
            break;
        }
        case BLE_GAP_EVENT_MTU:
            s_mtu = e->mtu.value; ESP_LOGI(TAG, "MTU %u", s_mtu);
            break;
        case BLE_GAP_EVENT_DISCONNECT:
            s_conn = BLE_HS_CONN_HANDLE_NONE; s_itvl = 0; s_mtu = 0;
            ESP_LOGI(TAG, "disconnected reason %d", e->disconnect.reason);
            ota_push_abort();
            if (s_adv_on) start_advertising();
            break;
        case BLE_GAP_EVENT_ADV_COMPLETE:
            if (s_adv_on) start_advertising();
            break;
        default: break;
    }
    return 0;
}

static int s_adv_rc = -1;   /* last ble_gap_adv_start result; -1 = never tried */

static void start_advertising(void)
{
    struct ble_hs_adv_fields f = {0};   /* a 128-bit UUID + the name don't both fit in 31 bytes: name goes in the scan response */
    f.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    f.uuids128 = (ble_uuid128_t *)&SVC_UUID; f.num_uuids128 = 1; f.uuids128_is_complete = 1;
    struct ble_hs_adv_fields rsp = {0};
    rsp.name = (uint8_t *)s_name; rsp.name_len = (uint8_t)strlen(s_name); rsp.name_is_complete = 1;
    ble_gap_adv_set_fields(&f);
    ble_gap_adv_rsp_set_fields(&rsp);
    struct ble_gap_adv_params adv = {0};
    adv.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv.disc_mode = BLE_GAP_DISC_MODE_GEN;
    adv.itvl_min = adv.itvl_max = 160;    /* 100 ms while no phone is connected (a 1 s interval was not caught while Wi-Fi was busy); advertising stops on connect */
    /* While waiting for a phone, let Bluetooth win the radio: the phone's connect request must be caught in a ~150 us window right
     * after an advertisement, which a busy Wi-Fi link (Switch session) otherwise occupies. Rebalanced once connected. */
    esp_coex_preference_set(ESP_COEX_PREFER_BT);
    int rc = ble_gap_adv_start(s_addr_type, NULL, BLE_HS_FOREVER, &adv, gap_event, NULL);
    s_adv_rc = rc;
    if (rc != 0) ESP_LOGW(TAG, "advertising %s FAILED (rc=%d)", s_name, rc);
    else ESP_LOGI(TAG, "advertising %s (rc=%d)", s_name, rc);
}

static void on_sync(void)
{
    ble_hs_id_infer_auto(0, &s_addr_type);
    start_advertising();
}
static void host_task(void *p) { (void)p; nimble_port_run(); nimble_port_freertos_deinit(); }

void kz_mode_ble_set_handlers(kz_mode_ble_op_fn op, kz_mode_ble_data_fn data) { s_op_fn = op; s_data_fn = data; }

void kz_mode_ble_notify(const char *json) { notify_ctrl(json); }

int kz_mode_ble_adv_rc(void) { return s_adv_rc; }
bool kz_mode_ble_adv_active(void) { return ble_gap_adv_active() != 0; }

void kz_mode_ble_set_advertising(bool on)
{
    s_adv_on = on;
    if (on) { start_advertising(); return; }
    ble_gap_adv_stop();
    if (s_conn != BLE_HS_CONN_HANDLE_NONE) ble_gap_terminate(s_conn, BLE_ERR_REM_USER_CONN_TERM);
}

esp_err_t kz_mode_ble_start(const char *mode, const char *device_name)
{
    if (s_running) return ESP_OK;              /* idempotent; restartable after kz_mode_ble_stop() */
    s_mode = mode; s_name = device_name; s_adv_on = true; s_conn = BLE_HS_CONN_HANDLE_NONE;
    ota_update_enable_spare_switch();          /* this firmware handles boot_spare */
    esp_err_t e = nimble_port_init();
    if (e != ESP_OK) { ESP_LOGE(TAG, "nimble_port_init: %s (free %u)", esp_err_to_name(e), (unsigned)heap_caps_get_free_size(MALLOC_CAP_8BIT)); return e; }
    ble_svc_gap_init();
    ble_svc_gatt_init();
    int rc = ble_gatts_count_cfg(GATT_SVCS);
    if (rc == 0) rc = ble_gatts_add_svcs(GATT_SVCS);
    if (rc != 0) { nimble_port_deinit(); return ESP_FAIL; }
    ble_svc_gap_device_name_set(device_name);
    ble_hs_cfg.sync_cb = on_sync;
    nimble_port_freertos_init(host_task);
    s_running = true;
    return ESP_OK;
}

bool kz_mode_ble_running(void) { return s_running; }

/* Full stop: unlike kz_mode_ble_set_advertising(false), which keeps the stack (and its ~52 KB of heap on the C3),
 * this deinitialises host + controller so the memory comes back for a session that needs it. */
esp_err_t kz_mode_ble_stop(void)
{
    if (!s_running) return ESP_OK;
    s_adv_on = false;
    ble_gap_adv_stop();
    if (s_conn != BLE_HS_CONN_HANDLE_NONE) {
        ble_gap_terminate(s_conn, BLE_ERR_REM_USER_CONN_TERM);
        for (int i = 0; i < 50 && s_conn != BLE_HS_CONN_HANDLE_NONE; i++) vTaskDelay(pdMS_TO_TICKS(10));
    }
    ota_push_abort();
    int rc = nimble_port_stop();               /* the host task leaves nimble_port_run() and deletes itself */
    esp_err_t e = rc == 0 ? nimble_port_deinit() : ESP_FAIL;
    s_running = false; s_conn = BLE_HS_CONN_HANDLE_NONE; s_itvl = 0; s_mtu = 0;
    /* The deleted host task's stack is freed by the idle task; until then it splits the region just given back (bench:
     * largest block 28 KB right after stop, 59 KB a moment later). */
    vTaskDelay(pdMS_TO_TICKS(150));
    ESP_LOGI(TAG, "stopped (rc=%d deinit=%s, heap %u largest %u)", rc, esp_err_to_name(e),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_8BIT), (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
    return e;
}
