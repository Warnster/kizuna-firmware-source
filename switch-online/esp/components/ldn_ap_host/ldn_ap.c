/* SPDX-License-Identifier: AGPL-3.0-only
 * Derived from frlg-ldn-trade-esp32 (https://github.com/easyworld/frlg-ldn-trade-esp32), Copyright (c) Easy World and the frlg-ldn-trade-esp32 authors.
 * Derived from LDN (https://github.com/kinnay/LDN), Copyright (c) kinnay and the LDN authors.
 * Derived from GB-Link-Switch-LDN (https://github.com/GB-Link/GB-Link-Switch-LDN), Copyright (c) the GB-Link-Switch-LDN authors.
 * Changes Copyright (c) 2026 Kizuna. See NOTICE.
 */
/* ldn_ap.c — see ldn_ap.h. The hosting sequence follows GB-Link's working ESP32-C6 host
 * (firmware/common/ldn_host.c: hooks installed before esp_wifi_start, keys installed from the poll
 * loop, the Switch's RSN element, ARP probes) on the real v6.1 driver header, as ldn_join does. */
#include "ldn_ap.h"
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_wifi_default.h"
#include "esp_event.h"
#include "esp_private/wifi.h"      /* esp_wifi_internal_tx / _reg_rxcb / _free_rx_buffer */
/* esp_wifi_driver.h is a private wpa_supplicant header meant to follow wpa's common.h. */
typedef uint8_t  u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;
#include "esp_wifi_driver.h"
#include "ldn_host.h"

/* hostapd's station table and association response (src/ap/hostapd.h, sta_info.h, esp_hostap.h);
 * declared here rather than pulling in wpa_supplicant's internal include tree. */
struct hostapd_data;
struct sta_info;
struct hostapd_data *hostapd_get_hapd_data(void);
struct sta_info *ap_get_sta(struct hostapd_data *hapd, const u8 *sta);
struct sta_info *ap_sta_add(struct hostapd_data *hapd, const u8 *addr);
u16 esp_send_assoc_resp(struct hostapd_data *data, const u8 *addr, u16 status_code, bool omit_rsnxe, int subtype);
extern struct wpa_funcs *wpa_cb;

static const char *TAG = "ldn_ap";

#define ADVERT_PERIOD_US   100000
#define ASSOC_TIMEOUT_MS   15000    /* a station that never authenticates is dropped */
#define ARP_PROBE_FOR_MS   12000
#define ARP_PROBE_EVERY_MS 1000
#define AUTH_FRAME_MAX     (14 + LDN_HOST_AUTH_REQ_LEN + 32)
#define PENDING_MAX        LDN_HOST_MAX_MEMBERS

/* The Switch's RSN element: CCMP group + pairwise, PSK, capabilities 0x000c (hostapd's lacks them). */
static uint8_t s_rsn_ie[] = {
    0x30, 0x14, 0x01, 0x00, 0x00, 0x0f, 0xac, 0x04, 0x01, 0x00, 0x00, 0x0f, 0xac, 0x04,
    0x01, 0x00, 0x00, 0x0f, 0xac, 0x02, 0x0c, 0x00,
};

typedef struct { uint16_t len; uint8_t b[AUTH_FRAME_MAX]; } auth_frame;
typedef struct { uint8_t mac[6]; int64_t assoc_ms; bool used; } assoc_station;

static struct {
    bool              running;
    const ldn_keys   *keys;
    ldn_ap_member_fn  member_fn;
    ldn_host_net      net;
    uint8_t           ccmp_key[16];
    uint64_t          device_id;
    esp_netif_t      *netif;
    esp_netif_t      *up_netif;     /* STA uplink netif (DHCP), created once */
    bool              uplink;
    bool              up_connected;
    uint32_t          up_ip;
    QueueHandle_t     rx;
    assoc_station     assoc[LDN_HOST_MAX_MEMBERS];
    int64_t           next_advert_us, next_rxcb_us, probe_until_ms, next_probe_ms;
    bool (*orig_join)(wpa_station_join_param_t *join);
    bool (*orig_remove)(u8 *bssid);
    uint8_t *(*orig_get_wpa_ie)(size_t *len);
} s;

/* Filled from the Wi-Fi task (hooks), drained by ldn_ap_poll. */
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static uint8_t s_join_q[PENDING_MAX][6], s_leave_q[PENDING_MAX][6];
static int     s_join_n, s_leave_n;

static uint8_t           s_frame[LDN_HOST_ADVERT_FRAME_MAX];   /* the encrypted advert, rebuilt on change */
static int               s_frame_len;
static ldn_host_auth_req s_req;
static uint8_t           s_reply[14 + LDN_HOST_AUTH_RES_LEN];

static volatile uint32_t s_adv_ok, s_adv_fail, s_assoc_n, s_auth_rx;
static volatile int      s_adv_last_err;
static volatile uint32_t s_rx_total, s_rx_88b7_other;
static uint8_t           s_rx_last[14], s_rx_last88[14];

static int64_t now_ms(void) { return esp_timer_get_time() / 1000; }

/* ---- wpa_supplicant hooks (Wi-Fi task) ---- */

/* Replaces the stock join, which would start a 4-way handshake LDN never runs: register the
 * station and answer the association; the poll loop installs its key and opens its port. */
static bool hook_ap_join(wpa_station_join_param_t *join)
{
    struct hostapd_data *hapd = hostapd_get_hapd_data();
    if (!hapd || !join || !join->bssid) return false;
    s_assoc_n++;
    struct sta_info *sta = ap_get_sta(hapd, join->bssid);
    if (!sta) sta = ap_sta_add(hapd, join->bssid);
    if (!sta) return false;
    if (esp_send_assoc_resp(hapd, join->bssid, 0, true, join->subtype) != 0) return false;
    if (join->pmf_enable) *join->pmf_enable = false;
    if (join->pairwise_cipher) *join->pairwise_cipher = 3;   /* the CCMP bit */
    *join->sm = sta;
    portENTER_CRITICAL(&s_lock);
    if (s_join_n < PENDING_MAX) memcpy(s_join_q[s_join_n++], join->bssid, 6);
    portEXIT_CRITICAL(&s_lock);
    return true;
}

static bool hook_ap_remove(u8 *mac)
{
    if (mac) {
        portENTER_CRITICAL(&s_lock);
        if (s_leave_n < PENDING_MAX) memcpy(s_leave_q[s_leave_n++], mac, 6);
        portEXIT_CRITICAL(&s_lock);
    }
    return s.orig_remove ? s.orig_remove(mac) : true;   /* the stock handler frees the station */
}

static uint8_t *hook_ap_get_wpa_ie(size_t *len)
{
    *len = sizeof s_rsn_ie;
    return s_rsn_ie;
}

/* 0x88b7 frames for us go to the auth queue; everything else to lwip (Pia's UDP). */
static esp_err_t ap_rx(void *buffer, uint16_t len, void *eb)
{
    const uint8_t *b = buffer;
    s_rx_total++;
    if (len >= 14) memcpy(s_rx_last, b, 14);
    if (len >= 14 && b[12] == 0x88 && b[13] == 0xb7 && memcmp(b, s.net.members[0].mac, 6)) { s_rx_88b7_other++; memcpy(s_rx_last88, b, 14); }
    if (len >= 14 && b[12] == 0x88 && b[13] == 0xb7 && !memcmp(b, s.net.members[0].mac, 6)) {
        s_auth_rx++;
        if (len <= AUTH_FRAME_MAX && s.rx) {
            static auth_frame f;   /* the Wi-Fi task is the only writer */
            f.len = len;
            memcpy(f.b, b, len);
            xQueueSend(s.rx, &f, 0);
        }
        esp_wifi_internal_free_rx_buffer(eb);
        return ESP_OK;
    }
    return esp_netif_receive(s.netif, buffer, len, eb);
}

/* ---- session-task side ---- */

static void reencode(void)
{
    long n = ldn_host_advert_frame(s.keys, &s.net, s_frame, sizeof s_frame);
    s_frame_len = n > 0 ? (int)n : 0;
    if (n <= 0) ESP_LOGE(TAG, "advertisement does not fit");
}

static void send_advert(void)
{
    if (!s_frame_len) return;
    static esp_err_t last;
    esp_err_t e = esp_wifi_80211_tx(WIFI_IF_AP, s_frame, s_frame_len, true);
    if (e == ESP_OK) s_adv_ok++; else { s_adv_fail++; s_adv_last_err = e; }
    if (e != last) { if (e != ESP_OK) ESP_LOGW(TAG, "advert not sent: %d", e); last = e; }
}

static assoc_station *find_assoc(const uint8_t mac[6])
{
    for (int i = 0; i < LDN_HOST_MAX_MEMBERS; i++)
        if (s.assoc[i].used && !memcmp(s.assoc[i].mac, mac, 6)) return &s.assoc[i];
    return NULL;
}

/* Pairwise key at index 0, then PTKINITDONE: opens the station's port. Never wait for STACONNECTED. */
static void on_associated(const uint8_t mac[6])
{
    uint8_t addr[6];
    memcpy(addr, mac, 6);
    int k = esp_wifi_set_ap_key_internal(WIFI_WPA_ALG_CCMP, addr, 0, s.ccmp_key, sizeof s.ccmp_key);
    if (k != 0) ESP_LOGW(TAG, "station key not installed: %d", k);
    if (!esp_wifi_wpa_ptk_init_done_internal(addr)) ESP_LOGW(TAG, "station port not opened");
    assoc_station *a = find_assoc(mac);
    for (int i = 0; !a && i < LDN_HOST_MAX_MEMBERS; i++)
        if (!s.assoc[i].used) { a = &s.assoc[i]; a->used = true; memcpy(a->mac, mac, 6); }
    if (a) a->assoc_ms = now_ms();
    ESP_LOGI(TAG, "station associated " MACSTR, MAC2STR(mac));
}

static void member_event(int idx, bool joined)
{
    if (!s.member_fn || idx < 1) return;
    ldn_ap_member m = {0};
    memcpy(m.mac, s.net.members[idx].mac, 6);
    memcpy(m.ip, s.net.members[idx].ip, 4);
    m.platform = s.net.members[idx].platform;
    m.index = (uint8_t)idx;
    snprintf(m.name, sizeof m.name, "%s", s.net.members[idx].name);
    s.member_fn(&m, joined);
}

static void on_left(const uint8_t mac[6])
{
    assoc_station *a = find_assoc(mac);
    if (a) a->used = false;
    int idx = ldn_host_member_find(&s.net, mac);
    if (idx < 1) return;
    member_event(idx, false);
    ldn_host_member_remove(&s.net, mac);
    reencode();
    ESP_LOGI(TAG, "member %d left", idx);
}

static void send_frame(const uint8_t dst[6], uint16_t ethertype, const uint8_t *body, size_t len)
{
    if (14 + len > sizeof s_reply) return;
    memcpy(s_reply, dst, 6);
    memcpy(s_reply + 6, s.net.members[0].mac, 6);
    s_reply[12] = (uint8_t)(ethertype >> 8);
    s_reply[13] = (uint8_t)ethertype;
    memcpy(s_reply + 14, body, len);
    int e = esp_wifi_internal_tx(WIFI_IF_AP, s_reply, (uint16_t)(14 + len));
    if (e != 0) ESP_LOGW(TAG, "frame not sent: %d", e);
}

static void handle_auth(const auth_frame *f)
{
    const uint8_t *src = f->b + 6;
    if (!find_assoc(src)) return;                       /* not associated with us */
    int st = ldn_host_auth_parse(s.keys, &s.net, f->b + 14, (size_t)f->len - 14, &s_req);
    if (st == LDN_HOST_AUTH_NOT_AUTH) return;
    int idx = ldn_host_member_find(&s.net, src);
    if (st == LDN_HOST_AUTH_OK && idx < 0 && ldn_host_member_count(&s.net) >= s.net.max_participants)
        st = LDN_HOST_AUTH_DENIED;                      /* room full */
    uint8_t body[LDN_HOST_AUTH_RES_LEN];
    long n = ldn_host_auth_response(s.keys, &s.net, &s_req, st, s.device_id, body, sizeof body);
    if (n > 0) send_frame(src, 0x88b7, body, (size_t)n);
    ESP_LOGI(TAG, "auth from " MACSTR " name=%s platform=%u status=%d", MAC2STR(src), s_req.name,
             s_req.platform, st);
    if (st != LDN_HOST_AUTH_OK || idx >= 0) return;    /* a repeated request just gets the answer again */
    idx = ldn_host_member_add(&s.net, src, s_req.name, s_req.platform);
    if (idx < 1) return;
    reencode();
    s.probe_until_ms = now_ms() + ARP_PROBE_FOR_MS;
    s.next_probe_ms = now_ms() + 200;
    member_event(idx, true);
}

/* ARP request to the member: its own ARP for us goes to a group address the AP path never
 * delivers, and without our answer the Switch deauthenticates (reason 3). */
static void probe_arp(int idx)
{
    uint8_t arp[28] = { 0x00, 0x01, 0x08, 0x00, 6, 4, 0x00, 0x01 };
    memcpy(arp + 8, s.net.members[0].mac, 6);
    memcpy(arp + 14, s.net.members[0].ip, 4);
    memcpy(arp + 24, s.net.members[idx].ip, 4);
    send_frame(s.net.members[idx].mac, 0x0806, arp, sizeof arp);
}

/* The netif glue re-registers the AP's rx callback on WIFI_EVENT_AP_START (wifi_default.c wifi_start), in the
 * event task, racing our own registration after esp_wifi_start() -- on the dual-core S3 either can win. When the
 * glue wins the LDN auth frames go to lwIP and the Switch is left "awaiting response". Registered after the
 * default AP handlers, so it runs after them for the same event. */
static void ap_start_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg; (void)base; (void)id; (void)data;
    if (s.keys) esp_wifi_internal_reg_rxcb(WIFI_IF_AP, ap_rx);
}

static void uplink_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg;
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        const wifi_event_sta_disconnected_t *d = data;
        ESP_LOGW(TAG, "uplink: disconnected (reason %d)", d->reason);
        s.up_connected = false; s.up_ip = 0;
        if (s.uplink) esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_CONNECTED) {
        const wifi_event_sta_connected_t *c = data;
        ESP_LOGI(TAG, "uplink: associated, channel %d", c->channel);
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        const ip_event_got_ip_t *e = data;
        s.up_connected = true; s.up_ip = e->ip_info.ip.addr;
        ESP_LOGI(TAG, "uplink: got ip " IPSTR " gw " IPSTR, IP2STR(&e->ip_info.ip), IP2STR(&e->ip_info.gw));
    } else if (base == IP_EVENT && id == IP_EVENT_STA_LOST_IP) {
        s.up_connected = false; s.up_ip = 0;
    }
}

void ldn_ap_uplink_status(char *buf, size_t n)
{
    uint8_t pri = 0; wifi_second_chan_t sec = 0;
    esp_wifi_get_channel(&pri, &sec);
    wifi_bandwidth_t bw = 0;
    esp_wifi_get_bandwidth(WIFI_IF_AP, &bw);
    wifi_ap_record_t ap = {0};
    int rssi = (s.uplink && esp_wifi_sta_get_ap_info(&ap) == ESP_OK) ? ap.rssi : 0;
    snprintf(buf, n, "{\"ev\":\"uplink\",\"sta\":%d,\"connected\":%d,\"ip\":\"%u.%u.%u.%u\",\"rssi\":%d,\"channel\":%u,\"sec\":%d,\"ap_bw\":%d,"
             "\"rx\":%u,\"rx88_other\":%u,\"ap_mac\":\"%02x%02x%02x%02x%02x%02x\",\"last\":\"%02x%02x%02x%02x%02x%02x>%02x%02x%02x%02x%02x%02x/%02x%02x\",\"last88\":\"%02x%02x%02x%02x%02x%02x>%02x%02x%02x%02x%02x%02x\",\"adv_ok\":%u,\"adv_fail\":%u,\"adv_err\":%d,\"assoc\":%u,\"auth_rx\":%u}",
             s.uplink, s.up_connected, (unsigned)(s.up_ip & 0xff), (unsigned)((s.up_ip >> 8) & 0xff),
             (unsigned)((s.up_ip >> 16) & 0xff), (unsigned)(s.up_ip >> 24), rssi, pri, (int)sec, (int)bw,
             (unsigned)s_rx_total, (unsigned)s_rx_88b7_other,
             s.net.members[0].mac[0], s.net.members[0].mac[1], s.net.members[0].mac[2], s.net.members[0].mac[3], s.net.members[0].mac[4], s.net.members[0].mac[5],
             s_rx_last[0],s_rx_last[1],s_rx_last[2],s_rx_last[3],s_rx_last[4],s_rx_last[5],s_rx_last[6],s_rx_last[7],s_rx_last[8],s_rx_last[9],s_rx_last[10],s_rx_last[11],s_rx_last[12],s_rx_last[13],
             s_rx_last88[0],s_rx_last88[1],s_rx_last88[2],s_rx_last88[3],s_rx_last88[4],s_rx_last88[5],s_rx_last88[6],s_rx_last88[7],s_rx_last88[8],s_rx_last88[9],s_rx_last88[10],s_rx_last88[11],
             (unsigned)s_adv_ok, (unsigned)s_adv_fail, s_adv_last_err, (unsigned)s_assoc_n, (unsigned)s_auth_rx);
}

esp_err_t ldn_ap_start(const ldn_keys *keys, const ldn_ap_config *cfg, ldn_ap_member_fn fn)
{
    if (s.running) ldn_ap_stop();
    if (!keys || !cfg || (cfg->channel != 1 && cfg->channel != 6 && cfg->channel != 11)) return ESP_ERR_INVALID_ARG;
    esp_netif_t *netif = s.netif, *up_netif = s.up_netif;
    QueueHandle_t rx = s.rx;
    memset(&s, 0, sizeof s);
    s.netif = netif; s.up_netif = up_netif; s.rx = rx; s.keys = keys; s.member_fn = fn;

    /* The room's identity, fixed for its life. */
    ldn_host_identity id = { .channel = (uint8_t)cfg->channel, .name = cfg->host_name, .platform = 0 };
    esp_fill_random(id.ssid, sizeof id.ssid);
    esp_fill_random(id.server_random, sizeof id.server_random);
    esp_fill_random(id.mac, sizeof id.mac);
    id.mac[0] = (uint8_t)((id.mac[0] & 0xfc) | 0x02);   /* locally administered unicast */
    id.challenge = ((uint64_t)esp_random() << 32) | esp_random();
    id.nonce = esp_random();
    id.subnet = (uint8_t)(1 + esp_random() % 127);
    s.device_id = ((uint64_t)esp_random() << 32) | esp_random();
    if (ldn_host_net_init(&s.net, &id) != 0) return ESP_ERR_INVALID_ARG;
    if (cfg->app_data && ldn_host_set_app_data(&s.net, cfg->app_data, cfg->app_data_len) < 0) return ESP_ERR_INVALID_SIZE;
    if (ldn_host_data_key(keys, &s.net, s.ccmp_key) != 0) return ESP_FAIL;

    esp_wifi_set_promiscuous(false);
    esp_wifi_stop();
    const bool uplink = cfg->sta_ssid && cfg->sta_ssid[0];
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_set_mode(uplink ? WIFI_MODE_APSTA : WIFI_MODE_AP));
    if (esp_wifi_set_mac(WIFI_IF_AP, id.mac) != ESP_OK) return ESP_FAIL;
    if (!s.netif) {
        s.netif = esp_netif_create_default_wifi_ap();
        esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_AP_START, ap_start_event, NULL);
    }
    if (!s.rx) s.rx = xQueueCreate(2, sizeof(auth_frame));
    if (!s.netif || !s.rx) return ESP_ERR_NO_MEM;
    xQueueReset(s.rx);
    portENTER_CRITICAL(&s_lock);
    s_join_n = s_leave_n = 0;
    portEXIT_CRITICAL(&s_lock);
    s_adv_ok = s_adv_fail = s_assoc_n = s_auth_rx = s_rx_total = s_rx_88b7_other = 0; s_adv_last_err = 0;
    esp_log_level_set(TAG, ESP_LOG_INFO);

    char ssid[33];
    ldn_host_ssid_text(id.ssid, ssid);
    wifi_config_t wc = {0};
    memcpy(wc.ap.ssid, ssid, 32);
    wc.ap.ssid_len = 32;
    wc.ap.channel = (uint8_t)cfg->channel;
    wc.ap.authmode = WIFI_AUTH_WPA2_PSK;
    wc.ap.pairwise_cipher = WIFI_CIPHER_TYPE_CCMP;
    memcpy(wc.ap.password, "kizuna00", 8);          /* unused: LDN authenticates, not the PSK */
    wc.ap.max_connection = (uint8_t)(s.net.max_participants - 1);
    wc.ap.beacon_interval = 100;
    wc.ap.ssid_hidden = 1;                          /* as a Switch's beacons */
    wc.ap.pmf_cfg.capable = false;
    wc.ap.pmf_cfg.required = false;
    if (esp_wifi_set_config(WIFI_IF_AP, &wc) != ESP_OK) return ESP_FAIL;
    esp_wifi_set_protocol(WIFI_IF_AP, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G);   /* no HT, as a Switch */

    if (uplink) {
        if (!s.up_netif) {
            /* The app already owns a static-IP STA netif under the default key; this one is the DHCP uplink. */
            static esp_netif_inherent_config_t base;
            base = (esp_netif_inherent_config_t)ESP_NETIF_INHERENT_DEFAULT_WIFI_STA();
            base.if_key = "KZ_UPLINK"; base.if_desc = "uplink";
            esp_netif_config_t ncfg = ESP_NETIF_DEFAULT_WIFI_STA();
            ncfg.base = &base;
            s.up_netif = esp_netif_new(&ncfg);
            if (s.up_netif) {
                ESP_ERROR_CHECK_WITHOUT_ABORT(esp_netif_attach_wifi_station(s.up_netif));
                ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_set_default_wifi_sta_handlers());
            }
            esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, uplink_event, NULL);
            esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, uplink_event, NULL);
            esp_event_handler_register(IP_EVENT, IP_EVENT_STA_LOST_IP, uplink_event, NULL);
        }
        if (!s.up_netif) return ESP_ERR_NO_MEM;
        wifi_config_t sc = {0};
        strlcpy((char *)sc.sta.ssid, cfg->sta_ssid, sizeof sc.sta.ssid);
        if (cfg->sta_pass) strlcpy((char *)sc.sta.password, cfg->sta_pass, sizeof sc.sta.password);
        sc.sta.threshold.authmode = (cfg->sta_pass && cfg->sta_pass[0]) ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;
        if (esp_wifi_set_config(WIFI_IF_STA, &sc) != ESP_OK) return ESP_FAIL;
        esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW20);   /* the AP shares the radio: no 40 MHz from the router */
        s.uplink = true;
    }

    /* Before esp_wifi_start, which reads the RSN element. */
    if (!wpa_cb) return ESP_ERR_INVALID_STATE;
    s.orig_join = wpa_cb->wpa_ap_join;
    s.orig_remove = wpa_cb->wpa_ap_remove;
    s.orig_get_wpa_ie = wpa_cb->wpa_ap_get_wpa_ie;
    wpa_cb->wpa_ap_join = hook_ap_join;
    wpa_cb->wpa_ap_remove = hook_ap_remove;
    wpa_cb->wpa_ap_get_wpa_ie = hook_ap_get_wpa_ie;
    if (esp_wifi_start() != ESP_OK) { ldn_ap_stop(); return ESP_FAIL; }
    esp_wifi_set_ps(WIFI_PS_NONE);
    if (s.uplink) esp_wifi_connect();

    esp_netif_dhcps_stop(s.netif);
    esp_netif_ip_info_t ip = {0};
    memcpy(&ip.ip.addr, s.net.members[0].ip, 4);
    ip.netmask.addr = ESP_IP4TOADDR(255, 255, 255, 0);
    esp_netif_set_ip_info(s.netif, &ip);

    /* Group key now (index 1, broadcast address); pairwise keys per station on association. */
    static const uint8_t bcast[6] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
    esp_wifi_set_ap_key_internal(WIFI_WPA_ALG_CCMP, bcast, 1, s.ccmp_key, sizeof s.ccmp_key);
    esp_wifi_internal_reg_rxcb(WIFI_IF_AP, ap_rx);

    reencode();
    s.next_advert_us = esp_timer_get_time();
    s.running = true;
    ESP_LOGI(TAG, "hosting on channel %d, ip 169.254.%u.1", cfg->channel, s.net.subnet);
    return ESP_OK;
}

void ldn_ap_poll(void)
{
    if (!s.running) return;
    uint8_t joins[PENDING_MAX][6], leaves[PENDING_MAX][6];
    int nj, nl;
    portENTER_CRITICAL(&s_lock);
    nj = s_join_n; memcpy(joins, s_join_q, (size_t)nj * 6); s_join_n = 0;
    nl = s_leave_n; memcpy(leaves, s_leave_q, (size_t)nl * 6); s_leave_n = 0;
    portEXIT_CRITICAL(&s_lock);
    for (int i = 0; i < nj; i++) on_associated(joins[i]);
    for (int i = 0; i < nl; i++) on_left(leaves[i]);

    static auth_frame f;
    while (xQueueReceive(s.rx, &f, 0) == pdTRUE) handle_auth(&f);

    int64_t now_us = esp_timer_get_time(), now = now_us / 1000;
    if (now_us >= s.next_rxcb_us) { esp_wifi_internal_reg_rxcb(WIFI_IF_AP, ap_rx); s.next_rxcb_us = now_us + 500000; }
    if (now_us >= s.next_advert_us) { send_advert(); s.next_advert_us = now_us + ADVERT_PERIOD_US; }
    if (now < s.probe_until_ms && now >= s.next_probe_ms) {
        for (int i = 1; i < LDN_HOST_MAX_MEMBERS; i++) if (s.net.members[i].used) probe_arp(i);
        s.next_probe_ms = now + ARP_PROBE_EVERY_MS;
    }
    for (int i = 0; i < LDN_HOST_MAX_MEMBERS; i++) {
        assoc_station *a = &s.assoc[i];
        if (!a->used || ldn_host_member_find(&s.net, a->mac) >= 0 || now - a->assoc_ms < ASSOC_TIMEOUT_MS) continue;
        esp_wifi_ap_deauth_internal(a->mac, WIFI_REASON_AUTH_LEAVE);
        a->used = false;
    }
}

void ldn_ap_stop(void)
{
    if (!s.running && !s.orig_join) return;
    for (int i = 1; i < LDN_HOST_MAX_MEMBERS; i++)
        if (s.net.members[i].used) esp_wifi_ap_deauth_internal(s.net.members[i].mac, WIFI_REASON_AUTH_LEAVE);
    if (wpa_cb && s.orig_join) {
        wpa_cb->wpa_ap_join = s.orig_join;
        wpa_cb->wpa_ap_remove = s.orig_remove;
        wpa_cb->wpa_ap_get_wpa_ie = s.orig_get_wpa_ie;
    }
    esp_wifi_stop();
    memset(s.ccmp_key, 0, sizeof s.ccmp_key);
    s.keys = NULL;
    s.running = false;
    s.uplink = false; s.up_connected = false; s.up_ip = 0;
    s.orig_join = NULL;
    ESP_LOGI(TAG, "stopped");
}

bool ldn_ap_running(void) { return s.running; }

esp_err_t ldn_ap_set_app_data(const uint8_t *data, size_t len)
{
    int r = ldn_host_set_app_data(&s.net, data, len);
    if (r < 0) return ESP_ERR_INVALID_SIZE;
    if (r > 0 && s.running) reencode();
    return ESP_OK;
}

const uint8_t *ldn_ap_ssid(void) { return s.net.ssid; }

void ldn_ap_identity(uint8_t mac[6], uint8_t ip[4])
{
    if (mac) memcpy(mac, s.net.members[0].mac, 6);
    if (ip) memcpy(ip, s.net.members[0].ip, 4);
}

esp_netif_t *ldn_ap_netif(void) { return s.netif; }
