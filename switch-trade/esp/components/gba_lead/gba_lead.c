/* SPDX-License-Identifier: AGPL-3.0-only
 * Derived from GB-Link-Switch-LDN (https://github.com/GB-Link/GB-Link-Switch-LDN), Copyright (c) the GB-Link-Switch-LDN authors.
 * Changes Copyright (c) 2026 Kizuna. See NOTICE.
 */
/* gba_lead.c -- see gba_lead.h. The BR_HOST role of GB-Link-Switch-LDN firmware/common/pia_bridge.c (commit
 * c972db4, AGPL-3.0, https://github.com/GB-Link/GB-Link-Switch-LDN; see NOTICE), on kizuna's own radio: ldn_ap (the
 * Switch-facing LDN room, proven on the C3) and dist_udp instead of GB-Link's ldn_host/ldn_udp, rfu1_bridge
 * instead of pico_link. The room record, the connect/accept/close sequence, the timers and the lead shim are GB-Link's. */
#include "gba_lead.h"

#include <stdio.h>
#include <string.h>
#include "esp_heap_caps.h"
#include "esp_random.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ldn_ap.h"
#include "dist_udp.h"
#include "rfu1.h"
#include "rfu1_bridge.h"
#include "glp_host.h"
#include "gl_lead_shim.h"

/* No GBA beacon for this long closes the group (only before a member joins: EndHost stops beacons). */
#define BEACON_TIMEOUT_MS 3000
/* The adapter drops a client silent for 4 s; keepalive interval. */
#define CLIENT_ACK_MS 500
/* Room kept up after the GBA closes the link, for the Switch's own close; then the client is dropped and the teardown
   given time. */
#define CLOSE_GRACE_MS 15000
#define TEARDOWN_MS 6000

/* Room application data: Pia's 92-byte system header, then the room record as base-85 text. */
#define APP_HEADER 92
#define APP_RECORD 24
#define PIA_SYSTEM_VERSION 22
#define GAME_VERSION 88

static struct
{
    bool active;
    glp_host_t *host;                 /* heap: ~30 KB, only while leading */
    const ldn_keys *keys;
    uint16_t gba_devid;
    uint8_t gba_beacon[24];
    int64_t gba_beacon_ms;
    uint16_t client_devid;
    uint8_t client_slot;
    bool client_requested, client_connected;
    int members;                      /* LDN members other than us */
    int member_index;                 /* the LDN member the session serves, -1 none */
    uint8_t member_ip[4], member_mac[6];
    int64_t next_client_ack;
    int64_t closing_ms;               /* when the GBA closed the link, 0 while it is open */
    int64_t session_ms;
    int64_t next_tick_us, tick_fraction;   /* 59.727 Hz: microseconds plus parts per 59727 */
    int frames_to_gba, frames_from_gba, to_gba_lost;
    uint8_t channel;
    char name[33];
    uint16_t stall_low;
    int64_t stall_since, next_report_ms;
    bool stall_reported;
    uint8_t app[GLP_HOST_APP_DATA];
    int app_len;
} g;

static int64_t now_ms(void) { return esp_timer_get_time() / 1000; }

static void ip_text(const uint8_t ip[4], char out[16]) { snprintf(out, 16, "%u.%u.%u.%u", ip[0], ip[1], ip[2], ip[3]); }
static bool ip_parse(const char *text, uint8_t out[4])
{
    unsigned a, b, c, d;
    if (sscanf(text, "%u.%u.%u.%u", &a, &b, &c, &d) != 4 || a > 255 || b > 255 || c > 255 || d > 255) return false;
    out[0] = (uint8_t)a; out[1] = (uint8_t)b; out[2] = (uint8_t)c; out[3] = (uint8_t)d;
    return true;
}

/* ---- RFU1 toward the GBA's adapter ------------------------------------------------ */

static const uint8_t kRfu1[4] = {'R', 'F', 'U', '1'};

static void to_pico(const uint8_t *frame, size_t length)
{
    if (!rfu1_bridge_send_raw(frame, (uint32_t)length)) ++g.to_gba_lost;
    ++g.frames_to_gba;
}

static void send_command(uint32_t type, uint32_t header)
{
    uint8_t f[16] = {0};
    memcpy(f, kRfu1, 4);
    glb_wb32(f + 4, type);
    glb_wb32(f + 8, header);
    to_pico(f, sizeof(f));
}

/* ---- the room's application data (GB-Link pia_bridge.c) ----------------------------- */

static uint8_t base85_digit(uint32_t v)
{
    uint8_t c = (uint8_t)(0x23 + v % 85);
    return c >= 0x5c ? (uint8_t)(c + 1) : c;   /* skips the backslash */
}

/* Four-byte little-endian groups, five digits each, least significant digit first. */
static int base85_encode(const uint8_t *in, int length, uint8_t *out)
{
    int n = 0;
    for (int i = 0; i + 4 <= length; i += 4)
    {
        uint32_t v = glb_u32(in + i);
        for (int d = 0; d < 5; ++d, v /= 85) out[n++] = base85_digit(v);
    }
    return n;
}

/* The trainer name in a beacon, as text. */
static void text_of_uname(const uint8_t *uname, char *out, size_t cap)
{
    size_t n = 0;
    for (int i = 0; i < 8 && n + 1 < cap; ++i)
    {
        uint8_t c = uname[i];
        if (c == 0xff) break;
        if (c >= 0xbb && c <= 0xd4) out[n++] = (char)('A' + c - 0xbb);
        else if (c >= 0xd5 && c <= 0xee) out[n++] = (char)('a' + c - 0xd5);
        else if (c >= 0xa1 && c <= 0xaa) out[n++] = (char)('0' + c - 0xa1);
        else if (c == 0) out[n++] = ' ';
        else out[n++] = '?';
    }
    out[n] = 0;
    if (n == 0) snprintf(out, cap, "GBA");
}

/* RFU beacon (the GBA's): serial at 0, then RfuGameData: compatibility word at 2 (language 0-3, has news 4, has card 5,
   can link nationally 7, national dex 8, game clear 9, version 10-13), trainer id 4, partner info 6-9, trade species 10
   (10 bits) and type (6 bits), activity 12 (bit 7 started), gender and trade level 13; name at 16.
   Room record (the Switch's): trainer id 0, name 2, adapter id 10, partner info 12-15, word 16 (activity 0-6, bit 7,
   version 8-10, language 11-13, has card 14, started 15), 18 (progress 0-1, trade type 2-7), gender and trade level 19,
   trade species 22. */
static void record_of_beacon(uint8_t *rec, const uint8_t *beacon, uint16_t adapter_id, bool started)
{
    const uint16_t compat = glb_u16(beacon + 2);
    const uint8_t nationally = (compat & 0x80) ? 3 : 0;
    memset(rec, 0, APP_RECORD);
    memcpy(rec, beacon + 4, 2);
    memcpy(rec + 2, beacon + 16, 8);
    glb_w16(rec + 10, adapter_id);
    memcpy(rec + 12, beacon + 6, 4);
    glb_w16(rec + 16, (uint16_t)((beacon[12] & 0x7f) | (compat & 0x80) | ((compat >> 10) & 7) << 8 |
                                 (compat & 7) << 11 | (compat & 0x20) << 9 | (started ? 0x8000 : 0)));
    rec[18] = (uint8_t)((beacon[11] & 0xfc) | ((compat >> 8) & 3) | nationally);
    rec[19] = beacon[13];
    glb_w16(rec + 22, glb_u16(beacon + 10) & 0x3ff);
}

static int build_app_data(uint8_t *app, const uint8_t *beacon, uint16_t adapter_id, int players, bool started)
{
    memset(app, 0, APP_HEADER);
    glb_wb16(app, APP_HEADER);
    app[2] = PIA_SYSTEM_VERSION;
    glb_wb16(app + 3, GAME_VERSION);
    app[0x15] = 1;                              /* player limit on */
    app[0x16] = (uint8_t)players;
    char name[33];
    text_of_uname(beacon + 16, name, sizeof(name));
    size_t n = strlen(name);
    glb_wb32(app + 0x17, (uint32_t)n);
    app[0x1b] = 1;                              /* UTF-8 */
    memcpy(app + 0x1c, name, n);
    uint8_t rec[APP_RECORD];
    record_of_beacon(rec, beacon, adapter_id, started);
    return APP_HEADER + base85_encode(rec, APP_RECORD, app + APP_HEADER);
}

/* Rebuild the application data: the GBA's beacon, the member count, and started once a client is in (the adapter stops
   beaconing then). */
static void update_app_data(void)
{
    if (!g.host) return;
    const bool started = (g.gba_beacon[12] & 0x80) || g.client_connected;
    g.app_len = build_app_data(g.app, g.gba_beacon, g.gba_devid, 1 + g.members, started);
    ldn_ap_set_app_data(g.app, (size_t)g.app_len);
    glp_host_set_app_data(g.host, g.app, g.app_len);
}

/* ---- Pia host callbacks ------------------------------------------------------------- */

static void on_send(const uint8_t *datagram, size_t length, const char *destination, void *user)
{
    (void)user;
    uint8_t ip[4];
    if (ip_parse(destination, ip)) dist_udp_send(ip, datagram, (int)length);
}

/* Shim-built parent frames to the Switch. */
static void to_child(const uint8_t *frames, size_t length)
{
    for (size_t o = 0; o + 73 <= length; o += 73) glp_host_parent_frame(g.host, frames + o, 73);
}

/* A Switch child frame to the GBA's adapter, through the shim. */
static void on_child_frame(const uint8_t *payload, size_t length, void *user)
{
    (void)user;
    if (!g.client_connected || length > 16) return;
    uint8_t f[104] = {0};
    memcpy(f, kRfu1, 4);
    glb_wb32(f + 4, RFU1_CLIENT_SEND);
    glb_wb32(f + 8, (uint32_t)g.client_devid | ((uint32_t)g.client_slot << 16) | ((uint32_t)length << 24));
    memcpy(f + 12, payload, length);
    static uint8_t reply[146];
    bool forward = true;
    size_t r = gls_lead_child(f + 12, length, now_ms(), reply, sizeof(reply), &forward);
    if (forward) to_pico(f, sizeof(f));
    to_child(reply, r);
}

static void on_log(const char *message, void *user) { (void)user; printf("  [lead] pia: %s\n", message); }

static void rx_datagram(const uint8_t src_ip[4], const uint8_t *data, int len)
{
    if (!g.host || len <= 0) return;
    char ip[16];
    ip_text(src_ip, ip);
    glp_host_receive(g.host, data, (size_t)len, ip);
}

/* ---- LDN members (the Switch) -------------------------------------------------------- */

static void print_session_summary(void)
{
    const glp_host_t *h = g.host;
    const int s = (int)((now_ms() - g.session_ms) / 1000);
    printf("  [lead] session %d s: Switch datagrams in %d out %d, frames to GBA %d (lost %d) from GBA %d, parent %d child %d, "
           "skipped %d shed %d dropped %d resends %d decrypt_fail %d, lowest heap %u\n",
           s, h->received, h->sent, g.frames_to_gba, g.to_gba_lost, g.frames_from_gba, h->parent_frames, h->child_frames,
           h->child_skipped, h->shed, h->parent_dropped, h->resends, h->decrypt_failures,
           (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT));
}

static void on_member(const ldn_ap_member *m, bool joined)
{
    if (!g.host) return;
    char ip[16];
    ip_text(m->ip, ip);
    if (joined)
    {
        ++g.members;
        dist_udp_peer(m->ip, m->mac);
        if (g.host->state != 0)
        {
            printf("  [lead] Switch %s joined the room too; only the first one is served\n", m->name);
            update_app_data();
            return;
        }
        printf("  [lead] Switch %s joined the room as %s (%d)\n", m->name, ip, m->index);
        g.member_index = m->index;
        memcpy(g.member_ip, m->ip, 4);
        memcpy(g.member_mac, m->mac, 6);
        update_app_data();
        glp_host_station_joined(g.host, m->mac, ip, m->index);
        return;
    }
    if (g.members > 0) --g.members;
    dist_udp_peer(m->ip, NULL);
    if (g.host->state == 0 || memcmp(m->mac, g.member_mac, 6) != 0) { update_app_data(); return; }
    printf("  [lead] Switch %s left the room\n", m->name);
    print_session_summary();
    g.member_index = -1;
    memset(g.member_mac, 0, 6);
    glp_host_station_left(g.host);
    if (g.client_connected) send_command(RFU1_DISCONNECT, (uint32_t)g.client_devid | ((uint32_t)g.client_slot << 16));
    g.client_connected = g.client_requested = false;
    g.closing_ms = 0;
    update_app_data();
}

/* ---- inbound from the GBA's adapter (rfu1_bridge's poll, this task) ----------------- */

static void on_rfu1(const rfu1_pkt_t *p)
{
    ++g.frames_from_gba;
    switch (p->ptype)
    {
    case RFU1_BROADCAST:
    {
        /* The GBA leads its group; beacon words arrive big-endian. */
        uint8_t beacon[24];
        for (int i = 0; i < 6; ++i) glb_w32(beacon + i * 4, glb_b32(p->payload + i * 4));
        const bool changed = memcmp(beacon, g.gba_beacon, sizeof(beacon)) != 0 || g.gba_devid != rfu1_devid(p->hdata);
        g.gba_devid = rfu1_devid(p->hdata);
        memcpy(g.gba_beacon, beacon, sizeof(beacon));
        g.gba_beacon_ms = now_ms();
        if (changed) update_app_data();
        break;
    }
    case RFU1_CONNECT_ACK:
        if (g.client_requested && !g.client_connected)
        {
            g.client_devid = rfu1_devid(p->hdata);
            g.client_slot = rfu1_clnum(p->hdata);
            g.client_connected = true;
            g.session_ms = now_ms();
            g.next_client_ack = now_ms() + CLIENT_ACK_MS;
            gls_reset();
            glp_host_accept(g.host, true);
            update_app_data();
            printf("  [lead] GBA accepted the Switch as client %u devid=%04x\n", g.client_slot, g.client_devid);
        }
        break;
    case RFU1_CONNECT_NACK:
        if (g.client_requested && !g.client_connected)
        {
            g.client_requested = false;
            glp_host_accept(g.host, false);
            printf("  [lead] GBA refused the Switch\n");
        }
        break;
    case RFU1_HOST_SEND:
    {
        size_t n = rfu1_host_len(p->hdata);
        if (n > 92) n = 92;
        static uint8_t payload[92];
        memcpy(payload, p->payload, n);
        gls_lead_parent(payload, n, now_ms());
        glp_host_parent_frame(g.host, payload, n);
        break;
    }
    case RFU1_DISCONNECT:
        if (g.client_connected && rfu1_devid(p->hdata) == g.client_devid)
        {
            g.client_connected = g.client_requested = false;
            g.closing_ms = now_ms();
            update_app_data();
            printf("  [lead] GBA closed the link; keeping the room while the Switch finishes (max %d s)\n",
                   (CLOSE_GRACE_MS + TEARDOWN_MS) / 1000);
        }
        break;
    default:
        break;   /* CLIENT_ACK (the adapter acks our CLIENT_SENDs), FLOWCTL */
    }
}

/* GB-Link's check_host_stall: signal and memory tell a Switch out of range from a board out of buffers. */
static void check_host_stall(int64_t now)
{
    const int pending = glp_reliable_pending(&g.host->reliable);
    if (pending == 0 || g.host->reliable.low != g.stall_low)
    {
        if (g.stall_reported) printf("  [lead] Switch is acknowledging our frames again\n");
        g.stall_low = g.host->reliable.low;
        g.stall_since = now;
        g.stall_reported = false;
        return;
    }
    if (g.stall_reported || now - g.stall_since < 1500) return;
    g.stall_reported = true;
    wifi_sta_list_t stations;
    int rssi = 0;
    if (esp_wifi_ap_get_sta_list(&stations) == ESP_OK && stations.num > 0) rssi = stations.sta[0].rssi;
    printf("  [lead] Switch has not acknowledged our frames for %d ms (%d waiting, %d resends, retry after %d ms; "
           "signal %d dBm, heap %u, lowest %u)\n",
           (int)(now - g.stall_since), pending, g.host->resends, (int)glp_reliable_rto(&g.host->reliable), rssi,
           (unsigned)heap_caps_get_free_size(MALLOC_CAP_8BIT), (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT));
}

/* ---- session ---------------------------------------------------------------------- */

static void finish(const char *why)
{
    printf("  [lead] %s -> restarting for a clean heap\n", why);
    if (g.host && g.session_ms) print_session_summary();
    rfu1_bridge_set_lead_sink(NULL);
    dist_udp_stop();
    ldn_ap_stop();
    fflush(stdout);
    vTaskDelay(pdMS_TO_TICKS(200));
    esp_restart();
}

bool gba_lead_wanted(const uint8_t beacon[24])
{
    const uint8_t activity = beacon[12] & 0x7f;
    return activity == 4 || activity == 1 || activity == 2;
}

bool gba_lead_start(const ldn_keys *keys, const uint8_t beacon[24], uint16_t gba_devid)
{
    if (g.active || !keys) return false;
    memset(&g, 0, sizeof(g));
    g.member_index = -1;
    g.keys = keys;
    g.gba_devid = gba_devid;
    memcpy(g.gba_beacon, beacon, 24);
    g.gba_beacon_ms = now_ms();
    g.host = heap_caps_calloc(1, sizeof(glp_host_t), MALLOC_CAP_8BIT);
    if (!g.host)
    {
        printf("  [lead] no memory for the Pia host (%u B, largest block %u)\n", (unsigned)sizeof(glp_host_t),
               (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
        return false;
    }
    static const uint8_t kChannels[3] = {1, 6, 11};
    g.channel = kChannels[esp_random() % 3];
    text_of_uname(g.gba_beacon + 16, g.name, sizeof(g.name));
    g.app_len = build_app_data(g.app, g.gba_beacon, g.gba_devid, 1, (g.gba_beacon[12] & 0x80) != 0);
    ldn_ap_config cfg = { .channel = g.channel, .host_name = g.name, .app_data = g.app, .app_data_len = (size_t)g.app_len };
    esp_err_t e = ldn_ap_start(keys, &cfg, on_member);
    if (e != ESP_OK)
    {
        printf("  [lead] could not host a room (%s)\n", esp_err_to_name(e));
        heap_caps_free(g.host); g.host = NULL;
        return false;
    }
    if (dist_udp_start(rx_datagram) != ESP_OK)
    {
        printf("  [lead] could not open the Pia socket\n");
        ldn_ap_stop();
        heap_caps_free(g.host); g.host = NULL;
        return false;
    }
    uint8_t mac[6], ip[4];
    char ipt[16];
    ldn_ap_identity(mac, ip);
    ip_text(ip, ipt);
    glp_host_init(g.host, ldn_ap_ssid(), mac, ipt, on_send, on_child_frame, on_log, NULL);
    /* The WA carries the adapter id the record names. */
    g.host->adapter_id = g.gba_devid;
    g.host->sheddable = gls_parent_sheddable;
    glp_host_set_app_data(g.host, g.app, g.app_len);
    gls_reset();
    g.next_tick_us = esp_timer_get_time();
    g.active = true;
    rfu1_bridge_set_lead_sink(on_rfu1);
    printf("  [lead] GBA %s is leading activity %u: hosting a room on channel %d (Pia host %u B, free %u largest %u)\n",
           g.name, (unsigned)(g.gba_beacon[12] & 0x7f), g.channel, (unsigned)sizeof(glp_host_t),
           (unsigned)heap_caps_get_free_size(MALLOC_CAP_8BIT), (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
    return true;
}

bool gba_lead_active(void) { return g.active; }

void gba_lead_poll(void)
{
    if (!g.active) return;
    dist_udp_poll();
    const int64_t now = now_ms();
    const int64_t now_us = esp_timer_get_time();
    if (now_us >= g.next_tick_us)
    {
        glp_host_tick(g.host, now);
        g.tick_fraction += 1000000000LL;
        g.next_tick_us += g.tick_fraction / 59727;
        g.tick_fraction %= 59727;
        if (g.next_tick_us < now_us) g.next_tick_us = now_us + 16742;
        /* Shim frames are added only while the queue is nearly empty, so the replayed block is not crowded out. */
        if (g.client_connected && g.host->parent_count < 2)
        {
            static uint8_t repeat[73];
            to_child(repeat, gls_lead_inject(now, repeat, sizeof(repeat)));
        }
    }
    if (g.host->state == 3) check_host_stall(now);
    if (now >= g.next_report_ms)
    {
        g.next_report_ms = now + 5000;
        printf("  [lead] members=%d client=%d pia=%d dg in=%d out=%d decrypt_fail=%d to_gba=%d from_gba=%d parent=%d child=%d "
               "resends=%d shed=%d free=%u\n",
               g.members, g.client_connected, g.host->state, g.host->received, g.host->sent, g.host->decrypt_failures,
               g.frames_to_gba, g.frames_from_gba, g.host->parent_frames, g.host->child_frames, g.host->resends, g.host->shed,
               (unsigned)heap_caps_get_free_size(MALLOC_CAP_8BIT));
    }
    if (glp_host_connect_pending(g.host) && !g.client_requested)
    {
        g.client_requested = true;
        send_command(RFU1_CONNECT_REQ, g.gba_devid);
        printf("  [lead] Switch asks to join the group: connecting it to the GBA\n");
    }
    if (g.client_connected && g.host->child_disconnected)
    {
        send_command(RFU1_DISCONNECT, (uint32_t)g.client_devid | ((uint32_t)g.client_slot << 16));
        g.client_connected = g.client_requested = false;
        printf("  [lead] Switch left the group\n");
    }
    if (g.client_connected && now >= g.next_client_ack)
    {
        send_command(RFU1_CLIENT_ACK, (uint32_t)g.client_devid | ((uint32_t)g.client_slot << 16));
        g.next_client_ack = now + CLIENT_ACK_MS;
    }
    if (g.closing_ms)
    {
        if (now - g.closing_ms > CLOSE_GRACE_MS + TEARDOWN_MS) finish("the room's close grace is over");
        else if (now - g.closing_ms > CLOSE_GRACE_MS && !g.host->child_disconnected) glp_host_disconnect_child(g.host);
    }
    else if (!g.client_connected && now - g.gba_beacon_ms > BEACON_TIMEOUT_MS)
        finish(g.session_ms ? "the GBA closed its group after the session" : "the GBA closed its group");
}

void gba_lead_status(char *out, size_t cap)
{
    if (!g.active) { snprintf(out, cap, "\"lead\":{\"on\":0}"); return; }
    const glp_host_t *h = g.host;
    snprintf(out, cap,
             "\"lead\":{\"on\":1,\"activity\":%u,\"ch\":%u,\"members\":%d,\"pia\":%d,\"requested\":%d,\"connected\":%d,"
             "\"closing\":%d,\"rx\":%d,\"tx\":%d,\"to_gba\":%d,\"from_gba\":%d,\"shed\":%d,\"resends\":%d}",
             (unsigned)(g.gba_beacon[12] & 0x7f), g.channel, g.members, h->state, g.client_requested, g.client_connected,
             g.closing_ms != 0, h->received, h->sent, g.frames_to_gba, g.frames_from_gba, h->shed, h->resends);
}
