/* relay_engine.c — see relay_engine.h. Mirrors gba_bridge's UP FIFO / DOWN slot semantics without a GBA:
 *   tick  : pop the next partner slot (7 u16) or hand the sim an all-zero idle slot
 *   feed  : print each NON-IDLE Switch slot once per frame as {"ev":"slot","mp":i,"hex":...}
 * Printing runs on the brain task; a line is ~50 bytes and the Switch sends <= 5 slots per 16.7 ms frame, so
 * the worst case is ~15 KB/s on the USB console, well under its capacity. Idle (all-zero) slots are not
 * printed: the partner synthesises its own idle, as GB-Link's relay does. */
#include "relay_engine.h"
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sim.h"
#include "ni.h"
#include "ldn_brain.h"
#include "kz_mode_ble.h"

#define UPQ 32
static uint8_t  s_upq[UPQ][14];
static volatile int s_up_head, s_up_tail;        /* console task pushes, brain task pops; indexes only */
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static unsigned s_up_in, s_up_out, s_up_drop, s_down, s_down_idle, s_ticks, s_ble_drop;
static relay_via_t s_via;
static sim_engine s_eng;
static bool s_installed;

static int upq_depth(void) { return (s_up_head - s_up_tail + UPQ) % UPQ; }

bool relay_engine_push(const uint8_t slot14[14])
{
    bool ok;
    portENTER_CRITICAL(&s_mux);
    int next = (s_up_head + 1) % UPQ;
    if ((ok = next != s_up_tail)) { memcpy(s_upq[s_up_head], slot14, 14); s_up_head = next; s_up_in++; }
    else s_up_drop++;
    portEXIT_CRITICAL(&s_mux);
    return ok;
}

static void eng_tick(void *ctx, uint16_t words[7])
{
    (void)ctx; s_ticks++;
    uint8_t s[14]; bool have = false;
    portENTER_CRITICAL(&s_mux);
    if (s_up_tail != s_up_head) { memcpy(s, s_upq[s_up_tail], 14); s_up_tail = (s_up_tail + 1) % UPQ; s_up_out++; have = true; }
    portEXIT_CRITICAL(&s_mux);
    for (int i = 0; i < 7; i++) words[i] = have ? (uint16_t)(s[2 * i] | (s[2 * i + 1] << 8)) : 0;
}

static int eng_has_backlog(void *ctx) { (void)ctx; return upq_depth() > 0; }

static void eng_feed(void *ctx, const gba_in *rec)
{
    (void)ctx;
    static const char H[] = "0123456789abcdef";
    for (int i = 0; i < rec->nslots && i < GBA_MAX_SLOTS; i++) {
        const uint8_t *s = rec->slots[i];
        bool idle = true;
        for (int k = 0; k < 14; k++) if (s[k]) { idle = false; break; }
        if (idle) { s_down_idle++; continue; }
        char hx[29];
        for (int k = 0; k < 14; k++) { hx[2 * k] = H[s[k] >> 4]; hx[2 * k + 1] = H[s[k] & 15]; }
        hx[28] = 0;
        s_down++;
        if (s_via == RELAY_VIA_BLE) {
            /* 15 bytes on the data characteristic: [mp][14-byte slot]. Non-blocking; a miss is counted, not retried
             * (the next frame carries the current state anyway, as GB-Link's relay model assumes). */
            uint8_t pkt[15]; pkt[0] = (uint8_t)i; memcpy(pkt + 1, s, 14);
            if (!kz_mode_ble_notify_data(pkt, sizeof pkt)) s_ble_drop++;
        } else {
            printf("{\"ev\":\"slot\",\"mp\":%d,\"hex\":\"%s\"}\n", i, hx);
        }
    }
}

void relay_engine_install(relay_via_t via)
{
    s_via = via;
    if (s_installed) { printf("  [relay] transport -> %s\n", via == RELAY_VIA_BLE ? "BLE" : "USB"); return; }
    memset(&s_eng, 0, sizeof s_eng);
    s_eng.tick           = eng_tick;
    s_eng.has_backlog    = eng_has_backlog;
    s_eng.feed_in_frame  = eng_feed;
    s_eng.ni_started     = 1;
    s_eng.ni_activity    = NI_ACTIVITY_TRADE;
    /* Identity: leave the LinkPlayer fields zero so sim.c applies the SAME defaults the T0 stub engine used
     * (the Switch accepted those); a different name/tid/version is one variable fewer in T1. */
    ldn_brain_set_engine(&s_eng);
    s_installed = true;
    printf("  [relay] relay engine installed, transport %s\n", via == RELAY_VIA_BLE ? "BLE (data char: notify out, write in)" : "USB (slot op in, ev slot out)");
}

bool relay_engine_installed(void) { return s_installed; }

void relay_engine_status(char *out, size_t cap)
{
    uint16_t itvl = 0, mtu = 0; kz_mode_ble_link_info(&itvl, &mtu);
    snprintf(out, cap, "\"relay\":{\"on\":%d,\"via\":\"%s\",\"up_in\":%u,\"up_out\":%u,\"up_drop\":%u,\"depth\":%d,\"down\":%u,\"down_idle\":%u,\"ble_drop\":%u,\"ticks\":%u,\"ble\":{\"connected\":%d,\"itvl_1p25ms\":%u,\"mtu\":%u}}",
             s_installed, s_via == RELAY_VIA_BLE ? "ble" : "usb", s_up_in, s_up_out, s_up_drop, upq_depth(), s_down, s_down_idle, s_ble_drop, s_ticks,
             kz_mode_ble_connected(), itvl, mtu);
}
