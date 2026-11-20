/* trade_host_session.h — the adapter HOSTS a Direct Corner trade room and trades with the
 * Switch. Glue only: ldn_ap (room + optional Wi-Fi uplink) + dist_udp (Pia datagrams on the AP) + pia_host
 * (Pia leader, cport) + trade_host (the trade engine, cport). The partner's Pokemon arrive from the other
 * side of the relay as 100-byte party-mon blobs (th_session_set_mon) and what the Switch offers goes back out
 * through the emit callback ({"ev":"th_received",...}). */
#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "ldn_advert.h"

typedef void (*th_emit_fn)(const char *json);

typedef struct {
    const char *name;        /* our trainer name, <= 7 chars */
    uint16_t    tid, sid;
    int         trades;      /* 1..6 */
    int         slot;        /* first offered party slot */
    const char *sta_ssid;    /* optional Wi-Fi uplink alongside the room */
    const char *sta_pass;
} th_session_cfg;

void th_session_set_emit(th_emit_fn fn);
/* Stage one party member (100 bytes, pokeldn Mon.raw / .ek3 form) before th_session_start. */
bool th_session_set_mon(int index, const uint8_t mon[100]);
void th_session_clear_party(void);
int  th_session_party_count(void);

/* Keys are borrowed until stop. Starts the room, the Pia leader and the session task. */
bool th_session_start(const ldn_keys *keys, const th_session_cfg *cfg);
/* Frame-level remote trade over Bluetooth (proxy): the same room and Pia leader, but the Switch's child rows are shipped to a
 * remote engine (BLE data characteristic -> phone -> relay) and its parent rows are played back; no party is needed. The wire format
 * is cport/proxy_activity.h. Register the BLE data-notify function once at boot (th_session_set_proxy_tx) and forward every BLE data
 * write that is not the 4-byte ping echo to th_session_proxy_data (true = consumed). */
/* One BLE data notification carrying a whole bundle (<= 240 bytes); false = the stack could not take it now (the bundle is kept and retried). */
typedef bool (*th_proxy_tx_fn)(const uint8_t *msg, size_t len);
void th_session_set_proxy_tx(th_proxy_tx_fn fn);
/* Bundle pacing: at most one BLE notification per flush_ms (default 15, 0 = every session pass); applies to the running session too. */
void th_session_set_proxy_flush_ms(uint32_t ms);
/* Where to read the negotiated BLE connection interval (1.25 ms units) and MTU from, for the 1 Hz th_tick. */
typedef void (*th_proxy_link_fn)(uint16_t *itvl_1p25ms, uint16_t *mtu);
void th_session_set_proxy_link(th_proxy_link_fn fn);
bool th_session_start_proxy(const ldn_keys *keys, const th_session_cfg *cfg);
bool th_session_proxy_data(const uint8_t *data, size_t len);
void th_session_stop(void);
bool th_session_active(void);
/* Emulate the internet for the frame-level proxy experiment: hold every datagram to and from the Switch for delay_ms (+ up to
 * jitter_ms extra) each way. 0 disables. Total added round trip = 2 x delay_ms. */
void th_session_set_delay(int delay_ms, int jitter_ms);
/* One th_* status line for the console. */
void th_session_status(char *buf, size_t n);
