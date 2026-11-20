/* rfu1_bridge.c — see include/rfu1_bridge.h. */
#include "rfu1_bridge.h"
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_timer.h"
#include "esp_random.h"
#include "gblink.h"
#include "rfu1.h"
#include "gba_bridge.h"

/* Our room id toward the GBA. The C6 presented 0x00002abc as the peer id; the RP2040 synthesizes the GBA's
 * candidate word 0 as (devid | next_slot << 16), so devid = 0x2abc, next_slot = 0 reproduces it exactly. */
#define OUR_DEVID       0x2abcu
#define BEACON_US       250000      /* RP2040 room table ages entries out at 4.25 s; its own rate is 500 ms */
#define FLOWCTL_LOG_MAX 20
#define UP_BP_HIGH      10          /* host link: hold the GBA side's echoes at this UP queue depth */
#define UP_BP_LOW       2           /* ... and release at or below this */

typedef struct { uint8_t len; uint8_t data[GBL_MAX_PAYLOAD]; } chunk_t;

static QueueHandle_t s_rxq;         /* data-channel frames from the gblink RX task */
static rfu1_rx_t     s_rx;
static bool          s_up;
static uint16_t      s_last_status;
static int64_t       s_last_beacon_us, s_last_rx_us;

/* GBA state as seen here */
static uint32_t s_gba_beacon[6];    /* the GBA's own 0x16 words (BROADCAST payload) = beacon template */
static bool     s_have_beacon, s_browsing;
static uint16_t s_gba_devid;        /* devid we assigned the child in CONNECT_ACK */
static bool     s_connected;        /* CONNECT_ACK sent, no DISCONNECT since */
static bool     s_hold;             /* RP2040 FLOWCTL backpressure */
static uint32_t s_n_uni_suppressed; /* host link: idle UNI frames not sent (the GBA side makes them) */
static uint8_t  s_login_state;      /* telemetry 0x0E byte 3 (>= 2 = past login) */
static int64_t  s_gba_seen_us;      /* last time the RP2040's GBA-side counters moved (GBA switched off = silence) */
static unsigned s_n_bc_in, s_n_cs, s_n_hs, s_n_acks, s_n_bc_out, s_n_fc, s_n_tele, s_n_junk, s_n_idle_cs;
/* (the GBA leads): the GBA's own group, from its BROADCASTs, and the lead session that takes every packet once it runs. */
static uint16_t s_gba_host_devid;
static int64_t  s_gba_bc_us;
static void   (*s_lead_sink)(const rfu1_pkt_t *p);

/* ---- UART side ---- */
static void on_data(const uint8_t *p, size_t n, void *ctx) {
    (void)ctx;
    chunk_t c; c.len = (uint8_t)n; memcpy(c.data, p, n);
    if (xQueueSend(s_rxq, &c, 0) != pdTRUE) printf("  [rfu1] rx queue full, chunk dropped\n");
}
static void on_status(uint16_t st, void *ctx) { (void)ctx; s_last_status = st; }

static bool send_packet(const uint8_t *pkt, uint32_t len) {
    /* The RP2040 parses ch1 as a byte stream: any chunking <= 64 B is fine. */
    for (uint32_t off = 0; off < len; off += GBL_MAX_PAYLOAD) {
        uint32_t k = len - off < GBL_MAX_PAYLOAD ? len - off : GBL_MAX_PAYLOAD;
        if (!gbl_send(GBL_CH_DATA, pkt + off, k)) return false;
    }
    return true;
}

/* ---- the parent frame toward the GBA (bytes = what the C6 answered to 0x26, minus the count word) ---- */
static bool host_send_words(const uint32_t *w, int nw) {
    /* w[0] = count header (bytes), w[1..] = frame bytes packed LE (gba_bridge_build_recv's layout). */
    uint32_t nb = w[0] & 0x7Fu;
    if (!nb) return false;
    if (nb > RFU1_SEND_DATA - 2) nb = RFU1_SEND_DATA - 2;   /* RP2040 sends <= 90 itself; stay inside */
    uint8_t bytes[RFU1_SEND_DATA];
    for (uint32_t i = 0; i < nb; i++) bytes[i] = (uint8_t)(w[1 + i / 4] >> (8 * (i % 4)));
    uint8_t pkt[RFU1_MAX_LEN];
    uint32_t len = rfu1_build_host_send(pkt, bytes, nb);
    if (!send_packet(pkt, len)) return false;
    s_n_hs++;
    (void)nw;
    return true;
}

static bool host_send_bytes(const uint8_t *b, uint32_t nb) {
    uint8_t pkt[RFU1_MAX_LEN];
    uint32_t len = rfu1_build_host_send(pkt, b, nb);
    if (!send_packet(pkt, len)) return false;
    s_n_hs++;
    return true;
}

/* FRLG character map for a visible-ASCII name char; 0xFF = terminator (verbatim from the C6 build). */
static uint8_t frlg_char(char c) {
    if (c >= 'A' && c <= 'Z') return 0xBB + (c - 'A');
    if (c >= 'a' && c <= 'z') return 0xD5 + (c - 'a');
    if (c >= '0' && c <= '9') return 0xA1 + (c - '0');
    if (c == ' ') return 0x00;
    return 0xFF;
}

/* The Switch's room as a 6-word beacon for the GBA: librfu's RFU_SERIAL_GAME + RfuGameData
 * compatibility word (language/version/link-flags), the Switch host's name + trainer id, and
 * activity = TRADE. Checksum per pret/pokefirered src/librfu_rfu.c (also the afska wireless
 * reference's "<sep>" byte).
 *
 * Kizuna: synthesized from scratch -- NOT copied from the GBA's own captured 0x16
 * beacon as before. That required the GBA to have called CMD_HOST_START (0x19) at least once
 * before ANY room could ever be shown to it (s_have_beacon gated this function entirely), which
 * is backwards for a Switch-hosts / GBA-only-searches trade: the GBA never hosts, so bc_in never
 * moves and the gate never opens (flagged this exact fallback as "not done"; confirmed
 * on the bench -- the RP2040's own command telemetry showed lastcmd=16/1d/3d while the
 * GBA sat searching for several minutes, never once 19).
 *
 * Field layout and the constant bytes below are ported byte-for-byte from GB-Link's own working
 * PC-side leader bridge (github.com/GB-Link/GB-Link-Switch-LDN, host/core/LeaderBridge.cs,
 * Beacon()), which builds this same beacon with no GBA-captured template at all. Byte offsets
 * there (0-indexed into a 24-byte buffer) map onto this function's c[4..27] 1:1. */
__attribute__((weak)) uint8_t rfu1_bridge_room_activity(void) { return 0; }

static bool build_room_beacon(uint32_t out[6]) {
    uint8_t c[28] = {0};
    c[4] = 0x02; c[5] = 0x00;           /* RFU_SERIAL_GAME */
    c[6] = 0x82; c[7] = 0x13;           /* compat: lang EN(2) | canLinkNationally | hasNationalDex |
                                            gameClear | version FireRed(4)<<10 = 0x1382 LE */
    /* c[10..15] (partnerInfo[4] + tradeSpecies/tradeType) stay zero, as the reference does. */
    char name[9] = {0}; uint16_t tid = 0;
    if (rfu1_bridge_room_identity(name, &tid)) {
        int nl = (int)strlen(name);
        for (int i = 0; i < 8; i++) c[20 + i] = (i < nl) ? frlg_char(name[i]) : (i == nl ? 0xFF : 0x00);
        if (tid) { c[8] = tid & 0xff; c[9] = tid >> 8; }
    }
    /* the host's real in-game name (FRLG charmap) from its advert record beats the ASCII nickname. */
    { uint8_t raw[8]; if (rfu1_bridge_room_identity_raw(raw)) memcpy(c + 20, raw, 8); }
    /* The GBA's Direct Corner lists only groups of the activity the player picked: show the Switch room's own
     * (4 trade, 1 single battle, 2 double battle). Unknown room: trade, as before. */
    uint8_t act = rfu1_bridge_room_activity();
    c[16] = act ? act : 0x04;
    uint8_t sum = 0; for (int j = 0; j < 8; j++) { sum += c[6+j]; sum += c[20+j]; }
    c[19] = (uint8_t)~sum;
    for (int i = 1; i < 7; i++) out[i-1] = c[i*4] | c[i*4+1]<<8 | c[i*4+2]<<16 | c[i*4+3]<<24;
    return true;
}

/* ---- inbound packets ---- */
static void on_pkt(const rfu1_pkt_t *p, void *ctx) {
    (void)ctx;
    s_last_rx_us = esp_timer_get_time();
    if (p->ptype == RFU1_BROADCAST) { s_gba_host_devid = rfu1_devid(p->hdata); s_gba_bc_us = s_last_rx_us; }
    if (s_lead_sink) { s_lead_sink(p); return; }   /* the GBA leads; the lead session owns the link */
    switch (p->ptype) {
    case RFU1_BROADCAST: {
        /* The GBA hosts (Union Room alternates host/scan): capture its beacon = identity + template. */
        uint32_t w[6];
        for (int i = 0; i < 6; i++)
            w[i] = (uint32_t)p->payload[4*i] << 24 | (uint32_t)p->payload[4*i+1] << 16
                 | (uint32_t)p->payload[4*i+2] << 8 | p->payload[4*i+3];
        bool first = !s_have_beacon;
        memcpy(s_gba_beacon, w, sizeof w); s_have_beacon = true; s_n_bc_in++;
        if (first) {
            printf("  [rfu1] GBA beacon (devid %04x): %08lx %08lx %08lx %08lx %08lx %08lx\n", rfu1_devid(p->hdata),
                   (unsigned long)w[0], (unsigned long)w[1], (unsigned long)w[2],
                   (unsigned long)w[3], (unsigned long)w[4], (unsigned long)w[5]);
            uint16_t tid = (uint16_t)(w[1] & 0xFFFF);
            uint8_t name[8] = { (uint8_t)w[4], (uint8_t)(w[4] >> 8), (uint8_t)(w[4] >> 16), (uint8_t)(w[4] >> 24),
                                (uint8_t)w[5], (uint8_t)(w[5] >> 8), (uint8_t)(w[5] >> 16), (uint8_t)(w[5] >> 24) };
            gba_bridge_set_gba_identity(tid, name, 1 /*JP*/, 0x04 /*TRADE*/);   /* as the C6 did */
        }
        if (!s_browsing) { s_browsing = true; printf("  [rfu1] GBA is browsing -> associate\n"); }
        /* Kizuna: call every time a beacon arrives, not just once. Called unconditionally
         * (see the matching note on the 0x0E telemetry handler below) so a "start" that arrives after the
         * GBA is already browsing still gets picked up -- the callback itself is a cheap, idempotent
         * no-op until trade_session is armed. */
        rfu1_bridge_on_gba_browsing();
        break;
    }
    case RFU1_CONNECT_REQ: {
        uint16_t target = rfu1_devid(p->hdata);
        if (target != OUR_DEVID) { printf("  [rfu1] CONNECT_REQ to %04x (not us) ignored\n", target); break; }
        if (!s_connected) {
            s_gba_devid = (uint16_t)(esp_random() & 0xFFFF); if (!s_gba_devid) s_gba_devid = 1;
            s_connected = true;
            printf("  [rfu1] GBA selected our room -> CONNECT_ACK devid=%04x slot 0\n", s_gba_devid);
            rfu1_bridge_on_gba_selected();
        }
        uint8_t pkt[16];
        send_packet(pkt, rfu1_build_connect_ack(pkt, s_gba_devid, 0, true));   /* resent per REQ retry */
        break;
    }
    case RFU1_CLIENT_SEND: {
        uint32_t nb = rfu1_client_len(p->hdata);
        if (nb > 16) nb = 16;
        bool nonzero = false;
        for (uint32_t i = 0; i < nb; i++) if (p->payload[i]) { nonzero = true; break; }
        if (!nonzero) { s_n_idle_cs++; break; }                 /* idle carrier (RP2040 fills these every 17 ms) */
        s_n_cs++;
        /* gba_bridge_on_gba_cmd wants the GBA's 0x24/0x25 words: data[0] = SEND length header (unused), then
         * the LLSF subframes packed LE — exactly the CLIENT_SEND payload. */
        uint32_t words[1 + 4] = { 0 };
        int nw = (int)((nb + 3) / 4);
        for (int i = 0; i < nw; i++)
            words[1 + i] = (uint32_t)p->payload[4*i] | (uint32_t)p->payload[4*i+1] << 8
                         | (uint32_t)p->payload[4*i+2] << 16 | (uint32_t)p->payload[4*i+3] << 24;
        gba_bridge_on_gba_cmd(0x25, words, 1 + nw);
        uint8_t ack[3];
        if (gba_bridge_take_child_ack(ack)) { if (host_send_bytes(ack, 3)) s_n_acks++; }
        break;
    }
    case RFU1_CLIENT_ACK: break;                                /* the RP2040 acks every HOST_SEND; informational */
    case RFU1_FLOWCTL: {
        bool hold = rfu1_fc_hold(p->hdata);
        if (hold != s_hold && s_n_fc < FLOWCTL_LOG_MAX) printf("  [rfu1] FLOWCTL hold=%d depth=%u\n", hold, rfu1_fc_depth(p->hdata));
        s_hold = hold; s_n_fc++;
        break;
    }
    case RFU1_DISCONNECT:
        printf("  [rfu1] GBA DISCONNECT (devid %04x)\n", rfu1_devid(p->hdata));
        if (s_connected) { s_connected = false; rfu1_bridge_on_gba_gone(); }
        break;
    default:
        printf("  [rfu1] %s hdata=%08lx (unhandled)\n", rfu1_name(p->ptype), (unsigned long)p->hdata);
        break;
    }
}

static inline uint32_t le32(const uint8_t *p) { return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24; }
static inline uint16_t le16(const uint8_t *p) { return (uint16_t)(p[0] | p[1] << 8); }

/* RP2040 session telemetry, printed on change / every 5 s so a login wall is legible. */
/* kizuna.10 live command trace (tag 0x2C): {kind, code, ret, adapterState, arg, resp, ms} x n. */
static const char *rfu_cmd_name(uint8_t c) {
    switch (c & 0x7F) {
    case 0x10: return "Reset";        case 0x11: return "LinkStatus";   case 0x13: return "SystemStatus";
    case 0x14: return "SlotStatus";   case 0x16: return "GameConfig";   case 0x17: return "SystemConfig";
    case 0x19: return "HostStart";    case 0x1a: return "HostPoll";     case 0x1b: return "HostEnd";
    case 0x1c: return "SearchStart";  case 0x1d: return "SearchPoll";   case 0x1e: return "SearchEnd";
    case 0x1f: return "ConnectStart"; case 0x20: return "ConnectPoll";  case 0x21: return "ConnectEnd";
    case 0x24: return "DataTx";       case 0x25: return "DataTxChange"; case 0x26: return "DataRx";
    case 0x27: return "Wait";         case 0x30: return "Disconnect";   case 0x3d: return "Init2";
    default: return "?";
    }
}
static void on_live_trace(const uint8_t *d, size_t n) {
    static const char *const st[] = { "idle", "HOST", "CONNECTING", "CLIENT" };
    for (uint8_t i = 0; i < d[1] && 2 + (size_t)(i + 1) * 16 <= n; i++) {
        const uint8_t *e = d + 2 + i * 16;
        uint32_t arg = le32(e + 4), resp = le32(e + 8), ms = le32(e + 12);
        const char *as = e[3] < 4 ? st[e[3]] : "?";
        switch (e[0]) {
        case 1: printf("  [rp-cmd %lu] %02x %-12s arg=%08lx ret=%d resp=%08lx (adapter %s)\n", (unsigned long)ms, e[1],
                       rfu_cmd_name(e[1]), (unsigned long)arg, (int8_t)e[2], (unsigned long)resp, as); break;
        case 3: printf("  [rp-cmd %lu] GBA RESTARTED a command (com %u, header %08lx)\n", (unsigned long)ms, e[1], (unsigned long)arg); break;
        case 5: printf("  [rp-cmd %lu] SD SOFT RESET (com %u)\n", (unsigned long)ms, e[1]); break;
        case 6: printf("  [rp-cmd %lu] GBA RESTARTED ITS LOGIN (site %u)\n", (unsigned long)ms, e[1]); break;
        case 9: printf("  [rp-cmd %lu] adapter state %s -> %s\n", (unsigned long)ms, arg < 4 ? st[arg] : "?", e[1] < 4 ? st[e[1]] : "?"); break;
        case 2: printf("  [rp-cmd %lu] GBA wait resolved: %s (event %08lx, frames pending %lu)\n", (unsigned long)ms,
                       (e[1] & 0x7F) == 0x28 ? "DATA" : (e[1] & 0x7F) == 0x27 ? "TIMEOUT" : (e[1] & 0x7F) == 0x29 ? "DISCONNECT" : "?", (unsigned long)arg, (unsigned long)resp); break;
        case 4: printf("  [rp-cmd %lu] delivery to GBA: %s\n", (unsigned long)ms, e[1] == 0 ? "done" : e[1] == 1 ? "RETRY" : "ABORTED"); break;
        case 10: printf("  [rp-cmd %lu] parent frame from us: %s (w0=%08lx, now %lu queued)\n", (unsigned long)ms,
                        e[1] == 0 ? "queued" : e[1] == 1 ? "replaced unread UNI" : "idle dropped", (unsigned long)arg, (unsigned long)resp); break;
        }
    }
}

static void on_telemetry(const uint8_t *d, size_t n) {
    s_n_tele++;
    if (d[0] == 0x2C && n >= 2) { on_live_trace(d, n); return; }
    if (d[0] == 0x2E && n >= 62) {
        static uint8_t sig[8]; static int64_t last; int64_t now = esp_timer_get_time();
        /* slave + master + cmds only move while a GBA is clocking the link; a GBA switched off never logs out, it goes quiet. */
        static uint32_t last_act;
        uint32_t act = le32(d+16) + le32(d+20) + le32(d+32);
        if (act != last_act) { last_act = act; s_gba_seen_us = now; }
        uint8_t cur[8] = { d[1], d[2], d[3], d[4], d[5], d[56], d[57], (uint8_t)d[32] };
        if (memcmp(sig, cur, 8) != 0 || now - last > 5000000) {
            memcpy(sig, cur, 8); last = now;
            printf("  [rp-2e] state=%u link=%u role=%s flags=%02x lastcmd=%02x/%u ring=%u | slave=%lu master=%lu wait=%lu cmds=%lu "
                   "aborts=%lu restarts=%lu errs=%lu lastrx=%08lx | fail=%02x(stage %u word %u) SO=%u retries=%lu\n",
                   d[1], d[2], d[3] ? "adapter-clocks" : "gba-clocks", d[4], d[5], d[6], d[7],
                   (unsigned long)le32(d+16), (unsigned long)le32(d+20), (unsigned long)le32(d+24), (unsigned long)le32(d+28),
                   (unsigned long)le32(d+32), (unsigned long)le32(d+36), (unsigned long)le32(d+40), (unsigned long)le32(d+44),
                   d[56], d[56] & 0xF, d[56] >> 4, d[57], (unsigned long)le32(d+58));
        }
        return;
    }
    if (d[0] == 0x2F && n >= 10) {
        static uint8_t sig[9];
        if (memcmp(sig, d + 1, 9) != 0) { memcpy(sig, d + 1, 9);
            printf("  [rp-2f] lines SO=%u SI=%u SC=%u adapter_clk=%u pc=%u txfifo=%u rxfifo=%u lastrx=%08lx\n",
                   d[1] & 1, (d[1] >> 1) & 1, (d[1] >> 2) & 1, (d[1] >> 3) & 1, d[2], d[3], d[4], (unsigned long)le32(d+5)); }
        return;
    }
    if (d[0] == 0x1E && n >= 19) {
        static uint8_t sig[18];
        if (memcmp(sig, d + 1, 18) != 0) { memcpy(sig, d + 1, 18);
            printf("  [rp-1e] uart_ovf=%u parked_hw=%u parked_drop=%u qfull=%u fifo_hw=%u aborts=%u retries=%u idle_drop=%u ring_drop=%u cmd_restarts=%u\n",
                   le16(d+1), d[3], le16(d+4), le16(d+6), d[8], le16(d+9), le16(d+11), le16(d+13), le16(d+15), le16(d+17)); }
        return;
    }
    if (d[0] == 0x4C && n >= 21) {
        printf("  [rp-isr] worst=%lu us  over30us=%lu  tightest_gap=%lu us  passes=%lu  sd_gpio=%02x\n",
               (unsigned long)le32(d+1), (unsigned long)le32(d+5), (unsigned long)le32(d+9), (unsigned long)le32(d+13), d[17]);
        return;
    }
    if (d[0] == 0x0E && n >= 16) {
        uint8_t st = d[3];
        if (st != s_login_state) {
            s_login_state = st;
            uint32_t last = (uint32_t)d[8] | (uint32_t)d[9] << 8 | (uint32_t)d[10] << 16 | (uint32_t)d[11] << 24;
            printf("  [rfu1] login telemetry: state=%u nintendo=%u last_rx=%08lx\n", st, d[2], (unsigned long)last);
            if (st >= 2 && !s_browsing) { s_browsing = true; printf("  [rfu1] GBA logged in -> associate\n"); }
            if (st < 2 && s_connected) { s_connected = false; printf("  [rfu1] login restarted\n"); rfu1_bridge_on_gba_gone(); }
        }
        /* Kizuna: called every poll, not just on the state transition, and not gated on
         * s_browsing. The RP2040 keeps running mode 07 across a C6 reflash, so the GBA can already be past
         * login (st>=2) on our very first telemetry frame -- before "start" has armed trade_session -- and
         * the one-shot transition above then latches s_browsing permanently while trade_session's s_armed
         * is still false, so rfu1_bridge_on_gba_browsing() was a no-op and never retried (symptom: no group
         * ever appears on the GBA; status stays member=0/assoc=0 forever). The callback is idempotent (sets
         * a flag only if armed), so retrying it every poll while state>=2 is free and catches a "start"
         * that arrives after the GBA was already logged in. */
        if (st >= 2) rfu1_bridge_on_gba_browsing();
    }
}

/* ---- public ---- */
bool rfu1_bridge_start(int tx_gpio, int rx_gpio) {
    if (!s_rxq) s_rxq = xQueueCreate(32, sizeof(chunk_t));
    static bool uart_up;
    if (tx_gpio < 0) {                    /* host link: the PC bridge plays the RP2040 over the relay */
        if (!gbl_host_active() && !gbl_init_host()) return false;
    } else if (!uart_up) { if (!gbl_init(tx_gpio, rx_gpio)) return false; uart_up = true; }
    gbl_set_tap(NULL, NULL, NULL);
    uint8_t ver[3], landing, cable;
    if (!gbl_get_info(ver, &landing, &cable, 800)) {
        /* An RP2040 left in a mode may answer slowly; cancel and retry once. */
        gbl_cancel();
        if (!gbl_get_info(ver, &landing, &cable, 800)) { printf("  [rfu1] no RP2040 on the bridge UART\n"); return false; }
    }
    printf("  [rfu1] GB-Link %u.%u.%u landing=%u cable=%u\n", ver[0], ver[1], ver[2], landing, cable);
    rfu1_rx_init(&s_rx); xQueueReset(s_rxq);
    s_have_beacon = false; s_browsing = false; s_connected = false; s_hold = false; s_login_state = 0;
    gbl_set_tap(on_data, on_status, NULL);
    if (!gbl_set_mode(GBL_MODE_RFU, 2 /* client hint; unused by the RP2040 */, GBL_ST_AWAIT_MODE, 1500)) {
        printf("  [rfu1] SetMode 07: no AwaitMode status\n"); gbl_set_tap(NULL, NULL, NULL); return false;
    }
    s_up = true;
    printf("  [rfu1] mode 07 up: adapter emulation running, waiting for the GBA\n");
    return true;
}

bool rfu1_bridge_up(void) { return s_up; }
bool rfu1_bridge_gba_hosting(uint8_t beacon[24], uint16_t *devid) {
    if (!s_have_beacon || !s_gba_bc_us || esp_timer_get_time() - s_gba_bc_us > 1000000) return false;
    for (int i = 0; i < 6; i++) {   /* the BROADCAST's big-endian words, stored little-endian: the 0x16 data's byte order */
        beacon[4*i] = (uint8_t)s_gba_beacon[i]; beacon[4*i+1] = (uint8_t)(s_gba_beacon[i] >> 8);
        beacon[4*i+2] = (uint8_t)(s_gba_beacon[i] >> 16); beacon[4*i+3] = (uint8_t)(s_gba_beacon[i] >> 24);
    }
    *devid = s_gba_host_devid;
    return true;
}
void rfu1_bridge_set_lead_sink(void (*fn)(const rfu1_pkt_t *p)) { s_lead_sink = fn; }
bool rfu1_bridge_send_raw(const uint8_t *pkt, uint32_t len) { return send_packet(pkt, len); }
int  rfu1_bridge_login_state(void) { return s_up ? (int)s_login_state : 0; }
int64_t rfu1_bridge_gba_quiet_us(void) { return s_gba_seen_us ? esp_timer_get_time() - s_gba_seen_us : esp_timer_get_time(); }
void rfu1_bridge_isr_stats(void) { static const uint8_t c = 0x4C; gbl_send(GBL_CH_CMD, &c, 1); }   /* reply -> [rp-isr] */

void rfu1_bridge_stop(void) {
    if (!s_up) return;
    gbl_cancel();
    s_up = false; s_connected = false; s_browsing = false;
    gbl_set_tap(NULL, NULL, NULL);
}

void rfu1_bridge_poll(void) {
    if (!s_up) return;
    chunk_t c;
    while (xQueueReceive(s_rxq, &c, 0) == pdTRUE) {
        /* The RP2040 streams RFU1 in exactly-64-byte chunks; telemetry/replies are always shorter. */
        if (c.len == GBL_MAX_PAYLOAD) rfu1_rx_feed(&s_rx, c.data, c.len, on_pkt, NULL);
        else on_telemetry(c.data, c.len);
    }
    s_n_junk = s_rx.resyncs;
    int64_t now = esp_timer_get_time();

    /* Beacon the Switch's room (only once we're a joinable member, as the C6 did) while the GBA browses. */
    if (!s_connected && now - s_last_beacon_us > BEACON_US) {
        uint32_t w[6];
        if (rfu1_bridge_room_visible() && build_room_beacon(w)) {
            uint8_t pkt[36];
            if (send_packet(pkt, rfu1_build_broadcast(pkt, OUR_DEVID, 0, w))) {
                if (!s_n_bc_out) printf("  [rfu1] beaconing the Switch's room to the GBA (devid %04x)\n", OUR_DEVID);
                s_n_bc_out++;
            }
        }
        s_last_beacon_us = now;
    }

    /* Parent frames toward the child: child acks first (already sent inline), then whatever gba_bridge
     * stages for this child frame: the JOIN_GROUP_OK NI subframes (one per tick, in order) or the UNI stream
     * (one 73-byte frame per tick: the RP2040 collapses idle UNI frames and paces real ones at 16 ms). A
     * count=0 reply means "nothing this frame" — the RP2040 then answers the GBA's 0x26 with an empty table,
     * which is the same keepalive the C6 served. FLOWCTL hold = the RP2040's queue is deep: don't pop more. */
    /* Diagnostics: every parent frame toward the GBA for the first 300 after a connect, with
     * the send result and FLOWCTL state, so "the GBA got no next frame" can be split into "we never sent it"
     * vs "the RP2040 never handed it over". Also logs the ticks where we were held or had nothing to send. */
    static int s_dn_logged; static bool s_was_connected;
    if (s_connected && !s_was_connected) s_dn_logged = 0;
    s_was_connected = s_connected;
    if (s_connected && s_hold && s_dn_logged < 300) { s_dn_logged++; printf("  [dn %lld] HELD by RP2040 FLOWCTL\n", (long long)(now / 1000)); }
    /* Online back-pressure (host link): the GBA side echoes the GBA's slots locally, which removed
     * the natural pacing of a real parent (it only echoes what it accepted). When our queue toward the Switch
     * builds up, tell the GBA side to hold its echoes (an RFU1 FLOWCTL over the relay; it never reaches the
     * RP2040) so the GBA re-sends instead of running ahead; release once drained. Re-sent every 500 ms while
     * held so a late joiner or a reconnect can't miss it. */
    /* DISABLED: holding the GBA side's echoes never slowed the GBA (in held-keys mode it
     * sends a slot every frame regardless); it only overflowed the echo queue, and the GBA's own block echoes
     * were dropped at the trade computer (135 lost, both consoles crashed). The drain now matches the GBA's rate
     * (12 ms floor) and back-to-back held keys merge (gba_bridge_set_coalesce_up_keys). Kept for reference. */
    if (0 && gbl_host_active() && s_connected) {
        static bool s_bp_hold; static int64_t s_bp_last_us;
        int depth = 0; gba_bridge_upq_stats(NULL, NULL, NULL, &depth);
        bool want = s_bp_hold ? depth > UP_BP_LOW : depth >= UP_BP_HIGH;
        if (want != s_bp_hold || (want && now - s_bp_last_us > 500000)) {
            if (want != s_bp_hold) printf("  [bp %lld] %s GBA echoes (queue toward the Switch depth=%d)\n",
                                          (long long)(now / 1000), want ? "HOLD" : "release", depth);
            s_bp_hold = want; s_bp_last_us = now;
            uint8_t pkt[16];
            send_packet(pkt, rfu1_build_flowctl(pkt, want, (uint8_t)depth));
        }
    }
    if (s_connected && !s_hold) {
        static uint32_t out[1 + 24];
        int nw = gba_bridge_build_recv(out, 1 + 24);
        /* Online (host link): the GBA side builds row 1 (its own echo) and sends one idle parent
         * frame per GBA frame by itself (the GBA side's uni_echo), so a UNI frame whose row 0 (the Switch's slot) is
         * empty carries nothing remote. Sending them anyway was ~115 relay messages/s that the GBA side's Wi-Fi
         * could not keep up with: real frames queued behind them, the Switch's 6600 arrived 5.3 s late. */
        if (gbl_host_active() && nw >= 6 && (out[0] & 0x7F) >= 3 + 14) {
            uint8_t b[3 + 14];
            for (int i = 0; i < (int)sizeof b; i++) b[i] = (uint8_t)(out[1 + i / 4] >> (8 * (i % 4)));
            bool uni = ((((uint32_t)b[0] | ((uint32_t)b[1] << 8) | ((uint32_t)b[2] << 16)) >> 14) & 0xF) == 4;
            bool row0_empty = true;
            for (int i = 3; i < 3 + 14; i++) if (b[i]) { row0_empty = false; break; }
            if (uni && row0_empty) { s_n_uni_suppressed++; nw = 0; }
        }
        if (nw >= 2 && (out[0] & 0x7F)) {
            bool ok = host_send_words(out, nw);
            if (s_dn_logged < 300) { s_dn_logged++;
                printf("  [dn %lld] HOST_SEND %u B w1=%08lx w2=%08lx -> %s\n", (long long)(now / 1000),
                       (unsigned)(out[0] & 0x7F), (unsigned long)out[1], (unsigned long)(nw > 2 ? out[2] : 0), ok ? "sent" : "SEND FAILED"); }
            if (ok) gba_bridge_recv_advance();   /* join-NI subframe confirmed queued */
        } else if (s_dn_logged < 300 && nw >= 1) { s_dn_logged++;
            printf("  [dn %lld] nothing to send (count=0)\n", (long long)(now / 1000)); }
    }
}

void rfu1_bridge_status(char *out, size_t cap) {
    snprintf(out, cap, "\"rfu1\":{\"up\":%d,\"status\":\"%04x\",\"login\":%u,\"beacon\":%d,\"browsing\":%d,\"connected\":%d,"
             "\"hold\":%d,\"bc_in\":%u,\"bc_out\":%u,\"cs\":%u,\"idle_cs\":%u,\"hs\":%u,\"acks\":%u,\"tele\":%u,\"resync\":%u,\"ni\":%d}",
             s_up, s_last_status, s_login_state, s_have_beacon, s_browsing, s_connected, s_hold,
             s_n_bc_in, s_n_bc_out, s_n_cs, s_n_idle_cs, s_n_hs, s_n_acks, s_n_tele, s_n_junk, gba_bridge_ni_stage());
}
