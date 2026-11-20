// gblink.c — host side of the GB-Link RP2040 control UART. See gblink.h.

#include "gblink.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/stream_buffer.h"
#include "esp_timer.h"
#include "driver/uart.h"
#include "esp_log.h"
#include <stdio.h>

static const char *TAG = "gblink";

#define GBL_UART   UART_NUM_1
#define GBL_BAUD   921600

static QueueHandle_t s_frames;
static StreamBufferHandle_t s_pipe;     // GBA->host bytes while the UART passthrough mode runs
static volatile bool s_pipe_on;
static int64_t s_pipe_drain_us;         // when the Pico will have sent everything we've queued
static gbl_data_cb_t   s_tap_data;      // mode 0x07 stream tap (see gblink.h)
static gbl_status_cb_t s_tap_status;
static void           *s_tap_ctx;
static volatile bool   s_host;          // host link: frames go to the console, not the UART
static gbl_host_tx_fn  s_host_tx;       // host link: optional sink instead of the console (NULL = "@GB" lines)

void gbl_set_tap(gbl_data_cb_t cb, gbl_status_cb_t st, void *ctx) {
    s_tap_ctx = ctx; s_tap_status = st; s_tap_data = cb;
}

static void deliver(const gbl_frame_t *f) {
    if (f->ch == GBL_CH_STATUS && f->len == 2 && s_tap_status)
        s_tap_status((uint16_t)(f->data[0] | f->data[1] << 8), s_tap_ctx);   // tapped AND queued
    if (s_tap_data && f->ch == GBL_CH_DATA) { s_tap_data(f->data, f->len, s_tap_ctx); return; }
    if (s_pipe_on && f->ch == GBL_CH_DATA) {
        if (xStreamBufferSend(s_pipe, f->data, f->len, 0) != f->len)
            ESP_LOGW(TAG, "pipe overflow: GBA bytes dropped");
        return;
    }
    if (xQueueSend(s_frames, f, 0) != pdTRUE) ESP_LOGW(TAG, "rx queue full, frame dropped");
}

// Byte-at-a-time parser: 'G' 'B' ch len_lo len_hi payload. Oversize frames resync.
static void rx_task(void *arg) {
    enum { S_G, S_B, S_CH, S_L0, S_L1, S_PAY } st = S_G;
    gbl_frame_t f;
    uint16_t got = 0;
    uint8_t buf[256];
    for (;;) {
        // Block for the first byte, then take whatever else is already buffered. (Asking for a
        // full buffer with a timeout made every reply wait out that timeout: ~20 ms per
        // multiboot word, long enough for the GBA BIOS to give up mid-handshake.)
        int n = uart_read_bytes(GBL_UART, buf, 1, portMAX_DELAY);
        if (n == 1) {
            size_t avail = 0;
            uart_get_buffered_data_len(GBL_UART, &avail);
            if (avail > sizeof buf - 1) avail = sizeof buf - 1;
            if (avail) {
                int m = uart_read_bytes(GBL_UART, buf + 1, avail, 0);
                if (m > 0) n += m;
            }
        }
        for (int i = 0; i < n; i++) {
            uint8_t b = buf[i];
            switch (st) {
            case S_G:  st = b == 'G' ? S_B : S_G; break;
            case S_B:  st = b == 'B' ? S_CH : (b == 'G' ? S_B : S_G); break;
            case S_CH: f.ch = b; st = S_L0; break;
            case S_L0: f.len = b; st = S_L1; break;
            case S_L1:
                f.len |= (uint16_t)b << 8;
                got = 0;
                if (f.len > GBL_MAX_PAYLOAD) { ESP_LOGW(TAG, "oversize frame len=%u", f.len); st = S_G; }
                else if (f.len == 0) { if (!s_host) deliver(&f); st = S_G; }
                else st = S_PAY;
                break;
            case S_PAY:
                f.data[got++] = b;
                /* Host link: the RP2040 is the REMOTE one (PC bridge, gbl_host_inject). A board that also has a
                 * local RP2040 on this UART (on the two-chip board) must not mix its frames in: its
                 * "no GBA, not logged in" telemetry alternated with the bridge's "logged in" and the bridge
                 * dropped the connected GBA every 0.5 s, so JOIN_GROUP_OK was never delivered. */
                if (got == f.len) { if (!s_host) deliver(&f); st = S_G; }
                break;
            }
        }
    }
}

bool gbl_init(int tx_gpio, int rx_gpio) {
    uart_config_t cfg = {
        .baud_rate = GBL_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    if (uart_driver_install(GBL_UART, 4096, 1024, 0, NULL, 0) != ESP_OK) return false;
    if (uart_param_config(GBL_UART, &cfg) != ESP_OK) return false;
    uart_set_rx_timeout(GBL_UART, 2);            // RX interrupt after 2 idle symbols (~22 us)
    if (uart_set_pin(GBL_UART, tx_gpio, rx_gpio, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK)
        return false;
    s_frames = xQueueCreate(16, sizeof(gbl_frame_t));
    s_pipe = xStreamBufferCreate(GBL_PIPE_RXBUF, 1);
    xTaskCreate(rx_task, "gbl_rx", 3072, NULL, 6, NULL);
    ESP_LOGI(TAG, "GB-Link UART1 up: TX=GPIO%d -> RP2040 GP9, RX=GPIO%d <- RP2040 GP8, %d baud",
             tx_gpio, rx_gpio, GBL_BAUD);
    return true;
}

/* Local-only RP2040 commands for a host-link mode (switch-online): the link's frames go to the REMOTE RP2040
 * (gbl_send in host mode), but the status LED is on this board's own RP2040. A write-only UART: the local RP2040's
 * replies and telemetry are never read (no rx task), so nothing of it can mix into the host link. */
static bool s_local_uart;
bool gbl_init_local_tx(int tx_gpio, int rx_gpio) {
    if (s_local_uart) return true;
    uart_config_t cfg = {
        .baud_rate = GBL_BAUD, .data_bits = UART_DATA_8_BITS, .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1, .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, .source_clk = UART_SCLK_DEFAULT,
    };
    if (uart_driver_install(GBL_UART, 256, 0, 0, NULL, 0) != ESP_OK) return false;   // RX buffer is the driver minimum, never read
    if (uart_param_config(GBL_UART, &cfg) != ESP_OK) return false;
    if (uart_set_pin(GBL_UART, tx_gpio, rx_gpio, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK) return false;
    s_local_uart = true;
    return true;
}
bool gbl_local_set_led(uint8_t r, uint8_t g, uint8_t b, bool on) {
    if (!s_local_uart) return false;
    uint8_t f[5 + 5] = { 'G', 'B', GBL_CH_CMD, 5, 0, 0x42, r, g, b, on ? 1 : 0 };
    uart_flush_input(GBL_UART);                  // drop whatever the local RP2040 said; nobody reads it
    return uart_write_bytes(GBL_UART, f, sizeof f) == (int)sizeof f;
}

bool gbl_set_link_voltage_5v(bool five_volts) {
    uint8_t c = five_volts ? 0x41 : 0x40;   /* GBLink-Firmware control.hpp: SetVoltage5V / SetVoltage3V3 */
    return gbl_send(GBL_CH_CMD, &c, 1);
}

bool gbl_set_led(uint8_t r, uint8_t g, uint8_t b, bool on) {
    uint8_t c[5] = { 0x42, r, g, b, on ? 1 : 0 };   /* control.hpp: SetLEDColor -> Hardware::setLED(r,g,b,on) */
    return gbl_send(GBL_CH_CMD, c, sizeof c);
}

bool gbl_send(uint8_t ch, const uint8_t *payload, size_t len) {
    if (len > GBL_MAX_PAYLOAD) return false;
    if (s_host) {
        gbl_host_tx_fn tx = s_host_tx;
        if (tx) { tx(ch, payload, len); return true; }      // e.g. switch-online: frames to the phone over BLE
        char line[16 + 2 * GBL_MAX_PAYLOAD];
        int n = snprintf(line, sizeof line, "@GB %u ", ch);
        for (size_t i = 0; i < len; i++) n += snprintf(line + n, sizeof line - n, "%02x", payload[i]);
        line[n++] = '\n';
        fwrite(line, 1, (size_t)n, stdout); fflush(stdout);   // one write: the line is not split by logs
        return true;
    }
    uint8_t hdr[5] = { 'G', 'B', ch, (uint8_t)len, (uint8_t)(len >> 8) };
    return uart_write_bytes(GBL_UART, hdr, 5) == 5 &&
           (len == 0 || uart_write_bytes(GBL_UART, payload, len) == (int)len);
}

bool gbl_init_host(void) {
    if (!s_frames) s_frames = xQueueCreate(16, sizeof(gbl_frame_t));
    if (!s_pipe) s_pipe = xStreamBufferCreate(GBL_PIPE_RXBUF, 1);
    s_host = true;
    ESP_LOGI(TAG, "GB-Link host link: RP2040 frames over the USB console (PC bridge)");
    return s_frames != NULL;
}
bool gbl_host_active(void) { return s_host; }
void gbl_set_host_tx(gbl_host_tx_fn fn) { s_host_tx = fn; }
void gbl_host_inject(uint8_t ch, const uint8_t *payload, size_t len) {
    if (!s_host || len > GBL_MAX_PAYLOAD) return;
    gbl_frame_t f = { .ch = ch, .len = (uint16_t)len };
    memcpy(f.data, payload, len);
    deliver(&f);
}

bool gbl_recv(gbl_frame_t *f, int timeout_ms) {
    return xQueueReceive(s_frames, f, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
}

void gbl_flush(void) { xQueueReset(s_frames); }

bool gbl_cancel(void) {
    static const uint8_t cancel = 0x01;
    return gbl_send(GBL_CH_CMD, &cancel, 1);
}

bool gbl_get_info(uint8_t ver[3], uint8_t *landing, uint8_t *cable, int timeout_ms) {
    static const uint8_t cmd = 0x0F;
    gbl_flush();
    if (!gbl_send(GBL_CH_CMD, &cmd, 1)) return false;
    gbl_frame_t f;
    TickType_t end = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);
    while (xTaskGetTickCount() < end) {
        if (!gbl_recv(&f, 20)) continue;
        if (f.ch == GBL_CH_DATA && f.len >= 6 && f.data[0] == 0x0F) {
            memcpy(ver, &f.data[1], 3);
            *landing = f.data[4];
            *cable = f.data[5];
            return true;
        }
    }
    return false;
}

bool gbl_set_mode(uint8_t mode, int variant, uint16_t wait_status, int timeout_ms) {
    uint8_t cmd[3] = { 0x00, mode, (uint8_t)variant };
    gbl_flush();
    if (!gbl_send(GBL_CH_CMD, cmd, variant < 0 ? 2 : 3)) return false;
    if (!wait_status) return true;
    gbl_frame_t f;
    TickType_t end = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);
    while (xTaskGetTickCount() < end) {
        if (!gbl_recv(&f, 20)) continue;
        if (f.ch == GBL_CH_STATUS && f.len == 2 && (f.data[0] | f.data[1] << 8) == wait_status) return true;
    }
    return false;
}

// ---- raw 32-bit SIO through mode 0x02 (multiboot) ----------------------------------------

bool gbl_set_word_timing(uint32_t gap_us) {
    uint8_t cmd[5] = { 0x30, (uint8_t)gap_us, (uint8_t)(gap_us >> 8), (uint8_t)(gap_us >> 16), 4 };
    return gbl_send(GBL_CH_CMD, cmd, sizeof cmd);
}

bool gbl_xfer32(const uint32_t *out, uint32_t *in, int n, int timeout_ms) {
    if (n < 1 || n > GBL_MAX_PAYLOAD / 4) return false;
    uint8_t p[GBL_MAX_PAYLOAD];
    for (int i = 0; i < n; i++) {               // the GBA shifts SIODATA32 MSB first
        p[4 * i] = out[i] >> 24; p[4 * i + 1] = out[i] >> 16;
        p[4 * i + 2] = out[i] >> 8; p[4 * i + 3] = out[i];
    }
    if (!gbl_send(GBL_CH_DATA, p, 4 * n)) return false;
    gbl_frame_t f;
    TickType_t end = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms) + 1;
    while (xTaskGetTickCount() <= end) {
        if (xQueueReceive(s_frames, &f, 1) != pdTRUE) continue;   // wakes as soon as a frame lands
        if (f.ch != GBL_CH_DATA) continue;      // statuses are not replies
        if (f.len != 4 * n) { ESP_LOGW(TAG, "xfer32: reply len %u != %d", f.len, 4 * n); return false; }
        if (in) for (int i = 0; i < n; i++)
            in[i] = (uint32_t)f.data[4 * i] << 24 | (uint32_t)f.data[4 * i + 1] << 16 |
                    (uint32_t)f.data[4 * i + 2] << 8 | f.data[4 * i + 3];
        return true;
    }
    return false;
}

// ---- real GB/GBC link through mode 0x02 (gb_serial_bridge) -----------------------
//
// Same GBModule (control.hpp Mode::gbLink) as gbl_xfer32 above, just reconfigured for 1-byte
// Game Boy shift-register transfers instead of 4-byte multiboot words: SetTimingConfig (0x30)
// bytesPerTransfer=1. clock_hz is accepted but NOT sent as SetClockKhz (0x33) -- checked against
// starlarkus/gblink-pokemon-web (pokemon.gblink.io's actual source): it never calls SetClockKhz
// for Gen 1/2 trading at all, leaving the RP2040 at its own default (~985 kHz per gbLinkLayer.h's
// own doc comment). That's correct, not an oversight: we are the master driving an
// externally-clocked GB/GBC slave, whose shift register has no fixed sample rate of its own to
// match (8192 Hz is only the GB's rate when IT is master) -- so there's no reason to slow our
// clock down to 8192 Hz, and doing so (an earlier version of this function did) skews the
// byte-to-byte timing relative to the proven 1000us inter-byte gap below, which was tuned
// against the ~985 kHz default, not an 8 kHz one.
bool gbl_gb_link_begin(uint32_t clock_hz) {
    (void)clock_hz;
    if (!gbl_set_link_voltage_5v(true)) return false;       // GB/GBC link is 5V (c3rp R2 translator rail)
    if (!gbl_set_mode(GBL_MODE_GB_LINK, -1, GBL_ST_GB_MODE_ACTIVE, 500)) return false;
    // 1000us inter-byte gap, 1 byte/transfer: matches starlarkus/gblink-pokemon-web's own
    // setTimingConfig(1000, 1) for GB/GBC trading (AppUI.js). A real console needs this gap to
    // keep up; 0 (back-to-back, what multiboot's 32-bit words use) desyncs it.
    uint32_t gap_us = 1000;
    uint8_t timing_cmd[5] = { 0x30, (uint8_t)gap_us, (uint8_t)(gap_us >> 8), (uint8_t)(gap_us >> 16), 1 };
    return gbl_send(GBL_CH_CMD, timing_cmd, sizeof timing_cmd);
}

bool gbl_gb_exchange_byte(uint8_t tx_byte, uint8_t *rx_byte, int timeout_ms) {
    if (!gbl_send(GBL_CH_DATA, &tx_byte, 1)) return false;
    gbl_frame_t f;
    TickType_t end = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms) + 1;
    while (xTaskGetTickCount() <= end) {
        if (xQueueReceive(s_frames, &f, 1) != pdTRUE) continue;
        if (f.ch != GBL_CH_DATA) continue;
        if (f.len != 1) { ESP_LOGW(TAG, "gb exchange: reply len %u != 1", f.len); return false; }
        if (rx_byte) *rx_byte = f.data[0];
        return true;
    }
    return false;
}

void gbl_gb_link_end(void) { gbl_cancel(); }

bool gbl_gb_set_clock_khz(uint16_t khz) {
    uint8_t cmd[3] = { 0x33, (uint8_t)khz, (uint8_t)(khz >> 8) };   /* gb.hpp GBCommand::SetClockKhz */
    return gbl_send(GBL_CH_CMD, cmd, sizeof cmd);
}

// ---- real GB/GBC link through mode 0x08 (gb_serial_bridge slave role) ---------------------

bool gbl_gb_link_begin_slave(void) {
    if (!gbl_set_link_voltage_5v(true)) return false;
    return gbl_set_mode(GBL_MODE_GB_LINK_SLAVE, -1, GBL_ST_GB_MODE_ACTIVE, 500);
}

bool gbl_gb_slave_exchange_byte(uint8_t tx_byte, uint8_t *rx_byte, int timeout_ms) {
    if (!gbl_send(GBL_CH_DATA, &tx_byte, 1)) return false;
    gbl_frame_t f;
    TickType_t end = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms) + 1;
    while (xTaskGetTickCount() <= end) {
        if (xQueueReceive(s_frames, &f, 1) != pdTRUE) continue;
        if (f.ch != GBL_CH_DATA) continue;
        if (f.len != 1) { ESP_LOGW(TAG, "gb slave exchange: reply len %u != 1", f.len); return false; }
        if (rx_byte) *rx_byte = f.data[0];
        return true;
    }
    return false;
}

void gbl_gb_link_end_slave(void) { gbl_cancel(); }

// ---- byte pipe through the UART passthrough mode (fork mode 0x80) -------------------------

bool gbl_pipe_begin(uint32_t baud_code) {
    xStreamBufferReset(s_pipe);
    s_pipe_drain_us = 0;
    s_pipe_on = true;
    if (gbl_set_mode(GBL_MODE_GBA_UART, (int)baud_code, GBL_ST_UART_ACTIVE, 500)) return true;
    s_pipe_on = false;
    return false;
}

void gbl_pipe_end(void) {
    s_pipe_on = false;
    gbl_cancel();
}

void gbl_pipe_flush(void) { xStreamBufferReset(s_pipe); }

// Paced so the Pico's TX ring (GBL_PICO_TXBUF) can't overflow: we track when the Pico will have
// shifted out everything already sent at 115200 baud (~87 us/byte) and wait while that backlog
// exceeds the ring. Small writes (acks, markers) never wait.
int gbl_pipe_write(const uint8_t *src, size_t n) {
    const int64_t us_per_byte = 87, budget = (int64_t)GBL_PICO_TXBUF * us_per_byte;
    size_t done = 0;
    while (done < n) {
        size_t k = n - done < GBL_MAX_PAYLOAD ? n - done : GBL_MAX_PAYLOAD;
        int64_t now = esp_timer_get_time();
        if (s_pipe_drain_us < now) s_pipe_drain_us = now;
        while (s_pipe_drain_us - now > budget) {
            vTaskDelay(pdMS_TO_TICKS(20) ? pdMS_TO_TICKS(20) : 1);
            now = esp_timer_get_time();
        }
        if (!gbl_send(GBL_CH_DATA, src + done, k)) break;
        s_pipe_drain_us += (int64_t)k * us_per_byte;
        done += k;
    }
    return (int)done;
}

// Like uart_read_bytes: wait until `n` bytes arrived or the timeout expires; returns the count.
int gbl_pipe_read(uint8_t *dst, size_t n, int timeout_ms) {
    size_t got = 0;
    TickType_t end = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);
    do {
        TickType_t now = xTaskGetTickCount();
        TickType_t wait = end > now ? end - now : 0;
        got += xStreamBufferReceive(s_pipe, dst + got, n - got, wait);
    } while (got < n && xTaskGetTickCount() < end);
    return (int)got;
}
