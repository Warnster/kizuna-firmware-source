// gblink.h — host side of the GB-Link RP2040 control UART.
//
// GB-Link (unmodified, GPL) accepts its launcher commands on uart1 (RP2040 GP8=TX, GP9=RX,
// 921600 8N1) framed as  'G' 'B' | channel | len_lo len_hi | payload[len<=64].
// Channels: 0 = command (host->dev), 1 = data (both ways), 2 = status (dev->host, u16 LE).
// This is our own implementation from that public interface; no GB-Link code is used.
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define GBL_CH_CMD     0
#define GBL_CH_DATA    1
#define GBL_CH_STATUS  2
#define GBL_MAX_PAYLOAD 64    // the RP2040's serial layer drops anything longer, both directions

// Launcher modes (SetMode = command [0x00, mode, (variant)]). Not persisted: resend after reset.
enum {
    GBL_MODE_GBA_TRADE_EMU = 0x00,
    GBL_MODE_GBA_LINK      = 0x01,
    GBL_MODE_GB_LINK       = 0x02,   // raw SPI-master passthrough (RP2040 clocks SC)
    GBL_MODE_GB_PRINTER    = 0x03,
    GBL_MODE_ADVANCE_WARS  = 0x04,
    GBL_MODE_EREADER       = 0x05,
    GBL_MODE_CHIP_GATE     = 0x06,
    GBL_MODE_RFU           = 0x07,   // wireless adapter emulation
    GBL_MODE_GB_LINK_SLAVE = 0x08,   // Kizuna fork: GPIO bit-bang GB slave, MSB-first, 1 byte/frame
    GBL_MODE_GBA_UART      = 0x80,   // Kizuna fork: GBA SIO-UART <-> data-channel byte passthrough
};

// Status words (channel 2)
#define GBL_ST_AWAIT_MODE     0xFF02
#define GBL_ST_LINK_CLOSED    0xFF07
#define GBL_ST_DEVICE_READY   0xFF08
#define GBL_ST_GB_MODE_ACTIVE 0xFF0A
#define GBL_ST_GB_SESSION_END 0xFF0C
#define GBL_ST_WRONG_CABLE    0xFF0D
#define GBL_ST_UART_ACTIVE    0xFF80   // Kizuna fork: mode 0x80 running

// Fork mode 0x80 variant byte = GBA SIO-UART baud
#define GBL_UART_115200 0
#define GBL_UART_57600  1
#define GBL_UART_38400  2
#define GBL_UART_9600   3

#define GBL_PIPE_RXBUF  8192    // our buffer for GBA->host bytes
#define GBL_PICO_TXBUF  4096    // fork mode 0x80's host->GBA ring on the RP2040

typedef struct {
    uint8_t  ch;
    uint16_t len;
    uint8_t  data[GBL_MAX_PAYLOAD];
} gbl_frame_t;

bool gbl_init(int tx_gpio, int rx_gpio);
// Host link (online Switch side): no RP2040 on a UART. Frames we send are printed on the console as
// "@GB <ch> <hex>" lines for a PC bridge, which plays the RP2040 over the
/* Host link sink: when set, frames the bridge would send to the (remote) RP2040 go to fn instead of the "@GB <ch>
 * <hex>" console lines. NULL (default) keeps the console path unchanged. Called from the sender's task. */
typedef void (*gbl_host_tx_fn)(uint8_t ch, const uint8_t *payload, size_t len);
void gbl_set_host_tx(gbl_host_tx_fn fn);
// relay; frames from the PC arrive through gbl_host_inject() and are delivered exactly like UART frames.
bool gbl_init_host(void);
bool gbl_host_active(void);
void gbl_host_inject(uint8_t ch, const uint8_t *payload, size_t len);
bool gbl_send(uint8_t ch, const uint8_t *payload, size_t len);
bool gbl_recv(gbl_frame_t *f, int timeout_ms);     // next parsed frame from the RP2040
void gbl_flush(void);                              // drop any queued frames

// Stream tap (mode 0x07 / RFU1): when set, every data-channel (ch 1) frame is handed to `cb` from the
// UART task instead of being queued, and every status word to `st` (both may be NULL). Command replies
// still go to the queue, so gbl_get_info() keeps working. Clear with gbl_set_tap(NULL, NULL).
typedef void (*gbl_data_cb_t)(const uint8_t *payload, size_t len, void *ctx);
typedef void (*gbl_status_cb_t)(uint16_t status, void *ctx);
void gbl_set_tap(gbl_data_cb_t cb, gbl_status_cb_t st, void *ctx);
bool gbl_cancel(void);                             // launcher Cancel (0x01): leave the current mode

// Convenience
bool gbl_get_info(uint8_t ver[3], uint8_t *landing, uint8_t *cable, int timeout_ms);
bool gbl_set_mode(uint8_t mode, int variant, uint16_t wait_status, int timeout_ms);  // variant<0: none

// Mode 0x02 (raw SPI master, RP2040 clocks SC ~1 MHz): exchange n<=16 32-bit words (one frame).
bool gbl_set_word_timing(uint32_t gap_us);        // per-word gap after each 4-byte chunk
bool gbl_xfer32(const uint32_t *out, uint32_t *in, int n, int timeout_ms);

// Mode 0x02 reused for the real Game Boy / GBC link (gb_serial_bridge): 1 byte/transfer
// at the GB's own ~8192 Hz internal clock instead of multiboot's 32-bit words. The RP2040 is
// always master on this path (mode 0x02 has no "listen as slave" primitive) -- see
// gb_serial_bridge's header for why that is an accepted simplification, not an oversight.
bool gbl_gb_link_begin(uint32_t clock_hz);        // SetMode 0x02 + SetClockKhz + 1 byte/transfer, 5V rail
bool gbl_gb_exchange_byte(uint8_t tx_byte, uint8_t *rx_byte, int timeout_ms);
void gbl_gb_link_end(void);                       // Cancel (0x01); leaves the 5V rail as-is
// SetClockKhz (0x33, Kizuna fork): SCK rate for mode 0x02's SPI passthrough. gb_link_init() resets
// it to the firmware default (~985 kHz) on every mode entry, so send it after gbl_gb_link_begin().
bool gbl_gb_set_clock_khz(uint16_t khz);

// Kizuna fork mode 0x08 (RP2040 as a real GB_SERIAL_ROLE_SLAVE): the RP2040 waits for the real
// Game Boy's own clock instead of driving SC itself -- gbl_gb_exchange_byte's reply can time out
// as soon as the real console stops clocking (no fixed per-byte rate to configure, unlike mode
// 0x02). timeout_ms should give the console a few VBlanks of room; the RP2040 itself replies
// 0xFF if nobody clocked it within its own internal bound.
bool gbl_gb_link_begin_slave(void);                                    // SetMode 0x08, 5V rail
bool gbl_gb_slave_exchange_byte(uint8_t tx_byte, uint8_t *rx_byte, int timeout_ms);
void gbl_gb_link_end_slave(void);                                      // Cancel (0x01)

// Fork mode 0x80: transparent byte pipe to the GBA's SIO-UART (savetool protocol).
/* Link-port logic level on boards with GB-Link's translator section (c3rp R2, hardware-updates §2):
 * the RP2040 drives VCCLOGIC = 3.3 V (GP11, command 0x40) or 5 V (GP12, command 0x41). GBA modes need
 * 3.3 V (the firmware's boot default); GB/GBC console modes need 5 V. Harmless on boards without the
 * section (GP11/GP12 unconnected). Fire-and-forget: the firmware acks nothing for hardware commands. */
bool gbl_set_link_voltage_5v(bool five_volts);

/* GBLink-Firmware control.hpp SetLEDColor (0x42): set the WS2812 status LED on RP2040 GP16 (c3rp R2 §3.4).
 * The firmware restores its own per-mode colour on the next mode change. */
bool gbl_set_led(uint8_t r, uint8_t g, uint8_t b, bool on);
/* Host-link modes only: this board's OWN RP2040 for its status LED, while gbl_send goes to the remote one.
 * gbl_init_local_tx opens a write-only UART (the local RP2040's output is never read); gbl_local_set_led sends 0x42. */
bool gbl_init_local_tx(int tx_gpio, int rx_gpio);
bool gbl_local_set_led(uint8_t r, uint8_t g, uint8_t b, bool on);
bool gbl_pipe_begin(uint32_t baud_code);          // SetMode 0x80 and route data frames to the pipe
void gbl_pipe_end(void);                          // Cancel the mode, frames go back to the queue
int  gbl_pipe_write(const uint8_t *src, size_t n);
int  gbl_pipe_read(uint8_t *dst, size_t n, int timeout_ms);
void gbl_pipe_flush(void);
