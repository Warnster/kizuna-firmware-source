#pragma once
#include <stdbool.h>

#include <zephyr/kernel.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize the GB SPI PIO program on pio0.
 * This disables any existing PIO program on pio0 first (e.g., GBA link).
 * Must be called before gb_link_transfer().
 */
void gb_link_init(void);

/**
 * Deinitialize the GB SPI PIO, freeing pio0 for other uses (e.g., GBA link).
 */
void gb_link_deinit(void);
// Set the SCK rate (Hz). The default is ~985 kHz (clkdiv 31.71 at 125 MHz, 4 cycles/bit).
void gb_link_set_clock_hz(uint32_t hz);
// Park MOSI (the GBA's SI) high between transfers instead of leaving the last data bit.
// A GBA must power up with SI high (else it plays multiplay parent) but, as a NORMAL-mode
// slave, only takes a transfer once SI has gone low, so a host parks high while waiting for
// the GBA to boot and turns this off (stock behaviour) while polling the multiboot hello.
void gb_link_set_park_si_high(bool on);

/**
 * Perform a full-duplex SPI transfer.
 * For each byte, clocks tx out while reading rx in simultaneously.
 * 
 * @param tx   Transmit buffer (len bytes)
 * @param rx   Receive buffer (len bytes)
 * @param len  Number of bytes to transfer
 * @param us_between  Microseconds to wait between individual byte transfers
 */
void gb_link_transfer(const uint8_t* tx, uint8_t* rx, uint32_t len, uint32_t us_between);

#ifdef __cplusplus
}
#endif
