#pragma once

#include <span>
#include <zephyr/kernel.h>

#include "moduleInterface.hpp"

class GBModule : public IModule
{
public:
    GBModule()
    {
        k_sem_init(&m_dataSemaphore, 0, 1);
    }

    void execute();
    void executePrinterMode();
    // Kizuna fork: generic GB-speed SLAVE byte exchange (mode 0x08). Unlike execute()'s mode 0x02
    // (always master, PIO), this bit-bangs GP0-3 as a slave -- we wait for the Game Boy's own
    // clock edges and shift our current byte out while sampling its byte in, MSB first (Pan
    // Docs), symmetric to executePrinterMode()'s bit-bang loop but without the GB Printer
    // protocol's sync bytes or bit order. One byte in, one byte out, over the data channel.
    void executeSlaveMode();
    void cancel() override;
    bool canHandle(uint8_t command) override;
    void receiveCommand(std::span<const uint8_t> command) override;

private:
    enum class GBCommand : uint8_t
    {
        SetTimingConfig = 0x30,
        EnterPrinterMode = 0x31,
        ExitPrinterMode = 0x32,
        SetClockKhz = 0x33,     // [0x33, khz_lo, khz_hi]: SCK rate for the SPI passthrough
        SetParkSiHigh = 0x34,   // [0x34, 0|1]: park the GBA's SI high between transfers
    };

    enum class SubMode { normal, printer };

    bool m_cancel = false;
    SubMode m_subMode = SubMode::normal;

    // Timing config (defaults match reconfigurable firmware)
    uint8_t m_bytesPerTransfer = 1;
    uint32_t m_usBetweenTransfer = 1000;

    struct k_sem m_dataSemaphore;

    // --- Normal SPI passthrough ---
    void normalModeLoop();

    // --- Printer mode (GPIO bit-bang SPI slave) ---
    void printerModeLoop();

    // --- Generic slave mode (GPIO bit-bang SPI slave, MSB-first, one byte per Transport frame) ---
    void slaveModeLoop();

    // --- USB data handler ---
    static void usbDataHandler(std::span<const uint8_t> data, void* userData);
};
