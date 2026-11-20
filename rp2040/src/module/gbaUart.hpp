#pragma once

#include <span>
#include <zephyr/kernel.h>

#include "moduleInterface.hpp"

// GBA SIO-UART passthrough (mode 0x80).
//
// Puts the link port's data lines into async UART mode and bridges them byte-for-byte to
// the host's data channel: GBA SO (GP1) -> host, host -> GBA SI (GP2). For homebrew that
// switches the GBA's SIO into UART mode after a multiboot (e.g. a save backup tool), so a
// companion MCU can talk to it through the adapter. Flow control is not used; SC/SD are
// left alone.
//
// Variant byte selects the baud rate: 0 = 115200 (default), 1 = 57600, 2 = 38400, 3 = 9600.
class GbaUartModule : public IModule
{
public:
    explicit GbaUartModule(uint8_t variant = 0);

    void execute();
    void cancel() override { m_cancel = true; }
    bool canHandle(uint8_t command) override { (void)command; return false; }
    void receiveCommand(std::span<const uint8_t> command) override { (void)command; }

private:
    volatile bool m_cancel = false;
    uint32_t m_baud;

    static void dataHandler(std::span<const uint8_t> data, void* userData);
};
