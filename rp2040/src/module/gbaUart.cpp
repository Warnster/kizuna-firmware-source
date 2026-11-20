#include "gbaUart.hpp"
#include "../linkStatus.hpp"
#include "../hardware.hpp"
#include "../persist.hpp"
#include "../layers/transport.hpp"

#include <zephyr/sys/ring_buffer.h>

extern "C"
{
    // No extern "C" guard of its own, so it must sit in this block.
    #include <zephyr/drivers/misc/pio_rpi_pico/pio_rpi_pico.h>
    #include "../layers/linkLayer.h"
    #include "hardware/pio.h"
    #include "hardware/gpio.h"
    #include "hardware/clocks.h"
    #include "hardware/timer.h"
}

// Link port data lines, as wired for the other modes: GP1 = GBA SO (in), GP2 = GBA SI (out).
#define PIN_RX 1
#define PIN_TX 2

// Standard PIO UART programs, 8 PIO cycles per bit, 8N1, LSB first.
//   tx: pull side 1 [7] / set x, 7 side 0 [7] / bitloop: out pins, 1 / jmp x-- bitloop [6]
static const uint16_t uart_tx_instructions[] = {
    0x9fa0, // pull   block           side 1 [7]
    0xf727, // set    x, 7            side 0 [7]
    0x6001, // out    pins, 1
    0x0642, // jmp    x--, 2                 [6]
};
static const struct pio_program uart_tx_program = {
    .instructions = uart_tx_instructions, .length = 4, .origin = -1,
};

//   rx: wait 0 pin 0 / set x, 7 [10] / bitloop: in pins, 1 / jmp x-- bitloop [6]
//       jmp pin good_stop / irq 4 rel / wait 1 pin 0 / jmp start / good_stop: push
// Bytes with a bad stop bit are discarded. irq 4 rel only raises PIO-internal flag 4/5,
// never the flags 0/1 that the GBA link layer routes to its interrupt handlers.
static const uint16_t uart_rx_instructions[] = {
    0x2020, // wait   0 pin, 0
    0xea27, // set    x, 7                   [10]
    0x4001, // in     pins, 1
    0x0642, // jmp    x--, 2                 [6]
    0x00c8, // jmp    pin, 8
    0xc014, // irq    nowait 4 rel
    0x20a0, // wait   1 pin, 0
    0x0000, // jmp    0
    0x8020, // push   block
};
static const struct pio_program uart_rx_program = {
    .instructions = uart_rx_instructions, .length = 9, .origin = -1,
};

// Host -> GBA bytes, filled by the transport's receive thread, drained by execute().
static constexpr uint32_t TX_RING_SIZE = 4096;
static uint8_t g_txRingBuf[TX_RING_SIZE];
static struct ring_buf g_txRing;
static struct k_spinlock g_txLock;
static uint32_t g_txDropped = 0;

// GBA -> host: flush a frame when it is full or the line has been quiet this long.
static constexpr uint32_t RX_FLUSH_IDLE_US = 500;
static constexpr uint32_t MAX_FRAME = 64;

GbaUartModule::GbaUartModule(uint8_t variant)
{
    m_baud = variant == 1 ? 57600 : variant == 2 ? 38400 : variant == 3 ? 9600 : 115200;
}

void GbaUartModule::dataHandler(std::span<const uint8_t> data, void* userData)
{
    (void)userData;
    k_spinlock_key_t key = k_spin_lock(&g_txLock);
    uint32_t put = ring_buf_put(&g_txRing, data.data(), data.size());
    k_spin_unlock(&g_txLock, key);
    g_txDropped += data.size() - put;
}

static void sendFrame(const uint8_t* buf, uint32_t len)
{
    // The transport refuses when its TX ring is full; retry briefly so the host isn't
    // missing bytes (the bridge drains ~90 bytes/ms, the GBA sends at most ~11.5/ms).
    for (int attempt = 0; attempt < 50; attempt++) {
        if (Transport::sendData(std::span<const uint8_t>(buf, len))) return;
        k_usleep(100);
    }
}

void GbaUartModule::execute()
{
    m_cancel = false;

    // Free pio0 from the GBA link programs (same hand-over the GB SPI mode uses).
    link_changeMode(DISABLED);

    const struct device* dev = DEVICE_DT_GET(DT_PROP(DT_NODELABEL(pio_link), pio));
    PIO pio = pio_rpi_pico_get_pio(dev);
    // SM0 is the GBA link's (disabled now); claim a second one once for the RX side.
    static size_t smRx = SIZE_MAX;
    if (smRx == SIZE_MAX && pio_rpi_pico_allocate_sm(dev, &smRx) != 0) {
        sendLinkStatus(LinkStatus::GBSessionFinished);
        return;
    }
    const uint smTx = 0;

    pio_sm_set_enabled(pio, smTx, false);
    pio_sm_set_enabled(pio, smRx, false);
    pio_clear_instruction_memory(pio);
    pio_sm_restart(pio, smTx);
    pio_sm_restart(pio, smRx);
    pio_sm_clear_fifos(pio, smTx);
    pio_sm_clear_fifos(pio, smRx);

    const float div = (float)clock_get_hz(clk_sys) / (8.0f * (float)m_baud);

    // TX on GP2, idle high.
    uint txOff = pio_add_program(pio, &uart_tx_program);
    pio_sm_set_pins_with_mask(pio, smTx, 1u << PIN_TX, 1u << PIN_TX);
    pio_sm_set_pindirs_with_mask(pio, smTx, 1u << PIN_TX, 1u << PIN_TX);
    pio_gpio_init(pio, PIN_TX);
    pio_sm_config tc = pio_get_default_sm_config();
    sm_config_set_wrap(&tc, txOff, txOff + 3);
    sm_config_set_sideset(&tc, 2, true, false);
    sm_config_set_out_shift(&tc, true, false, 32);
    sm_config_set_out_pins(&tc, PIN_TX, 1);
    sm_config_set_sideset_pins(&tc, PIN_TX);
    sm_config_set_fifo_join(&tc, PIO_FIFO_JOIN_TX);
    sm_config_set_clkdiv(&tc, div);
    pio_sm_init(pio, smTx, txOff, &tc);

    // RX on GP1, pulled up so an unplugged cable reads idle rather than a stream of breaks.
    uint rxOff = pio_add_program(pio, &uart_rx_program);
    pio_sm_set_consecutive_pindirs(pio, smRx, PIN_RX, 1, false);
    pio_gpio_init(pio, PIN_RX);
    gpio_pull_up(PIN_RX);
    pio_sm_config rc = pio_get_default_sm_config();
    sm_config_set_wrap(&rc, rxOff, rxOff + 8);
    sm_config_set_in_pins(&rc, PIN_RX);
    sm_config_set_jmp_pin(&rc, PIN_RX);
    sm_config_set_in_shift(&rc, true, false, 32);
    sm_config_set_fifo_join(&rc, PIO_FIFO_JOIN_RX);
    sm_config_set_clkdiv(&rc, div);
    pio_sm_init(pio, smRx, rxOff, &rc);

    ring_buf_init(&g_txRing, sizeof(g_txRingBuf), g_txRingBuf);
    g_txDropped = 0;
    Transport::registerDataHandler(dataHandler, this);

    pio_sm_set_enabled(pio, smTx, true);
    pio_sm_set_enabled(pio, smRx, true);

    applyLedForSlot(LED_SLOT_GBA);
    sendLinkStatus(LinkStatus::GbaUartModeActive);

    uint8_t frame[MAX_FRAME];
    uint32_t frameLen = 0;
    uint64_t lastRxUs = 0;

    while (!m_cancel)
    {
        // GBA -> host. The joined RX FIFO holds 8 bytes (~0.7 ms at 115200), so this loop
        // must come round well within that: it sleeps at most 100 us per pass.
        while (!pio_sm_is_rx_fifo_empty(pio, smRx)) {
            frame[frameLen++] = (uint8_t)(pio_sm_get(pio, smRx) >> 24);
            lastRxUs = time_us_64();
            if (frameLen == MAX_FRAME) { sendFrame(frame, frameLen); frameLen = 0; }
        }
        if (frameLen && time_us_64() - lastRxUs >= RX_FLUSH_IDLE_US) {
            sendFrame(frame, frameLen);
            frameLen = 0;
        }

        // Host -> GBA.
        while (!pio_sm_is_tx_fifo_full(pio, smTx)) {
            uint8_t b;
            k_spinlock_key_t key = k_spin_lock(&g_txLock);
            uint32_t got = ring_buf_get(&g_txRing, &b, 1);
            k_spin_unlock(&g_txLock, key);
            if (!got) break;
            pio_sm_put(pio, smTx, b);
        }

        k_usleep(100);
    }

    if (frameLen) sendFrame(frame, frameLen);
    Transport::registerDataHandler(nullptr, nullptr);

    pio_sm_set_enabled(pio, smTx, false);
    pio_sm_set_enabled(pio, smRx, false);
    pio_clear_instruction_memory(pio);
    pio_sm_restart(pio, smTx);
    pio_sm_restart(pio, smRx);
    pio_sm_clear_fifos(pio, smTx);
    pio_sm_clear_fifos(pio, smRx);

    // Back to the idle pin state (inputs with pull-ups) for the next mode.
    gpio_init(PIN_TX);
    gpio_pull_up(PIN_TX);
    gpio_init(PIN_RX);
    gpio_pull_up(PIN_RX);
}
