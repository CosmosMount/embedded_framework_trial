#include "led.hpp"

#include "bsp_spi.hpp"
#include "config.hpp"

namespace led
{

namespace
{


constexpr bsp::spi::bus ws2812_bus = board::device::led::spi;
constexpr uint8_t ws2812_low = 0xC0;
constexpr uint8_t ws2812_high = 0xF0;
constexpr uint32_t spi_timeout_ms = 1000;

bool set_rgb(uint8_t r, uint8_t g, uint8_t b)
{
    uint8_t txbuf[24]{};
    const uint8_t reset_byte = 0;

    for (int i = 0; i < 8; ++i)
    {
        txbuf[7 - i] =
            static_cast<uint8_t>((((g >> i) & 0x01U) ? ws2812_high : ws2812_low) >> 1);
        txbuf[15 - i] =
            static_cast<uint8_t>((((r >> i) & 0x01U) ? ws2812_high : ws2812_low) >> 1);
        txbuf[23 - i] =
            static_cast<uint8_t>((((b >> i) & 0x01U) ? ws2812_high : ws2812_low) >> 1);
    }

    if (bsp::spi::wait_ready(ws2812_bus, spi_timeout_ms) != types::status::ok)
    {
        return false;
    }
    if (bsp::spi::transmit(ws2812_bus, txbuf, sizeof(txbuf), spi_timeout_ms) != types::status::ok)
    {
        return false;
    }
    for (int i = 0; i < 100; ++i)
    {
        if (bsp::spi::transmit(ws2812_bus, &reset_byte, 1, spi_timeout_ms) != types::status::ok)
        {
            return false;
        }
    }

    return true;
}


} // namespace

void init()
{
    all_on();
}

void all_on()
{
    (void)set_rgb(7, 7, 7);
}

void all_off()
{
    (void)set_rgb(0, 0, 0);
}

void on(color c)
{
    switch (c)
    {
    case color::red:
        (void)set_rgb(7, 0, 0);
        break;
    case color::green:
        (void)set_rgb(0, 7, 0);
        break;
    case color::blue:
        (void)set_rgb(0, 0, 7);
        break;
    case color::white:
        (void)set_rgb(7, 7, 7);
        break;
    }
}

void toggle(color c)
{
    static bool enabled = false;
    enabled = !enabled;
    if (enabled)
    {
        on(c);
    }
    else
    {
        all_off();
    }
}

void blink(color c)
{
    static uint32_t flash_count = 0;
    ++flash_count;

    if (flash_count <= 250)
    {
        all_off();
    }
    else if (flash_count <= 500)
    {
        on(c);
    }

    if (flash_count >= 500)
    {
        flash_count = 0;
    }
}

} // namespace led
