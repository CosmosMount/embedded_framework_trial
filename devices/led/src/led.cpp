#include "led.hpp"
#include "bsp_spi.hpp"
#include "config.hpp"

namespace led
{
namespace
{
constexpr bsp::spi::bus ws2812_bus = board::device::led::spi;
constexpr uint8_t low_symbol = 0x60;
constexpr uint8_t high_symbol = 0x78;
// Preserve the established reset interval as 100 low bytes, sent in one transfer.
constexpr uint8_t reset_bytes[100]{};
constexpr uint8_t brightness = params::led::brightness;

types::status set_rgb(uint8_t r, uint8_t g, uint8_t b)
{
    uint8_t payload[24]{};
    for (uint8_t i = 0; i < 8; ++i)
    {
        payload[7 - i] = ((g >> i) & 1U) ? high_symbol : low_symbol;
        payload[15 - i] = ((r >> i) & 1U) ? high_symbol : low_symbol;
        payload[23 - i] = ((b >> i) & 1U) ? high_symbol : low_symbol;
    }
    auto status = bsp::spi::wait_ready(ws2812_bus, params::led::spi_timeout_ms);
    if (status != types::status::ok) return status;
    status = bsp::spi::transmit(ws2812_bus, payload, sizeof(payload), params::led::spi_timeout_ms);
    if (status != types::status::ok) return status;
    return bsp::spi::transmit(ws2812_bus, reset_bytes, sizeof(reset_bytes), params::led::spi_timeout_ms);
}
} // namespace

types::status init()
{
    const auto status = bsp::spi::init(ws2812_bus);
    return status == types::status::ok ? all_on() : status;
}

types::status all_on() { return set_rgb(brightness, brightness, brightness); }
types::status all_off() { return set_rgb(0, 0, 0); }

types::status on(color c)
{
    switch (c)
    {
    case color::red: return set_rgb(brightness, 0, 0);
    case color::green: return set_rgb(0, brightness, 0);
    case color::blue: return set_rgb(0, 0, brightness);
    case color::white: return all_on();
    }
    return types::status::invalid_arg;
}

types::status toggle(color c)
{
    static bool enabled = false;
    const auto status = enabled ? all_off() : on(c);
    if (status == types::status::ok) enabled = !enabled;
    return status;
}

types::status blink(color c)
{
    static uint16_t count = 0;
    const auto status = count < params::led::blink_half_period_calls ? all_off() : on(c);
    if (status == types::status::ok && ++count == 2U * params::led::blink_half_period_calls) count = 0;
    return status;
}
} // namespace led
