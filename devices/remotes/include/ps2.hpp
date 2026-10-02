#pragma once
#include "bsp_gpio.hpp"
#include "bsp_spi.hpp"
#include "usertypes.hpp"
#include <stdint.h>

namespace remotes::ps2
{
inline constexpr uint8_t frame_size = 9U;

struct frame
{
    uint8_t bytes[frame_size]{};
};

struct sample
{
    uint16_t buttons = 0; // Active-high PS2 button bitmap.
    float lx = 0.0f;
    float ly = 0.0f;
    float rx = 0.0f;
    float ry = 0.0f;
};

// Synchronous device operations; scheduling, button edges and publication belong to remoter.
types::status decode(const frame& raw, sample& out) noexcept;

template <typename Transport>
types::status poll(Transport& transport, sample& out, frame& raw)
{
    constexpr uint8_t query[frame_size]{
        0x01U, 0x42U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U};
    raw = {};
    const auto status = transport.transfer(query, raw.bytes, frame_size);
    return status == types::status::ok ? decode(raw, out) : status;
}

class spi
{
public:
    constexpr spi(bsp::spi::bus spi_bus, bsp::gpio::output cs) noexcept
        : spi_bus_(spi_bus), cs_(cs)
    {
    }

    types::status init();
    types::status transfer(const uint8_t* tx, uint8_t* rx, uint8_t size);

private:
    bsp::spi::bus spi_bus_;
    bsp::gpio::output cs_;
};

class gpio
{
public:
    constexpr gpio(bsp::gpio::output cmd, bsp::gpio::input data,
                       bsp::gpio::output clk, bsp::gpio::output cs) noexcept
        : cmd_(cmd), data_(data), clk_(clk), cs_(cs)
    {
    }

    types::status init();
    types::status transfer(const uint8_t* tx, uint8_t* rx, uint8_t size);

private:
    types::status transfer_byte(uint8_t tx, uint8_t& rx);

    bsp::gpio::output cmd_;
    bsp::gpio::input data_;
    bsp::gpio::output clk_;
    bsp::gpio::output cs_;
};

} // namespace remotes::ps2
