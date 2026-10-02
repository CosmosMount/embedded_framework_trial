#include "ps2.hpp"
#include "bsp_dwt.hpp"
#include "config.hpp"

namespace remotes::ps2
{
namespace
{
constexpr bool analog_id(uint8_t id)
{
    return id == 0x53U || id == 0x73U;
}

constexpr float normalize_axis(uint8_t raw, uint8_t center)
{
    if (raw >= center)
    {
        return static_cast<float>(raw - center) / static_cast<float>(UINT8_MAX - center);
    }
    return -static_cast<float>(center - raw) / static_cast<float>(center);
}

constexpr float apply_axis_deadzone(float value)
{
    return value > -params::ps2::axis_deadzone && value < params::ps2::axis_deadzone ? 0.0f : value;
}

} // namespace

types::status decode(const frame& raw, sample& out) noexcept
{
    if (raw.bytes[0] != 0xFFU || !analog_id(raw.bytes[1]) || raw.bytes[2] != 0x5AU)
    {
        return types::status::error;
    }

    sample decoded{};
    decoded.buttons = static_cast<uint16_t>(~(static_cast<uint16_t>(raw.bytes[3]) |
                                             (static_cast<uint16_t>(raw.bytes[4]) << 8U)));
    decoded.rx = apply_axis_deadzone(normalize_axis(raw.bytes[5], 127U));
    decoded.ry = apply_axis_deadzone(-normalize_axis(raw.bytes[6], 128U));
    decoded.lx = apply_axis_deadzone(normalize_axis(raw.bytes[7], 127U));
    decoded.ly = apply_axis_deadzone(-normalize_axis(raw.bytes[8], 128U));
    out = decoded;
    return types::status::ok;
}

types::status spi::init()
{
    const types::status status = bsp::spi::init(spi_bus_);
    return status == types::status::ok ? bsp::gpio::set_active(cs_, false) : status;
}

types::status spi::transfer(const uint8_t* tx, uint8_t* rx, uint8_t size)
{
    if (tx == nullptr || rx == nullptr || size == 0U)
    {
        return types::status::invalid_arg;
    }
    types::status status = bsp::gpio::set_active(cs_, true);
    if (status == types::status::ok)
    {
        status = bsp::spi::transmit_receive(spi_bus_, tx, rx, size, params::ps2::spi_timeout_ms);
    }
    const types::status release_status = bsp::gpio::set_active(cs_, false);
    return status == types::status::ok ? release_status : status;
}

types::status gpio::init()
{
    types::status status = bsp::dwt::init();
    if (status != types::status::ok) return status;
    status = bsp::gpio::set_active(cs_, false);
    if (status == types::status::ok)
    {
        status = bsp::gpio::write(clk_, true);
    }
    return status == types::status::ok ? bsp::gpio::write(cmd_, true) : status;
}

types::status gpio::transfer_byte(uint8_t tx, uint8_t& rx)
{
    rx = 0U;
    for (uint8_t bit = 0U; bit < 8U; ++bit)
    {
        types::status status = bsp::gpio::write(cmd_, (tx & (1U << bit)) != 0U);
        if (status != types::status::ok)
        {
            return status;
        }
        bsp::dwt::delay_us(params::ps2::gpio_half_period_us);
        status = bsp::gpio::write(clk_, false);
        if (status != types::status::ok)
        {
            return status;
        }
        bsp::dwt::delay_us(params::ps2::gpio_half_period_us);
        bool data_high = false;
        status = bsp::gpio::read(data_, data_high);
        if (status != types::status::ok)
        {
            return status;
        }
        if (data_high)
        {
            rx = static_cast<uint8_t>(rx | (1U << bit));
        }
        status = bsp::gpio::write(clk_, true);
        if (status != types::status::ok)
        {
            return status;
        }
        bsp::dwt::delay_us(params::ps2::gpio_half_period_us);
    }
    return types::status::ok;
}

types::status gpio::transfer(const uint8_t* tx, uint8_t* rx, uint8_t size)
{
    if (tx == nullptr || rx == nullptr || size == 0U)
    {
        return types::status::invalid_arg;
    }
    types::status status = bsp::gpio::set_active(cs_, true);
    if (status == types::status::ok)
    {
        bsp::dwt::delay_us(params::ps2::gpio_cs_setup_us);
        for (uint8_t index = 0U; index < size && status == types::status::ok; ++index)
        {
            status = transfer_byte(tx[index], rx[index]);
        }
        bsp::dwt::delay_us(params::ps2::gpio_cs_hold_us);
    }
    const types::status release_status = bsp::gpio::set_active(cs_, false);
    return status == types::status::ok ? release_status : status;
}

} // namespace remotes::ps2
