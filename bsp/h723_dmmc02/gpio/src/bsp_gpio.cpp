#include "bsp_gpio.hpp"

#include "main.h"

namespace bsp::gpio
{
namespace
{

GPIO_TypeDef* port_from_id(port_id id) noexcept
{
    switch (id)
    {
    case port_id::a: return GPIOA;
    case port_id::b: return GPIOB;
    case port_id::c: return GPIOC;
    case port_id::d: return GPIOD;
    case port_id::e: return GPIOE;
    case port_id::f: return GPIOF;
    case port_id::g: return GPIOG;
    case port_id::h: return GPIOH;
#if defined(GPIOI)
    case port_id::i: return GPIOI;
#endif
#if defined(GPIOJ)
    case port_id::j: return GPIOJ;
#endif
#if defined(GPIOK)
    case port_id::k: return GPIOK;
#endif
    default: return nullptr;
    }
}

const input_config* config_of(input input_id) noexcept
{
    const auto index = static_cast<std::size_t>(input_id);
    return index < input_count ? &input_configs[index] : nullptr;
}

const output_config* config_of(output output_id) noexcept
{
    const auto index = static_cast<std::size_t>(output_id);
    return index < output_count ? &output_configs[index] : nullptr;
}

std::uint16_t pin_mask(std::uint8_t pin) noexcept
{
    return static_cast<std::uint16_t>(1U << pin);
}

} // namespace

types::status read(input input_id, bool& is_high)
{
    const input_config* config = config_of(input_id);
    if (config == nullptr)
    {
        return types::status::invalid_arg;
    }
    GPIO_TypeDef* port = port_from_id(config->port);
    if (port == nullptr || config->pin > 15U)
    {
        return types::status::not_configured;
    }
    is_high = HAL_GPIO_ReadPin(port, pin_mask(config->pin)) == GPIO_PIN_SET;
    return types::status::ok;
}

types::status is_active(input input_id, bool& active)
{
    bool is_high = false;
    const types::status status = read(input_id, is_high);
    if (status != types::status::ok)
    {
        return status;
    }
    const input_config& config = input_configs[static_cast<std::size_t>(input_id)];
    active = config.active == active_level::high ? is_high : !is_high;
    return types::status::ok;
}

types::status write(output output_id, bool is_high)
{
    const output_config* config = config_of(output_id);
    if (config == nullptr)
    {
        return types::status::invalid_arg;
    }
    GPIO_TypeDef* port = port_from_id(config->port);
    if (port == nullptr || config->pin > 15U)
    {
        return types::status::not_configured;
    }
    HAL_GPIO_WritePin(port, pin_mask(config->pin), is_high ? GPIO_PIN_SET : GPIO_PIN_RESET);
    return types::status::ok;
}

types::status set_active(output output_id, bool active)
{
    const output_config* config = config_of(output_id);
    if (config == nullptr)
    {
        return types::status::invalid_arg;
    }
    const bool active_high = config->active == active_level::high;
    return write(output_id, active ? active_high : !active_high);
}

types::status toggle(output output_id)
{
    const output_config* config = config_of(output_id);
    if (config == nullptr)
    {
        return types::status::invalid_arg;
    }
    GPIO_TypeDef* port = port_from_id(config->port);
    if (port == nullptr || config->pin > 15U)
    {
        return types::status::not_configured;
    }
    HAL_GPIO_TogglePin(port, pin_mask(config->pin));
    return types::status::ok;
}

} // namespace bsp::gpio
