#pragma once

#include "config.hpp"
#include "usertypes.hpp"

namespace bsp::gpio
{

constexpr bool is_enabled(input input_id)
{
    const auto index = static_cast<std::size_t>(input_id);
    return index < input_count && input_configs[index].port != port_id::none;
}

constexpr bool is_enabled(output output_id)
{
    const auto index = static_cast<std::size_t>(output_id);
    return index < output_count && output_configs[index].port != port_id::none;
}

types::status read(input input_id, bool& is_high);
types::status is_active(input input_id, bool& active);
types::status write(output output_id, bool is_high);
types::status set_active(output output_id, bool active);
types::status toggle(output output_id);

} // namespace bsp::gpio
