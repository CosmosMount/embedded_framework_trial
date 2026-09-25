#pragma once

#include "config.hpp"
#include "tim.h"

#include <cstdint>

namespace bsp::pwm::detail
{

struct binding
{
    TIM_HandleTypeDef* timer = nullptr;
    std::uint32_t hal_channel = 0;
};

bool binding_for(channel channel_id, binding& out) noexcept;

} // namespace bsp::pwm::detail
