#pragma once

#include "config.hpp"
#include "tim.h"

#include <cstdint>

namespace bsp::pwm::failsafe::detail
{

struct binding
{
    TIM_HandleTypeDef* timeout_timer = nullptr;
    std::uint32_t dma_slot = 0;
    std::uint32_t timer_clock_hz = 0;
    std::uintptr_t compare_register = 0;
};

bool binding_for(channel channel_id, binding& out) noexcept;
void keep_timers_running_while_debugging(channel channel_id) noexcept;

} // namespace bsp::pwm::failsafe::detail
