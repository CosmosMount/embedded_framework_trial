#pragma once

#include "bsp_pwm.hpp"

namespace bsp::pwm::failsafe
{

bool enabled(channel channel_id) noexcept;
types::status init(channel channel_id);
types::status arm(channel channel_id);
void disarm(channel channel_id) noexcept;

} // namespace bsp::pwm::failsafe
