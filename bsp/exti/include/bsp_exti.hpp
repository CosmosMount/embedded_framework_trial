#pragma once

#include "callback.hpp"
#include "config.hpp"
#include "usertypes.hpp"

#include <cstdint>

namespace bsp::exti
{

// Invoked from the GPIO HAL interrupt path; it must not block.
using interrupt_callback = core::callback<void()>;
types::status attach(bsp::gpio::input input, interrupt_callback callback);

} // namespace bsp::exti
