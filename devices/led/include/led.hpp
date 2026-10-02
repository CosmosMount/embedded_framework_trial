#pragma once

#include <stdint.h>
#include "usertypes.hpp"

// Currently supports only the H7 SPI-driven WS2812 LED.
// F4 GPIO indicator LEDs are not supported by this device; use bsp::gpio directly.
// This implementation is linked only when robot.devices.led is selected.
namespace led
{

enum class color : uint8_t
{
    red = 0,
    green,
    blue,
    white,
};

types::status init();
types::status on(color c);
types::status toggle(color c);
// One step per call; half-period is configured in calls, not RTOS ticks.
types::status blink(color c);
types::status all_off();
types::status all_on();

} // namespace led
