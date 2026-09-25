#pragma once

#include <cstdint>

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

void init();
void on(color c);
void toggle(color c);
void blink(color c);
void all_off();
void all_on();

} // namespace led
