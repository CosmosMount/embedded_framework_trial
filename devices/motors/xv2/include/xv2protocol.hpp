#pragma once

#include "motor.hpp"

#include <stdint.h>

namespace motors::xv2
{

static constexpr uint32_t default_can_id = 0x000;
static constexpr uint8_t frame_tail = 0x6B;

enum class sys_param : uint8_t
{
    velocity = 0x35,
    position = 0x36,
    temperature = 0x39,
    flags = 0x3A,
};

enum class position_reference : uint8_t
{
    last_target = 0,
    absolute = 1,
    current = 2,
};

struct frame
{
    uint8_t data[16]{};
    uint8_t len = 0;
};

uint8_t direction_of(float value);
uint16_t scaled_abs_u16(float value, float scale);
uint32_t scaled_abs_u32(float value, float scale);

frame trigger_encoder_cal(uint8_t address);
frame reset_motor(uint8_t address);
frame reset_position(uint8_t address);
frame reset_clog_protection(uint8_t address);
frame enable(uint8_t address, bool state, bool sync);
frame torque(uint8_t address, int16_t current_ma, uint16_t ramp_ma_s, bool sync);
frame velocity(uint8_t address, float rpm, uint16_t acceleration_rpm_s, bool sync);
frame position_speed(uint8_t address, float position_deg, float velocity_rpm,
                     position_reference reference, bool sync);
frame stop_now(uint8_t address, bool sync);
frame synchronous_motion(uint8_t address);
frame read_sys_param(uint8_t address, sys_param param);

bool parse_feedback(uint8_t address, const uint8_t* data, uint8_t len, feedback& output);

} // namespace motors::xv2
