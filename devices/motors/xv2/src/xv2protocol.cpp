#include "xv2protocol.hpp"
#include "constants.hpp"

namespace motors::xv2
{

namespace
{

uint16_t be_u16(const uint8_t* data)
{
    return static_cast<uint16_t>((data[0] << 8) | data[1]);
}

uint32_t be_u32(const uint8_t* data)
{
    return (static_cast<uint32_t>(data[0]) << 24) | (static_cast<uint32_t>(data[1]) << 16) |
           (static_cast<uint32_t>(data[2]) << 8) | static_cast<uint32_t>(data[3]);
}

void put_u16(frame& f, uint16_t value)
{
    f.data[f.len++] = static_cast<uint8_t>(value >> 8);
    f.data[f.len++] = static_cast<uint8_t>(value);
}

void put_u32(frame& f, uint32_t value)
{
    f.data[f.len++] = static_cast<uint8_t>(value >> 24);
    f.data[f.len++] = static_cast<uint8_t>(value >> 16);
    f.data[f.len++] = static_cast<uint8_t>(value >> 8);
    f.data[f.len++] = static_cast<uint8_t>(value);
}

frame begin(uint8_t address, uint8_t command)
{
    frame f{};
    f.data[f.len++] = address;
    f.data[f.len++] = command;
    return f;
}

void end(frame& f)
{
    f.data[f.len++] = frame_tail;
}

} // namespace

uint8_t direction_of(float value)
{
    return value >= 0.0f ? 0U : 1U;
}

uint16_t scaled_abs_u16(float value, float scale)
{
    const float scaled = value >= 0.0f ? value * scale : -value * scale;
    return static_cast<uint16_t>(scaled);
}

uint32_t scaled_abs_u32(float value, float scale)
{
    const float scaled = value >= 0.0f ? value * scale : -value * scale;
    return static_cast<uint32_t>(scaled);
}

frame trigger_encoder_cal(uint8_t address)
{
    frame f = begin(address, 0x06);
    f.data[f.len++] = 0x45;
    end(f);
    return f;
}

frame reset_motor(uint8_t address)
{
    frame f = begin(address, 0x08);
    f.data[f.len++] = 0x97;
    end(f);
    return f;
}

frame reset_position(uint8_t address)
{
    frame f = begin(address, 0x0A);
    f.data[f.len++] = 0x6D;
    end(f);
    return f;
}

frame reset_clog_protection(uint8_t address)
{
    frame f = begin(address, 0x0E);
    f.data[f.len++] = 0x52;
    end(f);
    return f;
}

frame enable(uint8_t address, bool state, bool sync)
{
    frame f = begin(address, 0xF3);
    f.data[f.len++] = 0xAB;
    f.data[f.len++] = static_cast<uint8_t>(state);
    f.data[f.len++] = static_cast<uint8_t>(sync);
    end(f);
    return f;
}

frame torque(uint8_t address, int16_t current_ma, uint16_t ramp_ma_s, bool sync)
{
    frame f = begin(address, 0xF5);
    f.data[f.len++] = current_ma >= 0 ? 0U : 1U;
    put_u16(f, ramp_ma_s);
    put_u16(f, static_cast<uint16_t>(current_ma >= 0 ? current_ma : -current_ma));
    f.data[f.len++] = static_cast<uint8_t>(sync);
    end(f);
    return f;
}

frame velocity(uint8_t address, float rpm, uint16_t acceleration_rpm_s, bool sync)
{
    frame f = begin(address, 0xF6);
    f.data[f.len++] = direction_of(rpm);
    put_u16(f, acceleration_rpm_s);
    put_u16(f, scaled_abs_u16(rpm, 10.0f));
    f.data[f.len++] = static_cast<uint8_t>(sync);
    end(f);
    return f;
}

frame position_speed(uint8_t address, float position_deg, float velocity_rpm,
                     position_reference reference, bool sync)
{
    frame f = begin(address, 0xFB);
    f.data[f.len++] = direction_of(position_deg);
    put_u16(f, scaled_abs_u16(velocity_rpm, 10.0f));
    put_u32(f, scaled_abs_u32(position_deg, 10.0f));
    f.data[f.len++] = static_cast<uint8_t>(reference);
    f.data[f.len++] = static_cast<uint8_t>(sync);
    end(f);
    return f;
}

frame stop_now(uint8_t address, bool sync)
{
    frame f = begin(address, 0xFE);
    f.data[f.len++] = 0x98;
    f.data[f.len++] = static_cast<uint8_t>(sync);
    end(f);
    return f;
}

frame synchronous_motion(uint8_t address)
{
    frame f = begin(address, 0xFF);
    f.data[f.len++] = 0x66;
    end(f);
    return f;
}

frame read_sys_param(uint8_t address, sys_param param)
{
    frame f = begin(address, static_cast<uint8_t>(param));
    end(f);
    return f;
}

bool parse_feedback(uint8_t address, const uint8_t* data, uint8_t len, feedback& output)
{
    if (data == nullptr || len < 3 || data[0] != address || data[len - 1] != frame_tail)
    {
        return false;
    }

    switch (data[1])
    {
        case static_cast<uint8_t>(sys_param::velocity):
            if (len >= 5)
            {
                const float rpm = static_cast<float>(static_cast<int16_t>(be_u16(&data[2]))) * 0.1f;
                output.velocity = rpm * (math::two_pi / 60.0f);
                return true;
            }
            break;
        case static_cast<uint8_t>(sys_param::position):
            if (len >= 7)
            {
                output.position = math::degrees_to_radians(static_cast<float>(be_u32(&data[2])) * 0.1f);
                return true;
            }
            break;
        case static_cast<uint8_t>(sys_param::temperature):
            if (len >= 4)
            {
                output.temperature = static_cast<float>(data[2]);
                return true;
            }
            break;
        case static_cast<uint8_t>(sys_param::flags):
            if (len >= 4)
            {
                output.error_code = data[2];
                return true;
            }
            break;
        default:
            break;
    }

    return false;
}

} // namespace motors::xv2
