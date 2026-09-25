#include "djimotors.hpp"

#include "constants.hpp"

namespace motors
{

djimotor::djimotor(config cfg, float kt, float pos_to_rad, float rpm_to_rad_s,
                   bool multi_turn_feedback)
    : motor(cfg, kt),
      pos_to_rad_(pos_to_rad),
      rpm_to_rad_s_(rpm_to_rad_s),
      multi_turn_feedback_(multi_turn_feedback)
{
}

float djimotor::rpm_to_rad_s(int16_t rpm) const
{
    return static_cast<float>(rpm) * rpm_to_rad_s_;
}

float djimotor::ecd_to_rad(const rawfeedback& raw_fb) const
{
    return (static_cast<float>(raw_fb.ecd) + static_cast<float>(raw_fb.ecd_cnt) * 8192.0f - raw_fb.ecd_offset) * pos_to_rad_;
}

void djimotor::parse_feedback(const uint8_t* data, uint8_t len)
{
    if (data == nullptr || len != 8)
    {
        return;
    }
    mark_alive();

    raw.last_ecd = raw.ecd;
    raw.ecd = static_cast<uint16_t>((data[0] << 8) | data[1]);
    if (!feedback_seen_)
    {
        raw.last_ecd = raw.ecd; // First sample is not a wrap from an invented zero.
        feedback_seen_ = true;
    }
    raw.speed_rpm = static_cast<int16_t>((data[2] << 8) | data[3]);
    fdb.current = static_cast<float>(static_cast<int16_t>((data[4] << 8) | data[5]));
    fdb.temperature = c610_protocol_ ? 0.0f : static_cast<float>(data[6]);
    fdb.temperature_valid = !c610_protocol_;
    fdb.error_code = c610_protocol_ ? data[7] : 0;

    if (!multi_turn_feedback_)
    {
        fdb.velocity = rpm_to_rad_s(raw.speed_rpm);
        fdb.position = static_cast<float>(raw.ecd) * pos_to_rad_ - math::pi;
        fdb.torque = 0.0f;
        return;
    }

    if (raw.ecd - raw.last_ecd > 4096)
    {
        raw.ecd_cnt--;
    }
    else if (raw.ecd - raw.last_ecd < -4096)
    {
        raw.ecd_cnt++;
    }

    fdb.velocity = rpm_to_rad_s(raw.speed_rpm);
    fdb.position = ecd_to_rad(raw);
    fdb.torque = 0.0f;
}

void djimotor::reset_position()
{
    raw.ecd_offset = raw.ecd;
    raw.ecd_cnt = 0;
    raw.total_cnt = 0;
    fdb.position = 0.0f;
}

types::status djimotor::set_output()
{
    if (control_mode == mode::relax)
    {
        cmd.current = 0;
        cmd.voltage_raw = 0;
    }
    return types::status::ok;
}

m2006::m2006(config cfg) : djimotor(cfg, 0.0f, 0.00002130788978f, 0.002908882087f, true)
{
    c610_protocol_ = true;
    amps_to_command_ = 1000.0f; // C610: +/-10000 <-> +/-10 A.
    command_limit_ = 10000;
}

m3508::m3508(config cfg)
    : djimotor(cfg, 0.0f, 0.00003994074176f, 0.005453242609f, true)
{
    amps_to_command_ = 16384.0f / 20.0f;
    command_limit_ = 16384;
}

gm6020::gm6020(config cfg)
    : djimotor(cfg, 0.0f, 0.0007669903939f, 0.1047197551196f, false)
{
    gm6020_protocol_ = true;
    voltage_control_ = !cfg.protocol_options ||
        cfg.protocol_options->gm6020_control == gm6020_loop::voltage;
    amps_to_command_ = voltage_control_ ? 0.0f : 16384.0f / 3.0f;
    command_limit_ = voltage_control_ ? 25000 : 16384;
}

xroll::xroll(config cfg)
    : djimotor(cfg, 0.0f, 0.000048658315f, 0.006642670920f, true)
{
}

bool djimotor::valid_can_id() const
{
    return gm6020_protocol_ ? (can_id() >= 0x205 && can_id() <= 0x20b)
                           : (can_id() >= 0x201 && can_id() <= 0x208);
}

uint32_t djimotor::control_id() const
{
    if (gm6020_protocol_)
        return voltage_control_ ? (can_id() <= 0x208 ? 0x1ff : 0x2ff)
                                : (can_id() <= 0x208 ? 0x1fe : 0x2fe);
    return can_id() <= 0x204 ? 0x200 : 0x1ff;
}

uint8_t djimotor::control_slot() const
{
    return static_cast<uint8_t>((can_id() - (gm6020_protocol_ ? 0x205 : 0x201)) % 4);
}

} // namespace motors
