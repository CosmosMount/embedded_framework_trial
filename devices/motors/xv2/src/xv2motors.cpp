#include "xv2motors.hpp"

namespace motors
{

namespace
{

constexpr float rad_to_deg_scale = 57.29577951308232f;
constexpr float rad_s_to_rpm_scale = 9.54929658551372f;

} // namespace

xv2motor::xv2motor(config cfg, options opts) : motor(cfg, 0.0f), options_(opts)
{
    if (options_.tx_id == xv2::default_can_id && cfg.can_id != 0)
    {
        options_.tx_id = cfg.can_id;
    }
}

types::status xv2motor::set_output()
{
    switch (control_mode)
    {
        case mode::relax:
            return disable(options_.sync);
        case mode::current:
        case mode::torque:
            return transmit(xv2::torque(options_.address, cmd.current, options_.current_ramp_ma_s,
                                  options_.sync));
        case mode::pos_speed:
            return transmit(xv2::position_speed(options_.address, rad_to_deg(cmd.position),
                                          rad_s_to_rpm(cmd.velocity),
                                          options_.position_reference, options_.sync));
        case mode::speed:
            return transmit(xv2::velocity(options_.address, rad_s_to_rpm(cmd.velocity),
                                    options_.acceleration_rpm_s, options_.sync));
        default:
        {
            const auto status = stop_now(options_.sync);
            return status == types::status::ok ? types::status::invalid_arg : status;
        }
    }
}

void xv2motor::parse_feedback(const uint8_t* data, uint8_t len)
{
    if (xv2::parse_feedback(options_.address, data, len, fdb))
    {
        mark_alive();
    }
}

types::status xv2motor::enable(bool sync)
{
    return transmit(xv2::enable(options_.address, true, sync));
}

types::status xv2motor::disable(bool sync)
{
    return transmit(xv2::enable(options_.address, false, sync));
}

types::status xv2motor::stop_now(bool sync)
{
    return transmit(xv2::stop_now(options_.address, sync));
}

types::status xv2motor::reset_position()
{
    return transmit(xv2::reset_position(options_.address));
}

types::status xv2motor::trigger_encoder_calibration()
{
    return transmit(xv2::trigger_encoder_cal(options_.address));
}

types::status xv2motor::reset_clog_protection()
{
    return transmit(xv2::reset_clog_protection(options_.address));
}

types::status xv2motor::request_feedback(xv2::sys_param param)
{
    return transmit(xv2::read_sys_param(options_.address, param));
}

types::status xv2motor::transmit(const xv2::frame& frame)
{
    return bsp::can::transmit(can_bus(), options_.tx_id, frame.data, frame.len);
}

float xv2motor::rad_to_deg(float rad)
{
    return rad * rad_to_deg_scale;
}

float xv2motor::rad_s_to_rpm(float rad_s)
{
    return rad_s * rad_s_to_rpm_scale;
}

} // namespace motors
