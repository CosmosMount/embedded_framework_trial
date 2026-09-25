#include "dmmotors.hpp"
#include "config.hpp"

namespace motors
{

void dmmotor::parse_feedback(const uint8_t* data, uint8_t len)
{
    if (data == nullptr || len != 8 || !limits_valid())
    {
        return;
    }
    const limits lim = motion_limits();

    mark_alive();
    raw.first_byte = data[0];
    raw.id = data[0] & 0x0F;
    raw.err = static_cast<error>(data[0] >> 4);
    fdb.error_code = static_cast<uint8_t>(raw.err);

    const uint16_t p_int = static_cast<uint16_t>((data[1] << 8) | data[2]);
    const uint16_t v_int = static_cast<uint16_t>((data[3] << 4) | (data[4] >> 4));
    const uint16_t t_int = static_cast<uint16_t>(((data[4] & 0x0F) << 8) | data[5]);

    fdb.position = math::uint_to_float(static_cast<int>(p_int), lim.position_min, lim.position_max, 16);
    fdb.velocity = math::uint_to_float(static_cast<int>(v_int), lim.velocity_min, lim.velocity_max, 12);
    fdb.torque = math::uint_to_float(static_cast<int>(t_int), lim.torque_min, lim.torque_max, 12);
    raw.temp_mos = static_cast<float>(data[6]);
    raw.temp_rotor = static_cast<float>(data[7]);
    fdb.temperature = raw.temp_rotor;
    fdb.temperature_valid = true;
    fdb.torque_valid = true;
    // Single RX writer; publish status and sequence together for the start waiter.
    feedback_revision_ = (feedback_revision_ + 1U) & 0x0fffffffU;
    feedback_token_.store((feedback_revision_ << 4U) | static_cast<uint32_t>(raw.err), std::memory_order_release);
}

types::status dmmotor::transmit_frame(uint32_t frame_id, const uint8_t* data, uint16_t len)
{
    last_tx_id = frame_id;
    disable_id_ = frame_id;
    return bsp::can::transmit(can_bus(), frame_id, data, len);
}

types::status dmmotor::emit_disable()
{
    return transmit_frame(disable_id_, disable_frame, 8);
}

types::status dmmotor::emit_mit(float position, float velocity, float kp, float kd, float torque)
{
    uint8_t output[8] = {0};
    const limits lim = motion_limits();
    if (!limits_valid() || !std::isfinite(position) || !std::isfinite(velocity) || !std::isfinite(kp) ||
        !std::isfinite(kd) || !std::isfinite(torque))
    {
        const auto status = emit_disable();
        return status == types::status::ok ? types::status::invalid_arg : status;
    }
    const float position_set = math::clamp(position, lim.position_min, lim.position_max);
    const uint16_t pos_tmp = static_cast<uint16_t>(math::float_to_uint(
        position_set, lim.position_min, lim.position_max, 16));
    const uint16_t vel_tmp = static_cast<uint16_t>(math::float_to_uint(
        math::clamp(velocity, lim.velocity_min, lim.velocity_max), lim.velocity_min, lim.velocity_max, 12));
    const uint16_t kp_tmp = static_cast<uint16_t>(math::float_to_uint(math::clamp(kp, lim.kp_min, lim.kp_max), lim.kp_min, lim.kp_max, 12));
    const uint16_t kd_tmp = static_cast<uint16_t>(math::float_to_uint(math::clamp(kd, lim.kd_min, lim.kd_max), lim.kd_min, lim.kd_max, 12));
    const uint16_t tor_tmp = static_cast<uint16_t>(math::float_to_uint(
        math::clamp(torque, lim.torque_min, lim.torque_max), lim.torque_min, lim.torque_max, 12));

    output[0] = static_cast<uint8_t>(pos_tmp >> 8);
    output[1] = static_cast<uint8_t>(pos_tmp);
    output[2] = static_cast<uint8_t>(vel_tmp >> 4);
    output[3] = static_cast<uint8_t>(((vel_tmp & 0xF) << 4) | (kp_tmp >> 8));
    output[4] = static_cast<uint8_t>(kp_tmp);
    output[5] = static_cast<uint8_t>(kd_tmp >> 4);
    output[6] = static_cast<uint8_t>(((kd_tmp & 0xF) << 4) | (tor_tmp >> 8));
    output[7] = static_cast<uint8_t>(tor_tmp);

    return transmit_frame(can_id(), output, 8);
}

types::status dmmotor::emit_pos_speed(float position, float velocity)
{
    uint8_t output[8] = {0};
    std::memcpy(output, &position, 4);
    std::memcpy(output + 4, &velocity, 4);
    return transmit_frame(can_id() + 0x100, output, 8);
}

types::status dmmotor::emit_velocity(float velocity)
{
    uint8_t output[8] = {0};
    std::memcpy(output, &velocity, 4);
    return transmit_frame(can_id() + 0x200, output, 4);
}

dm4310::dm4310(config cfg)
    : dmmotor(cfg, {
          params::motors::dm_dm4310::position_min,
          params::motors::dm_dm4310::position_max,
          params::motors::dm_dm4310::velocity_min,
          params::motors::dm_dm4310::velocity_max,
          params::motors::dm_dm4310::torque_min,
          params::motors::dm_dm4310::torque_max,
          params::motors::dm_dm4310::kp_min,
          params::motors::dm_dm4310::kp_max,
          params::motors::dm_dm4310::kd_min,
          params::motors::dm_dm4310::kd_max})
{
    lower_pos_limit = motion_limits().position_min;
    upper_pos_limit = motion_limits().position_max;
}

types::status dm4310::set_output()
{
    cmd.position = math::clamp(cmd.position, lower_pos_limit, upper_pos_limit);

    switch (control_mode)
    {
        case mode::relax:
            return emit_disable();
        case mode::torque:
            return emit_mit(0.0f, 0.0f, 0.0f, 0.0f, cmd.torque);
        case mode::mit:
            return emit_mit(cmd.position, cmd.velocity, cmd.kp, cmd.kd, cmd.torque);
        case mode::pos_speed:
            return emit_pos_speed(cmd.position, cmd.velocity);
        case mode::speed:
            return emit_velocity(cmd.velocity);
        default:
        {
            const auto status = emit_disable();
            return status == types::status::ok ? types::status::invalid_arg : status;
        }
    }
}

dm8009p::dm8009p(config cfg)
    : dmmotor(cfg, {
          params::motors::dm_dm8009p::position_min,
          params::motors::dm_dm8009p::position_max,
          params::motors::dm_dm8009p::velocity_min,
          params::motors::dm_dm8009p::velocity_max,
          params::motors::dm_dm8009p::torque_min,
          params::motors::dm_dm8009p::torque_max,
          params::motors::dm_dm8009p::kp_min,
          params::motors::dm_dm8009p::kp_max,
          params::motors::dm_dm8009p::kd_min,
          params::motors::dm_dm8009p::kd_max})
{
    lower_pos_limit = motion_limits().position_min;
    upper_pos_limit = motion_limits().position_max;
}

types::status dm8009p::set_output()
{
    switch (control_mode)
    {
        case mode::relax:
            return emit_disable();
        case mode::torque:
            return emit_mit(0.0f, 0.0f, 0.0f, 0.0f, cmd.torque);
        case mode::mit:
            return emit_mit(cmd.position, cmd.velocity, cmd.kp, cmd.kd, cmd.torque);
        case mode::pos_speed:
            cmd.position = math::clamp(cmd.position, lower_pos_limit, upper_pos_limit);
            return emit_pos_speed(cmd.position, cmd.velocity);
        case mode::speed:
            return emit_velocity(cmd.velocity);
        default:
        {
            const auto status = emit_disable();
            return status == types::status::ok ? types::status::invalid_arg : status;
        }
    }
}

} // namespace motors
