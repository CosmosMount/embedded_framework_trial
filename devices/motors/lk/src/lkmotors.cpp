#include "lkmotors.hpp"

#include "constants.hpp"
#include "constrain.hpp"

namespace motors
{

lkmotor::lkmotor(config cfg, float kt, float model_reduction_ratio) : motor(cfg, kt)
{
    if (cfg.protocol_options) options_ = *cfg.protocol_options;
    reduction_ratio_ = options_.lk_reduction_ratio == 0.0f ? model_reduction_ratio : options_.lk_reduction_ratio;
    max_current = options_.lk_protocol == lk_version::v2_3 ? 2000 : 2048;
    amps_to_command_ = 2000.0f / 32.0f;
    feedback_amperes_per_unit_ = 33.0f / 2048.0f;
}

void lkmotor::parse_feedback(const uint8_t* data, uint8_t len)
{
    if (data == nullptr || len != 8 || !protocol_valid() ||
        (data[0] != 0x9c && data[0] != 0xa1))
    {
        return;
    }
    const uint16_t encoder = static_cast<uint16_t>((data[7] << 8) | data[6]);
    if (encoder >= options_.lk_encoder_counts) return;
    mark_alive();
    raw.last_ecd = raw.ecd;
    raw.ecd = encoder;
    raw.speed_dps = static_cast<int16_t>((data[5] << 8) | data[4]);
    raw.current_raw = static_cast<int16_t>((data[3] << 8) | data[2]);
    fdb.temperature = static_cast<float>(static_cast<int8_t>(data[1]));

    if (!feedback_seen_)
    {
        position_counts_ = zero_on_first_ ? 0 : raw.ecd;
        zero_on_first_ = false;
        raw.last_ecd = raw.ecd;
        feedback_seen_ = true;
    }
    else
    {
        const int32_t counts = static_cast<int32_t>(options_.lk_encoder_counts);
        int32_t delta = static_cast<int32_t>(raw.ecd) - raw.last_ecd;
        if (delta > counts / 2) delta -= counts;
        else if (delta < -counts / 2) delta += counts;
        position_counts_ += delta;
    }
    fdb.position = (static_cast<float>(position_counts_) - offset) * position_scale() / reduction_ratio_;
    fdb.velocity = static_cast<float>(raw.speed_dps) * (math::pi / 180.0f) / reduction_ratio_;
    fdb.current = static_cast<float>(raw.current_raw);
    fdb.current_amperes = fdb.current * feedback_amperes_per_unit_;
    fdb.current_amperes_valid = true;
    fdb.temperature_valid = true;
    // Command and feedback scales differ in V2.3.
    fdb.torque_valid = has_torque_calibration();
    fdb.torque = fdb.torque_valid ? torque_from_current(fdb.current_amperes * amps_to_command_) : 0.0f;
}

types::status lkmotor::set_output()
{
    if (control_mode == mode::relax)
    {
        cmd.current = 0;
    }
    return types::status::ok;
}

lk8016::lk8016(config cfg) : lkmotor(cfg, 0.0f, 6.0f)
{
    if (options_.lk_protocol == lk_version::v2_36) amps_to_command_ = 2048.0f / 33.0f;
}

lk9025::lk9025(config cfg) : lkmotor(cfg, 0.0f)
{
    if (options_.lk_protocol == lk_version::v2_36)
    {
        amps_to_command_ = 2048.0f / 16.5f;
        feedback_amperes_per_unit_ = 16.5f / 2048.0f;
    }
}

void lkmotor::reset_position()
{
    position_counts_ = 0;
    zero_on_first_ = !feedback_seen_;
    offset = 0.0f;
    fdb.position = 0.0f;
    // The next valid sample continues from the last encoder sample.
}

} // namespace motors
