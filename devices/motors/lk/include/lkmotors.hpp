#pragma once

#include "motor.hpp"
#include "constants.hpp"

#include <stdint.h>

namespace motors
{

class lkmotor : public motor
{
public:
    struct rawfeedback
    {
        uint16_t last_ecd = 0;
        uint16_t ecd = 0;
        int16_t speed_dps = 0;
        int16_t current_raw = 0;
    };

    explicit lkmotor(config cfg, float kt = 0.0f, float model_reduction_ratio = 1.0f);
    void reset_position();

    types::status set_output() override;
    void parse_feedback(const uint8_t* data, uint8_t len) override;
    protocol get_protocol() const noexcept final { return protocol::lk; }
    capabilities get_capabilities() const override { return {protocol_valid(), protocol_valid() && has_torque_calibration(), false, false, false}; }

    float command_units_per_ampere() const override { return amps_to_command_; }
    int32_t raw_command_limit() const override { return max_current; }
    bool broadcast() const { return options_.lk_broadcast; }
    bool protocol_valid() const
    {
        return std::isfinite(reduction_ratio_) && reduction_ratio_ > 0.0f &&
            (options_.lk_protocol == lk_version::v2_3 || options_.lk_protocol == lk_version::v2_36) &&
            (options_.lk_encoder_counts == 16384 || options_.lk_encoder_counts == 32768 || options_.lk_encoder_counts == 65536) &&
            (!options_.lk_broadcast || options_.lk_protocol == lk_version::v2_3);
    }
    rawfeedback raw{};

protected:
    protocol_settings options_{};
    float amps_to_command_ = 0.0f;
    float feedback_amperes_per_unit_ = 0.0f;
    float position_scale() const { return 2.0f * math::pi / options_.lk_encoder_counts; }
    float reduction_ratio_ = 1.0f;
    int64_t position_counts_ = 0;
    bool feedback_seen_ = false;
    bool zero_on_first_ = false;
};

class lk8016 : public lkmotor
{
public:

    explicit lk8016(config cfg);

};

class lk9025 : public lkmotor
{
public:

    explicit lk9025(config cfg);

};

} // namespace motors
