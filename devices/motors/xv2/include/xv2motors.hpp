#pragma once

#include "motor.hpp"
#include "xv2protocol.hpp"

#include <cstdint>

namespace motors
{

struct xv2_options
{
    uint8_t address = 1;
    uint32_t tx_id = xv2::default_can_id;
    uint16_t acceleration_rpm_s = 1000;
    uint16_t current_ramp_ma_s = 1000;
    bool sync = false;
    xv2::position_reference position_reference = xv2::position_reference::absolute;
};

class xv2motor : public motor
{
public:
    using options = xv2_options;

    explicit xv2motor(config cfg, options opts = options{});

    types::status set_output() override;
    void parse_feedback(const uint8_t* data, uint8_t len) override;
    protocol get_protocol() const noexcept final { return protocol::xv2; }
    capabilities get_capabilities() const override { return {true, has_torque_calibration(), false, true, true}; }

    types::status enable(bool sync = false);
    types::status disable(bool sync = false);
    types::status stop_now(bool sync = false);
    types::status reset_position();
    types::status trigger_encoder_calibration();
    types::status reset_clog_protection();
    types::status request_feedback(xv2::sys_param param);

    uint8_t address() const { return options_.address; }
    uint32_t tx_id() const { return options_.tx_id; }

private:
    types::status transmit(const xv2::frame& frame);
    static float rad_to_deg(float rad);
    static float rad_s_to_rpm(float rad_s);

    options options_{};
};

class x42 : public xv2motor
{
public:
    using xv2motor::xv2motor;
};

class y42 : public xv2motor
{
public:
    using xv2motor::xv2motor;
};

} // namespace motors
