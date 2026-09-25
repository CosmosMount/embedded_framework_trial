#pragma once

#include "motor.hpp"

#include <cstdint>

namespace motors
{

class djimotor : public motor
{
public:
    struct rawfeedback
    {
        int16_t last_ecd = 0;
        uint16_t ecd = 0;
        uint16_t ecd_offset = 0;
        int16_t speed_rpm = 0;
        int32_t ecd_cnt = 0;
        int16_t total_cnt = 0;
    };

    explicit djimotor(config cfg, float kt = 0.0f, float pos_to_rad = 0.0007669903939f,
                      float rpm_to_rad_s = 0.1047197551196f,
                      bool multi_turn_feedback = false);

    types::status set_output() override;
    void parse_feedback(const uint8_t* data, uint8_t len) override;
    protocol get_protocol() const noexcept final { return protocol::dji; }
    capabilities get_capabilities() const override
    { return {!voltage_control_, !voltage_control_ && has_torque_calibration(), false, false, false, voltage_control_}; }
    float command_units_per_ampere() const override { return amps_to_command_; }
    int32_t raw_command_limit() const override { return command_limit_; }
    bool valid_can_id() const;
    uint32_t control_id() const;
    uint8_t control_slot() const;
    int16_t raw_output() const { return voltage_control_ ? cmd.voltage_raw : cmd.current; }

    void reset_position();

    rawfeedback raw{};
    bool blocked = false;

protected:
    bool gm6020_protocol_ = false;
    bool c610_protocol_ = false;
    bool voltage_control_ = false;
    float amps_to_command_ = 0.0f;
    int32_t command_limit_ = 32767;

private:
    float rpm_to_rad_s(int16_t rpm) const;
    float ecd_to_rad(const rawfeedback& raw_fb) const;

    float pos_to_rad_ = 0.0007669903939f;
    float rpm_to_rad_s_ = 0.1047197551196f;
    bool multi_turn_feedback_ = false;
    bool feedback_seen_ = false;
};

class m2006 : public djimotor
{
public:
    explicit m2006(config cfg);
};

class m3508 : public djimotor
{
public:
    explicit m3508(config cfg);
};

class gm6020 : public djimotor
{
public:
    explicit gm6020(config cfg);
};

class xroll : public djimotor
{
public:
    explicit xroll(config cfg);
};

} // namespace motors
