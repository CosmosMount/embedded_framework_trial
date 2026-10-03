#pragma once

#include "bsp_can.hpp"
#include "tx_api.h"
#include "usertypes.hpp"

#include <stdint.h>
#include <cmath>

namespace motors
{

enum class state : uint8_t
{
    offline = 0,
    online,
    blocked,
};

enum class mode : uint8_t
{
    relax = 0,
    current,
    torque,
    mit,
    pos_speed,
    speed,
    voltage_raw = 7,
};

struct command
{
    // Legacy protocol command units, NOT amperes (use set_current_raw).
    int16_t current = 0;
    int16_t voltage_raw = 0;
    float torque = 0.0f;
    float position = 0.0f;
    float velocity = 0.0f;
    float kp = 0.0f;
    float kd = 0.0f;
};

struct feedback
{
    float position = 0.0f; // rad
    float velocity = 0.0f; // rad/s
    float torque = 0.0f;
    float current = 0.0f;
    float temperature = 0.0f;
    uint8_t error_code = 0;
    // Explicit physical feedback; legacy current remains the raw protocol field.
    float current_amperes = 0.0f;
    bool current_amperes_valid = false;
    bool torque_valid = false;
    bool temperature_valid = false;
};

struct capabilities
{
    bool current = false;
    bool torque = false;
    bool mit = false;
    bool pos_speed = false;
    bool velocity = false;
    bool voltage_raw = false;
};

class api
{
public:
    virtual ~api() = default;

    virtual types::status set_command(const command& next_command, mode next_mode) = 0;
    // Returns a value snapshot. CAN feedback is written from its RX interrupt.
    virtual feedback get_feedback() const = 0;
    virtual state status() const = 0;
    virtual capabilities get_capabilities() const = 0;
    virtual bool supports(mode request_mode) const = 0;
    virtual void relax() = 0;
};

enum class protocol : uint8_t
{
    dji = 0,
    dm,
    lk,
    xv2,
    count,
};

// Wire scaling must match the controller configuration; radians, rad/s, Nm.
// Gains use the motor protocol units. Not an actuator safety envelope.
struct mit_limits
{
    float position_min, position_max;
    float velocity_min, velocity_max;
    float torque_min, torque_max;
    float kp_min, kp_max;
    float kd_min, kd_max;
};

// Calibration of legacy wire command units, not a motor-shaft Nm/A constant.
// Zero explicitly means unknown: current-driven torque control is unavailable.
struct drive_parameters
{
    float torque_nm_per_command_unit = 0.0f;
    // Output-shaft torque per protocol torque-current ampere, not bus current.
    // Choose either physical or raw calibration, never both nonzero.
    float torque_nm_per_ampere = 0.0f;
};

enum class gm6020_loop : uint8_t { voltage, current };
enum class lk_version : uint8_t { v2_3, v2_36 };
struct protocol_settings
{
    gm6020_loop gm6020_control = gm6020_loop::voltage;
    lk_version lk_protocol = lk_version::v2_3;
    uint32_t lk_encoder_counts = 16384;
    bool lk_broadcast = false;
    float lk_reduction_ratio = 0.0f; // 0 selects model default; motor turns / output turn.
};

struct config
{
    bsp::can::bus can_bus = bsp::can::bus::none;
    bsp::can::bus_type can_type = bsp::can::bus_type::classic;
    uint32_t can_id = 0;
    mode control_mode = mode::relax;
    // Non-owning immutable limits; generated storage has static lifetime.
    // nullptr selects the model defaults from generated params.
    const mit_limits* mit = nullptr;
    // Copied at construction. nullptr leaves current-driven torque uncalibrated.
    const drive_parameters* drive = nullptr;
    const protocol_settings* protocol_options = nullptr;
    // ThreadX ticks. DM start validates 1..INT32_MAX, sends once, then waits.
    uint32_t enable_timeout_ticks = 0U; // Generated params supply the policy; zero rejects startup.
};


class motor : public api
{
public:
    explicit motor(config cfg, float kt = 0.0f) : torque_constant(cfg.drive ? cfg.drive->torque_nm_per_command_unit : kt), config_(cfg),
        torque_nm_per_ampere_(cfg.drive ? cfg.drive->torque_nm_per_ampere : 0.0f)
    {
        control_mode = cfg.control_mode;
    }

    virtual ~motor() = default;

    virtual types::status set_output() = 0;
    virtual void parse_feedback(const uint8_t* data, uint8_t len) = 0;
    virtual protocol get_protocol() const noexcept = 0;

    state alive_check();

    types::status set_command(const command& next_command, mode next_mode) override;
    feedback get_feedback() const override
    {
        // Both supported MCUs are single-core; mask the CAN writer only for the copy.
        TX_INTERRUPT_SAVE_AREA
        TX_DISABLE
        feedback snapshot = fdb;
        TX_RESTORE
        return snapshot;
    }
    state status() const override { return motor_state; }
    capabilities get_capabilities() const override { return {}; }
    bool supports(mode request_mode) const override;
    void relax() override;

    // Commands only update state; owner thread later calls set_output().
    // Failure rejects the command and clears pending output; missing physical scaling is not_configured.
    types::status set_current_raw(int16_t current);
    types::status set_current_amperes(float amperes);
    types::status set_voltage_raw(int16_t voltage);
    // A raw command unit can be current, voltage, or unsupported for this mode.
    virtual float command_units_per_ampere() const { return 0.0f; }
    virtual int32_t raw_command_limit() const { return 32767; }
    types::status set_torque(float torque);
    types::status set_mit(float position, float velocity, float kp, float kd, float torque);
    types::status set_pos_speed(float position, float velocity);
    types::status set_velocity(float velocity);

    float torque_from_current(float current) const { return current * effective_torque_nm_per_command_unit(); }
    float current_from_torque(float torque) const
    {
        const float scale = effective_torque_nm_per_command_unit();
        return scale > 0.0f ? torque / scale : 0.0f;
    }

    const config& cfg() const { return config_; }
    bsp::can::bus can_bus() const { return config_.can_bus; }
    bsp::can::bus_type can_type() const { return config_.can_type; }
    uint32_t can_id() const { return config_.can_id; }

    command cmd{};
    feedback fdb{};
    mode control_mode = mode::relax;
    state motor_state = state::offline;

    // Nm per raw command unit. Immutable; not Nm/A.
    const float torque_constant;
    bool has_torque_calibration() const
    {
        const float scale = effective_torque_nm_per_command_unit();
        return std::isfinite(scale) && scale > 0.0f;
    }

    float effective_torque_nm_per_command_unit() const
    {
        if (!std::isfinite(torque_constant) || torque_constant < 0.0f ||
            !std::isfinite(torque_nm_per_ampere_) || torque_nm_per_ampere_ < 0.0f ||
            (torque_constant > 0.0f && torque_nm_per_ampere_ > 0.0f)) return 0.0f;
        if (torque_nm_per_ampere_ > 0.0f)
        {
            const float units = command_units_per_ampere();
            return units > 0.0f ? torque_nm_per_ampere_ / units : 0.0f;
        }
        return torque_constant;
    }
    float torque_nm_per_ampere() const
    {
        return has_torque_calibration() ? effective_torque_nm_per_command_unit() * command_units_per_ampere() : 0.0f;
    }

    uint16_t max_current = 0;
    float offset = 0.0f;

    uint32_t alive = 0;
    uint32_t prealive = 0;

protected:
    void mark_alive() { ++alive; }

    config config_;

private:
    const float torque_nm_per_ampere_;
};

inline void motor::relax()
{
    control_mode = mode::relax;
    cmd = {};
}

inline types::status motor::set_command(const command& next_command, mode next_mode)
{
    if (next_mode == mode::torque && get_capabilities().current && !has_torque_calibration())
    {
        relax();
        return types::status::not_configured;
    }
    if (!supports(next_mode))
    {
        relax();
        return types::status::not_configured;
    }
    if (!std::isfinite(next_command.torque) ||
        !std::isfinite(next_command.position) || !std::isfinite(next_command.velocity) ||
        !std::isfinite(next_command.kp) || !std::isfinite(next_command.kd))
    {
        relax();
        return types::status::invalid_arg;
    }
    if ((next_mode == mode::current &&
         (next_command.current < -raw_command_limit() || next_command.current > raw_command_limit())) ||
        (next_mode == mode::voltage_raw &&
         (next_command.voltage_raw < -raw_command_limit() || next_command.voltage_raw > raw_command_limit())))
    {
        relax();
        return types::status::invalid_arg;
    }
    cmd = next_command;
    if (next_mode == mode::torque && get_capabilities().current)
    {
        const float raw_current = current_from_torque(cmd.torque);
        // Check before integer conversion; out-of-range float-to-int is undefined.
        if (!std::isfinite(raw_current) || raw_current < -raw_command_limit() || raw_current > raw_command_limit())
        {
            relax();
            return types::status::invalid_arg;
        }
        cmd.current = static_cast<int16_t>(raw_current);
    }
    control_mode = next_mode;
    return types::status::ok;
}

inline bool motor::supports(mode request_mode) const
{
    const capabilities caps = get_capabilities();
    switch (request_mode)
    {
        case mode::relax:
            return true;
        case mode::current:
            return caps.current;
        case mode::torque:
            return caps.torque;
        case mode::mit:
            return caps.mit;
        case mode::pos_speed:
            return caps.pos_speed;
        case mode::speed:
            return caps.velocity;
        case mode::voltage_raw:
            return caps.voltage_raw;
        default:
            return false;
    }
}

inline types::status motor::set_current_raw(int16_t current)
{
    command next{};
    next.current = current;
    return set_command(next, mode::current);
}

inline types::status motor::set_current_amperes(float amperes)
{
    const float scale = command_units_per_ampere();
    if (scale <= 0.0f)
    {
        relax();
        return types::status::not_configured;
    }
    const float raw = amperes * scale;
    if (!std::isfinite(amperes) || !std::isfinite(raw) ||
        raw < -raw_command_limit() || raw > raw_command_limit())
    {
        relax();
        return types::status::invalid_arg;
    }
    return set_current_raw(static_cast<int16_t>(raw));
}

inline types::status motor::set_voltage_raw(int16_t voltage)
{
    command next{};
    next.voltage_raw = voltage;
    return set_command(next, mode::voltage_raw);
}

inline types::status motor::set_torque(float torque)
{
    command next{};
    next.torque = torque;
    return set_command(next, mode::torque);
}

inline types::status motor::set_mit(float position, float velocity, float kp, float kd, float torque)
{
    command next{};
    next.position = position;
    next.velocity = velocity;
    next.kp = kp;
    next.kd = kd;
    next.torque = torque;
    return set_command(next, mode::mit);
}

inline types::status motor::set_pos_speed(float position, float velocity)
{
    command next{};
    next.position = position;
    next.velocity = velocity;
    return set_command(next, mode::pos_speed);
}

inline types::status motor::set_velocity(float velocity)
{
    command next{};
    next.velocity = velocity;
    return set_command(next, mode::speed);
}

inline state motor::alive_check()
{
    if (alive == prealive)
    {
        motor_state = state::offline;
    }
    else
    {
        prealive = alive;
        motor_state = state::online;
    }
    return motor_state;
}

} // namespace motors
