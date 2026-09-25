#pragma once

#include "motor.hpp"

#include "constants.hpp"
#include "constrain.hpp"
#include "trans.hpp"

#include <cstdint>
#include <atomic>
#include <cstring>

namespace motors
{

class dmmotor : public motor
{
public:
    enum class error : uint8_t
    {
        disable = 0x0,
        enable = 0x1,
        over_voltage = 0x8,
        under_voltage = 0x9,
        over_current = 0xA,
        mos_over_temp = 0xB,
        coil_over_temp = 0xC,
        comm_lost = 0xD,
        overload = 0xE,
    };

    using limits = mit_limits;

    struct rawfeedback
    {
        uint8_t first_byte = 0;
        uint8_t id = 0;
        error err = error::disable;
        float temp_mos = 0.0f;
        float temp_rotor = 0.0f;
    };

    static inline const uint8_t enable_frame[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
    static inline const uint8_t disable_frame[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
    static inline const uint8_t save_zero_frame[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};
    static inline const uint8_t clear_error_frame[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB};

    explicit dmmotor(config cfg, const limits& defaults, float kt = 1.0f)
        : motor(cfg, kt), enable_timeout_ticks(cfg.enable_timeout_ticks),
          limits_(cfg.mit ? *cfg.mit : defaults),
          disable_id_(cfg.can_id + (cfg.control_mode == mode::pos_speed ? 0x100U : cfg.control_mode == mode::speed ? 0x200U : 0U)) {}

    void parse_feedback(const uint8_t* data, uint8_t len) override;
    protocol get_protocol() const noexcept final { return protocol::dm; }
    capabilities get_capabilities() const override { return {false, limits_valid(), limits_valid(), limits_valid(), limits_valid()}; }

    types::status transmit_frame(uint32_t frame_id, const uint8_t* data, uint16_t len);

    // Thread-context handshake reads only this atomic token, never raw.err.
    uint32_t feedback_token() const noexcept { return feedback_token_.load(std::memory_order_acquire); }
    uint32_t wire_control_id() const noexcept { return disable_id_; }
    const uint32_t enable_timeout_ticks;
    rawfeedback raw{};
    bool enablefailed = false;
    uint32_t last_tx_id = 0;

    float lower_pos_limit = 0.0f;
    float upper_pos_limit = 0.0f;

protected:
    bool limits_valid() const
    {
        const auto& l = limits_;
        return std::isfinite(l.position_max) && l.position_max > 0 && l.position_min == -l.position_max &&
            std::isfinite(l.velocity_max) && l.velocity_max > 0 && l.velocity_min == -l.velocity_max &&
            std::isfinite(l.torque_max) && l.torque_max > 0 && l.torque_min == -l.torque_max &&
            l.kp_min == 0 && l.kp_max == 500 && l.kd_min == 0 && l.kd_max == 5;
    }
    const limits& motion_limits() const { return limits_; }
    types::status emit_disable();
    types::status emit_mit(float position, float velocity, float kp, float kd, float torque);
    types::status emit_pos_speed(float position, float velocity);
    types::status emit_velocity(float velocity);

private:
    static_assert(std::atomic<uint32_t>::is_always_lock_free);
    uint32_t feedback_revision_ = 0; // RX callback is the sole writer.
    std::atomic<uint32_t> feedback_token_{0};
    const limits limits_;
    uint32_t disable_id_;
};

class dm4310 : public dmmotor
{
public:
    explicit dm4310(config cfg);

    types::status set_output() override;
};

class dm8009p : public dmmotor
{
public:
    explicit dm8009p(config cfg);

    types::status set_output() override;
};

} // namespace motors
