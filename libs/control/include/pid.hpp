#pragma once

#include <cstdint>

namespace control
{

enum class pid_mode : uint8_t
{
    position = 0x01,
    delta = 0x02,
    trapezoid_integral = 0x04,
    changing_integral_rate = 0x08,
    integral_separation = 0x10,
    derivative_on_measurement = 0x20,
    integral_limit = 0x40,
    derivative_incomplete = 0x80,
};

// Select one base mode (position/delta), then combine the desired options.
constexpr pid_mode operator|(pid_mode lhs, pid_mode rhs) noexcept
{
    return static_cast<pid_mode>(static_cast<uint8_t>(lhs) | static_cast<uint8_t>(rhs));
}

class pid
{
public:
    pid(float kp, float ki, float kd, float max_out, float max_iout,
        pid_mode mode = pid_mode::position);

    void tune(float kp, float ki, float kd);
    void update(float velocity = 0.0f);
    void clear();
    void reset_state(float reference, float feedback);

    pid_mode mode = pid_mode::position;

    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
    float ref = 0.0f;
    float fdb = 0.0f;
    float result = 0.0f;

    float max_out = 0.0f;
    float max_iout = 0.0f;

    float scalar_a = 0.0f;
    float scalar_b = 0.0f;

private:
    void apply_integral_separation();
    void apply_changing_integral_rate();
    void apply_integral_limit();

    float last_fdb = 0.0f;
    float err[2]{};
    float p_out = 0.0f;
    float i_out = 0.0f;
    float d_out = 0.0f;
    float i_term = 0.0f;
};

} // namespace control
