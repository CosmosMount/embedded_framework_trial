#include "pid.hpp"

#include "constrain.hpp"

#include <cmath>

namespace control
{

void pid::apply_integral_separation()
{
    if (err[0] * i_out > 0.0f)
    {
        if (std::fabs(err[0]) <= max_out)
        {
            return;
        }
        i_term = 0.0f;
    }
}

void pid::apply_changing_integral_rate()
{
    if (err[0] * i_out > 0.0f)
    {
        if (std::fabs(err[0]) <= scalar_b)
        {
            return;
        }
        if (std::fabs(err[0]) <= (scalar_a + scalar_b))
        {
            i_term *= (scalar_a - std::fabs(err[0]) + scalar_b) / scalar_a;
            return;
        }
        i_term = 0.0f;
    }
}

void pid::apply_integral_limit()
{
    const float temp_i_out = i_out + i_term;
    const float temp_output = p_out + i_out + d_out;
    if (std::fabs(temp_output) > max_out)
    {
        if (err[0] * i_out > 0.0f)
        {
            i_term = 0.0f;
        }
    }

    if (temp_i_out > max_iout)
    {
        i_term = 0.0f;
        i_out = max_iout;
    }
    if (temp_i_out < -max_iout)
    {
        i_term = 0.0f;
        i_out = -max_iout;
    }
}

pid::pid(float kp, float ki, float kd, float max_out, float max_iout, pid_mode mode)
    : mode(mode), kp(kp), ki(ki), kd(kd), max_out(max_out), max_iout(max_iout)
{
}

void pid::tune(float new_kp, float new_ki, float new_kd)
{
    kp = new_kp;
    ki = new_ki;
    kd = new_kd;
}

void pid::update(float velocity)
{
    err[1] = err[0];
    err[0] = ref - fdb;

    p_out = kp * err[0];
    i_term = ki * err[0];

    const auto mode_bits = static_cast<uint8_t>(mode);
    if ((mode_bits & static_cast<uint8_t>(pid_mode::position)) != 0U)
    {
        d_out = kd * (err[0] - err[1]);
    }
    else if ((mode_bits & static_cast<uint8_t>(pid_mode::delta)) != 0U)
    {
        d_out = kd * velocity;
    }

    if ((mode_bits & static_cast<uint8_t>(pid_mode::trapezoid_integral)) != 0U)
    {
        i_term = ki * ((err[0] + err[1]) / 2.0f);
    }
    if ((mode_bits & static_cast<uint8_t>(pid_mode::derivative_on_measurement)) != 0U)
    {
        d_out = kd * (last_fdb - fdb);
    }
    if ((mode_bits & static_cast<uint8_t>(pid_mode::integral_separation)) != 0U)
    {
        apply_integral_separation();
    }
    if ((mode_bits & static_cast<uint8_t>(pid_mode::changing_integral_rate)) != 0U)
    {
        apply_changing_integral_rate();
    }
    if ((mode_bits & static_cast<uint8_t>(pid_mode::integral_limit)) != 0U)
    {
        apply_integral_limit();
    }

    i_out += i_term;
    i_out = math::limit_abs(i_out, max_iout);

    last_fdb = fdb;
    result = p_out + i_out + d_out;
    result = math::limit_abs(result, max_out);
}

void pid::clear()
{
    last_fdb = fdb = 0.0f;
    err[0] = err[1] = 0.0f;
    p_out = i_out = d_out = result = 0.0f;
    ref = 0.0f;
    i_term = 0.0f;
    scalar_a = 1.0f;
    scalar_b = 1.0f;
}

void pid::reset_state(float reference, float feedback)
{
    ref = reference;
    fdb = feedback;
    last_fdb = feedback;
    err[0] = err[1] = 0.0f;
    p_out = i_out = d_out = result = 0.0f;
    i_term = 0.0f;
}

} // namespace control
