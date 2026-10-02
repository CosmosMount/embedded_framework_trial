#include "kalman_1d.hpp"

namespace filter
{

void kalman_1d::reset()
{
    last_p_ = 0.02f;
    value_ = 0.0f;
    q_ = 0.001f;
    r_ = 0.543f;
}

void kalman_1d::set_process_noise(float q)
{
    q_ = q;
}

void kalman_1d::set_measurement_noise(float r)
{
    r_ = r;
}

float kalman_1d::update(float input)
{
    const float predicted_p = last_p_ + q_;
    const float gain = predicted_p / (predicted_p + r_);
    value_ += gain * (input - value_);
    last_p_ = (1.0f - gain) * predicted_p;
    return value_;
}

} // namespace filter
