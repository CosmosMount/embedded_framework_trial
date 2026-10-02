#include "iir.hpp"

namespace filter
{

iir::iir(float omega_n, float zeta, float dt)
{
    const float k = 2.0f / dt;
    const float wn2 = omega_n * omega_n;
    const float den = k * k + 2.0f * zeta * omega_n * k + wn2;

    b0_ = wn2 / den;
    b1_ = 2.0f * b0_;
    a1_ = -(2.0f * wn2 - 2.0f * k * k) / den;
    a2_ = -(k * k - 2.0f * zeta * omega_n * k + wn2) / den;
}

float iir::update(float input)
{
    const float output = b0_ * input + b1_ * x1_ + b0_ * x2_ + a1_ * y1_ + a2_ * y2_;
    x2_ = x1_;
    x1_ = input;
    y2_ = y1_;
    y1_ = output;
    return output;
}

void iir::reset()
{
    x1_ = x2_ = 0.0f;
    y1_ = y2_ = 0.0f;
}

} // namespace filter