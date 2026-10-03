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
    return state_.update(input, b0_, b1_, b0_, a1_, a2_);
}

void iir::reset()
{
    state_ = {};
}

} // namespace filter
