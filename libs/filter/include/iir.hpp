#pragma once

#include "biquad.hpp"

namespace filter
{

class iir
{
public:
    explicit iir(float omega_n, float zeta, float dt);

    float update(float input);
    void reset();

private:
    float b0_ = 0.0f;
    float b1_ = 0.0f;
    float a1_ = 0.0f;
    float a2_ = 0.0f;
    biquad_state state_{};
};

} // namespace filter
