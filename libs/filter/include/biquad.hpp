#pragma once

#include "constants.hpp"
#include <math.h>

namespace filter
{
// Direct-form I state; a1/a2 are signed feedback coefficients.
struct biquad_state
{
    float x1 = 0, x2 = 0, y1 = 0, y2 = 0;

    float update(float input, float b0, float b1, float b2, float a1, float a2)
    {
        const float output = b0 * input + b1 * x1 + b2 * x2 + a1 * y1 + a2 * y2;
        x2 = x1;
        x1 = input;
        y2 = y1;
        y1 = output;
        return output;
    }
};

struct biquad
{
    float b0 = 1, b1 = 0, b2 = 0, a1 = 0, a2 = 0;
    biquad_state state{};

    float update(float input) { return state.update(input, b0, b1, b2, a1, a2); }

    // Second-order Butterworth high pass. Nonpositive frequencies select bypass.
    static biquad high_pass(float cutoff_hz, float sample_rate_hz)
    {
        biquad result{};
        if (cutoff_hz <= 0 || sample_rate_hz <= 0) return result;
        const float k = tanf(math::pi * cutoff_hz / sample_rate_hz);
        constexpr float sqrt2 = 1.41421356237f;
        const float norm = 1.0f / (1.0f + sqrt2 * k + k * k);
        result.b0 = result.b2 = norm;
        result.b1 = -2.0f * norm;
        result.a1 = -2.0f * (k * k - 1.0f) * norm;
        result.a2 = -(1.0f - sqrt2 * k + k * k) * norm;
        return result;
    }
};
} // namespace filter
