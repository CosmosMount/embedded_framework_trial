#pragma once

#include <cmath>
#include <stdint.h>

namespace math
{

inline constexpr float pi = 3.14159265358979f;
inline constexpr float two_pi = 6.283185307f;

inline float inv_sqrt(float value)
{
    if (value <= 0.0f)
    {
        return 0.0f;
    }
    return 1.0f / std::sqrt(value);
}

} // namespace math
