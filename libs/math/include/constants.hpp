#pragma once

#include <math.h>
#include <stdint.h>

namespace math
{

inline constexpr float pi = 3.14159265358979f;
inline constexpr float two_pi = 6.283185307f;

inline constexpr float degrees_to_radians(float degrees) { return degrees * (pi / 180.0f); }
inline constexpr float radians_to_degrees(float radians) { return radians * (180.0f / pi); }

inline float inv_sqrt(float value)
{
    if (value <= 0.0f)
    {
        return 0.0f;
    }
    return 1.0f / sqrtf(value);
}

} // namespace math
