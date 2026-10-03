#include "constrain.hpp"
#include <cmath>

namespace math
{

float clamp(float input, float min_value, float max_value)
{
    if (max_value < min_value)
    {
        return input;
    }
    if (input > max_value)
    {
        return max_value;
    }
    if (input < min_value)
    {
        return min_value;
    }
    return input;
}

float clamp_loop(float input, float min_value, float max_value)
{
    if (max_value < min_value)
    {
        return input;
    }

    const float range = max_value - min_value;
    if (range == 0.0f)
    {
        return min_value;
    }

    float normalized = input - min_value;
    normalized = std::fmod(normalized, range);
    if (normalized < 0.0f)
    {
        normalized += range;
    }
    return min_value + normalized;
}

float limit_abs(float input, float max_value)
{
    if (input > max_value)
    {
        return max_value;
    }
    if (input < -max_value)
    {
        return -max_value;
    }
    return input;
}

} // namespace math
