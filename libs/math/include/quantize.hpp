#pragma once

#include <stdint.h>

namespace math
{

// Protocol callers supply 1..31 bits, finite min < max and an in-range value.
// Validation/clamping belongs at the command or configuration boundary.
inline uint32_t quantize(float value, float min, float max, uint8_t bits)
{
    const uint32_t full_scale = (UINT32_C(1) << bits) - 1U;
    return static_cast<uint32_t>((value - min) * static_cast<float>(full_scale) / (max - min));
}

inline float dequantize(uint32_t value, float min, float max, uint8_t bits)
{
    const uint32_t full_scale = (UINT32_C(1) << bits) - 1U;
    return static_cast<float>(value) * (max - min) / static_cast<float>(full_scale) + min;
}

} // namespace math
