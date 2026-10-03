#pragma once

namespace math
{

float clamp(float input, float min_value, float max_value);
float clamp_loop(float input, float min_value, float max_value);
float limit_abs(float input, float max_value);

} // namespace math
