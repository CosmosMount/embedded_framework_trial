#pragma once

#include "constants.hpp"
#include <math.h>

namespace math
{
// Fixed-size value types; quaternion order is W, X, Y, Z.
struct vector3 { float x = 0, y = 0, z = 0; };
struct quaternion { float w = 1, x = 0, y = 0, z = 0; };
struct euler_angles { float roll, pitch, yaw; }; // radians, right-handed XYZ

inline vector3 add(vector3 a, vector3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline vector3 subtract(vector3 a, vector3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline vector3 scale(vector3 v, float s) { return {v.x * s, v.y * s, v.z * s}; }
inline float dot(vector3 a, vector3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline float magnitude_squared(vector3 v) { return dot(v, v); }
inline float magnitude(vector3 v) { return sqrtf(magnitude_squared(v)); }
inline vector3 normalise(vector3 v) { return scale(v, inv_sqrt(magnitude_squared(v))); }
inline vector3 cross(vector3 a, vector3 b)
{
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
inline quaternion add(quaternion a, quaternion b)
{
    return {a.w + b.w, a.x + b.x, a.y + b.y, a.z + b.z};
}
// Hamilton product with the pure quaternion (0, v). This is not vector rotation.
inline quaternion multiply(quaternion q, vector3 v)
{
    return {-q.x * v.x - q.y * v.y - q.z * v.z,
            q.w * v.x + q.y * v.z - q.z * v.y,
            q.w * v.y - q.x * v.z + q.z * v.x,
            q.w * v.z + q.x * v.y - q.y * v.x};
}
// A zero quaternion remains zero; callers own invalid-attitude recovery.
inline quaternion normalise(quaternion q)
{
    const float reciprocal = inv_sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    return {q.w * reciprocal, q.x * reciprocal, q.y * reciprocal, q.z * reciprocal};
}
inline euler_angles to_euler(quaternion q)
{
    const float sin_pitch = 2 * (q.w * q.y - q.z * q.x);
    const float pitch = sin_pitch <= -1 ? -pi / 2 : sin_pitch >= 1 ? pi / 2 : asinf(sin_pitch);
    return {atan2f(2 * (q.w * q.x + q.y * q.z), q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z),
            pitch,
            atan2f(2 * (q.w * q.z + q.x * q.y), q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z)};
}
} // namespace math
