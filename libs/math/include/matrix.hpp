#pragma once

#include "dsp/matrix_functions.h"
#include <math.h>
#include <stddef.h>
#include <stdint.h>

namespace math
{
// Fixed row-major arrays. Output must not alias either input.
// Shapes are checked at compile time; CMSIS descriptors borrow the storage.
template <size_t Rows, size_t Inner, size_t Cols>
inline void multiply(const float (&a)[Rows][Inner], const float (&b)[Inner][Cols],
                     float (&out)[Rows][Cols])
{
    static_assert(Rows <= UINT16_MAX && Inner <= UINT16_MAX && Cols <= UINT16_MAX);
    const arm_matrix_instance_f32 left{Rows, Inner, const_cast<float*>(&a[0][0])};
    const arm_matrix_instance_f32 right{Inner, Cols, const_cast<float*>(&b[0][0])};
    arm_matrix_instance_f32 result{Rows, Cols, &out[0][0]};
    (void)arm_mat_mult_f32(&left, &right, &result);
}
template <size_t Rows, size_t Cols>
inline void transpose(const float (&input)[Rows][Cols], float (&out)[Cols][Rows])
{
    static_assert(Rows <= UINT16_MAX && Cols <= UINT16_MAX);
    const arm_matrix_instance_f32 source{Rows, Cols, const_cast<float*>(&input[0][0])};
    arm_matrix_instance_f32 result{Cols, Rows, &out[0][0]};
    (void)arm_mat_trans_f32(&source, &result);
}
// Cofactor inverse for fixed 3x3 matrices; leaves output untouched on rejection.
// The solver supplies its conditioning threshold. Output must not alias input.
inline bool inverse(const float (&m)[3][3], float (&inv)[3][3], float determinant_floor) {
    const float det =
        m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
        m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
        m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);

    if (!isfinite(det) || det == 0.0f || fabsf(det) < determinant_floor) {
        return false;
    }

    const float invDet = 1.0f / det;

    inv[0][0] = (m[1][1] * m[2][2] - m[1][2] * m[2][1]) * invDet;
    inv[0][1] = (m[0][2] * m[2][1] - m[0][1] * m[2][2]) * invDet;
    inv[0][2] = (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * invDet;

    inv[1][0] = (m[1][2] * m[2][0] - m[1][0] * m[2][2]) * invDet;
    inv[1][1] = (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * invDet;
    inv[1][2] = (m[0][2] * m[1][0] - m[0][0] * m[1][2]) * invDet;

    inv[2][0] = (m[1][0] * m[2][1] - m[1][1] * m[2][0]) * invDet;
    inv[2][1] = (m[0][1] * m[2][0] - m[0][0] * m[2][1]) * invDet;
    inv[2][2] = (m[0][0] * m[1][1] - m[0][1] * m[1][0]) * invDet;

    return true;
}

} // namespace math
