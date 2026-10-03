#include "quaternion_ekf.hpp"
#include "tactical_ekf.hpp"
#include "matrix.hpp"
#include <cmath>
#include <stdexcept>
#include <limits>
#include <cstdio>

void require(bool condition, const char* message)
{
    if (!condition) throw std::runtime_error(message);
}

int main() try
{
    require(math::inv_sqrt(0) == 0 && math::inv_sqrt(-1) == 0,
            "nonpositive inverse square root contract");
    require(math::inv_sqrt(std::numeric_limits<float>::infinity()) == 0,
            "infinite inverse square root");
    require(std::isnan(math::inv_sqrt(std::numeric_limits<float>::quiet_NaN())),
            "inverse square root preserves NaN");
    for (int exponent = -30; exponent <= 30; ++exponent)
    {
        const float value = std::ldexp(1.25f, exponent);
        const double reference = 1.0 / std::sqrt(static_cast<double>(value));
        require(std::fabs(math::inv_sqrt(value) / reference - 1.0) < 2e-7,
                "inverse square root precision across scales");
    }
    const float a[2][3] = {{1, 2, 3}, {4, 5, 6}};
    float at[3][2]{}, product[2][2]{};
    math::transpose(a, at);
    math::multiply(a, at, product);
    require(product[0][0] == 14 && product[0][1] == 32 &&
            product[1][0] == 32 && product[1][1] == 77, "CMSIS rectangular matrix operations");
    const float diagonal[3][3] = {{2, 0, 0}, {0, 4, 0}, {0, 0, 8}};
    float inverse[3][3]{};
    require(math::inverse(diagonal, inverse, 1e-9f) && inverse[0][0] == 0.5f &&
            inverse[1][1] == 0.25f && inverse[2][2] == 0.125f, "fixed matrix inverse");
    const float singular[3][3]{};
    require(!math::inverse(singular, inverse, 1e-9f) && inverse[0][0] == 0.5f,
            "singular inverse preserves output");
    require(!math::inverse(singular, inverse, 0) && inverse[0][0] == 0.5f,
            "zero determinant is rejected even without a conditioning floor");
    const float nonfinite[3][3] = {{std::numeric_limits<float>::quiet_NaN(), 0, 0},
                                  {0, 1, 0}, {0, 0, 1}};
    require(!math::inverse(nonfinite, inverse, 1e-9f) && inverse[0][0] == 0.5f,
            "nonfinite determinant preserves output");
    const float overflow[3][3] = {{std::numeric_limits<float>::max(), 0, 0},
                                 {0, 2, 0}, {0, 0, 2}};
    require(!math::inverse(overflow, inverse, 1e-9f) && inverse[0][0] == 0.5f,
            "overflowed determinant preserves output");
    const float asymmetric[3][3] = {{1, 2, 3}, {0, 1, 4}, {5, 6, 0}};
    require(math::inverse(asymmetric, inverse, 1e-9f), "invertible asymmetric matrix");
    float identity[3][3]{};
    math::multiply(asymmetric, inverse, identity);
    for (unsigned row = 0; row < 3; ++row)
        for (unsigned col = 0; col < 3; ++col)
            require(std::fabs(identity[row][col] - (row == col ? 1.0f : 0.0f)) < 1e-6f,
                    "matrix multiplied by its inverse is identity");
    require(math::magnitude(math::normalise(math::vector3{})) == 0, "zero vector normalisation");
    ahrs::quaternion_ekf original;
    ahrs::tactical::ekf tactical;
    tactical.reset();
    for (unsigned i = 0; i < 2000; ++i)
    {
        original.update_kalman(0, 0, 0, 0, 0, 9.80665f, 0.001f);
        tactical.update(0, 0, 0, 0, 0, 9.80665f, 0.001f);
    }
    require(std::isfinite(original.gyro_norm) && original.gyro_norm == 0,
            "zero angular rate must have zero norm");
    require(original.StableFlag, "stationary original EKF must be stable");
    require(std::fabs(original.roll) < 0.001f && std::fabs(original.pitch) < 0.001f,
            "stationary original EKF tilt");
    require(std::fabs(tactical.state().roll) < 0.001f && std::fabs(tactical.state().pitch) < 0.001f,
            "stationary tactical EKF tilt");
    original.reset();
    tactical.reset();
    for (unsigned i = 0; i < 2000; ++i)
    {
        original.update_kalman(0, 0, math::pi / 4, 0, 0, 9.80665f, 0.001f);
        tactical.update(0, 0, math::pi / 4, 0, 0, 9.80665f, 0.001f);
    }
    require(std::fabs(original.yaw - math::pi / 2) < 0.005f, "original EKF constant yaw rate");
    require(std::fabs(tactical.state().yaw - math::pi / 2) < 0.005f, "tactical EKF constant yaw rate");
    float norm = 0;
    for (float q : tactical.state().quaternion) norm += q * q;
    require(std::fabs(norm - 1) < 0.00001f, "tactical quaternion must be unit length");
    original.update_kalman(0, 0, 0, 0, 0, 0, 0.001f);
    require(std::isfinite(original.yaw), "zero acceleration cannot corrupt attitude");
}
catch (const std::exception& error)
{
    std::fprintf(stderr, "%s\n", error.what());
    return 1;
}
