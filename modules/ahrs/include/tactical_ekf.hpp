#pragma once

#include "tactical_fusion.hpp"

#include <stdint.h>

namespace ahrs::tactical
{

struct diagnostics
{
    float quaternion[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    float yaw = 0.0f;
    float pitch = 0.0f;
    float roll = 0.0f;
    float gyro_bias[3] = {};
    float accel_weight = 1.0f;
    float accel_direction_error = 0.0f;
    float accel_magnitude_g = 0.0f;
    float gyro_magnitude_rad_s = 0.0f;
    float accel_magnitude_variance = 0.0f;
    float gyro_magnitude_variance = 0.0f;
    float impact_acc_delta_g = 0.0f;
    float impact_gyro_delta_rad_s = 0.0f;
    uint8_t motion_state = 0;
    uint8_t impact_state = 0;
    bool paddling = false;
    bool linear_motion = false;
    float earth_acceleration[3] = {};
    float heave_velocity = 0.0f;
    float heave_position = 0.0f;
    uint32_t update_count = 0;
};

class ekf
{
public:
    void reset()
    {
        Tactical_Init(&system_, 2000.0f, 0.0f);
        // These are the values used by the old AHRS integration, not merely
        // the tactical fusion library defaults.
        system_.params.processNoiseAngle = 8.5e-8f;
        system_.params.processNoiseBias = 8.5e-10f;
        system_.params.measureNoiseAcc = 1.5e-5f;
        diagnostics_ = {};
    }

    void update(float gx_rad_s, float gy_rad_s, float gz_rad_s,
                float ax_m_s2, float ay_m_s2, float az_m_s2, float dt)
    {
        math::vector3 gyro{};
        gyro.x = math::radians_to_degrees(gx_rad_s);
        gyro.y = math::radians_to_degrees(gy_rad_s);
        gyro.z = math::radians_to_degrees(gz_rad_s);
        math::vector3 accel{};
        accel.x = ax_m_s2 / 9.80665f;
        accel.y = ay_m_s2 / 9.80665f;
        accel.z = az_m_s2 / 9.80665f;
        accel_ = accel;
        system_.samplePeriod = dt;
        Tactical_Update(&system_, gyro, accel);
        sync(gyro);
    }

    const diagnostics& state() const { return diagnostics_; }

private:
    void sync(const math::vector3& gyro)
    {
        diagnostics_.quaternion[0] = system_.quaternion.w;
        diagnostics_.quaternion[1] = system_.quaternion.x;
        diagnostics_.quaternion[2] = system_.quaternion.y;
        diagnostics_.quaternion[3] = system_.quaternion.z;
        const math::euler_angles euler = math::to_euler(system_.quaternion);
        diagnostics_.roll = euler.roll;
        diagnostics_.pitch = euler.pitch;
        diagnostics_.yaw = euler.yaw;
        diagnostics_.gyro_bias[0] = math::degrees_to_radians(system_.gyroBias.x);
        diagnostics_.gyro_bias[1] = math::degrees_to_radians(system_.gyroBias.y);
        diagnostics_.gyro_bias[2] = math::degrees_to_radians(system_.gyroBias.z);
        diagnostics_.accel_weight = system_.accWeight;
        diagnostics_.accel_direction_error = system_.accDirectionError;
        diagnostics_.accel_magnitude_g = system_.accMagnitude;
        diagnostics_.gyro_magnitude_rad_s = math::degrees_to_radians(
            math::magnitude(Tactical_GetCalibratedGyro(&system_, gyro)));
        diagnostics_.accel_magnitude_variance = system_.noise.accMagVar;
        diagnostics_.gyro_magnitude_variance = system_.noise.gyroMagVar * 0.0003046174198f;
        diagnostics_.impact_acc_delta_g = system_.impactAccDelta;
        diagnostics_.impact_gyro_delta_rad_s = math::degrees_to_radians(system_.impactGyroDelta);
        diagnostics_.motion_state = static_cast<uint8_t>(system_.motionState);
        diagnostics_.impact_state = static_cast<uint8_t>(system_.impactState);
        diagnostics_.paddling = system_.isPaddling;
        diagnostics_.linear_motion = system_.isLinearMotion;
        const math::vector3 earth = Tactical_GetEarthAcceleration(&system_, accel_);
        diagnostics_.earth_acceleration[0] = earth.x;
        diagnostics_.earth_acceleration[1] = earth.y;
        diagnostics_.earth_acceleration[2] = earth.z;
        diagnostics_.heave_velocity = system_.heaveVelocity;
        diagnostics_.heave_position = system_.heavePosition;
        ++diagnostics_.update_count;
    }

    TacticalSystem system_{};
    math::vector3 accel_{};
    diagnostics diagnostics_{};
};

} // namespace ahrs::tactical
