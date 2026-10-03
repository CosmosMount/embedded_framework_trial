/**
 * @file tactical_fusion.cpp
 * @brief Tactical IMU fusion implementation.
 */

#include "tactical_fusion.hpp"
#include "matrix.hpp"
#include "constrain.hpp"
#include <math.h>
#include <string.h>

#define G_CONST 9.80665f
static math::vector3 GravityBody(const math::quaternion q, const TacticalConvention convention) {
    const float qw = q.w;
    const float qx = q.x;
    const float qy = q.y;
    const float qz = q.z;

    math::vector3 halfGravity;
    if (convention == TacticalConvention::ned) {
        halfGravity.x = qw * qy - qx * qz;
        halfGravity.y = -(qy * qz + qw * qx);
        halfGravity.z = 0.5f - qw * qw - qz * qz;
    } else {
        halfGravity.x = qx * qz - qw * qy;
        halfGravity.y = qy * qz + qw * qx;
        halfGravity.z = qw * qw - 0.5f + qz * qz;
    }

    return math::scale(halfGravity, 2.0f);
}

static void UpdateMeanVar(const float value, const float alpha, float *const mean, float *const var) {
    const float delta = value - *mean;
    *mean += alpha * delta;
    *var += alpha * (delta * delta - *var);
}

static void UpdateNoiseStats(TacticalSystem *const sys, const math::vector3 acc, const math::vector3 gyro, const float dt) {
    const float tau = sys->params.noiseTau;
    const float alpha = (tau <= 0.0f) ? 1.0f : (dt / (tau + dt));

    UpdateMeanVar(acc.x, alpha, &sys->noise.accMean.x, &sys->noise.accVar.x);
    UpdateMeanVar(acc.y, alpha, &sys->noise.accMean.y, &sys->noise.accVar.y);
    UpdateMeanVar(acc.z, alpha, &sys->noise.accMean.z, &sys->noise.accVar.z);

    UpdateMeanVar(gyro.x, alpha, &sys->noise.gyroMean.x, &sys->noise.gyroVar.x);
    UpdateMeanVar(gyro.y, alpha, &sys->noise.gyroMean.y, &sys->noise.gyroVar.y);
    UpdateMeanVar(gyro.z, alpha, &sys->noise.gyroMean.z, &sys->noise.gyroVar.z);

    const float accMag = math::magnitude(acc);
    const float gyroMag = math::magnitude(gyro);

    UpdateMeanVar(accMag, alpha, &sys->noise.accMagMean, &sys->noise.accMagVar);
    UpdateMeanVar(gyroMag, alpha, &sys->noise.gyroMagMean, &sys->noise.gyroMagVar);

    const float accVarMag = sys->noise.accVar.x + sys->noise.accVar.y + sys->noise.accVar.z;
    const float gyroVarMag = sys->noise.gyroVar.x + sys->noise.gyroVar.y + sys->noise.gyroVar.z;

    sys->noise.accNoise = sqrtf(fmaxf(accVarMag, 0.0f));
    sys->noise.gyroNoise = sqrtf(fmaxf(gyroVarMag, 0.0f));
}

static TacticalMotionState AnalyzeMotion(TacticalSystem *const sys, const math::vector3 acc, const math::vector3 gyro) {
    const float accMag = math::magnitude(acc);
    const float gyroMag = math::magnitude(gyro);
    const float accErr = fabsf(accMag - 1.0f);

    if ((accErr < sys->params.accStaticThreshold) &&
        (gyroMag < sys->params.gyroStaticThreshold) &&
        (sys->noise.accMagVar < sys->params.accVarStatic) &&
        (sys->noise.gyroMagVar < sys->params.gyroVarStatic)) {
        return TACTICAL_MOTION_STATIC;
    }

    if ((accErr < sys->params.accStableThreshold) &&
        (gyroMag < sys->params.gyroStableThreshold)) {
        return TACTICAL_MOTION_STABLE;
    }

    return TACTICAL_MOTION_DYNAMIC;
}

static void ComputeAdaptiveNoise(const TacticalSystem *const sys, const float accError, float *const qAngle,
                                 float *const qBias, float *const rAcc) {
    float accNoise = sys->noise.accNoise;
    float gyroNoise = sys->noise.gyroNoise;

    if (accNoise <= 0.0f) {
        accNoise = 0.01f;
    }
    if (gyroNoise <= 0.0f) {
        gyroNoise = 0.05f;
    }

    const float gyroNoiseRad = math::degrees_to_radians(gyroNoise);
    const float gyroNoiseVar = gyroNoiseRad * gyroNoiseRad;
    const float accNoiseVar = accNoise * accNoise;

    float qAngleLocal = sys->params.processNoiseAngle + sys->params.gyroNoiseScale * gyroNoiseVar;
    float qBiasLocal = sys->params.processNoiseBias + sys->params.biasNoiseScale * gyroNoiseVar;
    float rAccLocal = sys->params.measureNoiseAcc + sys->params.accNoiseScale * accNoiseVar +
                      sys->params.accErrorScale * accError * accError;

    if (sys->motionState == TACTICAL_MOTION_STABLE) {
        rAccLocal *= 3.0f;
    } else if (sys->motionState == TACTICAL_MOTION_DYNAMIC) {
        rAccLocal *= 12.0f;
    }

    qAngleLocal = math::clamp(qAngleLocal, 1e-9f, 1.0f);
    qBiasLocal = math::clamp(qBiasLocal, 1e-12f, 1.0f);
    rAccLocal = math::clamp(rAccLocal, 1e-6f, 1.0f);

    *qAngle = qAngleLocal;
    *qBias = qBiasLocal;
    *rAcc = rAccLocal;
}

static void EkfPredict(TacticalSystem *const sys, const math::vector3 gyro, const float dt) {
    const math::vector3 gyroCorrected = math::subtract(gyro, sys->gyroBias);
    const math::vector3 omega = math::scale(gyroCorrected, math::degrees_to_radians(1.0f));

    const math::vector3 delta = math::scale(omega, 0.5f * dt);
    sys->quaternion = math::add(sys->quaternion,
                                math::multiply(sys->quaternion, delta));
    sys->quaternion = math::normalise(sys->quaternion);

    float F[6][6] = {0};
    for (int i = 0; i < 6; ++i) {
        F[i][i] = 1.0f;
    }

    const float wx = omega.x;
    const float wy = omega.y;
    const float wz = omega.z;

    const float skew[3][3] = {
        {0.0f, -wz, wy},
        {wz, 0.0f, -wx},
        {-wy, wx, 0.0f}
    };

    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            F[r][c] -= skew[r][c] * dt;
        }
        F[r][r + 3] -= dt;
    }

    const float accError = fabsf(sys->accMagnitude - 1.0f);
    float qAngle = 0.0f;
    float qBias = 0.0f;
    float rAccUnused = 0.0f;
    ComputeAdaptiveNoise(sys, accError, &qAngle, &qBias, &rAccUnused);

    float FP[6][6]{};
    float Ft[6][6]{};
    math::multiply(F, sys->P, FP);
    math::transpose(F, Ft);
    math::multiply(FP, Ft, sys->P);

    for (int i = 0; i < 3; ++i) {
        sys->P[i][i] += qAngle * dt;
    }
    for (int i = 3; i < 6; ++i) {
        sys->P[i][i] += qBias * dt;
    }
}

static void EkfUpdateAccel(TacticalSystem *const sys, const math::vector3 acc) {
    const float accMagSq = math::magnitude_squared(acc);
    if (accMagSq < 1e-6f) {
        return;
    }

    const float accMag = sqrtf(accMagSq);
    math::vector3 accNorm = math::scale(acc, 1.0f / accMag);

    math::vector3 gravity = GravityBody(sys->quaternion, sys->convention);
    if (math::magnitude_squared(gravity) > 0.0f) {
        gravity = math::normalise(gravity);
    }

    const math::vector3 error = math::cross(accNorm, gravity);

    const float accError = fabsf(sys->accMagnitude - 1.0f);
    float qAngle = 0.0f;
    float qBias = 0.0f;
    float rAcc = 0.0f;
    ComputeAdaptiveNoise(sys, accError, &qAngle, &qBias, &rAcc);

    // Apply impact-based adaptive weighting to observation noise
    // Lower accWeight → higher rAcc → less trust in accelerometer during impact/recovery
    if (sys->accWeight > 0.0f) {
        rAcc /= sys->accWeight;
    }

    float P11[3][3];
    float P21[3][3];

    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            P11[r][c] = sys->P[r][c];
            P21[r][c] = sys->P[r + 3][c];
        }
    }

    float S[3][3];
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            S[r][c] = P11[r][c];
        }
        S[r][r] += rAcc;
    }

    float SInv[3][3];
    if (!math::inverse(S, SInv, 1e-9f)) {
        return;
    }

    float K1[3][3] = {0};
    float K2[3][3] = {0};

    math::multiply(P11, SInv, K1);
    math::multiply(P21, SInv, K2);

    math::vector3 deltaTheta;
    deltaTheta.x = K1[0][0] * error.x + K1[0][1] * error.y + K1[0][2] * error.z;
    deltaTheta.y = K1[1][0] * error.x + K1[1][1] * error.y + K1[1][2] * error.z;
    deltaTheta.z = K1[2][0] * error.x + K1[2][1] * error.y + K1[2][2] * error.z;

    math::vector3 deltaBias;
    deltaBias.x = K2[0][0] * error.x + K2[0][1] * error.y + K2[0][2] * error.z;
    deltaBias.y = K2[1][0] * error.x + K2[1][1] * error.y + K2[1][2] * error.z;
    deltaBias.z = K2[2][0] * error.x + K2[2][1] * error.y + K2[2][2] * error.z;

    // A six-axis accelerometer update has no yaw observability.  Z-axis
    // gyro offset is calibrated by BMI088 at startup and must not be driven
    // by this roll/pitch-only measurement.
    deltaBias.z = 0.0f;

    const math::vector3 correction = math::scale(deltaTheta, 0.5f);
    sys->quaternion = math::add(sys->quaternion,
                                math::multiply(sys->quaternion, correction));
    sys->quaternion = math::normalise(sys->quaternion);

    const math::vector3 deltaBiasDeg = math::scale(deltaBias, math::radians_to_degrees(1.0f));
    sys->gyroBias = math::add(sys->gyroBias, deltaBiasDeg);

    float K[6][3] = {0};
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            K[r][c] = K1[r][c];
            K[r + 3][c] = K2[r][c];
        }
    }

    float I_KH[6][6] = {0};
    for (int i = 0; i < 6; ++i) {
        I_KH[i][i] = 1.0f;
    }
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            I_KH[r][c] -= K1[r][c];
            I_KH[r + 3][c] -= K2[r][c];
        }
    }

    float temp[6][6]{};
    float I_KHt[6][6]{};
    float Pnew[6][6]{};
    math::multiply(I_KH, sys->P, temp);
    math::transpose(I_KH, I_KHt);
    math::multiply(temp, I_KHt, Pnew);

    float Kt[3][6]{};
    float KKt[6][6]{};
    math::transpose(K, Kt);
    math::multiply(K, Kt, KKt);

    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            sys->P[i][j] = Pnew[i][j] + rAcc * KKt[i][j];
        }
    }

    // Enforce P matrix symmetry to prevent numerical drift
    for (int i = 0; i < 6; ++i) {
        for (int j = i + 1; j < 6; ++j) {
            const float avg = 0.5f * (sys->P[i][j] + sys->P[j][i]);
            sys->P[i][j] = avg;
            sys->P[j][i] = avg;
        }
    }
}

void Tactical_Init(TacticalSystem *const sys, const float sampleRate, const float heaveCutoffFreq) {
    *sys = {};

    sys->samplePeriod = (sampleRate > 0.0f) ? (1.0f / sampleRate) : 0.0f;
    sys->convention = TacticalConvention::nwu;
    sys->quaternion = math::quaternion{};
    sys->motionState = TACTICAL_MOTION_STATIC;

    // [FIX] Initialize noise statistics with reasonable initial values to avoid
    // large variance spike on first update (fixes static motion misdetection)
    // Initialize means to expected values (1g for acc, 0 for gyro)
    sys->noise.accMean = math::vector3{0.0f, 0.0f, 1.0f};
    sys->noise.gyroMean = math::vector3{};
    sys->noise.accMagMean = 1.0f;
    sys->noise.gyroMagMean = 0.0f;
    // Initialize variances with expected sensor noise levels
    const float expectedAccNoise = 0.01f;  // Expected accelerometer noise (g)
    const float expectedGyroNoise = 0.5f;  // Expected gyroscope noise (deg/s)
    sys->noise.accVar = math::vector3{expectedAccNoise*expectedAccNoise, expectedAccNoise*expectedAccNoise, expectedAccNoise*expectedAccNoise};
    sys->noise.gyroVar = math::vector3{expectedGyroNoise*expectedGyroNoise, expectedGyroNoise*expectedGyroNoise, expectedGyroNoise*expectedGyroNoise};
    sys->noise.accMagVar = expectedAccNoise * expectedAccNoise;
    sys->noise.gyroMagVar = 0.01f;  // Smaller initial variance for gyro to pass static detection

    sys->params.processNoiseAngle = 1.0e-5f;
    sys->params.processNoiseBias = 1.0e-7f;
    sys->params.measureNoiseAcc = 1.0e-2f;
    sys->params.accNoiseScale = 2.0f;
    sys->params.gyroNoiseScale = 1.0f;
    sys->params.biasNoiseScale = 0.1f;
    sys->params.accErrorScale = 2.0f;
    sys->params.noiseTau = 1.0f;
    sys->params.accStaticThreshold = 0.02f;
    sys->params.gyroStaticThreshold = 0.5f;
    sys->params.accStableThreshold = 0.15f;
    sys->params.gyroStableThreshold = 50.0f;
    sys->params.accVarStatic = 0.0004f;
    sys->params.gyroVarStatic = 0.25f;
    sys->params.heaveCutoffFreq = heaveCutoffFreq;
    sys->params.verticalBiasAlpha = 0.01f;
    sys->params.verticalAccLp = 0.2f;
    sys->params.gyroBiasAlpha = 0.001f;

    // [NEW] Impact detection and recovery parameters
    sys->params.impactAccThreshold = 0.5f;       // Detect impact when acc changes > 0.5g
    sys->params.impactGyroThreshold = 100.0f;     // Detect impact when gyro changes > 100 deg/s
    sys->params.impactRecoveryGain = 5.0f;        // Higher gain during recovery for fast convergence
    sys->params.impactRecoveryDuration = 0.5f;     // Recovery duration 0.5 seconds
    sys->params.accWeightNormal = 1.0f;          // Normal accelerometer weight
    sys->params.accWeightImpact = 0.05f;          // Very low weight during impact to ignore false acc
    sys->params.prevAccMag = 1.0f;                // Initialize previous acc magnitude
    sys->params.prevGyroMag = 0.0f;               // Initialize previous gyro magnitude

    // [NEW] Linear motion detection parameters
    sys->params.linearAccThreshold = 0.15f;       // Detect linear motion when acc deviates > 0.15g from 1g
    sys->params.linearMotionDecay = 0.95f;        // Decay rate for linear acceleration estimation
    sys->params.accCompensationEnabled = 1;

    // Direction-based translational acceleration detection
    sys->params.accDirectionThreshold = 0.15f;   // ~8.6 degrees deviation from predicted gravity
    sys->params.accDirectionScale = 10.0f;        // R multiplier per radian of direction error

    // Paddling / periodic vibration detection
    sys->params.paddlingVarThreshold = 0.05f;     // Variance threshold (g^2) for periodic motion
    sys->params.paddlingMeanTolerance = 0.1f;     // Mean must be within 0.1g of 1g
    sys->params.paddlingRScale = 20.0f;           // Heavily distrust accel during paddling
    sys->params.paddlingWindowAlpha = 0.01f;      // ~2s window at 500Hz for variance estimation

    // Sustained linear acceleration
    sys->params.sustainedLinearAlpha = 0.05f;     // Slow decay for sustained detection

    // Initialize linear acceleration estimation
    sys->linearAccBody = math::vector3{};
    sys->linearAccMagnitude = 0.0f;
    sys->isLinearMotion = false;

    // Initialize direction-based and paddling state
    sys->accDirectionError = 0.0f;
    sys->accVarWindow = 0.0f;
    sys->accMeanWindow = 1.0f;
    sys->accVarWindowAlpha = sys->params.paddlingWindowAlpha;
    sys->isPaddling = false;

    // Initialize impact state
    sys->impactState = TACTICAL_IMPACT_NONE;
    sys->impactRecoveryTimer = 0.0f;
    sys->accWeight = sys->params.accWeightNormal;
    sys->impactAccDelta = 0.0f;
    sys->impactGyroDelta = 0.0f;
    sys->impactTriggerAccDelta = 0.0f;
    sys->impactTriggerGyroDelta = 0.0f;
    sys->impactTriggerSource = 0U;

    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            sys->P[i][j] = 0.0f;
        }
    }
    sys->P[0][0] = 1.0e-3f;
    sys->P[1][1] = 1.0e-3f;
    sys->P[2][2] = 1.0e-3f;
    sys->P[3][3] = 1.0e-2f;
    sys->P[4][4] = 1.0e-2f;
    sys->P[5][5] = 1.0e-2f;

    sys->hpFilterVel = filter::biquad::high_pass(heaveCutoffFreq, sampleRate);
    sys->hpFilterPos = filter::biquad::high_pass(heaveCutoffFreq, sampleRate);
}

math::vector3 Tactical_GetCalibratedGyro(const TacticalSystem *const sys, const math::vector3 rawGyro) {
    return math::subtract(rawGyro, sys->gyroBias);
}

void Tactical_Update(TacticalSystem *const sys, const math::vector3 gyro, const math::vector3 acc) {
    const float dt = sys->samplePeriod;
    if (dt <= 0.0f) {
        return;
    }

    const math::vector3 calibGyro = Tactical_GetCalibratedGyro(sys, gyro);

    sys->accMagnitude = math::magnitude(acc);
    UpdateNoiseStats(sys, acc, calibGyro, dt);
    sys->motionState = AnalyzeMotion(sys, acc, calibGyro);

    // ===== IMPACT DETECTION AND RECOVERY =====
    const float accMag = sys->accMagnitude;
    const float gyroMag = math::magnitude(calibGyro);
    
    // Calculate rate of change
    const float accDelta = fabsf(accMag - sys->params.prevAccMag);
    const float gyroDelta = fabsf(gyroMag - sys->params.prevGyroMag);
    sys->impactAccDelta = accDelta;
    sys->impactGyroDelta = gyroDelta;
    
    // Store current values for next iteration
    sys->params.prevAccMag = accMag;
    sys->params.prevGyroMag = gyroMag;
    
    // Impact detection logic
    if (sys->impactState == TACTICAL_IMPACT_NONE) {
        // Check for sudden acceleration or rotation change
        const bool accImpact = accDelta > sys->params.impactAccThreshold;
        const bool gyroImpact = gyroDelta > sys->params.impactGyroThreshold;
        if (accImpact || gyroImpact) {
            sys->impactTriggerAccDelta = accDelta;
            sys->impactTriggerGyroDelta = gyroDelta;
            sys->impactTriggerSource = (accImpact ? 1U : 0U) | (gyroImpact ? 2U : 0U);
            sys->impactState = TACTICAL_IMPACT_DETECTED;
            sys->impactRecoveryTimer = 0.0f;
        }
    }
    
    // Handle impact recovery
    if (sys->impactState == TACTICAL_IMPACT_DETECTED) {
        sys->impactState = TACTICAL_IMPACT_RECOVERING;
        sys->impactRecoveryTimer = 0.0f;
        // Immediately drop accelerometer weight
        sys->accWeight = sys->params.accWeightImpact;
    } else if (sys->impactState == TACTICAL_IMPACT_RECOVERING) {
        sys->impactRecoveryTimer += dt;
        
        // Linear ramp up gain during recovery (faster convergence)
        const float recoveryProgress = sys->impactRecoveryTimer / sys->params.impactRecoveryDuration;

        // Ramp up accelerometer weight
        sys->accWeight = sys->params.accWeightImpact +
                        (sys->params.accWeightNormal - sys->params.accWeightImpact) * recoveryProgress;
        
        // Check if recovery is complete
        if (sys->impactRecoveryTimer >= sys->params.impactRecoveryDuration) {
            sys->impactState = TACTICAL_IMPACT_NONE;
            sys->accWeight = sys->params.accWeightNormal;
            sys->impactTriggerAccDelta = 0.0f;
            sys->impactTriggerGyroDelta = 0.0f;
            sys->impactTriggerSource = 0U;
        }
    } else {
        // Normal state - adapt weight based on motion
        if (sys->motionState == TACTICAL_MOTION_STATIC) {
            sys->accWeight = sys->params.accWeightNormal;
        } else if (sys->motionState == TACTICAL_MOTION_STABLE) {
            sys->accWeight = sys->params.accWeightNormal * 0.5f;
        } else {
            sys->accWeight = sys->params.accWeightNormal * 0.2f;
        }
    }
    // ===== END IMPACT DETECTION =====

    // ===== TRANSLATIONAL ACCELERATION COMPENSATION (3 mechanisms) =====
    if (sys->params.accCompensationEnabled) {

        // --- Mechanism 1: Direction-based detection ---
        // Use gyro-predicted gravity to detect when accel direction is wrong,
        // even if magnitude is ~1g (catches centripetal / constant-velocity turns).
        const math::vector3 gravPredicted = GravityBody(sys->quaternion, sys->convention);
        const float gravMagSq = math::magnitude_squared(gravPredicted);
        const float accMagSqDir = math::magnitude_squared(acc);

        if ((gravMagSq > 1e-6f) && (accMagSqDir > 1e-6f)) {
            const math::vector3 gravNorm = math::scale(gravPredicted,
                math::inv_sqrt(gravMagSq));
            const math::vector3 accNormDir = math::scale(acc,
                math::inv_sqrt(accMagSqDir));

            // cos(angle) = dot(accNorm, gravNorm)
            const float cosAngle = math::clamp(
                math::dot(accNormDir, gravNorm), -1.0f, 1.0f);

            // Use 1 - cos as a cheap proxy for angle^2/2 (avoids acosf)
            sys->accDirectionError = 1.0f - cosAngle;

            if (sys->accDirectionError > sys->params.accDirectionThreshold) {
                // Direction mismatch: translational acceleration present
                // Scale R proportional to direction error
                const float dirScale = 1.0f + sys->params.accDirectionScale *
                    sys->accDirectionError;
                sys->accWeight *= (1.0f / dirScale);
            }
        }

        // --- Mechanism 2: Paddling / periodic vibration detection ---
        // High short-window variance + mean near 1g = periodic vibration.
        // Gate accel updates by dramatically increasing R.
        {
            const float alpha = sys->params.paddlingWindowAlpha;
            const float magDelta = accMag - sys->accMeanWindow;

            // Exponential moving average of mean and variance
            sys->accMeanWindow += alpha * magDelta;
            sys->accVarWindow += alpha * (magDelta * magDelta - sys->accVarWindow);

            const float meanError = fabsf(sys->accMeanWindow - 1.0f);

            if ((sys->accVarWindow > sys->params.paddlingVarThreshold) &&
                (meanError < sys->params.paddlingMeanTolerance)) {
                // Periodic vibration detected: high variance but mean ≈ 1g
                sys->isPaddling = true;
                sys->accWeight *= (1.0f / sys->params.paddlingRScale);
            } else {
                sys->isPaddling = false;
            }
        }

        // --- Mechanism 3: Sustained linear acceleration (residual-based) ---
        // Use gyro-predicted gravity direction to estimate linear acceleration.
        // Don't rely on fixed decay — use the actual measured residual.
        {
            const math::vector3 gravBody = GravityBody(sys->quaternion, sys->convention);
            // linear_acc = measured_acc - predicted_gravity
            sys->linearAccBody = math::subtract(acc, gravBody);
            const float residualMag = math::magnitude(sys->linearAccBody);

            // EMA of residual for smooth sustained detection
            const float salpha = sys->params.sustainedLinearAlpha;
            sys->linearAccMagnitude = sys->linearAccMagnitude * (1.0f - salpha)
                                    + residualMag * salpha;

            if (sys->linearAccMagnitude > sys->params.linearAccThreshold) {
                sys->isLinearMotion = true;
                const float sustainedScale = sys->linearAccMagnitude /
                    sys->params.linearAccThreshold;
                sys->accWeight *= (1.0f / sustainedScale);
            } else {
                sys->isLinearMotion = false;
            }
        }
    }
    // ===== END TRANSLATIONAL ACCELERATION COMPENSATION =====

    EkfPredict(sys, gyro, dt);
    EkfUpdateAccel(sys, acc);

    // Note: gyro bias is estimated by the EKF state vector (updated in EkfUpdateAccel).
    // A separate LPF bias update here would conflict with the EKF estimate.
    // Only apply supplementary static bias correction when EKF P diagonal for bias
    // states is large (EKF has not yet converged on bias).
    if (sys->motionState == TACTICAL_MOTION_STATIC) {
        const float biasUncertainty = sys->P[3][3] + sys->P[4][4] + sys->P[5][5];
        if (biasUncertainty > 1.0e-3f) {
            const float alpha = math::clamp(sys->params.gyroBiasAlpha, 0.0f, 1.0f);
            sys->gyroBias.x += (gyro.x - sys->gyroBias.x) * alpha;
            sys->gyroBias.y += (gyro.y - sys->gyroBias.y) * alpha;
        }
    }

    math::vector3 earthAcc = Tactical_GetEarthAcceleration(sys, acc);
    float verticalAcc = earthAcc.z;

    if (sys->motionState != TACTICAL_MOTION_DYNAMIC) {
        const float beta = math::clamp(sys->params.verticalBiasAlpha, 0.0f, 1.0f);
        sys->verticalAccBias += (verticalAcc - sys->verticalAccBias) * beta;
    }
    verticalAcc -= sys->verticalAccBias;

    const float accAlpha = math::clamp(sys->params.verticalAccLp, 0.0f, 1.0f);
    sys->verticalAccFiltered += (verticalAcc - sys->verticalAccFiltered) * accAlpha;

    const float azMss = sys->verticalAccFiltered * G_CONST;
    sys->heaveVelocity += azMss * dt;
    sys->heaveVelocity = sys->hpFilterVel.update(sys->heaveVelocity);

    sys->heavePosition += sys->heaveVelocity * dt;
    sys->heavePosition = sys->hpFilterPos.update(sys->heavePosition);
}

math::vector3 Tactical_GetEarthAcceleration(const TacticalSystem *const sys, const math::vector3 acc) {
    const math::quaternion q = sys->quaternion;
    const float qw = q.w;
    const float qx = q.x;
    const float qy = q.y;
    const float qz = q.z;

    const float ax = acc.x;
    const float ay = acc.y;
    const float az = acc.z;

    math::vector3 earthAcc;
    earthAcc.x = (qw * qw + qx * qx - qy * qy - qz * qz) * ax +
                      2.0f * (qx * qy - qw * qz) * ay +
                      2.0f * (qx * qz + qw * qy) * az;

    earthAcc.y = 2.0f * (qx * qy + qw * qz) * ax +
                      (qw * qw - qx * qx + qy * qy - qz * qz) * ay +
                      2.0f * (qy * qz - qw * qx) * az;

    earthAcc.z = 2.0f * (qx * qz - qw * qy) * ax +
                      2.0f * (qy * qz + qw * qx) * ay +
                      (qw * qw - qx * qx - qy * qy + qz * qz) * az;

    if (sys->convention == TacticalConvention::ned) {
        earthAcc.z += 1.0f;
    } else {
        earthAcc.z -= 1.0f;
    }

    return earthAcc;
}
