#pragma once

// Stationary raw capture SHA256: 83aad377ec0f25fb464508f511ba74b694322c73328780283959a35b11f66dcb
// One session only; verify residual bias after flashing, restart and warm-up.
#define CHASSIS_IMU_BIAS_ENABLED 1
#define CHASSIS_IMU_BIAS_X_RADS (-0.0119512194f)
#define CHASSIS_IMU_BIAS_Y_RADS (-0.0046553588f)
#define CHASSIS_IMU_BIAS_Z_RADS (0.0230571526f)

static inline float chassis_imu_correct_gyro(float raw, float bias)
{
    return CHASSIS_IMU_BIAS_ENABLED ? raw - bias : raw;
}
