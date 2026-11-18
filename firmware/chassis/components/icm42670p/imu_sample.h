#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <limits.h>

// Sensor axes, SI units. No board-to-base rotation or zero-bias estimate applied.
typedef struct {
    float accel_mps2[3];
    float gyro_rads[3];
    int64_t time_us;  // Local time when the latest FIFO sample was received.
    uint32_t sequence;
    bool valid;
} icm42670p_sample_t;

static inline bool imu_sample_from_raw(icm42670p_sample_t *out,
                                      const int16_t accel[3], const int16_t gyro[3],
                                      uint16_t accel_fsr_g, uint16_t gyro_fsr_dps,
                                      int64_t time_us, uint32_t sequence)
{
    out->valid = false;
    if (!accel_fsr_g || !gyro_fsr_dps) return false;
    for (int i = 0; i < 3; ++i)
        if (accel[i] == INT16_MIN || gyro[i] == INT16_MIN) return false;
    for (int i = 0; i < 3; ++i) {
        // Official sample already converts to SI despite its _g/_dps names.
        out->accel_mps2[i] = (float)accel[i] * accel_fsr_g * 9.80665f / INT16_MAX;
        out->gyro_rads[i] = (float)gyro[i] * gyro_fsr_dps *
                            (3.14159265358979323846f / 180.0f) / INT16_MAX;
    }
    out->time_us = time_us;
    out->sequence = sequence;
    out->valid = true;
    return true;
}

static inline bool imu_sample_is_fresh(const icm42670p_sample_t *sample,
                                       int64_t now_us, int64_t max_age_us,
                                       uint32_t last_sequence)
{
    return sample->valid && sample->sequence != last_sequence &&
           now_us >= sample->time_us && now_us - sample->time_us <= max_age_us;
}
