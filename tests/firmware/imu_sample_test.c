// Native checks of the same conversion/freshness path used by the ESP32 driver.
// gcc -std=c11 -Wall -Wextra -Werror -Ifirmware/chassis/components/icm42670p
//     tests/firmware/imu_sample_test.c -lm -o /tmp/imu_sample_test
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "imu_sample.h"

int main(void)
{
    icm42670p_sample_t sample = {0};
    int16_t accel[] = {0, 8192, -8192}; // +/-1 g at +/-4 g full scale.
    int16_t gyro[] = {16384, -16384, 0}; // +/-1000 deg/s at +/-2000 deg/s.
    assert(imu_sample_from_raw(&sample, accel, gyro, 4, 2000, 1000000, 7));
    assert(fabsf(sample.accel_mps2[1] - 9.80665f) < 0.001f);
    assert(fabsf(sample.accel_mps2[2] + 9.80665f) < 0.001f);
    assert(fabsf(sample.gyro_rads[0] - 17.45329252f) < 0.001f);
    assert(fabsf(sample.gyro_rads[1] + 17.45329252f) < 0.001f);
    assert(sample.accel_mps2[0] == 0 && sample.gyro_rads[2] == 0);
    assert(sample.time_us == 1000000 && sample.sequence == 7);
    assert(imu_sample_is_fresh(&sample, 1000000, 100000, 6));
    assert(imu_sample_is_fresh(&sample, 1100000, 100000, 6));
    assert(!imu_sample_is_fresh(&sample, 1100001, 100000, 6)); // Stale.
    assert(!imu_sample_is_fresh(&sample, 999999, 100000, 6)); // Future time.
    assert(!imu_sample_is_fresh(&sample, 1000000, 100000, 7)); // Duplicate.
    sample.valid = false; // A failed I2C read invalidates the cached sample.
    assert(!imu_sample_is_fresh(&sample, 1000000, 100000, 6));
    accel[0] = INT16_MIN; // FIFO invalid-data sentinel must not become motion.
    assert(!imu_sample_from_raw(&sample, accel, gyro, 4, 2000, 2000000, 8));
    assert(!sample.valid);
    accel[0] = 0;
    gyro[0] = INT16_MIN;
    assert(!imu_sample_from_raw(&sample, accel, gyro, 4, 2000, 2000000, 8));
    gyro[0] = 0;
    assert(!imu_sample_from_raw(&sample, accel, gyro, 0, 2000, 2000000, 8));
    assert(!imu_sample_from_raw(&sample, accel, gyro, 4, 0, 2000000, 8));
    assert(imu_sample_from_raw(&sample, accel, gyro, 4, 2000, 2000000, 8));
    assert(imu_sample_is_fresh(&sample, 2000000, 100000, 7)); // Recovery.
    puts("PASS: SI units/sign, timestamps, stale/duplicate/invalid data, recovery");
    return 0;
}
