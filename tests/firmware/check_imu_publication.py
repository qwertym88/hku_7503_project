#!/usr/bin/env python3
"""Run the actual main.c IMU callback with fake hardware, time and ROS publish.

No device access or commands. Complements imu_sample_test.c and the IDF build.
"""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
source = (ROOT / 'firmware/chassis/main/main.c').read_text()
start = source.index('static void imu_callback(')
end = source.index('\nstatic void battery_init(', start)
callback = source[start:end]
stubs = r'''
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include "icm42670p.h"
#include "imu_calibration.h"
typedef int rcl_timer_t;
#define RCL_RET_OK 0
struct vector { double x, y, z; };
static struct {
    struct { struct { int32_t sec; uint32_t nanosec; } stamp; } header;
    struct vector angular_velocity, linear_acceleration;
} imu;
static int imu_publisher;
static bool fault;
static uint32_t imu_last_sequence, imu_published;
static icm42670p_sample_t fake_sample;
static icm42670p_status_t fake_status = {.state = 1, .who_am_i = 0x67};
static int64_t fake_now = 2000000;
static const int64_t fake_epoch = 1700000000000000000LL;
static int publish_calls, publish_result;
void Icm42670p_Get_Snapshot(icm42670p_sample_t *s, icm42670p_status_t *h)
{ *s = fake_sample; *h = fake_status; }
static int64_t esp_timer_get_time(void) { return fake_now; }
static int64_t rmw_uros_epoch_nanos(void) { return fake_epoch; }
static int rcl_publish(void *publisher, const void *msg, const void *options)
{ (void)publisher; (void)msg; (void)options; ++publish_calls; return publish_result; }
'''
tests = r'''
int main(void)
{
    int16_t a[] = {0, 0, 8192}, g[] = {0, 0, 16384};
    assert(imu_sample_from_raw(&fake_sample, a, g, 4, 2000, 1995000, 1));
    imu_callback(NULL, 0);
    assert(publish_calls == 1 && imu_published == 1 && imu_last_sequence == 1);
    assert(fabs(imu.angular_velocity.x -
        chassis_imu_correct_gyro(0, CHASSIS_IMU_BIAS_X_RADS)) < 0.0001);
    assert(fabs(imu.angular_velocity.y -
        chassis_imu_correct_gyro(0, CHASSIS_IMU_BIAS_Y_RADS)) < 0.0001);
    assert(fabs(imu.angular_velocity.z -
        chassis_imu_correct_gyro(17.45329252f, CHASSIS_IMU_BIAS_Z_RADS)) < 0.001); // No double conversion.
    assert(fabs(imu.linear_acceleration.z - 9.80665) < 0.001);
    assert((int64_t)imu.header.stamp.sec * 1000000000 + imu.header.stamp.nanosec
           == fake_epoch - 5000000); // FIFO receipt time, not publish time.
    imu_callback(NULL, 0);
    assert(publish_calls == 1); // Same sample cannot get a new timestamp.
    fake_sample.sequence = 2;
    fake_now = 2100001;
    imu_callback(NULL, 0);
    assert(publish_calls == 1); // Stale.
    fake_now = 2000000;
    fake_sample.valid = false;
    imu_callback(NULL, 0);
    assert(publish_calls == 1); // Bus error.
    fake_sample.valid = true;
    fake_status.state = -1;
    imu_callback(NULL, 0);
    assert(publish_calls == 1); // Initialization/WHOAMI/self-test failure.
    fake_status.state = 1;
    imu_callback(NULL, 0);
    assert(publish_calls == 2 && imu_published == 2); // Recovery.
    fake_sample.sequence = 3;
    publish_result = 1;
    imu_callback(NULL, 0);
    assert(fault && imu_published == 2 && imu_last_sequence == 2);
    puts("PASS: actual IMU callback SI payload, acquisition stamp, validity gates, publish fault");
}
'''
with tempfile.TemporaryDirectory(prefix='chassis-imu-check-') as tmp:
    unit = Path(tmp) / 'callback.c'
    binary = Path(tmp) / 'check'
    unit.write_text(stubs + callback + tests)
    subprocess.run(['gcc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                    '-Wno-unused-parameter',
                    '-I' + str(ROOT / 'firmware/chassis/components/icm42670p'),
                    '-I' + str(ROOT / 'firmware/chassis/main'),
                    str(unit), '-lm', '-o', str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
    # Recompile the SAME callback with a nonzero, enabled calibration to catch
    # wrong sign, wrong axis and accidental acceleration changes.
    calibration = (ROOT / 'firmware/chassis/main/imu_calibration.h').read_text()
    calibration = calibration.replace('CHASSIS_IMU_BIAS_ENABLED 0', 'CHASSIS_IMU_BIAS_ENABLED 1')
    for axis, value in [('X', '-0.013f'), ('Y', '-0.005f'), ('Z', '0.024f')]:
        calibration = calibration.replace(f'CHASSIS_IMU_BIAS_{axis}_RADS 0.0f',
                                          f'CHASSIS_IMU_BIAS_{axis}_RADS {value}')
    (Path(tmp) / 'imu_calibration.h').write_text(calibration)
    subprocess.run(['gcc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                    '-Wno-unused-parameter',
                    '-I' + str(ROOT / 'firmware/chassis/components/icm42670p'),
                    str(unit), '-lm', '-o', str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
