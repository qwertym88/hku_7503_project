#!/usr/bin/env python3
"""Compile actual ROS callbacks and diagnostic formatter with host message shims."""
from pathlib import Path
import json
import subprocess
import tempfile

ROOT=Path(__file__).resolve().parents[2]
main=(ROOT/'firmware/chassis/main/main.c').read_text()
restart=main[main.index('static void __attribute__((noreturn)) restart_disconnected('):
             main.index('static bool motion_requests_locked(')]
callbacks=main[main.index('static bool motion_requests_locked('):main.index('static void odom_callback(')]
fmt=main[main.index('    int length = snprintf('):main.index('    if (length > 0')]
stubs=r'''
#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <limits.h>
#include <setjmp.h>
#include <stdio.h>
#include <string.h>
#include "car_motion.h"
#include "lifter.h"
#define CHASSIS_IMU_BIAS_ENABLED 0
#define CHASSIS_IMU_BIAS_X_RADS 0.0
#define CHASSIS_IMU_BIAS_Y_RADS 0.0
#define CHASSIS_IMU_BIAS_Z_RADS 0.0
typedef struct { char *data;size_t size,capacity; } rosidl_runtime_c__String;
typedef struct { uint8_t data; } std_msgs__msg__UInt8;
typedef struct { bool success;rosidl_runtime_c__String message; } std_srvs__srv__Trigger_Response;
typedef struct { bool data; } std_srvs__srv__SetBool_Request;
typedef std_srvs__srv__Trigger_Response std_srvs__srv__SetBool_Response;
static bool rosidl_runtime_c__String__assign(rosidl_runtime_c__String *s,const char*text)
{size_t n=strlen(text);assert(n<s->capacity);memcpy(s->data,text,n+1);s->size=n;return true;}
static lifter_control_t test_lift;
static bool fault;
static bool command_locked;
static const uint32_t recovery_cookie=0x43484b31;
static uint32_t recovery_marker;
static lifter_fault_t recovery_lifter_fault;
static jmp_buf reboot;
static int64_t last_locked_command_us;
static int64_t fake_now=10000;
static uint32_t command_count, rejected_commands;
static unsigned wheel_stop_calls, wheel_move_calls;
#define STOP_BRAKE true
typedef struct {struct {double x,y,z;} linear,angular;} geometry_msgs__msg__Twist;
static int64_t esp_timer_get_time(void) {return fake_now;}
static float test_wheel_speed;
void Motor_Get_Diagnostics(motor_diagnostics_t *out) {*out=(motor_diagnostics_t){.speed={test_wheel_speed}};}
void Motor_Stop(bool brake) {assert(brake);++wheel_stop_calls;}
bool Lifter_Blocks_Chassis(void) {return lifter_control_blocks_wheels(&test_lift);}
void Motion_Ctrl(float vx,float vy,float wz) {(void)vx;assert(vy==0);(void)wz;++wheel_move_calls;}
lifter_result_t Lifter_Command(uint8_t value) {return lifter_control_command(&test_lift,value,true,fake_now);}
void Lifter_Stop(lifter_fault_t reason) {lifter_control_stop(&test_lift,reason);}
void Lifter_Get_Snapshot(lifter_control_t *out) {*out=test_lift;}
#define pdMS_TO_TICKS(ms) (ms)
static void vTaskDelay(int ticks) {assert(ticks>=250);}
static _Noreturn void esp_restart(void) {longjmp(reboot,1);}
'''
tests=r'''
int main(void) {
    const lifter_config_t config={.enabled=true,.lower_deg=0,.upper_deg=80,
        .pulse_0_us=500,.pulse_180_us=2500,.up_ms=600,.down_ms=600,.timeout_ms=5000};
    lifter_control_init(&test_lift,&config);
    char message[96];
    std_srvs__srv__Trigger_Response response={.message={.data=message,.capacity=sizeof(message)}};
    std_msgs__msg__UInt8 command={.data=1};
    lifter_callback(&command);
    assert(test_lift.state==LIFTER_RAISING && test_lift.pulse_us==1389); // first command, no arm
    geometry_msgs__msg__Twist velocity={.linear.x=.03};
    command_callback(&velocity);
    assert(wheel_move_calls==0 && rejected_commands==1);
    velocity.linear.x=0;command_callback(&velocity);
    assert(wheel_stop_calls==1 && test_lift.state==LIFTER_RAISING);
    int64_t deadline=test_lift.end_us;
    fake_now+=100000;lifter_callback(&command);
    assert(test_lift.end_us==deadline);
    command.data=255;lifter_callback(&command);
    assert(test_lift.end_us==deadline && test_lift.state==LIFTER_RAISING);
    lifter_control_tick(&test_lift,deadline,true);
    assert(test_lift.pulse_us==1389 && test_lift.state==LIFTER_IDLE);
    fake_now=deadline+10000;command.data=0;lifter_callback(&command);
    assert(test_lift.state==LIFTER_LOWERING && test_lift.pulse_us==500);
    lifter_stop_callback(NULL,&response);
    assert(response.success && test_lift.pulse_us==0 && test_lift.position==-1);
    command.data=1;lifter_callback(&command);
    assert(test_lift.state==LIFTER_RAISING); // no re-arm after operator stop
    lifter_stop_callback(NULL,&response);
    assert(!fault);

    lifter_control_init(&test_lift,&config);
    lifter_callback(&command);
    assert(test_lift.pulse_us!=0);
    const unsigned previous_stops=wheel_stop_calls;
    if (setjmp(reboot)==0) restart_disconnected();
    assert(wheel_stop_calls==previous_stops+1 && test_lift.pulse_us==0 && test_lift.fault==LIFTER_COMM_LOST);
    assert(recovery_marker==recovery_cookie);

    lifter_control_init(&test_lift,&config);
    Lifter_Stop(LIFTER_IO_FAULT);
    if (setjmp(reboot)==0) restart_disconnected();
    assert(recovery_marker==recovery_cookie && recovery_lifter_fault==LIFTER_IO_FAULT);

    lifter_control_init(&test_lift,&config);
    command_locked=true;
    unlock_callback(NULL,&response);
    assert(!response.success && command_locked);
    velocity.linear.x=.03;
    fake_now+=500000;
    command_callback(&velocity);
    fake_now+=900000;
    lifter_callback(&command);
    assert(wheel_move_calls==0 && test_lift.rx==0 && test_lift.pulse_us==0);
    unlock_callback(NULL,&response);
    assert(!response.success && recovery_marker==recovery_cookie);
    fake_now+=1000000;
    test_wheel_speed=.02;
    unlock_callback(NULL,&response);
    assert(!response.success && command_locked);
    test_wheel_speed=0;
    unlock_callback(NULL,&response);
    assert(response.success && !command_locked && recovery_marker==0);
    assert(wheel_move_calls==0 && test_lift.rx==0);
    lifter_callback(&command);
    assert(test_lift.state==LIFTER_RAISING);
    lifter_stop_callback(NULL,&response);

    // Worst-sized representative integer diagnostics must fit the message and
    // XRCE best-effort MTU. Use the production snprintf expression verbatim.
    motor_diagnostics_t motor={.enabled=false,.command_age_us=INT64_MAX,.control_cycles=UINT32_MAX};
    for(int i=0;i<4;++i) {motor.counts[i]=INT_MIN;motor.target_counts[i]=-1000;
        motor.speed[i]=-1234.5f;motor.pid_output[i]=-200;}
    lifter_control_t lift=test_lift;
    lift.rx=lift.rejected=lift.completed=UINT32_MAX;
    command_count=rejected_commands=UINT32_MAX;
    uint32_t imu_published=UINT32_MAX;
    int raw=4095,millivolts=3300;
    int64_t imu_age_ms=INT64_MAX;
    struct {int state;unsigned who_am_i;uint32_t read_errors;} imu_status={.state=-1,.who_am_i=103,.read_errors=UINT32_MAX};
    struct {bool valid;} sample={.valid=false};
    char buffer[1792];
    struct {rosidl_runtime_c__String data;} diagnostic={.data={.data=buffer,.capacity=sizeof(buffer)}};
'''
finish=r'''
    assert(length>0 && length<(int)sizeof(buffer) && length+16<2048);
    puts(buffer);
}
'''
with tempfile.TemporaryDirectory(prefix='lifter-ros-') as temp:
    unit=Path(temp)/'ros.c';binary=Path(temp)/'check'
    unit.write_text(stubs+restart+callbacks+tests+fmt+finish)
    subprocess.run(['gcc','-std=c11','-Wall','-Wextra','-Werror',
        '-I'+str(ROOT/'firmware/chassis/components/car_motion'),
        '-I'+str(ROOT/'firmware/chassis/components/lifter'),
        '-I'+str(ROOT/'firmware/chassis/components/motor'),
        '-I'+str(ROOT/'firmware/chassis/components/pwm_motor'),str(unit),
        str(ROOT/'firmware/chassis/components/lifter/lifter_control.c'),'-lm','-o',str(binary)],check=True)
    output=subprocess.check_output([str(binary)],text=True)
    data=json.loads(output)
    assert data['feedback_guard_enabled'] is False
    assert data['command_locked'] is False
    assert 'feedback_missing_mask' not in data and 'feedback_reversed_mask' not in data
    assert data['lift_inhibited'] == 0
    assert data['lifter_position_confirmed'] is False
    assert data['lifter_position']==-1
    assert data['lifter_lower_deg']==0 and data['lifter_upper_deg']==80
    assert data['lifter_target_deg']==-1 and data['lifter_pulse_us']==0
    assert data['lifter_pulse_0_deg_us']==500 and data['lifter_pulse_180_deg_us']==2500
print('PASS: ROS lift controls, reconnect command lock, quiet/stationary unlock and JSON/MTU bounds')
