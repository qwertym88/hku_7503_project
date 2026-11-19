#!/usr/bin/env python3
"""Run actual motor/kinematics: feedback feeds PID without fault latches."""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
source = (ROOT / 'firmware/chassis/components/motor/motor.c').read_text()
# Keep all production function bodies; replace only ESP/FreeRTOS includes.
source = '\n'.join(line for line in source.splitlines() if not line.startswith('#include'))
stubs = r'''
#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "motor.h"
#include "car_motion.h"
#include <limits.h>
#define ENCODER_ID_M1 1
#define PID_CAL_TYPE_INCREMENTAL 0
#define portMAX_DELAY 0
#define pdMS_TO_TICKS(v) (v)
#define ESP_LOGI(...) ((void)0)
#define ESP_ERROR_CHECK(v) assert((v)==0)
#define configASSERT(v) assert(v)
typedef int SemaphoreHandle_t;
typedef int TickType_t;
typedef void *pid_ctrl_block_handle_t;
typedef struct { float kp,ki,kd; int cal_type,max_output,min_output,max_integral,min_integral; } pid_ctrl_parameter_t;
typedef struct { pid_ctrl_parameter_t init_param; } pid_ctrl_config_t;
static int64_t fake_us;
static int counts[4], writes, brakes;
static float pid_errors[4];
static int wheel_outputs[4];
#define xPortGetCoreID() 1
static void xSemaphoreTake(int s,int wait) {(void)s;(void)wait;}
static void xSemaphoreGive(int s) {(void)s;}
static int xSemaphoreCreateMutex(void) {return 1;}
static int64_t esp_timer_get_time(void) {return fake_us;}
static int Encoder_Get_Count(int id) {return counts[id-1];}
static void Encoder_Init(void) {}
void PwmMotor_Init(void) {}
void PwmMotor_Stop(motor_id_t id, bool brake) {assert(id==MOTOR_ID_ALL); if(brake) ++brakes;}
void PwmMotor_Set_Speed(motor_id_t id,int speed) {wheel_outputs[id-1]=speed;++writes;}
static void pid_compute(void *p,float err,float *out) {pid_errors[(uintptr_t)p-1]=err; *out=err*10;}
static void pid_reset_control_block(void *p) {(void)p;}
static int pid_new_control_block(void *cfg,void **out) {(void)cfg;*out=(void*)1;return 0;}
static void pid_update_parameters(void *p,void *cfg) {(void)p;(void)cfg;}
static void vTaskDelay(int t) {(void)t;}
static int xTaskGetTickCount(void) {return 0;}
static void vTaskDelayUntil(int *t,int d) {(void)t;(void)d;}
static void vTaskDelete(void *p) {(void)p;}
static void xTaskCreatePinnedToCore(void (*f)(void*),const char*n,int s,void*a,int pr,void*o,int c)
{(void)f;(void)n;(void)s;(void)a;(void)pr;(void)o;(void)c;}
void Motor_Stop(bool brake);
'''
tests = r'''
static void reset_fixture(void) {
    pid_enable=0;
    fake_us=10000; writes=brakes=0; lift_inhibited=false;
    for(int i=0;i<4;++i) {
        pid_motor[i]=(void*)(uintptr_t)(i+1);
    }
    Motor_PID_Ctrl(); // synchronize production function's last encoder counts.
}
int main(void) {
    Motor_Init();
    reset_fixture();
    // Refreshing commands keeps drive available even with one missing encoder.
    for(int step=0;step<=150;++step) {
        fake_us=10000+step*10000;
        Motor_Set_Speed(.03f,.03f,.03f,.03f);
        for(int i=0;i<3;++i) ++counts[i];
        Motor_PID_Ctrl();
        assert(pid_enable);
    }
    motor_diagnostics_t d;
    Motor_Get_Diagnostics(&d);
    assert(d.enabled && d.speed[3]==0 && !d.lift_inhibited);
    int previous=writes;
    Motor_Stop(STOP_BRAKE);Motor_PID_Ctrl();
    assert(!pid_enable && writes==previous);
    Motor_Set_Speed(.03f,.03f,.03f,.03f);Motor_PID_Ctrl();
    assert(pid_enable && writes>previous); // no RESET needed after stopped feedback
    reset_fixture();
    for(int step=0;step<=150;++step) {
        fake_us=10000+step*10000;
        Motor_Set_Speed(.03f,.03f,.03f,.03f);
        for(int i=0;i<4;++i) counts[i]+=(i==1 ? -1 : 1);
        Motor_PID_Ctrl();
        assert(pid_enable); // reverse feedback still reaches PID but never latches
    }
    Motor_Get_Diagnostics(&d);assert(d.speed[1]<0 && d.enabled);
    reset_fixture();
    Motor_Set_Speed(.03f,.03f,.03f,.03f);
    for(int step=0;step<48;++step) {
        fake_us=10000+step*10000;
        for(int i=0;i<4;++i) ++counts[i];
        Motor_PID_Ctrl();
    }
    assert(pid_enable);
    previous=writes;
    fake_us=490000;
    Motor_PID_Ctrl();
    assert(!pid_enable && writes==previous);
    fake_us=500000;Motor_Set_Speed(.03f,.03f,.03f,.03f);Motor_PID_Ctrl();
    assert(pid_enable && writes>previous); // watchdog stop accepts a fresh command
    reset_fixture();
    Motor_Set_Lift_Inhibit(true);
    previous=writes;
    Motor_Set_Speed(.03f,.03f,.03f,.03f);
    Motor_PID_Ctrl();
    assert(!pid_enable && writes==previous);
    Motor_Set_Lift_Inhibit(false);
    Motor_PID_Ctrl();
    assert(!pid_enable && writes==previous); // releasing interlock cannot resume a stale command
    Motor_Set_Speed(.03f,.03f,.03f,.03f);
    Motor_PID_Ctrl();
    assert(pid_enable && writes>previous);
    // Actual forward/reverse and turning feedback must reach PID/odom with
    // chassis signs, while diagnostics preserve the raw installation convention.
    reset_fixture();
    Motion_Ctrl(.03f,0,0);
    for(int i=0;i<4;++i) counts[i]+=2;
    Motor_PID_Ctrl();
    Motor_Get_Diagnostics(&d);
    car_motion_t motion;
    Motion_Get_Speed(&motion);
    float meters_per_count=MOTOR_WHEEL_CIRCLE/MOTOR_ENCODER_CIRCLE/MOTOR_PID_PERIOD;
    for(int i=0;i<4;++i) {
        assert(d.counts[i]==counts[i] && d.speed[i]>0 && d.target_counts[i]>0);
        assert(fabsf(d.speed[i]-2*meters_per_count)<1e-6f);
        assert(fabsf(pid_errors[i]-(d.target_counts[i]-2))<1e-6f);
        assert(wheel_outputs[i]<0); // measured speed exceeds target, PID corrects down
    }
    assert(motion.Vx>0 && fabsf(motion.Wz)<1e-6f);
    reset_fixture();
    Motion_Ctrl(-.03f,0,0);
    for(int i=0;i<4;++i) counts[i]-=2;
    Motor_PID_Ctrl();Motor_Get_Diagnostics(&d);Motion_Get_Speed(&motion);
    assert(motion.Vx<0 && fabsf(motion.Wz)<1e-6f);
    for(int i=0;i<4;++i) {
        assert(d.speed[i]<0 && d.target_counts[i]<0);
        assert(fabsf(pid_errors[i]-(d.target_counts[i]+2))<1e-6f);
    }
    for(int turn=-1;turn<=1;turn+=2) {
        reset_fixture();
        Motion_Ctrl(0,0,turn*.15f);
        for(int step=0;step<35;++step) {
            fake_us+=10000;
            Motion_Ctrl(0,0,turn*.15f);
            for(int i=0;i<4;++i) counts[i]+=(i<2 ? -turn : turn);
            Motor_PID_Ctrl();
        }
        Motor_Get_Diagnostics(&d);Motion_Get_Speed(&motion);
        assert(pid_enable && fabsf(motion.Vx)<1e-6f && turn*motion.Wz>0);
        assert(turn*d.speed[0]<0 && turn*d.speed[2]>0);
    }
    // Raw counter wrap must preserve the signed one-count displacement.
    reset_fixture();
    for(int i=0;i<4;++i) counts[i]=INT_MIN;
    Motor_PID_Ctrl();
    for(int i=0;i<4;++i) counts[i]=INT_MAX;
    Motor_PID_Ctrl();Motor_Get_Diagnostics(&d);
    for(int i=0;i<4;++i) assert(fabsf(d.speed[i]+meters_per_count)<1e-6f);
    for(int i=0;i<4;++i) counts[i]=INT_MIN;
    Motor_PID_Ctrl();Motor_Get_Diagnostics(&d);
    for(int i=0;i<4;++i) assert(fabsf(d.speed[i]-meters_per_count)<1e-6f);
    puts("PASS: baseline raw-positive feedback, PID errors, forward/reverse/turn odometry, wrap, no feedback latch, interlock and watchdog");
}
'''
with tempfile.TemporaryDirectory(prefix='chassis-motor-test-') as temp:
    unit = Path(temp) / 'motor_test.c'
    binary = Path(temp) / 'check'
    motion_source=(ROOT/'firmware/chassis/components/car_motion/car_motion.c').read_text()
    motion_source='\n'.join(line for line in motion_source.splitlines() if not line.startswith('#include'))
    unit.write_text(stubs + source + motion_source + tests)
    subprocess.run(['gcc','-std=c11','-Wall','-Wextra','-Werror','-Wno-unused-variable','-Wno-unused-parameter',
                    '-I'+str(ROOT/'firmware/chassis/components/motor'),
                    '-I'+str(ROOT/'firmware/chassis/components/pwm_motor'),
                    '-I'+str(ROOT/'firmware/chassis/components/car_motion'),
                    str(unit),'-lm','-o',str(binary)],check=True)
    subprocess.run([str(binary)],check=True)
