#!/usr/bin/env python3
"""Run actual lifter.c with fake GPIO/PWM, time and wheel state; no devices."""
from pathlib import Path
import subprocess
import tempfile

ROOT=Path(__file__).resolve().parents[2]
source=(ROOT/'firmware/chassis/components/lifter/lifter.c').read_text()
source='\n'.join(line for line in source.splitlines() if not line.startswith('#include'))
stubs=r'''
#include <assert.h>
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <setjmp.h>
#include "lifter.h"
#define CONFIG_CHASSIS_LIFTER_PORT 1
#define CONFIG_CHASSIS_LIFTER_LOWER_DEG 0
#define CONFIG_CHASSIS_LIFTER_UPPER_DEG 80
#define CONFIG_CHASSIS_LIFTER_PULSE_0_DEG_US 500
#define CONFIG_CHASSIS_LIFTER_PULSE_180_DEG_US 2500
#define CONFIG_CHASSIS_LIFTER_UP_MS 600
#define CONFIG_CHASSIS_LIFTER_DOWN_MS 600
#define CONFIG_CHASSIS_LIFTER_TIMEOUT_MS 5000
#define ESP_OK 0
#define MCPWM_TIMER_CLK_SRC_DEFAULT 0
#define MCPWM_TIMER_COUNT_MODE_UP 0
#define MCPWM_TIMER_DIRECTION_UP 0
#define MCPWM_TIMER_EVENT_EMPTY 0
#define MCPWM_GEN_ACTION_HIGH 1
#define MCPWM_GEN_ACTION_LOW 0
#define MCPWM_TIMER_START_NO_STOP 0
#define MCPWM_GEN_TIMER_EVENT_ACTION(...) 0
#define MCPWM_GEN_COMPARE_EVENT_ACTION(...) 0
#define portMAX_DELAY 0
#define pdPASS 1
#define configASSERT(v) assert(v)
#define pdMS_TO_TICKS(v) (v)
typedef int esp_err_t;
typedef int SemaphoreHandle_t;
typedef int TickType_t;
typedef void *mcpwm_timer_handle_t;
typedef void *mcpwm_oper_handle_t;
typedef void *mcpwm_cmpr_handle_t;
typedef void *mcpwm_gen_handle_t;
typedef struct {int group_id,clk_src,resolution_hz,period_ticks,count_mode;} mcpwm_timer_config_t;
typedef struct {int group_id;} mcpwm_operator_config_t;
typedef struct {struct {int update_cmp_on_tez;} flags;} mcpwm_comparator_config_t;
typedef struct {int gen_gpio_num;} mcpwm_generator_config_t;
typedef struct {bool enabled; float speed[4];} motor_diagnostics_t;
static int64_t clock_us=10000;
static motor_diagnostics_t wheel;
static bool motor_inhibited;
static unsigned actual_pulse, pwm_updates, forced_low;
static bool fail_pwm;
static jmp_buf iteration_finished;
static void (*task_fn)(void*);
static int xSemaphoreCreateMutex(void) {return 1;}
static void xSemaphoreTake(int m,int wait) {(void)m;(void)wait;}
static void xSemaphoreGive(int m) {(void)m;}
static int64_t esp_timer_get_time(void) {return clock_us;}
static void Motor_Get_Diagnostics(motor_diagnostics_t *out) {*out=wheel;}
static void Motor_Set_Lift_Inhibit(bool on) {motor_inhibited=on;}
static int xTaskGetTickCount(void) {return 0;}
static void vTaskDelayUntil(int *prev,int interval) {(void)prev;assert(interval==10);longjmp(iteration_finished,1);}
static int xTaskCreatePinnedToCore(void (*f)(void*),const char*n,int stack,void*a,int pr,void*out,int core)
{(void)n;(void)stack;(void)a;(void)out;assert(pr==6 && core==0);task_fn=f;return 1;}
static int mcpwm_new_timer(const mcpwm_timer_config_t*c,void**out)
{assert(c->group_id==1 && c->resolution_hz==1000000 && c->period_ticks==20000);*out=(void*)1;return 0;}
static int mcpwm_new_operator(const mcpwm_operator_config_t*c,void**out)
{assert(c->group_id==1);*out=(void*)2;return 0;}
static int mcpwm_operator_connect_timer(void*op,void*t) {assert(op&&t);return 0;}
static int mcpwm_new_comparator(void*op,const mcpwm_comparator_config_t*c,void**out)
{assert(op&&c->flags.update_cmp_on_tez);*out=(void*)3;return 0;}
static int mcpwm_new_generator(void*op,const mcpwm_generator_config_t*c,void**out)
{assert(op && c->gen_gpio_num==8);*out=(void*)4;return 0;}
static int mcpwm_comparator_set_compare_value(void*cmp,unsigned pulse)
{assert(cmp);if(fail_pwm)return -1;assert(motor_inhibited);actual_pulse=pulse;++pwm_updates;return 0;}
static int mcpwm_generator_set_action_on_timer_event(void*g,int e) {assert(g);(void)e;return 0;}
static int mcpwm_generator_set_action_on_compare_event(void*g,int e) {assert(g);(void)e;return 0;}
static int mcpwm_timer_enable(void*t) {assert(t);return 0;}
static int mcpwm_timer_start_stop(void*t,int mode) {assert(t);(void)mode;return 0;}
static int mcpwm_generator_set_force_level(void*g,int level,bool hold)
{assert(g&&(level==0||level==-1)&&hold);if(level==0)++forced_low;return 0;}
'''
tests=r'''
static void tick(void) {if(!setjmp(iteration_finished)) task_fn(NULL);}
int main(void) {
    Lifter_Init();
    assert(!pwm_updates && !motor_inhibited); // absolutely no startup target
#ifndef CONFIG_CHASSIS_LIFTER_ENABLE
    Lifter_Heartbeat();clock_us=300000;Lifter_Heartbeat();
    assert(Lifter_Command(1)==LIFTER_NOT_READY && !pwm_updates);
    tick();assert(!motor_inhibited);
#else
    assert(Lifter_Command(1)==LIFTER_NOT_READY); // no communication lease
    Lifter_Heartbeat();tick();
    assert(Lifter_Command(1)==LIFTER_WHEELS_MOVING); // 200ms stable-stop interval
    clock_us=250000;Lifter_Heartbeat();
    assert(Lifter_Command(1)==LIFTER_OK); // no arm or travel calibration
    assert(actual_pulse==1389 && motor_inhibited && Lifter_Blocks_Chassis());
    clock_us=750000;Lifter_Heartbeat();tick();
    clock_us=850000;Lifter_Heartbeat();tick();
    assert(actual_pulse==1389 && motor_inhibited); // completed, still holds upper angle
    lifter_control_t snapshot;Lifter_Get_Snapshot(&snapshot);
    assert(snapshot.position==1 && snapshot.completed==1 && snapshot.state==LIFTER_IDLE);
    unsigned updates=pwm_updates;
    clock_us=1000000;Lifter_Heartbeat();tick();
    assert(!motor_inhibited && !Lifter_Blocks_Chassis() && actual_pulse==1389 && pwm_updates==updates);
    assert(Lifter_Command(1)==LIFTER_DUPLICATE && pwm_updates==updates);
    assert(Lifter_Command(0)==LIFTER_OK && actual_pulse==500 && motor_inhibited);
    clock_us=1500000;Lifter_Heartbeat();tick();
    clock_us=1600000;Lifter_Heartbeat();tick();
    assert(actual_pulse==500); // lower completion also keeps its target
    Lifter_Stop(LIFTER_STOPPED);
    assert(forced_low==1 && motor_inhibited && applied_pulse==0);
    assert(Lifter_Command(1)==LIFTER_OK && applied_pulse==1389); // direct resume releases forced low
    Lifter_Stop(LIFTER_STOPPED);
    fail_pwm=true;
    assert(Lifter_Command(0)==LIFTER_NOT_READY);
    Lifter_Get_Snapshot(&snapshot);
    assert(snapshot.fault==LIFTER_IO_FAULT && snapshot.pulse_us==0 && motor_inhibited);
    Lifter_Stop(LIFTER_STOPPED);
    assert(Lifter_Command(1)==LIFTER_NOT_READY); // operator stop cannot clear IO fault
#endif
    return 0;
}
'''
with tempfile.TemporaryDirectory(prefix='lifter-driver-') as temp:
    unit=Path(temp)/'driver.c';binary=Path(temp)/'check'
    unit.write_text(stubs+source+tests)
    for enabled in [False,True]:
        command=['gcc','-std=c11','-Wall','-Wextra','-Werror','-Wno-unused-variable',
                 '-I'+str(ROOT/'firmware/chassis/components/lifter'),str(unit),
                 str(ROOT/'firmware/chassis/components/lifter/lifter_control.c'),'-lm','-o',str(binary)]
        if enabled:command.append('-DCONFIG_CHASSIS_LIFTER_ENABLE=1')
        subprocess.run(command,check=True)
        subprocess.run([str(binary)],check=True)
print('PASS: real positional driver, no startup PWM, direct commands, persistent hold, interlock, release/resume and IO fault')
