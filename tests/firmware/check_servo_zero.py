#!/usr/bin/env python3
"""Exercise actual installation app: brake first, one channel and fixed pulse."""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
source = (ROOT/'firmware/servo_zero/main/main.c').read_text()
source = '\n'.join(line for line in source.splitlines() if not line.startswith('#include'))
stubs = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <setjmp.h>
#define MOTOR_ID_ALL 0
#define STOP_BRAKE true
#define ESP_LOGI(tag,...) ((void)(tag))
#define ESP_ERROR_CHECK(expr) assert((expr)==0)
#define pdMS_TO_TICKS(ms) (ms)
#define MCPWM_TIMER_CLK_SRC_DEFAULT 0
#define MCPWM_TIMER_COUNT_MODE_UP 0
#define MCPWM_TIMER_DIRECTION_UP 0
#define MCPWM_TIMER_EVENT_EMPTY 0
#define MCPWM_GEN_ACTION_HIGH 1
#define MCPWM_GEN_ACTION_LOW 0
#define MCPWM_TIMER_START_NO_STOP 0
#define MCPWM_GEN_TIMER_EVENT_ACTION(...) 0
#define MCPWM_GEN_COMPARE_EVENT_ACTION(...) 0
typedef void *mcpwm_timer_handle_t;
typedef void *mcpwm_oper_handle_t;
typedef void *mcpwm_cmpr_handle_t;
typedef void *mcpwm_gen_handle_t;
typedef struct { int group_id,clk_src,resolution_hz,period_ticks,count_mode; } mcpwm_timer_config_t;
typedef struct { int group_id; } mcpwm_operator_config_t;
typedef struct { struct { bool update_cmp_on_tez; } flags; } mcpwm_comparator_config_t;
typedef struct { int gen_gpio_num; } mcpwm_generator_config_t;
static jmp_buf finished;
static int braked, timer_started, pulse_writes, generators, delays;
static void PwmMotor_Init(void) {}
static void PwmMotor_Stop(int id,bool brake) {assert(id==0 && brake);braked=1;}
static void vTaskDelay(int delay) {
    assert(braked);
    if(delays++==0) {assert(!timer_started && delay==3000);}
    else {assert(timer_started && delay==1000);longjmp(finished,1);}
}
static int mcpwm_new_timer(const mcpwm_timer_config_t *cfg,void **out) {
    assert(braked && delays==1);
    assert(cfg->group_id==1 && cfg->resolution_hz==1000000 && cfg->period_ticks==20000);
    *out=(void*)1;return 0;
}
static int mcpwm_new_operator(const mcpwm_operator_config_t *cfg,void **out)
{assert(cfg->group_id==1);*out=(void*)2;return 0;}
static int mcpwm_operator_connect_timer(void *op,void *timer)
{assert(op && timer);return 0;}
static int mcpwm_new_comparator(void *op,const mcpwm_comparator_config_t *cfg,void **out)
{assert(op && cfg->flags.update_cmp_on_tez);*out=(void*)3;return 0;}
static int mcpwm_new_generator(void *op,const mcpwm_generator_config_t *cfg,void **out)
{assert(op && cfg->gen_gpio_num==(CONFIG_SERVO_ZERO_PORT==1 ? 8 : 21));++generators;*out=(void*)4;return 0;}
static int mcpwm_comparator_set_compare_value(void *cmp,unsigned pulse)
{assert(cmp && pulse==CONFIG_SERVO_ZERO_PULSE_US && !timer_started);++pulse_writes;return 0;}
static int mcpwm_generator_set_action_on_timer_event(void *gen,int event)
{assert(gen && event==0);return 0;}
static int mcpwm_generator_set_action_on_compare_event(void *gen,int event)
{assert(gen && event==0);return 0;}
static int mcpwm_timer_enable(void *timer) {assert(timer);return 0;}
static int mcpwm_timer_start_stop(void *timer,int mode)
{assert(timer && mode==0 && pulse_writes==1 && generators==1);timer_started=1;return 0;}
'''
tests = r'''
int main(void) {
    if(!setjmp(finished)) app_main();
    assert(braked && timer_started && generators==1 && pulse_writes==1);
    puts("PASS: actual servo app brakes wheels before one fixed servo PWM output");
}
'''
with tempfile.TemporaryDirectory(prefix='servo-zero-test-') as temp:
    unit=Path(temp)/'check.c'
    binary=Path(temp)/'check'
    unit.write_text(stubs+source+tests)
    for port,pulse in [(1,1500),(2,1500),(2,1475)]:
        subprocess.run(['gcc','-std=c11','-Wall','-Wextra','-Werror',
            f'-DCONFIG_SERVO_ZERO_PORT={port}',f'-DCONFIG_SERVO_ZERO_PULSE_US={pulse}',
            '-DCONFIG_SERVO_ZERO_START_DELAY_MS=3000',str(unit),'-o',str(binary)],check=True)
        subprocess.run([str(binary)],check=True)
    for port,pulse in [(0,1500),(3,1500),(1,1399),(1,1601)]:
        result=subprocess.run(['gcc','-std=c11',
            f'-DCONFIG_SERVO_ZERO_PORT={port}',f'-DCONFIG_SERVO_ZERO_PULSE_US={pulse}',
            '-DCONFIG_SERVO_ZERO_START_DELAY_MS=3000',str(unit),'-o',str(binary)],
            capture_output=True,text=True)
        assert result.returncode != 0, (port,pulse)
    print('PASS: invalid connectors and out-of-neutral calibration pulses rejected')
