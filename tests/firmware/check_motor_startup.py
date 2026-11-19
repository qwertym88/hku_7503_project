#!/usr/bin/env python3
"""Compare committed motor baseline and current control with actual incremental PID.

A deterministic plant with a chosen breakaway PWM reproduces delayed starting;
its threshold is synthetic, not a measurement of the user's motors.
"""
import json
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]

def strip_includes(source):
    return '\n'.join(line for line in source.splitlines() if not line.startswith('#include'))

stubs = r'''
#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "motor.h"
#define ENCODER_ID_M1 1
#define portMAX_DELAY 0
#define pdMS_TO_TICKS(v) (v)
#define ESP_LOGI(...) ((void)0)
#define ESP_ERROR_CHECK(v) assert((v)==0)
#define configASSERT(v) assert(v)
#define MIN(a,b) ((a)<(b)?(a):(b))
#define MAX(a,b) ((a)>(b)?(a):(b))
#define ESP_GOTO_ON_FALSE(cond,err,label,...) do {if(!(cond)){ret=err;goto label;}}while(0)
#define ESP_GOTO_ON_ERROR(expr,label,...) do {ret=expr;if(ret)goto label;}while(0)
#define ESP_RETURN_ON_FALSE(cond,err,...) do {if(!(cond))return err;}while(0)
#include "pid_ctrl.h"
typedef int SemaphoreHandle_t;
typedef int TickType_t;
static int64_t fake_us;
static int counts[4], pwm[4];
static bool started[4];
static void xSemaphoreTake(int s,int wait) {(void)s;(void)wait;}
static void xSemaphoreGive(int s) {(void)s;}
static int xSemaphoreCreateMutex(void) {return 1;}
static int64_t esp_timer_get_time(void) {return fake_us;}
static int Encoder_Get_Count(int id) {return counts[id-1];}
static void Encoder_Init(void) {}
void PwmMotor_Init(void) {}
void PwmMotor_Stop(motor_id_t id, bool brake) {(void)brake;assert(id==MOTOR_ID_ALL);for(int i=0;i<4;++i)pwm[i]=0;}
void PwmMotor_Set_Speed(motor_id_t id,int speed) {pwm[id-1]=speed;}
static void vTaskDelay(int t) {(void)t;}
static int xTaskGetTickCount(void) {return 0;}
static void vTaskDelayUntil(int *t,int d) {(void)t;(void)d;}
static void vTaskDelete(void *p) {(void)p;}
static void xTaskCreatePinnedToCore(void (*f)(void*),const char*n,int s,void*a,int pr,void*o,int c)
{(void)f;(void)n;(void)s;(void)a;(void)pr;(void)o;(void)c;}
#define xPortGetCoreID() 1
'''
test = r'''
int main(int argc,char**argv) {
    bool stalled=argc>1;
    bool command_lost=argc>2;
    (void)argv;
    motor_mutex=1;
    const pid_ctrl_parameter_t parameters={.kp=1,.ki=.2,.kd=.2,
        .cal_type=PID_CAL_TYPE_INCREMENTAL,.max_output=PWM_MOTOR_MAX_VALUE,
        .min_output=-PWM_MOTOR_MAX_VALUE,.max_integral=1000,.min_integral=-1000};
    const pid_ctrl_config_t config={.init_param=parameters};
    for(int i=0;i<4;++i) assert(pid_new_control_block(&config,&pid_motor[i])==ESP_OK);
    int first_motion_ms=-1, stopped_ms=-1;
    for(int step=0;step<200;++step) {
        fake_us=10000+step*10000;
        if(step%5==0 && (!command_lost || step==0)) Motor_Set_Speed(.03f,.03f,.03f,.03f);
        for(int i=0;i<4;++i) {
            // Mirror the production integer PWM plus 200 tick dead-zone offset.
            int ticks=pwm[i] ? abs(pwm[i])+PWM_MOTOR_DEAD_ZONE : 0;
            if(!stalled && ticks>=220) started[i]=true;
            if(!stalled && started[i] && ticks>200) {
                counts[i]+=pwm[i]>0 ? 1 : -1;
                if(first_motion_ms<0)first_motion_ms=step*10;
            }
        }
        Motor_PID_Ctrl();
        if(command_lost && step>0 && !pid_enable && stopped_ms<0)stopped_ms=step*10;
    }
    printf("{\"first_motion_ms\":%d,\"stopped_ms\":%d,\"enabled\":%d,\"counts\":%d",first_motion_ms,stopped_ms,(int)pid_enable,counts[0]);
    puts("}");
    for(int i=0;i<4;++i)pid_del_control_block(pid_motor[i]);
}
'''
with tempfile.TemporaryDirectory(prefix='motor-startup-') as temp:
    directory=Path(temp)
    (directory/'esp_err.h').write_text('#pragma once\ntypedef int esp_err_t;\n#define ESP_OK 0\n#define ESP_ERR_INVALID_ARG 1\n#define ESP_ERR_NO_MEM 2\n')
    pid_source=strip_includes((ROOT/'firmware/chassis/components/espressif__pid_ctrl/src/pid_ctrl.c').read_text())
    versions={'baseline':subprocess.check_output(['git','show','4943022:firmware/chassis/components/motor/motor.c'],cwd=ROOT,text=True),
              'current':(ROOT/'firmware/chassis/components/motor/motor.c').read_text()}
    reports={}
    for name,source in versions.items():
        unit=directory/f'{name}.c';binary=directory/name
        unit.write_text(stubs+'\n#define TAG PID_TAG\n'+pid_source+'\n#undef TAG\n'+strip_includes(source)+test)
        command=['gcc','-std=c11','-Wall','-Wextra','-Werror','-Wno-unused-variable','-Wno-unused-parameter',
            '-I'+str(directory),'-I'+str(ROOT/'firmware/chassis/components/motor'),
            '-I'+str(ROOT/'firmware/chassis/components/pwm_motor'),
            '-I'+str(ROOT/'firmware/chassis/components/espressif__pid_ctrl/include')]
        subprocess.run(command+[str(unit),'-lm','-o',str(binary)],check=True)
        reports[name]={}
        for mode,args in [('delayed_start',[]),('stalled',['stalled']),('command_lost',['stalled','no-refresh'])]:
            reports[name][mode]=json.loads(subprocess.check_output([str(binary)]+args,text=True))
    print(json.dumps(reports,indent=2))
    assert reports['baseline']['delayed_start']['first_motion_ms']>300
    assert reports['current']['delayed_start']['first_motion_ms']>300, 'new protection prevents baseline delayed start'
    assert reports['current']['delayed_start']['enabled'] and reports['current']['delayed_start']['stopped_ms']==-1
    assert reports['current']['stalled']['enabled'] and reports['current']['stalled']['first_motion_ms']==-1
    assert reports['current']['delayed_start']==reports['baseline']['delayed_start']
    assert reports['current']['stalled']==reports['baseline']['stalled']
    assert reports['current']['command_lost']['stopped_ms']==480
    assert not reports['current']['command_lost']['enabled']
print('PASS: actual vendor PID delayed-start regression, removed feedback latches and command watchdog')
