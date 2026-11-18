#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "stdbool.h"
#include "stdint.h"
#include "pwm_motor.h"

// 电机数量
#define MOTOR_MAX_NUM                   (4)
// 电机转动一圈产生的脉冲数量：13*20*4
#define MOTOR_ENCODER_CIRCLE            (1040)
// 轮子周长，单位：mm
#define MOTOR_WHEEL_CIRCLE              (65.0f * 3.14159265358979323846f)
// PID算法计算周期，单位：ms
#define MOTOR_PID_PERIOD                (10)
// 设置电机最大速度，单位：ms/s。
#define MOTOR_MAX_SPEED                 (0.20f)

// Check in the independent motor task; leaves one 10ms cycle of margin below 0.5s.
#define MOTOR_COMMAND_TIMEOUT_US        (480000)



void Motor_Init(void);
void Motor_Set_Speed(float speed_m1, float speed_m2, float speed_m3, float speed_m4);
void Motor_Get_Speed(float* speed_m1, float* speed_m2, float* speed_m3, float* speed_m4);
void Motor_Stop(bool brake);

void Motor_Update_PID_Parm(float pid_p, float pid_i, float pid_d);
void Motor_Read_PID_Parm(float* out_p, float* out_i, float* out_d);

typedef struct {
    bool enabled;
    int64_t command_age_us;
    uint32_t control_cycles;
    int counts[4];
    float target_counts[4];
    float speed[4];
    float pid_output[4];
} motor_diagnostics_t;
void Motor_Get_Diagnostics(motor_diagnostics_t *out);


#ifdef __cplusplus
}
#endif
