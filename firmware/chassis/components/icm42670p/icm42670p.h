#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "imu_sample.h"

// I2C设备地址
#define ICM42670P_I2C_ADDR        0x68
#define IMU_GPIO_INT              41


typedef struct {
    int state;  // 0: starting, 1: configured, -1: initialization failed.
    uint8_t who_am_i;
    uint32_t read_errors;
} icm42670p_status_t;

// Atomic snapshot of the latest complete six-axis sample and health state.
void Icm42670p_Get_Snapshot(icm42670p_sample_t *sample, icm42670p_status_t *status);
void Icm42670p_Init(void);
void Icm42670p_Get_Accel_RawData(int16_t accel[3]);
void Icm42670p_Get_Gyro_RawData(int16_t gyro[3]);
int Icm42670p_Start_OK(void);

void Icm42670p_Get_Accel_g(float accel_g[3]);
void Icm42670p_Get_Gyro_dps(float gyro_dps[3]);
void Icm42670p_Get_Accel_Gyro_FSR(uint16_t *accel_fsr_g, uint16_t *gyro_fsr_dps);

#ifdef __cplusplus
}
#endif
