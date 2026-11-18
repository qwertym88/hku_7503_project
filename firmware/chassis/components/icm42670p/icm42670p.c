#include "icm42670p.h"

#include "stdio.h"
#include <inttypes.h>
#include <unistd.h>
#include "math.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_timer.h"

#include "esp_system.h"
#include "esp_log.h"
#include "driver/gpio.h"


#include "i2c_master.h"
#include "inv_imu_driver.h"



#define ENABLE_ICM42670P_DEBUG      0


static const char *TAG = "ICM42670P";


static portMUX_TYPE sample_lock = portMUX_INITIALIZER_UNLOCKED;
static icm42670p_sample_t latest_sample;
static icm42670p_status_t health;
static uint32_t sample_sequence;

static void set_start_state(int state)
{
    portENTER_CRITICAL(&sample_lock);
    health.state = state;
    if (state < 0) latest_sample.valid = false;
    portEXIT_CRITICAL(&sample_lock);
}


static int16_t  icm_accel[3];
static int16_t  icm_gyro[3];
static float    icm_accel_g[3];
static float    icm_gyro_dps[3];
static uint16_t icm_accel_fsr_g = 0;
static uint16_t icm_gyro_fsr_dps = 0;


struct inv_imu_serif icm_serif;
static struct inv_imu_device icm_driver;



// IMU读取数据接口
// IMU interface for reading data
int inv_io_hal_read_reg(struct inv_imu_serif *serif, uint8_t reg, uint8_t *rbuffer, uint32_t rlen)
{
	(void)serif;

	if (rlen > UINT16_MAX - 1)
		return -1; /* Not supported */

    return I2C_Master_Read(ICM42670P_I2C_ADDR, reg, (uint16_t)rlen, rbuffer) == ESP_OK ? 0 : -1;
}

// IMU写入数据接口
// IMU interface for writing data
int inv_io_hal_write_reg(struct inv_imu_serif *serif, uint8_t reg, const uint8_t *wbuffer, uint32_t wlen)
{
	(void)serif;

	if (wlen > UINT16_MAX - 1)
		return -1; /* Not supported */

    return I2C_Master_Write(ICM42670P_I2C_ADDR, reg, (uint16_t)wlen,
                            (uint8_t *)wbuffer) == ESP_OK ? 0 : -1;
}


// 微秒级延时函数
// Microsecond delay function
void inv_imu_sleep_us(uint32_t us)
{
	usleep(us);
}

// 获取时间
// Get time
uint64_t inv_imu_get_time_us(void)
{
	return (uint64_t)esp_timer_get_time();
}

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

// Poll the FIFO in a dedicated task: no interrupt shared with motor control.
/*
    GYRO_CONFIG寄存器
    陀螺仪量程为:±250 dps     获取到的陀螺仪数据除以131           可以转化为带物理单位的数据，单位为：°/s
    陀螺仪量程为:±500 dps     获取到的陀螺仪数据除以65.5          可以转化为带物理单位的数据，单位为：°/s
    陀螺仪量程为:±1000dps     获取到的陀螺仪数据除以32.8          可以转化为带物理单位的数据，单位为：°/s
    陀螺仪量程为:±2000dps     获取到的陀螺仪数据除以16.4          可以转化为带物理单位的数据，单位为：°/s

    ACCEL_CONFIG寄存器
    加速度计量程为:±2g        获取到的加速度计数据 除以16384      可以转化为带物理单位的数据，单位：g(m/s^2)
    加速度计量程为:±4g        获取到的加速度计数据 除以8192       可以转化为带物理单位的数据，单位：g(m/s^2)
    加速度计量程为:±8g        获取到的加速度计数据 除以4096       可以转化为带物理单位的数据，单位：g(m/s^2)
    加速度计量程为:±16g       获取到的加速度计数据 除以2048       可以转化为带物理单位的数据，单位：g(m/s^2)
*/
// 获取加速度计和陀螺仪的量程
// Get the range of the accelerometer and gyroscope
static int get_accel_and_gyr_fsr(uint16_t *accel_fsr_g, uint16_t *gyro_fsr_dps)
{
	ACCEL_CONFIG0_FS_SEL_t accel_fsr_bitfield;
	GYRO_CONFIG0_FS_SEL_t  gyro_fsr_bitfield;

	if (inv_imu_get_accel_fsr(&icm_driver, &accel_fsr_bitfield)) return -1;
	switch (accel_fsr_bitfield) {
	case ACCEL_CONFIG0_FS_SEL_2g:
		*accel_fsr_g = 2;
		break;
	case ACCEL_CONFIG0_FS_SEL_4g:
		*accel_fsr_g = 4;
		break;
	case ACCEL_CONFIG0_FS_SEL_8g:
		*accel_fsr_g = 8;
		break;
	case ACCEL_CONFIG0_FS_SEL_16g:
		*accel_fsr_g = 16;
		break;
	default:
		return -1;
	}

	if (inv_imu_get_gyro_fsr(&icm_driver, &gyro_fsr_bitfield)) return -1;
	switch (gyro_fsr_bitfield) {
	case GYRO_CONFIG0_FS_SEL_250dps:
		*gyro_fsr_dps = 250;
		break;
	case GYRO_CONFIG0_FS_SEL_500dps:
		*gyro_fsr_dps = 500;
		break;
	case GYRO_CONFIG0_FS_SEL_1000dps:
		*gyro_fsr_dps = 1000;
		break;
	case GYRO_CONFIG0_FS_SEL_2000dps:
		*gyro_fsr_dps = 2000;
		break;
	default:
		return -1;
	}
	return 0;
}

// IMU读取数据完成后回调函数，在此函数内提取IMU原始数据并根据量程换算单位。
// After the IMU has read the data, it calls back the function.
// in which the original IMU data is extracted and the units are converted according to the range.
static void imu_callback(inv_imu_sensor_event_t *event)
{
    const int required = (1 << INV_SENSOR_ACCEL) | (1 << INV_SENSOR_GYRO);
    if ((event->sensor_mask & required) != required) return;
    icm42670p_sample_t sample;
    if (!imu_sample_from_raw(&sample, event->accel, event->gyro,
                             icm_accel_fsr_g, icm_gyro_fsr_dps,
                             esp_timer_get_time(), sample_sequence + 1)) return;
    ++sample_sequence;
    portENTER_CRITICAL(&sample_lock);
    latest_sample = sample;
    for (int i = 0; i < 3; ++i) {
        icm_accel[i] = event->accel[i];
        icm_gyro[i] = event->gyro[i];
        icm_accel_g[i] = sample.accel_mps2[i];
        icm_gyro_dps[i] = sample.gyro_rads[i];
    }
    portEXIT_CRITICAL(&sample_lock);
}

// 配置IMU结构体相关信息
// Configure IMU structure information
static int setup_mcu(struct inv_imu_serif *icm_serif)
{
	int rc = 0;

	/* Initialize serial interface between MCU and IMU */
	icm_serif->context    = 0; /* no need */
	icm_serif->read_reg   = inv_io_hal_read_reg;
	icm_serif->write_reg  = inv_io_hal_write_reg;
	icm_serif->max_read   = 1024 * 32; /* maximum number of bytes allowed per serial read */
	icm_serif->max_write  = 1024 * 32; /* maximum number of bytes allowed per serial write */
	icm_serif->serif_type = UI_I2C;    /* I2C */
	return rc;
}

// 设置IMU设备，检测芯片ID是否符合要求
// Set the IMU and check whether the chip ID meets the requirements
static int setup_imu_device(const struct inv_imu_serif *icm_serif)
{
	int     rc = 0;
	uint8_t who_am_i = 0;

	/* Init device */
	rc = inv_imu_init(&icm_driver, icm_serif, imu_callback);
	if (rc != INV_ERROR_SUCCESS) {
		ESP_LOGE(TAG, "Failed to initialize IMU!");
		return rc;
	}

	/* Check WHOAMI */
	rc = inv_imu_get_who_am_i(&icm_driver, &who_am_i);
	if (rc != INV_ERROR_SUCCESS) {
		ESP_LOGE(TAG, "Failed to read whoami!");
		return rc;
	}

    portENTER_CRITICAL(&sample_lock);
    health.who_am_i = who_am_i;
    portEXIT_CRITICAL(&sample_lock);
	if (who_am_i != ICM_WHOAMI) {
		ESP_LOGE(TAG, "Bad WHOAMI value!");
		ESP_LOGE(TAG, "Read 0x%02x, expected 0x%02x", who_am_i, ICM_WHOAMI);
		return INV_ERROR;
	}
    ESP_LOGI(TAG, "Read ICM_WHOAMI: 0x%02x", who_am_i);
	return rc;
}

// 配置IMU设备，写入相关参数
// Configure the IMU and write related parameters
static int configure_imu_device(void)
{
	int rc = 0;
    // Low resolution mode
    rc |= inv_imu_set_accel_fsr(&icm_driver, ACCEL_CONFIG0_FS_SEL_4g);
    rc |= inv_imu_set_gyro_fsr(&icm_driver, GYRO_CONFIG0_FS_SEL_2000dps);

    // low-noise mode
    rc |= inv_imu_set_accel_frequency(&icm_driver, ACCEL_CONFIG0_ODR_400_HZ);
    rc |= inv_imu_set_gyro_frequency(&icm_driver, GYRO_CONFIG0_ODR_400_HZ);
    rc |= inv_imu_enable_accel_low_noise_mode(&icm_driver);

	rc |= inv_imu_enable_gyro_low_noise_mode(&icm_driver);

	return rc;
}

// IMU自检测功能
// IMU self-test function
static int run_self_test(void)
{
	inv_imu_selftest_output_t     out;
	inv_imu_selftest_parameters_t params;
	int                           rc   = INV_ERROR_SUCCESS;
	static int                    iter = 0;

	rc |= inv_imu_init_selftest_parameters_struct(&icm_driver, &params);
	/* Update `params` if needed here */

	rc |= inv_imu_run_selftest(&icm_driver, params, &out);

	if (rc != INV_ERROR_SUCCESS)
    {
		ESP_LOGE(TAG, "[%u] An error occurred while running selftest (rc=%d)", iter, rc);
	}
    else
    {
		/* Print self-test status */
		ESP_LOGI(TAG, "[%u] Accel self-test %s", iter, out.accel_status == 1 ? "OK" : "KO");
		if (out.accel_status != 1) {
			ESP_LOGI(TAG, "  - Accel X: %s", out.ax_status == 1 ? "OK" : "KO");
			ESP_LOGI(TAG, "  - Accel Y: %s", out.ay_status == 1 ? "OK" : "KO");
			ESP_LOGI(TAG, "  - Accel Z: %s", out.az_status == 1 ? "OK" : "KO");
			rc |= INV_ERROR;
		}

		ESP_LOGI(TAG, "[%u] Gyro self-test %s", iter, out.gyro_status == 1 ? "OK" : "KO");
		if (out.gyro_status != 1) {
			ESP_LOGI(TAG, "  - Gyro X: %s", out.gx_status == 1 ? "OK" : "KO");
			ESP_LOGI(TAG, "  - Gyro Y: %s", out.gy_status == 1 ? "OK" : "KO");
			ESP_LOGI(TAG, "  - Gyro Z: %s", out.gz_status == 1 ? "OK" : "KO");
			rc |= INV_ERROR;
		}

		/* Check incomplete state */
		if (out.gyro_status & 0x2) {
			ESP_LOGE(TAG, "[%u] Gyro self-test are incomplete.", iter);
			rc |= INV_ERROR;
		}
	}

	iter++;
	return rc;
}


// 更新IMU数据
// Update IMU data
static int Icm42670p_Update_Imu_Data(void)
{
	return inv_imu_get_data_from_fifo(&icm_driver);
}


// ICM42670P读取数据的任务
// ICM42670P Data reading task
static void Icm42670p_Task(void *arg)
{
    int rc = I2C_Master_Init() == ESP_OK ? 0 : -1;
    if (!rc) rc = setup_mcu(&icm_serif);
    if (!rc) rc = setup_imu_device(&icm_serif);
    if (!rc) rc = run_self_test();
    if (!rc) rc = configure_imu_device();
    if (!rc) rc = get_accel_and_gyr_fsr(&icm_accel_fsr_g, &icm_gyro_fsr_dps);
    if (rc) {
        set_start_state(-1);
        vTaskDelete(NULL);
        return;
    }
    set_start_state(1);
    TickType_t wake = xTaskGetTickCount();
    while (1) {
        rc = Icm42670p_Update_Imu_Data();
        if (rc < 0) {
            portENTER_CRITICAL(&sample_lock);
            ++health.read_errors;
            latest_sample.valid = false;
            portEXIT_CRITICAL(&sample_lock);
        }
        // Sleep after each read; an I2C fault must not trigger catch-up bursts.
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(5));
        if (rc < 0) wake = xTaskGetTickCount();
    }
}

void Icm42670p_Get_Snapshot(icm42670p_sample_t *sample, icm42670p_status_t *status)
{
    portENTER_CRITICAL(&sample_lock);
    *sample = latest_sample;
    *status = health;
    portEXIT_CRITICAL(&sample_lock);
}

// 获取加速度计的原始数据
// Get the raw accelerometer data
void Icm42670p_Get_Accel_RawData(int16_t accel[3])
{
    portENTER_CRITICAL(&sample_lock);
    accel[0] = icm_accel[0];
    accel[1] = icm_accel[1];
    accel[2] = icm_accel[2];
    portEXIT_CRITICAL(&sample_lock);
}

// 获取陀螺仪的原始数据
// Get the raw data of the gyroscope
void Icm42670p_Get_Gyro_RawData(int16_t gyro[3])
{
    portENTER_CRITICAL(&sample_lock);
    gyro[0] = icm_gyro[0];
    gyro[1] = icm_gyro[1];
    gyro[2] = icm_gyro[2];
    portEXIT_CRITICAL(&sample_lock);
}

// 获取加速度计缩放后的数据
// Get the accelerometer scaled data
void Icm42670p_Get_Accel_g(float accel_g[3])
{
    portENTER_CRITICAL(&sample_lock);
    accel_g[0] = icm_accel_g[0];
    accel_g[1] = icm_accel_g[1];
    accel_g[2] = icm_accel_g[2];
    portEXIT_CRITICAL(&sample_lock);
}

// 获取陀螺仪的缩放后的数据
// Get the zoom data of the gyroscope
void Icm42670p_Get_Gyro_dps(float gyro_dps[3])
{
    portENTER_CRITICAL(&sample_lock);
    gyro_dps[0] = icm_gyro_dps[0];
    gyro_dps[1] = icm_gyro_dps[1];
    gyro_dps[2] = icm_gyro_dps[2];
    portEXIT_CRITICAL(&sample_lock);
}

// IMU初始化成功返回1，失败返回-1，正在初始化返回0
// Returns 1 when IMU initializing successfully, returns -1 when failed, and returns 0 when initializing
int Icm42670p_Start_OK(void)
{
    portENTER_CRITICAL(&sample_lock);
    int state = health.state;
    portEXIT_CRITICAL(&sample_lock);
    return state;
}

// 获取加速度和陀螺仪的量程
// Get the acceleration and gyroscope FSR
void Icm42670p_Get_Accel_Gyro_FSR(uint16_t *accel_fsr_g, uint16_t *gyro_fsr_dps)
{
    portENTER_CRITICAL(&sample_lock);
    *accel_fsr_g = icm_accel_fsr_g;
    *gyro_fsr_dps = icm_gyro_fsr_dps;
    portEXIT_CRITICAL(&sample_lock);
}

// 初始化ICM42670P Initialize ICM42670P
void Icm42670p_Init(void)
{
    if (xTaskCreatePinnedToCore(Icm42670p_Task, "Icm42670p_Task", 6*1024,
                                NULL, 5, NULL, 0) != pdPASS)
        set_start_state(-1);
}
