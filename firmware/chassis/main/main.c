// Chassis bring-up: raw wheel odometry, onboard IMU and /cmd_vel, without TF or lift.
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "car_motion.h"
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include "icm42670p.h"
#include <rosidl_runtime_c/string_functions.h>
#include <std_msgs/msg/string.h>
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

bool chassis_serial_init(void);
static nav_msgs__msg__Odometry odom;
static geometry_msgs__msg__Twist command;
static rcl_publisher_t publisher;
static int64_t previous_update_us;
static double x, y, yaw;
static bool fault;
static rcl_publisher_t diagnostic_publisher;
static std_msgs__msg__String diagnostic;
static uint32_t command_count;
static uint32_t rejected_commands;
static adc_oneshot_unit_handle_t battery_adc;
static adc_cali_handle_t battery_calibration;
static rcl_publisher_t imu_publisher;
static sensor_msgs__msg__Imu imu;
static uint32_t imu_last_sequence;
static uint32_t imu_published;

static void imu_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    icm42670p_sample_t sample;
    icm42670p_status_t status;
    Icm42670p_Get_Snapshot(&sample, &status);
    int64_t now_us = esp_timer_get_time();
    // Never publish a startup zero, duplicate, failed or stale sample as fresh.
    if (status.state != 1 ||
        !imu_sample_is_fresh(&sample, now_us, 100000, imu_last_sequence)) return;
    int64_t epoch_ns = rmw_uros_epoch_nanos();
    // Stamp at FIFO receipt time, rather than hiding publication delay.
    int64_t stamp_ns = epoch_ns - (now_us - sample.time_us) * 1000LL;
    if (stamp_ns <= 0) return;
    imu.header.stamp.sec = stamp_ns / 1000000000LL;
    imu.header.stamp.nanosec = stamp_ns % 1000000000LL;
    imu.angular_velocity.x = sample.gyro_rads[0];
    imu.angular_velocity.y = sample.gyro_rads[1];
    imu.angular_velocity.z = sample.gyro_rads[2];
    imu.linear_acceleration.x = sample.accel_mps2[0];
    imu.linear_acceleration.y = sample.accel_mps2[1];
    imu.linear_acceleration.z = sample.accel_mps2[2];
    if (rcl_publish(&imu_publisher, &imu, NULL) != RCL_RET_OK) {
        fault = true;
        return;
    }
    imu_last_sequence = sample.sequence;
    ++imu_published;
}

static void battery_init(void)
{
    adc_oneshot_unit_init_cfg_t unit = {.unit_id = ADC_UNIT_1};
    adc_oneshot_chan_cfg_t channel = {
        .atten = ADC_ATTEN_DB_11, .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (adc_oneshot_new_unit(&unit, &battery_adc) != ESP_OK) return;
    if (adc_oneshot_config_channel(battery_adc, ADC_CHANNEL_2, &channel) != ESP_OK) {
        adc_oneshot_del_unit(battery_adc);
        battery_adc = NULL;
        return;
    }
    adc_cali_curve_fitting_config_t calibration = {
        .unit_id = ADC_UNIT_1, .chan = ADC_CHANNEL_2,
        .atten = ADC_ATTEN_DB_11, .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (adc_cali_create_scheme_curve_fitting(&calibration, &battery_calibration) != ESP_OK)
        battery_calibration = NULL;
}

static void publish_diagnostics(void)
{
    motor_diagnostics_t motor;
    Motor_Get_Diagnostics(&motor);
    icm42670p_sample_t sample;
    icm42670p_status_t imu_status;
    Icm42670p_Get_Snapshot(&sample, &imu_status);
    int64_t imu_age_ms = sample.sequence ?
        (esp_timer_get_time() - sample.time_us) / 1000 : -1;
    int raw = -1, millivolts = -1;
    if (battery_adc && adc_oneshot_read(battery_adc, ADC_CHANNEL_2, &raw) == ESP_OK &&
        battery_calibration)
        adc_cali_raw_to_voltage(battery_calibration, raw, &millivolts);
    // Yahboom board GPIO3 voltage divider: battery V = ADC millivolts * 4.03 / 1000.
    // This observes the voltage sense net; it does not prove motor wiring/power is good.
    int length = snprintf(diagnostic.data.data, diagnostic.data.capacity,
        "{\"rx\":%lu,\"rejected\":%lu,\"enabled\":%d,\"age_ms\":%lld,\"cycles\":%lu,"
        "\"battery_v\":%.3f,\"adc_raw\":%d,"
        "\"counts\":[%d,%d,%d,%d],\"target\":[%.2f,%.2f,%.2f,%.2f],"
        "\"speed\":[%.4f,%.4f,%.4f,%.4f],\"pid_output\":[%.1f,%.1f,%.1f,%.1f],"
        "\"imu_state\":%d,\"imu_whoami\":%u,\"imu_valid\":%d,"
        "\"imu_age_ms\":%lld,\"imu_errors\":%lu,\"imu_published\":%lu}",
        (unsigned long)command_count, (unsigned long)rejected_commands, motor.enabled,
        (long long)(motor.command_age_us / 1000), (unsigned long)motor.control_cycles,
        millivolts < 0 ? -1.0 : millivolts * .00403, raw,
        motor.counts[0], motor.counts[1], motor.counts[2], motor.counts[3],
        motor.target_counts[0], motor.target_counts[1], motor.target_counts[2], motor.target_counts[3],
        motor.speed[0], motor.speed[1], motor.speed[2], motor.speed[3],
        motor.pid_output[0], motor.pid_output[1], motor.pid_output[2], motor.pid_output[3],
        imu_status.state, (unsigned)imu_status.who_am_i,
        sample.valid && imu_age_ms >= 0 && imu_age_ms <= 100,
        (long long)imu_age_ms, (unsigned long)imu_status.read_errors,
        (unsigned long)imu_published);
    if (length > 0 && (size_t)length < diagnostic.data.capacity) {
        diagnostic.data.size = length;
        // Debug telemetry must not stop the robot just because a subscriber is absent.
        rcl_ret_t rc = rcl_publish(&diagnostic_publisher, &diagnostic, NULL);
        if (rc != RCL_RET_OK) rcl_reset_error();
    }
}

// On communication failure remain stopped until a physical reset.
static void halt(void)
{
    Motor_Stop(STOP_BRAKE);
    vTaskDelete(NULL);
}
#define CHECK(call) do { if ((call) != RCL_RET_OK) halt(); } while (0)

static void command_callback(const void *input)
{
    const geometry_msgs__msg__Twist *msg = input;
    ++command_count;
    if (!isfinite(msg->linear.x) || !isfinite(msg->linear.y) ||
        !isfinite(msg->angular.z) || msg->linear.y != 0.0) {
        ++rejected_commands;
        Motor_Stop(STOP_BRAKE);
        return;
    }
    // Initial bench limits. Revisit only after wheel directions and stopping tests.
    float vx = fmaxf(-0.10f, fminf(0.10f, msg->linear.x));
    float wz = fmaxf(-0.40f, fminf(0.40f, msg->angular.z));
    if (vx == 0.0f && wz == 0.0f) Motor_Stop(STOP_BRAKE);
    else Motion_Ctrl(vx, 0.0f, wz);
}

static void odom_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    int64_t now = esp_timer_get_time();
    double dt = (now - previous_update_us) / 1e6;
    previous_update_us = now;
    car_motion_t speed;
    Motion_Get_Speed(&speed);
    double turn = speed.Wz * dt;
    // Midpoint integration; geometry and feedback all stay in firmware.
    x += speed.Vx * cos(yaw + turn / 2.0) * dt;
    y += speed.Vx * sin(yaw + turn / 2.0) * dt;
    yaw = remainder(yaw + turn, 2.0 * M_PI);
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.orientation.z = sin(yaw / 2.0);
    odom.pose.pose.orientation.w = cos(yaw / 2.0);
    odom.twist.twist.linear.x = speed.Vx;
    odom.twist.twist.angular.z = speed.Wz;
    int64_t epoch_ns = rmw_uros_epoch_nanos();
    odom.header.stamp.sec = epoch_ns / 1000000000LL;
    odom.header.stamp.nanosec = epoch_ns % 1000000000LL;
    if (rcl_publish(&publisher, &odom, NULL) != RCL_RET_OK) fault = true;
    static unsigned diagnostic_divider;
    if (++diagnostic_divider >= 10) {
        diagnostic_divider = 0;
        publish_diagnostics();
    }
}

void app_main(void)
{
    Motor_Init();
    Motor_Stop(STOP_BRAKE);
    battery_init();
    Icm42670p_Init();
    if (!chassis_serial_init()) halt();
    while (rmw_uros_ping_agent(100, 1) != RMW_RET_OK)
        vTaskDelay(pdMS_TO_TICKS(100));

    rcl_allocator_t allocator = rcl_get_default_allocator();
    rcl_init_options_t options = rcl_get_zero_initialized_init_options();
    CHECK(rcl_init_options_init(&options, allocator));
    CHECK(rcl_init_options_set_domain_id(&options, CONFIG_CHASSIS_ROS_DOMAIN_ID));
    rclc_support_t support;
    CHECK(rclc_support_init_with_options(&support, 0, NULL, &options, &allocator));
    CHECK(rcl_init_options_fini(&options));
    if (rmw_uros_sync_session(1000) != RMW_RET_OK || !rmw_uros_epoch_synchronized()) halt();

    rcl_node_t node = rcl_get_zero_initialized_node();
    CHECK(rclc_node_init_default(&node, "chassis", "", &support));
    publisher = rcl_get_zero_initialized_publisher();
    CHECK(rclc_publisher_init_default(&publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "/odom"));
    diagnostic_publisher = rcl_get_zero_initialized_publisher();
    CHECK(rclc_publisher_init_best_effort(&diagnostic_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/chassis/diagnostics"));
    imu_publisher = rcl_get_zero_initialized_publisher();
    CHECK(rclc_publisher_init_best_effort(&imu_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "/imu/data"));
    if (!sensor_msgs__msg__Imu__init(&imu) ||
        !rosidl_runtime_c__String__assign(&imu.header.frame_id, "imu_link")) halt();
    imu.orientation.w = 1.0;
    // No attitude estimate. All-zero gyro/accel covariance means unknown until measured.
    imu.orientation_covariance[0] = -1.0;
    char diagnostic_storage[896];
    memset(diagnostic_storage, ' ', sizeof(diagnostic_storage) - 1);
    diagnostic_storage[sizeof(diagnostic_storage) - 1] = '\0';
    if (!std_msgs__msg__String__init(&diagnostic) ||
        !rosidl_runtime_c__String__assign(&diagnostic.data, diagnostic_storage)) halt();
    rcl_subscription_t subscription = rcl_get_zero_initialized_subscription();
    CHECK(rclc_subscription_init_default(&subscription, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/cmd_vel"));
    if (!nav_msgs__msg__Odometry__init(&odom) || !geometry_msgs__msg__Twist__init(&command) ||
        !rosidl_runtime_c__String__assign(&odom.header.frame_id, "odom") ||
        !rosidl_runtime_c__String__assign(&odom.child_frame_id, "base_footprint")) halt();
    // Conservative initial uncertainties, to be replaced by measured values.
    for (int i = 0; i < 6; ++i) {
        odom.pose.covariance[7 * i] = (i == 0 || i == 1 || i == 5) ? 0.05 : 1e6;
        odom.twist.covariance[7 * i] = (i == 0 || i == 5) ? 0.05 : 1e6;
    }
    rcl_timer_t timer = rcl_get_zero_initialized_timer();
    CHECK(rclc_timer_init_default2(&timer, &support, RCL_MS_TO_NS(50), odom_callback, true));
    rcl_timer_t imu_timer = rcl_get_zero_initialized_timer();
    CHECK(rclc_timer_init_default2(&imu_timer, &support, RCL_MS_TO_NS(20), imu_callback, true));
    rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
    CHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
    CHECK(rclc_executor_add_subscription(&executor, &subscription, &command,
                                         command_callback, ON_NEW_DATA));
    CHECK(rclc_executor_add_timer(&executor, &timer));
    CHECK(rclc_executor_add_timer(&executor, &imu_timer));
    previous_update_us = esp_timer_get_time();
    int64_t health_check_us = previous_update_us;
    while (!fault) {
        rcl_ret_t rc = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
        if (rc != RCL_RET_OK && rc != RCL_RET_TIMEOUT) break;
        if (esp_timer_get_time() - health_check_us >= 1000000) {
            if (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) break;
            health_check_us = esp_timer_get_time();
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    halt();
}
