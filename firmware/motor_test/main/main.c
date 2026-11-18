// Bench-only adaptation of Yahboom's esp32_samples/{pwm_motor,encoder}.
// No Motor_Init(): its PID task would overwrite direct PWM commands every 10 ms.
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "encoder.h"
#include "pwm_motor.h"
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <rosidl_runtime_c/primitives_sequence_functions.h>
#include <rosidl_runtime_c/string_functions.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <std_msgs/msg/string.h>

#define WHEELS 4
#define INPUT_LIMIT 100
#define COMMAND_TIMEOUT_US 480000
#define BURST_LIMIT_US 1000000

bool chassis_serial_init(void);

static SemaphoreHandle_t mutex;
static struct {
    uint32_t rx, rejected, cycles;
    bool enabled, latched, fault;
    int64_t last_command_us, burst_start_us;
    int requested[WHEELS], applied[WHEELS], counts[WHEELS];
} state = {.latched = true};
static rcl_publisher_t publisher;
static std_msgs__msg__String diagnostic;
static std_msgs__msg__Int32MultiArray command;

static void halt(void)
{
    xSemaphoreTake(mutex, portMAX_DELAY);
    state.fault = true;
    state.enabled = false;
    state.latched = true;
    xSemaphoreGive(mutex);
    // The independent task keeps applying coast even if ROS has failed.
    vTaskDelete(NULL);
}
#define CHECK(call) do { if ((call) != RCL_RET_OK) halt(); } while (0)

static void motor_task(void *arg)
{
    TickType_t wake = xTaskGetTickCount();
    while (true) {
        xSemaphoreTake(mutex, portMAX_DELAY);
        int64_t now = esp_timer_get_time();
        ++state.cycles;
        if (state.enabled &&
            (now - state.last_command_us >= COMMAND_TIMEOUT_US ||
             now - state.burst_start_us >= BURST_LIMIT_US)) {
            state.enabled = false;
            state.latched = true;
        }
        if (state.fault) state.enabled = false;
        if (state.enabled) {
            for (int i = 0; i < WHEELS; ++i) {
                PwmMotor_Set_Speed(MOTOR_ID_M1 + i, state.requested[i]);
                state.applied[i] = state.requested[i];
            }
        } else {
            // Coast permits a separate manual encoder test with no PWM drive.
            PwmMotor_Stop(MOTOR_ID_ALL, STOP_COAST);
            memset(state.applied, 0, sizeof(state.applied));
        }
        for (int i = 0; i < WHEELS; ++i)
            state.counts[i] = Encoder_Get_Count(ENCODER_ID_M1 + i);
        xSemaphoreGive(mutex);
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(10));
    }
}

static void command_callback(const void *input)
{
    const std_msgs__msg__Int32MultiArray *msg = input;
    int nonzero = 0;
    bool valid = msg->data.size == WHEELS &&
                 msg->layout.dim.size == 0 && msg->layout.data_offset == 0;
    if (valid) {
        for (int i = 0; i < WHEELS; ++i) {
            if (msg->data.data[i] < -INPUT_LIMIT || msg->data.data[i] > INPUT_LIMIT)
                valid = false;
            if (msg->data.data[i] != 0) ++nonzero;
        }
    }
    xSemaphoreTake(mutex, portMAX_DELAY);
    ++state.rx;
    if (!valid || nonzero > 1 || state.fault) {
        ++state.rejected;
        state.enabled = false;
        state.latched = true;
    } else if (nonzero == 0) {
        state.enabled = false;
        state.latched = false;
        state.burst_start_us = 0;
        memset(state.requested, 0, sizeof(state.requested));
        state.last_command_us = esp_timer_get_time();
    } else if (state.latched ||
               (state.enabled && memcmp(state.requested, msg->data.data,
                                        sizeof(state.requested)) != 0)) {
        // Require a zero between pulses, wheel changes, and direction changes.
        ++state.rejected;
        state.enabled = false;
        state.latched = true;
    } else {
        int64_t now = esp_timer_get_time();
        if (!state.enabled) state.burst_start_us = now;
        state.last_command_us = now;
        for (int i = 0; i < WHEELS; ++i) state.requested[i] = msg->data.data[i];
        state.enabled = true;
    }
    xSemaphoreGive(mutex);
}

static void publish_diagnostics(rcl_timer_t *timer, int64_t last_call_time)
{
    xSemaphoreTake(mutex, portMAX_DELAY);
    int ticks[WHEELS];
    for (int i = 0; i < WHEELS; ++i) {
        int input = state.applied[i];
        ticks[i] = input == 0 ? 0 : input + (input > 0 ? 1 : -1) * PWM_MOTOR_DEAD_ZONE;
    }
    int length = snprintf(diagnostic.data.data, diagnostic.data.capacity,
        "{\"mode\":\"open_loop_test\",\"rx\":%lu,\"rejected\":%lu,"
        "\"enabled\":%d,\"latched\":%d,\"fault\":%d,\"cycles\":%lu,\"age_ms\":%lld,"
        "\"counts\":[%d,%d,%d,%d],\"pwm_input\":[%d,%d,%d,%d],"
        "\"pwm_ticks\":[%d,%d,%d,%d],\"pwm_period_ticks\":%d}",
        (unsigned long)state.rx, (unsigned long)state.rejected,
        state.enabled, state.latched, state.fault, (unsigned long)state.cycles,
        (long long)(state.last_command_us ?
            (esp_timer_get_time() - state.last_command_us) / 1000 : -1),
        state.counts[0], state.counts[1], state.counts[2], state.counts[3],
        state.applied[0], state.applied[1], state.applied[2], state.applied[3],
        ticks[0], ticks[1], ticks[2], ticks[3], PWM_MOTOR_DUTY_TICK_MAX);
    xSemaphoreGive(mutex);
    if (length > 0 && (size_t)length < diagnostic.data.capacity) {
        diagnostic.data.size = length;
        // These are software commands, not measured pin duty or motor voltage.
        rcl_ret_t rc = rcl_publish(&publisher, &diagnostic, NULL);
        if (rc != RCL_RET_OK) rcl_reset_error();
    }
}

void app_main(void)
{
    mutex = xSemaphoreCreateMutex();
    configASSERT(mutex);
    Encoder_Init();
    PwmMotor_Init();
    PwmMotor_Stop(MOTOR_ID_ALL, STOP_COAST);
    if (xTaskCreatePinnedToCore(motor_task, "motor_bench", 4096, NULL, 10, NULL, 1)
        != pdPASS) halt();
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
    rcl_node_t node = rcl_get_zero_initialized_node();
    CHECK(rclc_node_init_default(&node, "chassis_motor_test", "", &support));
    publisher = rcl_get_zero_initialized_publisher();
    CHECK(rclc_publisher_init_best_effort(&publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/chassis/diagnostics"));
    rcl_subscription_t subscription = rcl_get_zero_initialized_subscription();
    CHECK(rclc_subscription_init_default(&subscription, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray), "/chassis/motor_pwm"));
    char storage[640];
    memset(storage, ' ', sizeof(storage) - 1);
    storage[sizeof(storage) - 1] = '\0';
    if (!std_msgs__msg__String__init(&diagnostic) ||
        !rosidl_runtime_c__String__assign(&diagnostic.data, storage) ||
        !std_msgs__msg__Int32MultiArray__init(&command) ||
        !rosidl_runtime_c__int32__Sequence__init(&command.data, WHEELS)) halt();
    rcl_timer_t timer = rcl_get_zero_initialized_timer();
    CHECK(rclc_timer_init_default2(&timer, &support, RCL_MS_TO_NS(100),
                                 publish_diagnostics, true));
    rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
    CHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    CHECK(rclc_executor_add_subscription(&executor, &subscription, &command,
                                        command_callback, ON_NEW_DATA));
    CHECK(rclc_executor_add_timer(&executor, &timer));
    int64_t health_check_us = esp_timer_get_time();
    while (true) {
        rcl_ret_t rc = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
        if (rc != RCL_RET_OK && rc != RCL_RET_TIMEOUT) halt();
        if (esp_timer_get_time() - health_check_us >= 1000000) {
            if (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) halt();
            health_check_us = esp_timer_get_time();
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
