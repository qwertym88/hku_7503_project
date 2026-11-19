#include "lifter.h"
#include <math.h>
#include "sdkconfig.h"
#include "driver/mcpwm_prelude.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "motor.h"

#define LIFTER_MAIN_LEASE_US 480000
#define LIFTER_WHEELS_STILL_US 200000

static lifter_control_t control;
static SemaphoreHandle_t mutex;
static int64_t heartbeat_us, still_since_us, wheel_inhibit_until_us;
static mcpwm_cmpr_handle_t comparator;
static mcpwm_gen_handle_t generator;
static bool pwm_ready;
static uint16_t applied_pulse;

static esp_err_t pwm_init(void)
{
    mcpwm_timer_handle_t timer = NULL;
    mcpwm_oper_handle_t oper = NULL;
    mcpwm_timer_config_t timer_config = {
        .group_id = 1, .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = 1000000, .period_ticks = 20000,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    mcpwm_operator_config_t oper_config = {.group_id = 1};
    mcpwm_comparator_config_t cmp_config = {.flags.update_cmp_on_tez = true};
    mcpwm_generator_config_t gen_config = {
        .gen_gpio_num = CONFIG_CHASSIS_LIFTER_PORT == 1 ? 8 : 21,
    };
#define TRY(expr) do { esp_err_t rc = (expr); if (rc != ESP_OK) return rc; } while (0)
    TRY(mcpwm_new_timer(&timer_config, &timer));
    TRY(mcpwm_new_operator(&oper_config, &oper));
    TRY(mcpwm_operator_connect_timer(oper, timer));
    TRY(mcpwm_new_comparator(oper, &cmp_config, &comparator));
    TRY(mcpwm_new_generator(oper, &gen_config, &generator));
    TRY(mcpwm_comparator_set_compare_value(comparator, control.pulse_us));
    TRY(mcpwm_generator_set_action_on_timer_event(generator,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    TRY(mcpwm_generator_set_action_on_compare_event(generator,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator, MCPWM_GEN_ACTION_LOW)));
    TRY(mcpwm_timer_enable(timer));
    TRY(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));
#undef TRY
    applied_pulse = control.pulse_us;
    pwm_ready = true;
    return ESP_OK;
}

static void apply_output(void)
{
    // Inhibit is enforced inside Motor_Set_Speed under the motor mutex, so a ROS
    // callback race cannot re-enable wheels during a lift or a lift fault.
    int64_t now = esp_timer_get_time();
    if (lifter_control_moving(&control)) wheel_inhibit_until_us = now + 200000;
    Motor_Set_Lift_Inhibit(lifter_control_blocks_wheels(&control) || now < wheel_inhibit_until_us);
    bool failed = false;
    // Allocate/start PWM only for the first accepted target; never seek on boot.
    if (!pwm_ready && control.pulse_us) failed = pwm_init() != ESP_OK;
    if (!failed && pwm_ready && applied_pulse != control.pulse_us) {
        if (!control.pulse_us) {
            failed = mcpwm_generator_set_force_level(generator, 0, true) != ESP_OK;
        } else {
            failed = mcpwm_comparator_set_compare_value(comparator, control.pulse_us) != ESP_OK;
            if (!failed && !applied_pulse)
                failed = mcpwm_generator_set_force_level(generator, -1, true) != ESP_OK;
        }
        if (!failed) applied_pulse = control.pulse_us;
    }
    if (failed) {
        lifter_control_stop(&control, LIFTER_IO_FAULT);
        if (generator) mcpwm_generator_set_force_level(generator, 0, true);
        Motor_Set_Lift_Inhibit(true);
    }
}

static bool wheels_still(int64_t now_us)
{
    motor_diagnostics_t motor;
    Motor_Get_Diagnostics(&motor);
    bool moving = motor.enabled;
    for (int i = 0; i < 4; ++i) moving |= fabsf(motor.speed[i]) > 0.005f;
    if (moving) still_since_us = 0;
    else if (!still_since_us) still_since_us = now_us;
    return still_since_us && now_us - still_since_us >= LIFTER_WHEELS_STILL_US;
}

static void lifter_task(void *arg)
{
    (void)arg;
    TickType_t previous = xTaskGetTickCount();
    for (;;) {
        xSemaphoreTake(mutex, portMAX_DELAY);
        int64_t now = esp_timer_get_time();
        wheels_still(now);
        lifter_control_tick(&control, now, heartbeat_us > 0 && now-heartbeat_us <= LIFTER_MAIN_LEASE_US);
        apply_output();
        xSemaphoreGive(mutex);
        vTaskDelayUntil(&previous, pdMS_TO_TICKS(10));
    }
}

void Lifter_Init(void)
{
    mutex = xSemaphoreCreateMutex();
    configASSERT(mutex);
    const lifter_config_t config = {
#ifdef CONFIG_CHASSIS_LIFTER_ENABLE
        .enabled = true,
#endif
        .lower_deg = CONFIG_CHASSIS_LIFTER_LOWER_DEG,
        .upper_deg = CONFIG_CHASSIS_LIFTER_UPPER_DEG,
        .pulse_0_us = CONFIG_CHASSIS_LIFTER_PULSE_0_DEG_US,
        .pulse_180_us = CONFIG_CHASSIS_LIFTER_PULSE_180_DEG_US,
        .up_ms = CONFIG_CHASSIS_LIFTER_UP_MS,
        .down_ms = CONFIG_CHASSIS_LIFTER_DOWN_MS,
        .timeout_ms = CONFIG_CHASSIS_LIFTER_TIMEOUT_MS,
    };
    lifter_control_init(&control, &config);
    configASSERT(xTaskCreatePinnedToCore(lifter_task, "lifter", 4096, NULL, 6, NULL, 0) == pdPASS);
}

void Lifter_Heartbeat(void)
{
    xSemaphoreTake(mutex, portMAX_DELAY);
    heartbeat_us = esp_timer_get_time();
    xSemaphoreGive(mutex);
}

lifter_result_t Lifter_Command(uint8_t value)
{
    xSemaphoreTake(mutex, portMAX_DELAY);
    int64_t now = esp_timer_get_time();
    bool stationary = wheels_still(now);
    bool fresh = heartbeat_us > 0 && now-heartbeat_us <= LIFTER_MAIN_LEASE_US;
    lifter_result_t result;
    if (!fresh) {
        ++control.rx;
        ++control.rejected;
        result = LIFTER_NOT_READY;
    } else result = lifter_control_command(&control, value, stationary, now);
    apply_output();
    if (control.state == LIFTER_FAULT && result == LIFTER_OK) {
        ++control.rejected;
        result = LIFTER_NOT_READY;
    }
    control.last_result = result;
    xSemaphoreGive(mutex);
    return result;
}
void Lifter_Stop(lifter_fault_t reason)
{
    if (!mutex) return;
    xSemaphoreTake(mutex, portMAX_DELAY);
    lifter_control_stop(&control, reason);
    apply_output();
    xSemaphoreGive(mutex);
}
void Lifter_Get_Snapshot(lifter_control_t *out)
{
    xSemaphoreTake(mutex, portMAX_DELAY);
    *out = control;
    xSemaphoreGive(mutex);
}
bool Lifter_Blocks_Chassis(void)
{
    xSemaphoreTake(mutex, portMAX_DELAY);
    bool blocked = lifter_control_blocks_wheels(&control) || esp_timer_get_time() < wheel_inhibit_until_us;
    xSemaphoreGive(mutex);
    return blocked;
}
bool Lifter_Is_Moving(void)
{
    lifter_control_t snapshot;
    Lifter_Get_Snapshot(&snapshot);
    return lifter_control_moving(&snapshot);
}
