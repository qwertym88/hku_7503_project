// MG996R continuous-rotation neutral calibration: fixed pulse, wheels braked.
// Pinout and 50Hz/1us MCPWM setup follow Yahboom's esp32_samples/pwm_servo.
#include <stddef.h>
#include "driver/mcpwm_prelude.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "pwm_motor.h"
#include "sdkconfig.h"

#define SERVO_GPIO (CONFIG_SERVO_ZERO_PORT == 1 ? 8 : 21)
#define SERVO_GROUP 1
#define SERVO_PERIOD_US 20000

_Static_assert(CONFIG_SERVO_ZERO_PORT == 1 || CONFIG_SERVO_ZERO_PORT == 2,
               "Only S1 and S2 are available on this board");
_Static_assert(CONFIG_SERVO_ZERO_PULSE_US >= 1400 && CONFIG_SERVO_ZERO_PULSE_US <= 1600,
               "Neutral calibration stays within 1400..1600us; this is not an angle command");

static const char *TAG = "servo_zero";

static void servo_pwm_start(void)
{
    mcpwm_timer_handle_t timer = NULL;
    mcpwm_timer_config_t timer_config = {
        .group_id = SERVO_GROUP,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = 1000000,
        .period_ticks = SERVO_PERIOD_US,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));
    mcpwm_oper_handle_t oper = NULL;
    mcpwm_operator_config_t operator_config = {.group_id = SERVO_GROUP};
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &oper));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));
    mcpwm_cmpr_handle_t comparator = NULL;
    mcpwm_comparator_config_t comparator_config = {.flags.update_cmp_on_tez = true};
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &comparator_config, &comparator));
    mcpwm_gen_handle_t generator = NULL;
    mcpwm_generator_config_t generator_config = {.gen_gpio_num = SERVO_GPIO};
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &generator_config, &generator));
    // Program the requested stop pulse before PWM starts. No startup sweep
    // and no default pulse on the unselected servo connector.
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, CONFIG_SERVO_ZERO_PULSE_US));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                    MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                      comparator, MCPWM_GEN_ACTION_LOW)));
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));
}

void app_main(void)
{
    // Reuse the existing board driver. No encoder task, PID or ROS subscription
    // is started, so velocity commands cannot drive the chassis in this image.
    PwmMotor_Init();
    PwmMotor_Stop(MOTOR_ID_ALL, STOP_BRAKE);
    ESP_LOGI(TAG, "Installation firmware: wheels braked, no ROS interfaces");
    ESP_LOGI(TAG, "S%d / GPIO%d: start in %dms, then send neutral %dus at 50Hz",
             CONFIG_SERVO_ZERO_PORT, SERVO_GPIO, CONFIG_SERVO_ZERO_START_DELAY_MS,
             CONFIG_SERVO_ZERO_PULSE_US);
    vTaskDelay(pdMS_TO_TICKS(CONFIG_SERVO_ZERO_START_DELAY_MS));
    servo_pwm_start();
    ESP_LOGI(TAG, "PWM active: continuous servo STOP request, no angle positioning or holding");
    // MCPWM maintains pulses in hardware while the app task sleeps.
    while (true) vTaskDelay(pdMS_TO_TICKS(1000));
}
