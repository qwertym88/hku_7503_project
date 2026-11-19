#pragma once
#include <stdbool.h>
#include <stdint.h>

typedef enum {
    LIFTER_DISABLED, LIFTER_UNCONFIGURED, LIFTER_IDLE,
    LIFTER_RAISING, LIFTER_LOWERING, LIFTER_FAULT
} lifter_state_t;
typedef enum {
    LIFTER_NO_FAULT, LIFTER_STOPPED, LIFTER_TIMEOUT, LIFTER_COMM_LOST,
    LIFTER_IO_FAULT
} lifter_fault_t;
typedef enum {
    LIFTER_OK, LIFTER_DUPLICATE, LIFTER_BAD_VALUE, LIFTER_NOT_READY,
    LIFTER_BUSY, LIFTER_WHEELS_MOVING
} lifter_result_t;

typedef struct {
    bool enabled;
    uint16_t lower_deg, upper_deg, pulse_0_us, pulse_180_us;
    uint32_t up_ms, down_ms, timeout_ms;
} lifter_config_t;
typedef struct {
    lifter_config_t config;
    lifter_state_t state;
    lifter_fault_t fault;
    lifter_result_t last_result;
    int position; // -1 unknown; 0/1 target after settling. Not measured feedback.
    int goal;
    int target_deg; // -1 before a command or after output is released.
    uint16_t pulse_us; // 0 requests PWM release; never means angle zero.
    int64_t start_us, end_us, timeout_us;
    uint32_t rx, rejected, completed;
} lifter_control_t;

void lifter_control_init(lifter_control_t *l, const lifter_config_t *config);
bool lifter_control_available(const lifter_control_t *l);
bool lifter_control_moving(const lifter_control_t *l);
bool lifter_control_blocks_wheels(const lifter_control_t *l);
uint16_t lifter_control_angle_pulse(const lifter_config_t *config, unsigned degrees);
lifter_result_t lifter_control_command(lifter_control_t *l, uint8_t value,
                                       bool wheels_still, int64_t now_us);
void lifter_control_stop(lifter_control_t *l, lifter_fault_t reason);
void lifter_control_tick(lifter_control_t *l, int64_t now_us,
                         bool lease_fresh);
const char *lifter_state_name(lifter_state_t value);
const char *lifter_fault_name(lifter_fault_t value);
const char *lifter_result_name(lifter_result_t value);
