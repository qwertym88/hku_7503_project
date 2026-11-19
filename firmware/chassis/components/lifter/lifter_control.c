#include "lifter_control.h"
#include <stddef.h>

bool lifter_control_moving(const lifter_control_t *l)
{
    return l->state == LIFTER_RAISING || l->state == LIFTER_LOWERING;
}
bool lifter_control_available(const lifter_control_t *l)
{
    return l->state != LIFTER_DISABLED && l->state != LIFTER_UNCONFIGURED;
}
bool lifter_control_blocks_wheels(const lifter_control_t *l)
{
    return lifter_control_moving(l) || l->state == LIFTER_FAULT;
}
uint16_t lifter_control_angle_pulse(const lifter_config_t *c, unsigned degrees)
{
    // Linear endpoint calibration, including reversed pulse/angle direction.
    int32_t pulse = (int32_t)c->pulse_0_us * (180 - (int32_t)degrees)
                  + (int32_t)c->pulse_180_us * (int32_t)degrees;
    return (uint16_t)((pulse + 90) / 180);
}
void lifter_control_init(lifter_control_t *l, const lifter_config_t *c)
{
    *l = (lifter_control_t){.config = *c, .state = LIFTER_DISABLED,
        .last_result = LIFTER_NOT_READY, .position = -1, .goal = -1, .target_deg = -1};
    if (!c->enabled) return;
    l->state = LIFTER_UNCONFIGURED;
    int span = (int)c->pulse_180_us - c->pulse_0_us;
    if (c->lower_deg >= c->upper_deg || c->upper_deg > 180 ||
        c->pulse_0_us < 400 || c->pulse_0_us > 2600 ||
        c->pulse_180_us < 400 || c->pulse_180_us > 2600 ||
        (span > -180 && span < 180) ||
        !c->up_ms || !c->down_ms || c->timeout_ms > 30000 ||
        c->up_ms >= c->timeout_ms || c->down_ms >= c->timeout_ms) return;
    // No position assumption, no startup movement, and no arm requirement.
    l->state = LIFTER_IDLE;
}
lifter_result_t lifter_control_command(lifter_control_t *l, uint8_t value,
                                       bool wheels_still, int64_t now_us)
{
    ++l->rx;
    lifter_result_t result;
    if (value > 1) result = LIFTER_BAD_VALUE;
    else if (!lifter_control_available(l) ||
             (l->state == LIFTER_FAULT && l->fault != LIFTER_STOPPED)) result = LIFTER_NOT_READY;
    else if (lifter_control_moving(l)) result = l->goal == value ? LIFTER_DUPLICATE : LIFTER_BUSY;
    else if (!wheels_still) result = LIFTER_WHEELS_MOVING;
    else if (l->position == value && l->pulse_us) result = LIFTER_DUPLICATE;
    else {
        l->fault = LIFTER_NO_FAULT; // A new target can resume after an operator stop.
        l->goal = value;
        l->position = -1;
        l->target_deg = value ? l->config.upper_deg : l->config.lower_deg;
        l->pulse_us = lifter_control_angle_pulse(&l->config, l->target_deg);
        l->state = value ? LIFTER_RAISING : LIFTER_LOWERING;
        l->start_us = now_us;
        l->end_us = now_us + (int64_t)(value ? l->config.up_ms : l->config.down_ms) * 1000;
        l->timeout_us = now_us + (int64_t)l->config.timeout_ms * 1000;
        result = LIFTER_OK;
    }
    if (result != LIFTER_OK && result != LIFTER_DUPLICATE) ++l->rejected;
    l->last_result = result;
    return result;
}
void lifter_control_stop(lifter_control_t *l, lifter_fault_t reason)
{
    if (!lifter_control_available(l)) return;
    l->position = -1;
    l->target_deg = -1;
    if (l->state != LIFTER_FAULT) l->fault = reason;
    l->state = LIFTER_FAULT;
    // No measured intermediate angle: release signal rather than send 90 degrees.
    // Actual stopping/holding after signal loss depends on the servo hardware.
    l->pulse_us = 0;
}
void lifter_control_tick(lifter_control_t *l, int64_t now_us,
                         bool lease_fresh)
{
    if (!lifter_control_available(l) || l->state == LIFTER_FAULT) return;
    if (l->pulse_us && !lease_fresh) { lifter_control_stop(l, LIFTER_COMM_LOST); return; }
    if (!lifter_control_moving(l)) return;
    if (now_us >= l->timeout_us || now_us > l->end_us + 30000) {
        lifter_control_stop(l, LIFTER_TIMEOUT); return;
    }
    if (now_us >= l->end_us) {
        l->position = l->goal;
        l->state = LIFTER_IDLE;
        ++l->completed;
        // Keep the target pulse indefinitely. Never return to a neutral angle.
    }
}
const char *lifter_state_name(lifter_state_t s)
{
    static const char *names[] = {"disabled","unconfigured","idle","raising","lowering","fault"};
    return s >= 0 && s <= LIFTER_FAULT ? names[s] : "invalid";
}
const char *lifter_fault_name(lifter_fault_t f)
{
    static const char *names[] = {"none","operator_stop","timeout","communication","pwm_error"};
    return f >= 0 && f <= LIFTER_IO_FAULT ? names[f] : "invalid";
}
const char *lifter_result_name(lifter_result_t r)
{
    static const char *names[] = {"accepted","duplicate","invalid_value","not_ready","busy","wheels_moving"};
    return r >= 0 && r <= LIFTER_WHEELS_MOVING ? names[r] : "invalid";
}
