#pragma once

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

// Initial bench thresholds, not yet qualified with a loaded chassis.
#define MOTOR_FEEDBACK_WINDOW_US 300000LL
#define MOTOR_FEEDBACK_MIN_TARGET 1.0f  // counts per 10ms, about 0.0196m/s
#define MOTOR_FEEDBACK_MIN_COUNTS 2
#define MOTOR_FEEDBACK_REVERSE_COUNTS 4

typedef struct {
    bool active;
    int direction;
    int64_t since_us;
    int64_t travel;
    int64_t signed_travel;
} motor_feedback_guard_t;

enum {
    MOTOR_FEEDBACK_OK = 0,
    MOTOR_FEEDBACK_MISSING = 1,
    MOTOR_FEEDBACK_REVERSED = 2,
};

// Called every motor cycle, not every ROS command. Repeated commands must not
// restart the window and mask a failure. Near-zero targets intentionally bypass
// detection; two counts/window detect missing feedback, not accurate tracking.
static inline int motor_feedback_guard_update(motor_feedback_guard_t *g,
        bool enabled, float target, int64_t delta, int64_t now_us)
{
    if (!enabled || !isfinite(target) || fabsf(target) < MOTOR_FEEDBACK_MIN_TARGET) {
        *g = (motor_feedback_guard_t){0};
        return MOTOR_FEEDBACK_OK;
    }
    int direction = target > 0 ? 1 : -1;
    if (!g->active || direction != g->direction) {
        *g = (motor_feedback_guard_t){.active = true, .direction = direction,
                                      .since_us = now_us};
        return MOTOR_FEEDBACK_OK; // delta may belong to the previous direction.
    }
    g->travel += delta < 0 ? -(int64_t)delta : delta;
    g->signed_travel += (int64_t)direction * delta;
    if (now_us - g->since_us < MOTOR_FEEDBACK_WINDOW_US) return MOTOR_FEEDBACK_OK;
    int result = MOTOR_FEEDBACK_OK;
    if (g->travel < MOTOR_FEEDBACK_MIN_COUNTS) result = MOTOR_FEEDBACK_MISSING;
    else if (g->signed_travel <= -MOTOR_FEEDBACK_REVERSE_COUNTS)
        result = MOTOR_FEEDBACK_REVERSED;
    g->since_us = now_us;
    g->travel = g->signed_travel = 0;
    return result;
}
