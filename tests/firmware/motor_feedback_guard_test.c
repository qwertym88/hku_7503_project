#include <assert.h>
#include <stdio.h>
#include "motor_feedback_guard.h"

int main(void)
{
    motor_feedback_guard_t g = {0};
    assert(!motor_feedback_guard_update(&g, true, 1.5f, 0, 10000));
    assert(!motor_feedback_guard_update(&g, true, 1.5f, 0, 309999));
    assert(motor_feedback_guard_update(&g, true, 1.5f, 0, 310000) == MOTOR_FEEDBACK_MISSING);
    g = (motor_feedback_guard_t){0};
    for (int t = 0; t <= 1000000; t += 10000)
        assert(!motor_feedback_guard_update(&g, true, 1.5f, 1, t));
    g = (motor_feedback_guard_t){0};
    for (int t = 0; t < 300000; t += 10000)
        assert(!motor_feedback_guard_update(&g, true, 1.5f, -1, t));
    assert(motor_feedback_guard_update(&g, true, 1.5f, -1, 300000) == MOTOR_FEEDBACK_REVERSED);
    g = (motor_feedback_guard_t){0};
    for (int t = 0; t <= 1000000; t += 10000)
        assert(!motor_feedback_guard_update(&g, true, -1.5f, -1, t));
    // A direction change resets the observation window, not a new ROS message.
    assert(!motor_feedback_guard_update(&g, true, 1.5f, -1, 1010000));
    assert(!motor_feedback_guard_update(&g, true, 1.5f, 0, 1309999));
    assert(motor_feedback_guard_update(&g, true, 1.5f, 0, 1310000) == MOTOR_FEEDBACK_MISSING);
    // Stationary wheel in a mixed translation/rotation target must not trip.
    for (int t = 0; t <= 1000000; t += 10000)
        assert(!motor_feedback_guard_update(&g, true, 0, 0, t));
    assert(!motor_feedback_guard_update(&g, true, 0.99f, 0, 2000000));
    assert(!motor_feedback_guard_update(&g, false, 1.5f, 0, 3000000));
    assert(!g.active);
    puts("PASS: feedback timeout, direction, normal forward/reverse, zero target and stop");
}
