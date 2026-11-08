#pragma once
#include <stdbool.h>
#include <stdint.h>

/* Use the bits so the checks also survive the firmware's fast-math options. */
static inline bool motor_policy_finite(float value)
{
    union { float f; uint32_t u; } bits = {.f = value};
    return (bits.u & 0x7f800000U) != 0x7f800000U;
}

static inline bool motor_policy_speed_trip(float speed, float limit)
{
    return !motor_policy_finite(speed) || !motor_policy_finite(limit) ||
        limit <= 0 || speed > limit || speed < -limit;
}

static inline bool motor_policy_thermal_valid(float rated, float peak, float duration, float cooling)
{
    return motor_policy_finite(rated) && motor_policy_finite(peak) &&
        motor_policy_finite(duration) && motor_policy_finite(cooling) &&
        rated > 0 && peak > rated && duration > 0 && cooling > 0;
}
