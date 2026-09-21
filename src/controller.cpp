#include "controller.h"
#include <math.h>

int propCTRL(float KP, float setP_mm, float measP_mm)
{
    constexpr float STOP_ERROR_MM = 1.0f;
    constexpr float RESTART_ERROR_MM = 2.0f;

    constexpr int PWM_MIN = 30;
    constexpr int PWM_MAX = 100;

    // State for this single motor controller.
    static bool stopped = true;

    float err = setP_mm - measP_mm;
    float absErr = fabsf(err);

    if (stopped) {
        if (absErr <= RESTART_ERROR_MM) {
            return 0;
        }
        stopped = false;
    } else if (absErr <= STOP_ERROR_MM) {
        stopped = true;
        return 0;
    }

    // Clamp in floating point before converting to int.
    float effort = KP * absErr;
    if (effort < PWM_MIN) effort = PWM_MIN;
    if (effort > PWM_MAX) effort = PWM_MAX;

    int pwm = static_cast<int>(effort);
    return (err > 0.0f) ? pwm : -pwm;
}