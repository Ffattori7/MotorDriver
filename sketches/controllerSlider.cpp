#include "controller.h"
#include <math.h>

int motionCTRL(MotionController &state, const MotionControlConfig &config,
               float positionRef, float velocityRef, float position,
               float measuredVelocity, float dt, bool referenceMoving)
{
    if (!isfinite(dt) || dt <= 0.0f || !isfinite(positionRef) ||
        !isfinite(velocityRef) || !isfinite(position) || !isfinite(measuredVelocity)) {
        state = MotionController{};
        return 0;
    }

    const float alpha = dt / (config.velocityFilterS + dt);
    state.filteredVelocity += alpha * (measuredVelocity - state.filteredVelocity);
    const float error = positionRef - position;
    // Check raw speed too: filtering must not hide a recent movement at arrival.
    const bool slow = fabsf(state.filteredVelocity) <= config.stopSpeedMmS &&
                      fabsf(measuredVelocity) <= config.stopSpeedMmS;

    if (referenceMoving) {
        state.holding = false;
    } else if (state.holding) {
        if (fabsf(error) <= config.restartErrorMm && slow) return 0;
        state.holding = false;
    } else if (fabsf(error) <= config.stopErrorMm && slow) {
        state.holding = true;
        return 0;
    }

    // Same law during travel and settling; no minimum-PWM kick or integral windup.
    float effort = config.kp * error
                 + config.kd * (velocityRef - state.filteredVelocity)
                 + config.kv * velocityRef;
    if (effort > config.pwmMax) effort = config.pwmMax;
    if (effort < -config.pwmMax) effort = -config.pwmMax;
    return static_cast<int>(roundf(effort));
}
