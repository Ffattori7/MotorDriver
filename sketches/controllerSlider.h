#pragma once

// Units: mm, mm/s, seconds and signed PWM counts.
struct MotionController {
    float filteredVelocity = 0.0f;
    bool holding = true;
};

struct MotionControlConfig {
    float kp = 4.5f;             // PWM/mm
    float kd = 0.20f;             // PWM/(mm/s), initial damping trial
    float kv = 0.15f;              // PWM/(mm/s), calibrate before enabling feedforward
    float velocityFilterS = 0.040f;
    float stopErrorMm = 0.5f;
    float restartErrorMm = 1.0f;
    float stopSpeedMmS = 5.0f;
    int pwmMax = 255;
};

int motionCTRL(MotionController &state, const MotionControlConfig &config,
               float positionRef, float velocityRef, float position,
               float measuredVelocity, float dt, bool referenceMoving);
