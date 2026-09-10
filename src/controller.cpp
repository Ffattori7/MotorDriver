#include "controller.h"


// Proportional Position Controller - Revolutions (revs)
int propCTRL(
    float KP,
    float setP_revs, 
    float measP_revs) 
{
    float err = setP_revs - measP_revs;

    const int PWM_MIN = 30;  // To be tuned
    // const float PWM_ERR_MIN = 1.0f;  // Minimum error to apply PWM

    // const int PWM_HOLD = 10;
    // const float HOLD_ZONE = 0.2f;  // Error threshold for holding position

    // const float POS_DEADBAND = 0.20f;  // Deadband for position error in revolutions

    // // If close enough, do nothing (deadband)
    // if (abs(err) < POS_DEADBAND) {
    //     return 0;  // No PWM applied
    // }

    int pwm = static_cast<int>(KP * err);

    // Min useful torque/PWM to overcome static friction
    if (pwm > 0 && pwm < PWM_MIN) {
        pwm = PWM_MIN;
    }

    if (pwm < 0 && pwm > -PWM_MIN) {
        pwm = -PWM_MIN;
    }

    // // Stop completely if inside very small deadband
    // if (abs(err) < 0.02f) {
    //     pwm = 0;
    // }

    // Saturation
    if (pwm > 130)
        pwm = 130;
        
    if (pwm < -130) 
        pwm = -130;

    return pwm;
}