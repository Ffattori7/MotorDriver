#include <Arduino.h>
#include "motor.h"
#include "encoder.h"
#include "controller.h"


// Runtime
uint32_t startTime = 0;
const uint32_t MAX_RUNTIME_MS = 5000;  // 5 seconds

// Revolutions (Motor) Setpoint
const float SETPOINT_REV = -35.0f;  // Target encoder revolutions

// // Setpoints
// float velRef = 0.0f;  // [revs/s^2]
// float posRef = 0.0f;  // [revs]

// Tolerance
const float POS_TOL = 1.00f;  // Position tolerance in revolutions

// Maximum Current 
const float CURRENT_LIMIT = 1.4f;  // [A]
float maxCurrent = 0.0f;  // [A]

// Proportional Gain
const float KP = 25.0f;  // Adjust this value for tuning

// Control Loop Frequecncy
const float TS = 0.020f;  // 20 ms = 50 Hz

void setup() {
    Serial.begin(115200);

    motorInit();
    encoderInit(TS);

    // Start Disabled
    shutdown();
    motorStop();
    // delay(1000);

    // Reset encoder count
    resetEncoderCount();
    // delay(3000);

    // Serial.println("READY - type s");
    while (!Serial.available()) {
        // Wait for user input
    }
    char c = Serial.read();
    if (c == 's') {
        // Serial.println("Starting in 3 seconds:...");
        delay(3000);

        // Wake Driver
        driverEnable();

        // Record start time
        startTime = millis();

        // Print CSV Header
        Serial.println("time_ms,position_rev,error_rev,pwm,current_A,maxCurrent_A");
    }

    // // Wake Driver
    // driverEnable();
    // delay(3000);

    // // Record start time
    // startTime = millis();

    // // Print CSV Header
    // Serial.println("time_ms,position_rev,error_rev,pwm,current_A,maxCurrent_A");
}

void loop() {
    encoderUpdate();

    float pos = getMotorRevs();
    long encoderCount = getEncoderCount();
    float err = SETPOINT_REV - pos;
    float speedRPM = getMotorSpeedRPM();
    float current = getMotorCurrent();

    if (current > maxCurrent) {
        maxCurrent = current;
    }

    // Check if current is too high
    if (current >= CURRENT_LIMIT) {
        motorStop();

        Serial.print(millis() - startTime);
        Serial.print(",");

        Serial.print(pos, 3);
        Serial.print(",");

        // Serial.print(encoderCount);
        // Serial.print(",");

        Serial.print(err, 3);
        Serial.print(",");

        Serial.print("0,");

        // Serial.print(speedRPM, 1);
        // Serial.print(",");

        Serial.print(current, 3);
        Serial.print(",");

        Serial.println(maxCurrent, 3);

        shutdown();

        while (true)
        {
            // Stay here forever
        }
    }

    // Check if target reached within tolerance
    if (abs(err) <= POS_TOL) {
        motorStop();
        // delay(1000);

        Serial.print(millis() - startTime);
        Serial.print(",");

        Serial.print(pos,3);
        Serial.print(",");

        // Serial.print(encoderCount);
        // Serial.print(",");

        Serial.print(err,3);
        Serial.print(",");

        // Serial.print(pwm);
        Serial.print("0,");

        // Serial.print(speedRPM,1);
        // Serial.print(",");

        Serial.print(current, 3);
        Serial.print(",");

        Serial.println(maxCurrent, 3);

        shutdown();

        while (true) {
            // Stay here forever
        }
    }

    // Stop if maximum test time is exceeded
    if (millis() - startTime >= MAX_RUNTIME_MS)
    {
        motorStop();

        Serial.print(millis() - startTime);
        Serial.print(",");

        Serial.print(pos,3);
        Serial.print(",");

        // Serial.print(encoderCount);
        // Serial.print(",");

        Serial.print(err,3);
        Serial.print(",");

        // Serial.print(pwm);
        Serial.print("0,");

        // Serial.print(speedRPM,1);
        // Serial.print(",");

        Serial.print(current, 3);
        Serial.print(",");

        Serial.println(maxCurrent, 3);

        shutdown();

        // Serial.println();
        // Serial.println("===== TIMEOUT =====");
        // Serial.println("Maximum test time reached.");
        // Serial.println("===================");

        while (true)
        {
            // Stay here forever
        }
    }

    // Compute PWM using proportional control
    int pwm = propCTRL(
      KP, 
      SETPOINT_REV, 
      pos);

    if (pwm >= 0) {
        motorForward(pwm);
    } else if (pwm < 0) {
        motorReverse(-pwm);
    } else {
        motorStop();
    }

    // Print Feedback CSV-like
    Serial.print(millis() - startTime);
    Serial.print(",");

    Serial.print(pos,3);
    Serial.print(",");

    // Serial.print(encoderCount);
    // Serial.print(",");

    Serial.print(err,3);
    Serial.print(",");

    Serial.print(pwm);
    Serial.print(",");

    // Serial.print(speedRPM,1);
    // Serial.print(",");

    Serial.print(current, 3);
    Serial.print(",");

    Serial.println(maxCurrent, 3);

    delay(20);  // 50 Hz control loop
}