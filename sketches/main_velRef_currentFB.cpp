#include <Arduino.h>
#include "motor.h"
#include "encoder.h"
#include "controller.h"


// Runtime
uint32_t startTime = 0;
const uint32_t MAX_RUNTIME_MS = 10000;  // 5 seconds

// Setpoints - No Initial CMD
float velRef = 0.0f;  // [rev/s]
float posRef = 0.0f;  // [rev]

// Tolerance
const float POS_TOL = 2.00f;  // Position tolerance in revolutions

// Proportional Gain
const float KP = 10.0f;  // Adjust this value for tuning

// Current Limits
const float CURRENT_LIMIT = 1.4f;  // [A]
float maxCurrent = 0.0f;            // [A]

// Control Loop Frequency
const float TS = 0.020f;  // 20 ms = 50 Hz

const uint32_t LOOP_PERIOD_US = 10000;  // 20 ms = 50 Hz
uint32_t next_loopTime = 0;
uint32_t prev_loopTime = 0;
bool firstLoop = true;

// Serial Buffer
char velBuffer[20];
uint8_t velBuffer_idx = 0;

// Serial-reading Function - Velocity Reference
void readVelRef() {
    while (Serial.available() > 0) {
        char c = Serial.read();

        // End CMD
        if (c == '\n' || c == '\r') {
            if (velBuffer_idx > 0) {
                velBuffer[velBuffer_idx] = '\0';
                velRef = atof(velBuffer);
                velBuffer_idx = 0;
            }
        }

        // Add char to buffer
        else if (velBuffer_idx < sizeof(velBuffer) - 1) {
            velBuffer[velBuffer_idx++] = c;
        }
    }
}

// ---------------------------------------------------------
// Setup
// ---------------------------------------------------------

void setup() {
    Serial.begin(115200);

    motorInit();
    encoderInit(TS);

    // Start Disabled
    shutdown();
    motorStop();
    delay(1000);

    // Reset encoder count
    resetEncoderCount();

    // Wake Driver
    // Serial.println("Enabling Driver");
    driverEnable();
    delay(3000);

    // Record start time
    startTime = millis();

    uint32_t now = micros();
    next_loopTime = now;
    prev_loopTime = now;

    // Print CSV Header
    Serial.println(
        "time_ms,"
        "velocity_ref_rev_s,"
        "velocity_rev_s,"
        "position_ref_rev,"
        "position_rev,"
        "encoder_count,"
        "error_rev,"
        "pwm,"
        "speed_rpm,"
        "current_A,"
        "maxCurrent_A,"
        "loop_period_us,"
        "loop_exe_us");
}

// ---------------------------------------------------------
// Main loop
// ---------------------------------------------------------

void loop() {
    uint32_t now = micros();

    // 50 Hz scheduler

    if ((int32_t)(now - next_loopTime) < 0) {
        return;
    }

    uint32_t loopPeriod = 0;
    
    if (!firstLoop) {
        loopPeriod = now - prev_loopTime;
    }

    prev_loopTime = now;
    firstLoop = false;
    next_loopTime += LOOP_PERIOD_US;

    // Start execution-time measurement
    uint32_t loopStart = micros();

    // Read velocity reference from serial
    readVelRef();

    // Intergate to generate position reference
    posRef += velRef * TS;

    // Update encoder feedback
    encoderUpdate();

    // Read feedback
    float pos = getMotorRevs();
    long encoderCount = getEncoderCount();
    float err = posRef - pos;
    float vel = getMotorSpeedRevs();
    float speedRPM = getMotorSpeedRPM();

    float current = getMotorCurrent();
    if (current > maxCurrent) {
        maxCurrent = current;
    }

    // Current Hard Stop
    if (current >= CURRENT_LIMIT) {
        motorStop();

        uint32_t elapsed = millis() - startTime;

        Serial.print(elapsed);
        Serial.print(",");

        Serial.print(velRef, 3);
        Serial.print(",");

        Serial.print(vel, 3);
        Serial.print(",");

        Serial.print(posRef, 3);
        Serial.print(",");

        Serial.print(pos, 3);
        Serial.print(",");

        Serial.print(encoderCount);
        Serial.print(",");

        Serial.print(err, 3);
        Serial.print(",");

        Serial.print(0);
        Serial.print(",");

        Serial.print(current, 3);
        Serial.print(",");

        Serial.print(maxCurrent, 3);
        Serial.print(",");

        Serial.print(loopPeriod);
        Serial.print(",");

        Serial.println(micros() - loopStart);

        shutdown();

        while (true)
        {
            // Hard stop: remain stopped
        }
    }


    // Compute PWM using proportional control
    int pwm = propCTRL(
      KP, 
      posRef, 
      pos);

    // Motor direction
    if (pwm > 0) {
        motorForward(pwm);
    } else if (pwm < 0) {
        motorReverse(-pwm);
    } else {
        motorStop();
    }

    // Measure control loop execution time
    uint32_t loopExeTime = micros() - loopStart;

    // Stop condition -  Maximum test time exceeded
    if (millis() - startTime >= MAX_RUNTIME_MS)
    {
        motorStop();

        Serial.print(millis() - startTime);
        Serial.print(",");

        Serial.print(velRef,3);
        Serial.print(",");

        Serial.print(vel,3);
        Serial.print(",");

        Serial.print(posRef,3);
        Serial.print(",");

        Serial.print(pos,3);
        Serial.print(",");

        Serial.print(encoderCount);
        Serial.print(",");

        Serial.print(err,3);
        Serial.print(",");

        // Serial.print(pwm);
        Serial.print("0,");

        Serial.print(speedRPM,1);
        Serial.print(",");

        Serial.print(current, 3);
        Serial.print(",");

        Serial.print(maxCurrent, 3);
        Serial.print(",");

        Serial.print(loopPeriod);
        Serial.print(",");

        Serial.println(loopExeTime);

        shutdown();

        while (true)
        {
            // Stay here forever
        }
    }

    // Print Feedback CSV-like
    Serial.print(millis() - startTime);
    Serial.print(",");

    Serial.print(velRef,3);
    Serial.print(",");

    Serial.print(vel,3);
    Serial.print(",");

    Serial.print(posRef,3);
    Serial.print(",");

    Serial.print(pos,3);
    Serial.print(",");

    Serial.print(encoderCount);
    Serial.print(",");

    Serial.print(err,3);
    Serial.print(",");

    Serial.print(pwm);
    Serial.print(",");

    Serial.print(speedRPM,1);
    Serial.print(",");

    Serial.print(current, 3);
    Serial.print(",");

    Serial.print(maxCurrent, 3);
    Serial.print(",");

    Serial.print(loopPeriod);
    Serial.print(",");

    Serial.println(loopExeTime);

    // delay(20);  // 50 Hz control loop
}