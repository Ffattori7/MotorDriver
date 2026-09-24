#include <Arduino.h>
#include <math.h>
#include <stdlib.h>
#include <ctype.h>
#include "motor.h"
#include "encoder.h"
#include "controller.h"


// Runtime
uint32_t startTime = 0;
const uint32_t MAX_RUNTIME_MS = 60000;  // 60 seconds after setup

// Serial commands are absolute belt targets relative to startup zero [mm].
// This speed sets the reference ramp, not a closed-loop velocity command.
constexpr float MOTION_SPEED_MM_S = 300.0f;
float targetPos = 0.0f;  // Serial destination [mm]
float posRef = 0.0f;     // Integrated tracking reference [mm]
float velRef = 0.0f;     // Signed reference velocity [mm/s]

// Tunable gains, velocity filter and final holding thresholds: controller.h.
// kv defaults to zero until PWM-versus-speed is measured under the actual load.
const MotionControlConfig controlConfig;
MotionController controlState;
float previousPosition = 0.0f;
bool havePositionSample = false;

// Current Limits
const float CURRENT_LIMIT = 5.0f;  // [A]
float maxCurrent = 0.0f;            // [A]

// Control Loop Frequency
const float TS = 0.020f;  // 20 ms = 50 Hz

const uint32_t LOOP_PERIOD_US = 20000;  // 20 ms = 50 Hz
uint32_t next_loopTime = 0;
uint32_t prev_loopTime = 0;
bool firstLoop = true;

// Serial Buffer: ignore malformed or overlong commands as a whole.
char posBuffer[32];
uint8_t posBuffer_idx = 0;
bool posBufferOverflow = false;

void readPosRef() {
    while (Serial.available() > 0) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (posBuffer_idx > 0 && !posBufferOverflow) {
                posBuffer[posBuffer_idx] = '\0';
                char *end = nullptr;
                float value = strtod(posBuffer, &end);
                bool hasNumber = end != posBuffer;
                while (isspace(static_cast<unsigned char>(*end))) ++end;
                if (hasNumber && *end == '\0' && isfinite(value)) {
                    targetPos = value;
                }
            }
            posBuffer_idx = 0;
            posBufferOverflow = false;
        } else if (posBuffer_idx < sizeof(posBuffer) - 1) {
            posBuffer[posBuffer_idx++] = c;
        } else {
            posBufferOverflow = true;
        }
    }
}

void updatePositionReference(float dt) {
    float remaining = targetPos - posRef;
    float maxStep = MOTION_SPEED_MM_S * dt;
    float step = remaining;
    if (step > maxStep) step = maxStep;
    if (step < -maxStep) step = -maxStep;
    if (fabsf(remaining) <= maxStep) {
        posRef = targetPos;  // Land exactly on the destination without overshoot.
    } else {
        posRef += step;
    }
    velRef = step / dt;
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
        "velocity_ref_mm_s,"
        "velocity_rev_s,"
        "position_ref_mm,"
        "position_mm,"
        "encoder_count,"
        "error_mm,"
        "pwm,"
        "speed_rpm,"
        "current_A,"
        "maxCurrent_A,"
        "loop_period_us,"
        "loop_exe_us,"
        "tracking_ref_mm,"
        "tracking_error_mm,"
        "velocity_mm_s,"
        "filtered_velocity_mm_s,"
        "holding");
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
    // Do not execute rapid catch-up iterations after an overrun.
    if ((int32_t)(now - next_loopTime) >= 0) next_loopTime = now + LOOP_PERIOD_US;
    const float dt = loopPeriod > 0 ? loopPeriod * 1.0e-6f : TS;

    // Start execution-time measurement
    uint32_t loopStart = micros();

    // Ramp continuously from the previous reference, including on retargeting.
    readPosRef();
    updatePositionReference(dt);

    // Update encoder feedback
    encoderUpdate();

    // Read feedback
    float pos = getBeltPos();
    long encoderCount = getEncoderCount();
    float err = targetPos - pos;  // Destination error for tracking evaluation
    float vel = getMotorSpeedRevs();
    float speedRPM = getMotorSpeedRPM();
    // Derive belt velocity from the existing mm position API and actual elapsed time.
    float beltVelocity = havePositionSample ? (pos - previousPosition) / dt : 0.0f;
    previousPosition = pos;
    havePositionSample = true;

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

        Serial.print(targetPos, 3);
        Serial.print(",");

        Serial.print(pos, 3);
        Serial.print(",");

        Serial.print(encoderCount);
        Serial.print(",");

        Serial.print(err, 3);
        Serial.print(",");

        Serial.print(0);
        Serial.print(",");

        Serial.print(speedRPM, 1);
        Serial.print(",");

        Serial.print(current, 3);
        Serial.print(",");

        Serial.print(maxCurrent, 3);
        Serial.print(",");

        Serial.print(loopPeriod);
        Serial.print(",");

        Serial.print(micros() - loopStart);
        Serial.print(",");
        Serial.print(posRef, 3);
        Serial.print(",");
        Serial.print(posRef - pos, 3);
        Serial.print(",");
        Serial.print(beltVelocity, 3);
        Serial.print(",");
        Serial.print(controlState.filteredVelocity, 3);
        Serial.print(",");
        Serial.println(controlState.holding ? 1 : 0);

        shutdown();

        while (true)
        {
            // Hard stop: remain stopped
        }
    }


    // Reference still uses constant speed: no acceleration profile introduced.
    const bool referenceMoving = fabsf(velRef) > 0.001f || posRef != targetPos;
    int pwm = motionCTRL(controlState, controlConfig, posRef, velRef,
                         pos, beltVelocity, dt, referenceMoving);

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

        Serial.print(targetPos,3);
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

        Serial.print(loopExeTime);
        Serial.print(",");
        Serial.print(posRef, 3);
        Serial.print(",");
        Serial.print(posRef - pos, 3);
        Serial.print(",");
        Serial.print(beltVelocity, 3);
        Serial.print(",");
        Serial.print(controlState.filteredVelocity, 3);
        Serial.print(",");
        Serial.println(controlState.holding ? 1 : 0);

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

    Serial.print(targetPos,3);
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

    Serial.print(loopExeTime);
    Serial.print(",");
    Serial.print(posRef, 3);
    Serial.print(",");
    Serial.print(posRef - pos, 3);
        Serial.print(",");
        Serial.print(beltVelocity, 3);
        Serial.print(",");
        Serial.print(controlState.filteredVelocity, 3);
        Serial.print(",");
        Serial.println(controlState.holding ? 1 : 0);

    // delay(20);  // 50 Hz control loop
}