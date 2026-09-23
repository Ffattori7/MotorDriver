#include <Arduino.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "motor.h"
#include "encoder.h"
#include "controller.h"


// =========================================================
// Mechanical configuration
// =========================================================

// Must correspond to ROS l_max = 0.4825 m
constexpr float SLIDER_TRAVEL_MM = 482.5f;


// =========================================================
// Controller
// =========================================================

constexpr float KP = 5.0f;

// Current safety limit
constexpr float CURRENT_LIMIT = 4.5f;  // [A]

// Control loop
constexpr float TS = 0.020f;                 // [s]
constexpr uint32_t LOOP_PERIOD_US = 20000;  // 50 Hz


// =========================================================
// ROS / Serial interface
// =========================================================

// ROS sends:
//      0.000\n
//      0.500\n
//      1.000\n
//
// Arduino sends:
//      POS,0.497\n
//
// ROS can also send:
//      STOP\n

constexpr uint32_t FEEDBACK_PERIOD_MS = 50;  // 20 Hz

char serialBuffer[32];
uint8_t serialBufferIdx = 0;
bool discardSerialLine = false;


// =========================================================
// State
// =========================================================

// Normalized command received from ROS [0,1]
float positionCmdNorm = 0.0f;

// Physical controller reference [mm]
float posRef = 0.0f;

// Actuator state
bool motionEnabled = false;

// Latched current fault.
// Once triggered, reboot/reset is currently required.
bool currentFault = false;


// =========================================================
// Scheduler
// =========================================================

uint32_t nextLoopTime = 0;
uint32_t lastFeedbackTime = 0;


// =========================================================
// Conversion functions
// =========================================================

float normalizedToBeltPos(float normalized)
{
    if (normalized < 0.0f) normalized = 0.0f;
    if (normalized > 1.0f) normalized = 1.0f;

    return normalized * SLIDER_TRAVEL_MM;
}


float beltPosToNormalized(float pos_mm)
{
    float normalized = pos_mm / SLIDER_TRAVEL_MM;

    // ROS driver expects feedback within [0,1].
    // Clamp small overshoots caused by controller dynamics.
    if (normalized < 0.0f) normalized = 0.0f;
    if (normalized > 1.0f) normalized = 1.0f;

    return normalized;
}


// =========================================================
// Position command
// =========================================================

void applyPositionCommand(float normalized)
{
    // Reject commands outside the ROS interface definition.
    if (normalized < 0.0f || normalized > 1.0f) {
        return;
    }

    // Do not automatically restart after a safety fault.
    if (currentFault) {
        return;
    }

    positionCmdNorm = normalized;

    // Convert ROS normalized coordinate to the physical
    // quantity already used by your controller.
    posRef = normalizedToBeltPos(positionCmdNorm);

    // Enable actuator
    driverEnable();
    motionEnabled = true;
}


// =========================================================
// STOP command
// =========================================================

void stopFromCommand()
{
    motorStop();
    shutdown();

    motionEnabled = false;
}


// =========================================================
// Serial command processing
// =========================================================

void processSerialLine(char *line)
{
    // -------------------------
    // STOP command
    // -------------------------
    if (strcmp(line, "STOP") == 0) {
        stopFromCommand();
        return;
    }

    // -------------------------
    // Position command
    // -------------------------

    char *endPtr = nullptr;

    // float command = strtof(line, &endPtr);
    float command = static_cast<float>(strtod(line, &endPtr));

    // strtof() must actually have found a number
    if (endPtr == line) {
        return;
    }

    // Allow trailing spaces/tabs
    while (*endPtr == ' ' || *endPtr == '\t') {
        endPtr++;
    }

    // Reject strings such as:
    //
    // 0.5abc
    //
    if (*endPtr != '\0') {
        return;
    }

    // Reject NaN / Inf
    if (isnan(command) || isinf(command)) {
        return;
    }

    // Valid normalized position
    if (command >= 0.0f && command <= 1.0f) {
        applyPositionCommand(command);
    }
}


void readSerialCommands()
{
    while (Serial.available() > 0) {

        char c = Serial.read();

        // End of command
        if (c == '\n' || c == '\r') {

            // End discard mode at the end of the corrupted line
            if (discardSerialLine) {
                discardSerialLine = false;
                serialBufferIdx = 0;
                continue;
            }

            if (serialBufferIdx > 0) {

                serialBuffer[serialBufferIdx] = '\0';

                processSerialLine(serialBuffer);

                serialBufferIdx = 0;
            }
        }

        // Ignore everything until EOL after buffer overflow
        else if (discardSerialLine) {
            continue;
        }

        // Store character
        else if (serialBufferIdx < sizeof(serialBuffer) - 1) {

            serialBuffer[serialBufferIdx++] = c;
        }

        // Buffer overflow:
        // reject the complete command
        else {

            serialBufferIdx = 0;
            discardSerialLine = true;
        }
    }
}


// =========================================================
// ROS feedback
// =========================================================

void sendPositionFeedback(float pos_mm)
{
    float normalized = beltPosToNormalized(pos_mm);

    Serial.print("POS,");
    Serial.println(normalized, 4);
}


// =========================================================
// Setup
// =========================================================

void setup()
{
    Serial.begin(115200);

    motorInit();
    encoderInit(TS);

    // Always start with actuator disabled.
    motorStop();
    shutdown();

    // IMPORTANT:
    // This defines the CURRENT physical slider position
    // as x = 0 mm.
    resetEncoderCount();

    uint32_t now = micros();

    nextLoopTime = now + LOOP_PERIOD_US;

    lastFeedbackTime = millis();
}


// =========================================================
// Main loop
// =========================================================

void loop()
{
    // Read serial continuously.
    //
    // In particular, STOP does not need to wait for the
    // next complete control calculation.
    readSerialCommands();


    // -----------------------------------------------------
    // 50 Hz scheduler
    // -----------------------------------------------------

    uint32_t now = micros();

    if ((int32_t)(now - nextLoopTime) < 0) {
        return;
    }

    nextLoopTime += LOOP_PERIOD_US;


    // -----------------------------------------------------
    // Encoder feedback
    // -----------------------------------------------------

    encoderUpdate();

    float pos = getBeltPos();


    // -----------------------------------------------------
    // Motor control
    // -----------------------------------------------------

    if (motionEnabled && !currentFault) {

        float current = getMotorCurrent();


        // -------------------------------------------------
        // Current hard stop
        // -------------------------------------------------

        if (current >= CURRENT_LIMIT) {

            motorStop();
            shutdown();

            motionEnabled = false;
            currentFault = true;
        }

        // -------------------------------------------------
        // Position controller
        // -------------------------------------------------

        else {

            int pwm = propCTRL(
                KP,
                posRef,
                pos);

            if (pwm > 0) {

                motorForward(pwm);
            }
            else if (pwm < 0) {

                motorReverse(-pwm);
            }
            else {

                motorStop();
            }
        }
    }

    else {

        motorStop();
    }


    // -----------------------------------------------------
    // Periodic ROS feedback
    // -----------------------------------------------------

    uint32_t nowMs = millis();

    if (nowMs - lastFeedbackTime >= FEEDBACK_PERIOD_MS) {

        lastFeedbackTime = nowMs;

        sendPositionFeedback(pos);
    }
}