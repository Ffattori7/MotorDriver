#include "encoder.h"

const uint8_t ENC_A_PIN = 2;
const uint8_t ENC_B_PIN = 3;

volatile long encoderCount = 0;
const float COUNTS_PER_REV = 48.0f;  // Encoder resolution (48 counts per revolution)

// long prevCount = 0;
// unsigned long prevTime = 0;
static float sampleTime = 0.00f;  // [s]

static float posRevs = 0.0f;
static float velRevs = 0.0f;
static float accRevs = 0.0f;

static float prevPosRevs = 0.0f;
static float prevVelRevs = 0.0f;


// Interrupt Service Routine
void encoderISR() 
{   
    static uint8_t lastState = 0;

    uint8_t A = digitalRead(ENC_A_PIN);
    uint8_t B = digitalRead(ENC_B_PIN);

    uint8_t currState = (A << 1) | B;

    uint8_t transition = (lastState << 2) | currState;

    // Quadrature lookup table
    switch (transition) {
        case 0b0001:
        case 0b0111:
        case 0b1110:
        case 0b1000:
            encoderCount++;
            break;
        case 0b0010:
        case 0b0100:
        case 0b1101:
        case 0b1011:
            encoderCount--;
            break;
    }

    lastState = currState;
}

void encoderInit(float ts) {
    sampleTime = ts;  // Initialize sample time

    pinMode(ENC_A_PIN, INPUT);
    pinMode(ENC_B_PIN, INPUT);

    // prevTime = millis();

    attachInterrupt(
        digitalPinToInterrupt(ENC_A_PIN), 
        encoderISR, 
        CHANGE);

    attachInterrupt(
        digitalPinToInterrupt(ENC_B_PIN), 
        encoderISR, 
        CHANGE);
}

long getEncoderCount() {
    noInterrupts();  // Disable interrupts to read encoderCount safely
    long count = encoderCount;
    interrupts();    // Re-enable interrupts

    return count;
}

void resetEncoderCount() {
    noInterrupts();  // Disable interrupts to read encoderCount safely
    encoderCount = 0;
    interrupts();    // Re-enable interrupts

    posRevs = 0.0f;
    velRevs = 0.0f;
    accRevs = 0.0f;

    prevPosRevs = 0.0f;
    prevVelRevs = 0.0f;
}

void encoderUpdate() {
    long count = getEncoderCount();

    posRevs = static_cast<float>(count) / COUNTS_PER_REV;

    velRevs = (posRevs - prevPosRevs) / sampleTime;

    accRevs = (velRevs - prevVelRevs) / sampleTime;

    prevPosRevs = posRevs;
    prevVelRevs = velRevs;
}

float getMotorRevs() {
    return posRevs;
}

float getMotorSpeedRevs() {
    return velRevs;
}

float getMotorAccelRevs() {
    return accRevs;
}

float getMotorSpeedRPM() {
    return velRevs * 60.0f;  // Convert rev/s to RPM
}