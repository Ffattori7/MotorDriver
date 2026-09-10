#pragma once

#include <Arduino.h>

void encoderInit(float ts);
long getEncoderCount();
float getMotorRevs();
float getMotorSpeedRevs();
float getMotorAccelRevs();
float getMotorSpeedRPM();
void resetEncoderCount();
void encoderUpdate();