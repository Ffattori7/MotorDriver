#pragma once

#include <Arduino.h>

void motorInit();

void driverEnable();
void shutdown();

void motorForward(uint8_t pwm);
void motorReverse(uint8_t pwm);
void motorStop();

float getMotorCurrent();

void monitorFeedback(uint32_t duration_ms);