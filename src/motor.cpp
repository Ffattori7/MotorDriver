#include "motor.h"

const uint8_t EN_PIN  = 7;

const uint8_t IN1_PIN = 8;
const uint8_t IN2_PIN = 9;

const uint8_t D1_PIN = 11;
const uint8_t D2_PIN = 12;

const uint8_t FB_PIN = A0;

void motorInit() {
    pinMode(EN_PIN, OUTPUT);

    pinMode(IN1_PIN, OUTPUT);
    pinMode(IN2_PIN, OUTPUT);

    pinMode(D1_PIN, OUTPUT);
    pinMode(D2_PIN, OUTPUT);

    pinMode(FB_PIN, INPUT);
}

void driverEnable() {
  digitalWrite(EN_PIN, HIGH);

  // Disable Pins Inactive
  digitalWrite(D1_PIN, LOW);  // D1 HIGH = disable, LOW = enable
  digitalWrite(D2_PIN, HIGH);  //D2 LOW = disable, HIGH = enable
}

void shutdown() {
  digitalWrite(EN_PIN, LOW);

  // Disable Pins Active
  digitalWrite(D1_PIN, HIGH);  // D1 HIGH = disable, LOW = enable
  digitalWrite(D2_PIN, LOW);  //D2 LOW = disable, HIGH = enable
}

void motorStop() {
  analogWrite(IN1_PIN, 0);
  analogWrite(IN2_PIN, 0);
}

void motorForward(uint8_t pwm) {
  analogWrite(IN1_PIN, pwm);
  analogWrite(IN2_PIN, 0);
}

void motorReverse(uint8_t pwm) {
  analogWrite(IN1_PIN, 0);
  analogWrite(IN2_PIN, pwm);
}

float getMotorCurrent() {
  int rawFB = analogRead(FB_PIN);
  
  float voltage = (rawFB / 1023.0f) * 5.0f;

  float current = voltage / 0.525f;  // 525 mV/A

  return current;
}

// void monitorFeedback(uint32_t duration_ms) {
//   uint32_t start_time = millis();

//   long sumADC = 0;
//   uint32_t nSamples = 0;

//   // encoderCount = 0;  // Reset encoder count at the start of monitoring
  
//   while (millis() - start_time < duration_ms) {
//     sumADC += analogRead(FB_PIN);
//     nSamples++;
//     delay(100);  // Sample every 100 ms (10 Hz)
//   }

//   float avgADC = (float)sumADC / nSamples;
//   float voltage = (avgADC / 1023.0) * 5.0;  // Assuming a 10-bit ADC and 5V reference
//   float current = voltage / 0.525;  // Assuming a 525mV/A current sense amplifier

//   Serial.println("===== Feedback Summary =====");
//   Serial.print("Samples: ");
//   Serial.println(nSamples);

//   // Serial.print("PWM: ");
//   // Serial.println(PWM_TEST);

//   Serial.print("Average ADC: ");
//   Serial.println(avgADC, 1);

//   Serial.print("Average Voltage: ");
//   Serial.print(voltage, 3);
//   Serial.println(" V");

//   Serial.print("Average Current: ");
//   Serial.print(current, 3);
//   Serial.println(" A");

//   // Serial.print("Encoder Counts: ");
//   // Serial.println(encoderCount);
//   // Serial.println("============================");
// }
