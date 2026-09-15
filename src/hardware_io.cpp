#include "hardware_io.h"

#include <Arduino.h>
#include <cstdio>

namespace {
// L293D motor driver
constexpr uint8_t kIn1 = PA2;
constexpr uint8_t kIn2 = PA3;
constexpr uint8_t kIn3 = PA4;
constexpr uint8_t kIn4 = PA5;

// 6x Sharp analog IR distance sensors
constexpr uint8_t kSensorFarLeft = PA0;
constexpr uint8_t kSensorLeft = PA5;
constexpr uint8_t kSensorFrontLeft = PA6;
constexpr uint8_t kSensorFrontRight = PA7;
constexpr uint8_t kSensorRight = PB0;
constexpr uint8_t kSensorFarRight = PB1;

constexpr float kWallThresholdCm = 18.0f;
constexpr float kEmaAlpha = 0.3f;

float filtered[6] = {0};

float readDistanceCm(uint8_t pin) {
  int raw = analogRead(pin);
  if (raw < 100) {
    return 80.0f;
  }

  float voltage = (raw / 4095.0f) * 3.3f;
  if (voltage < 0.4f) {
    return 80.0f;
  }

  float distance = 27.619f * pow(voltage, -1.173f);
  return constrain(distance, 10.0f, 80.0f);
}

float filteredDistanceCm(uint8_t index, uint8_t pin) {
  float current = readDistanceCm(pin);
  if (filtered[index] == 0) {
    filtered[index] = current;
  } else {
    filtered[index] =
        (kEmaAlpha * current) + ((1.0f - kEmaAlpha) * filtered[index]);
  }
  return filtered[index];
}

void setMotors(int leftSpeed, int rightSpeed) {
  digitalWrite(kIn1, leftSpeed > 0 ? HIGH : LOW);
  digitalWrite(kIn2, leftSpeed < 0 ? HIGH : LOW);
  digitalWrite(kIn3, rightSpeed > 0 ? HIGH : LOW);
  digitalWrite(kIn4, rightSpeed < 0 ? HIGH : LOW);
}

void stopMotors() { setMotors(0, 0); }
} // namespace

void HardwareIO::emitLog(LogLevel level, const char* message) {
  Serial.print('[');
  Serial.print(millis());
  Serial.print("][");
  Serial.print(logLevelName(level));
  Serial.print("] ");
  Serial.println(message);
}

void HardwareIO::logDistances(const char* label, float a, float b) {
  char buffer[64];
  std::snprintf(buffer, sizeof(buffer), "IR %s: %d %d cm", label, (int)a, (int)b);
  log(LogLevel::Debug, buffer);
}

void HardwareIO::begin() {
  pinMode(kIn1, OUTPUT);
  pinMode(kIn2, OUTPUT);
  pinMode(kIn3, OUTPUT);
  pinMode(kIn4, OUTPUT);

  stopMotors();
}

bool HardwareIO::wallFront() {
  float fl = filteredDistanceCm(2, kSensorFrontLeft);
  float fr = filteredDistanceCm(3, kSensorFrontRight);
  if (logEnabled(LogLevel::Debug)) {
    logDistances("front", fl, fr);
  }
  return ((fl + fr) / 2.0f) < kWallThresholdCm;
}

bool HardwareIO::wallLeft() {
  float l = filteredDistanceCm(1, kSensorLeft);
  float fl = filteredDistanceCm(0, kSensorFarLeft);
  if (logEnabled(LogLevel::Debug)) {
    logDistances("left", l, fl);
  }
  return (l < kWallThresholdCm) || (fl < (kWallThresholdCm - 3.0f));
}

bool HardwareIO::wallRight() {
  float r = filteredDistanceCm(4, kSensorRight);
  float fr = filteredDistanceCm(5, kSensorFarRight);
  if (logEnabled(LogLevel::Debug)) {
    logDistances("right", r, fr);
  }
  return (r < kWallThresholdCm) || (fr < (kWallThresholdCm - 3.0f));
}

void HardwareIO::moveForward() {
  setMotors(1, 1);
  delay(400);
  stopMotors();
  delay(100);
}

void HardwareIO::turnRight() {
  setMotors(1, -1);
  delay(200);
  stopMotors();
}

void HardwareIO::turnLeft() {
  setMotors(-1, 1);
  delay(200);
  stopMotors();
}
