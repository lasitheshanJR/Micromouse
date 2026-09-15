#include "hardware_io.h"

#include <Arduino.h>
#include <cstdio>

#include "config.h"

namespace {
float filtered[config::kSensorCount] = {0};

float readDistanceCm(uint8_t pin) {
  int raw = analogRead(pin);
  if (raw < 100) {
    return config::kNoWallCm;
  }

  float voltage = (raw / 4095.0f) * 3.3f;
  if (voltage < 0.4f) {
    return config::kNoWallCm;
  }

  float distance = 27.619f * pow(voltage, -1.173f);
  return constrain(distance, 10.0f, config::kNoWallCm);
}

// Read one sensor through the EMA filter. Call once per sensor per step.
float filteredDistanceCm(int index) {
  float current = readDistanceCm(config::kSensorPins[index]);
  if (filtered[index] == 0) {
    filtered[index] = current;
  } else {
    filtered[index] =
        (config::kSensorAlpha * current) +
        ((1.0f - config::kSensorAlpha) * filtered[index]);
  }
  return filtered[index];
}

void setMotors(int leftSpeed, int rightSpeed) {
  digitalWrite(config::kMotorIn1, leftSpeed > 0 ? HIGH : LOW);
  digitalWrite(config::kMotorIn2, leftSpeed < 0 ? HIGH : LOW);
  digitalWrite(config::kMotorIn3, rightSpeed > 0 ? HIGH : LOW);
  digitalWrite(config::kMotorIn4, rightSpeed < 0 ? HIGH : LOW);
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

void HardwareIO::logDistance(const char* label, float cm) {
  char buffer[48];
  std::snprintf(buffer, sizeof(buffer), "IR %-5s: %d cm", label, (int)cm);
  log(LogLevel::Debug, buffer);
}

void HardwareIO::begin() {
  pinMode(config::kMotorIn1, OUTPUT);
  pinMode(config::kMotorIn2, OUTPUT);
  pinMode(config::kMotorIn3, OUTPUT);
  pinMode(config::kMotorIn4, OUTPUT);

  stopMotors();
}

bool HardwareIO::wallFront() {
  float fl = filteredDistanceCm(config::kFrontLeft);
  float fr = filteredDistanceCm(config::kFrontRight);
  if (logEnabled(LogLevel::Debug)) {
    logDistance("frontL", fl);
    logDistance("frontR", fr);
  }
  return fl < config::kFrontWallCm || fr < config::kFrontWallCm;
}

bool HardwareIO::wallLeft() {
  float l = filteredDistanceCm(config::kFarLeft);
  if (logEnabled(LogLevel::Debug)) {
    logDistance("left", l);
  }
  return l < config::kSideWallCm;
}

bool HardwareIO::wallRight() {
  float r = filteredDistanceCm(config::kFarRight);
  if (logEnabled(LogLevel::Debug)) {
    logDistance("right", r);
  }
  return r < config::kSideWallCm;
}

bool HardwareIO::wallDiagonalLeft() {
  float dl = filteredDistanceCm(config::kDiagLeft);
  if (logEnabled(LogLevel::Debug)) {
    logDistance("diagL", dl);
  }
  return dl < config::kDiagonalWallCm;
}

bool HardwareIO::wallDiagonalRight() {
  float dr = filteredDistanceCm(config::kDiagRight);
  if (logEnabled(LogLevel::Debug)) {
    logDistance("diagR", dr);
  }
  return dr < config::kDiagonalWallCm;
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

void HardwareIO::delayMs(int ms) {
  if (ms > 0) {
    delay((unsigned long)ms);
  }
}
