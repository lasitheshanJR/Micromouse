/*
  CalibrationIO - hardware bring-up helper (replaces the maze solver).
  --------------------------------------------------------------------
  Two modes, toggled over Serial at 115200:

    Stream (default) - prints raw ADC + computed distance (cm) for all
                       6 IR sensors, ~10 Hz.
    Motor            - single-character motor commands:
                         f = forward    b = backward   l = pivot left
                         r = pivot right  s = stop     i = back to stream
                         h = help

  Uses the config.h pin mapping (single source of truth). Motor pins
  (L293D, PWM): right = PA2/PA3, left = PA4/PA5.
*/

#include <Arduino.h>

#include "config.h"

namespace {
constexpr int kMotorSpeed = 150;
const char* const kSensorLabel[config::kSensorCount] = {
    "farL", "diagL", "frontL", "frontR", "diagR", "farR"};

float rawToCm(int raw) {
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
} // namespace

class CalibrationIO {
public:
  void begin() {
    Serial.begin(115200);
    analogReadResolution(12);

    pinMode(config::kMotorIn1, OUTPUT);
    pinMode(config::kMotorIn2, OUTPUT);
    pinMode(config::kMotorIn3, OUTPUT);
    pinMode(config::kMotorIn4, OUTPUT);
    stopMotors();

    Serial.println("CalibrationIO ready");
    printHelp();
  }

  void update() {
    if (mode_ == Mode::Motor) {
      if (Serial.available()) {
        handleMotorCommand((char)Serial.read());
      }
      return;
    }
    streamReadings();
  }

private:
  enum class Mode : uint8_t { Stream, Motor };

  Mode mode_ = Mode::Stream;

  void setMotors(int leftSpeed, int rightSpeed) {
    leftSpeed = constrain(leftSpeed, -255, 255);
    rightSpeed = constrain(rightSpeed, -255, 255);

    // Left motor: IN3 = PA4, IN4 = PA5
    if (leftSpeed >= 0) {
      analogWrite(config::kMotorIn3, leftSpeed);
      analogWrite(config::kMotorIn4, 0);
    } else {
      analogWrite(config::kMotorIn3, 0);
      analogWrite(config::kMotorIn4, -leftSpeed);
    }

    // Right motor: IN1 = PA2, IN2 = PA3
    if (rightSpeed >= 0) {
      analogWrite(config::kMotorIn1, rightSpeed);
      analogWrite(config::kMotorIn2, 0);
    } else {
      analogWrite(config::kMotorIn1, 0);
      analogWrite(config::kMotorIn2, -rightSpeed);
    }
  }

  void stopMotors() {
    analogWrite(config::kMotorIn1, 0);
    analogWrite(config::kMotorIn2, 0);
    analogWrite(config::kMotorIn3, 0);
    analogWrite(config::kMotorIn4, 0);
  }

  void printHelp() {
    Serial.println("m = motor mode   i = stream IR   h = help");
    Serial.println("motor: f=fwd  b=back  l=left  r=right  s=stop");
  }

  void streamReadings() {
    Serial.print("IR ");
    for (int i = 0; i < config::kSensorCount; i++) {
      int raw = analogRead(config::kSensorPins[i]);
      Serial.print(kSensorLabel[i]);
      Serial.print('=');
      Serial.print(raw);
      Serial.print('(');
      Serial.print((int)rawToCm(raw));
      Serial.print("cm) ");
    }
    Serial.println();

    if (Serial.available()) {
      char c = (char)Serial.read();
      if (c == 'm') {
        mode_ = Mode::Motor;
        Serial.println("motor mode: f/b/l/r/s, i = stream, h = help");
      } else if (c == 'h') {
        printHelp();
      }
    }

    delay(100);
  }

  void handleMotorCommand(char command) {
    switch (command) {
    case 'f':
      setMotors(kMotorSpeed, kMotorSpeed);
      break;
    case 'b':
      setMotors(-kMotorSpeed, -kMotorSpeed);
      break;
    case 'l':
      setMotors(-kMotorSpeed, kMotorSpeed);
      break;
    case 'r':
      setMotors(kMotorSpeed, -kMotorSpeed);
      break;
    case 's':
      stopMotors();
      break;
    case 'i':
      stopMotors();
      mode_ = Mode::Stream;
      Serial.println("streaming IR readings");
      break;
    case 'h':
      printHelp();
      break;
    default:
      break;
    }
  }
};

namespace {
CalibrationIO io;
}

void setup() { io.begin(); }

void loop() { io.update(); }
