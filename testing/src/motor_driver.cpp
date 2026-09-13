/*
  Motor Test Routine — STM32F103 "Blue Pill" + L293D
  --------------------------------------------------
  Right Motor : IN1 -> PA2, IN2 -> PA3
  Left Motor  : IN3 -> PA4, IN4 -> PA5
*/

#include <Arduino.h>

const uint8_t IN1 = PA2; // Right motor
const uint8_t IN2 = PA3; // Right motor
const uint8_t IN3 = PA4; // Left motor
const uint8_t IN4 = PA5; // Left motor

const uint8_t LED_PIN = PC13;

void setMotors(int leftSpeed, int rightSpeed);
void stopMotors();

void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  stopMotors();
  delay(2000); // 2-second delay before starting test
}

void loop() {
  digitalWrite(LED_PIN, LOW); // LED ON during movement

  // 1. Move Forward at medium speed
  setMotors(150, 150);
  delay(2000);

  // 2. Stop
  stopMotors();
  digitalWrite(LED_PIN, HIGH);
  delay(1000);

  // 3. Move Reverse
  setMotors(-150, -150);
  delay(2000);

  // 4. Stop
  stopMotors();
  delay(1000);

  // 5. Pivot Left
  setMotors(-140, 140);
  delay(1000);

  // 6. Pivot Right
  setMotors(140, -140);
  delay(1000);

  // 7. Stop and wait
  stopMotors();
  delay(3000);
}

void setMotors(int leftSpeed, int rightSpeed) {
  leftSpeed  = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  // Left Motor (IN3, IN4)
  if (leftSpeed >= 0) {
    analogWrite(IN3, leftSpeed);
    analogWrite(IN4, 0);
  } else {
    analogWrite(IN3, 0);
    analogWrite(IN4, -leftSpeed);
  }

  // Right Motor (IN1, IN2)
  if (rightSpeed >= 0) {
    analogWrite(IN1, rightSpeed);
    analogWrite(IN2, 0);
  } else {
    analogWrite(IN1, 0);
    analogWrite(IN2, -rightSpeed);
  }
}

void stopMotors() {
  analogWrite(IN1, 0);
  analogWrite(IN2, 0);
  analogWrite(IN3, 0);
  analogWrite(IN4, 0);
}
