/*
  Hardware bridge: STM32F103 "Blue Pill" + L293D + 6x analog IR sensors.
  Implements MouseIO so the shared Mouse core can drive the real robot.
*/

#pragma once

#include "mouse.h"

class HardwareIO : public MouseIO {
public:
  void begin();

  bool wallFront() override;
  bool wallRight() override;
  bool wallLeft() override;
  bool wallDiagonalLeft() override;
  bool wallDiagonalRight() override;

  void moveForward() override;
  void turnRight() override;
  void turnLeft() override;

protected:
  void emitLog(LogLevel level, const char* message) override;

private:
  void logDistance(const char* label, float cm);
};
