/*
  Simulator bridge: mackorone/mms over stdin/stdout.
  Implements MouseIO so the shared Mouse core runs on the host.

  IMPORTANT: stdout is the mms protocol channel. All logs go to stderr,
  which mms displays in the "Run Output" tab.
*/

#pragma once

#include <string>

#include "mouse.h"

class SimIO : public MouseIO {
public:
  bool wallFront() override;
  bool wallRight() override;
  bool wallLeft() override;

  void moveForward() override;
  void turnRight() override;
  void turnLeft() override;

  void showWall(int x, int y, int dir) override;
  void showText(int x, int y, const char* text) override;

  bool resetRequested() override;
  void resetAck() override;

protected:
  void emitLog(LogLevel level, const char* message) override;

private:
  std::string send(const std::string& command);
  void sendCommand(const std::string& command);
  bool query(const std::string& command);
};
