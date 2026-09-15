/*
  Shared maze-solving core.
  --------------------------------------------------------------------
  This header is platform-agnostic: it must not include <Arduino.h> or
  any simulator headers. The two "bridges" (adapters) implement MouseIO:

    * HardwareIO (src/hardware_io.cpp) - real robot via IR sensors + L293D
    * SimIO      (sim/sim_io.cpp)      - mackorone/mms over stdin/stdout
*/

#pragma once

#include <cstdint>

enum class LogLevel : uint8_t {
  Debug = 0,
  Info = 1,
  Warn = 2,
  Error = 3,
  None = 4,
};

const char* logLevelName(LogLevel level);

// Bridge between the maze algorithm and whatever is driving the robot.
class MouseIO {
public:
  virtual ~MouseIO() = default;

  // --- Logging ---------------------------------------------------------
  // The bridge owns where logs go (stderr, Serial, ...). Set the level to
  // None to silence, Debug to capture everything.
  void setLogLevel(LogLevel level) { logLevel_ = level; }
  LogLevel logLevel() const { return logLevel_; }
  bool logEnabled(LogLevel level) const {
    return logLevel_ != LogLevel::None && level >= logLevel_;
  }
  void log(LogLevel level, const char* message) {
    if (logEnabled(level)) {
      emitLog(level, message);
    }
  }

  // --- Wall sensing relative to the robot's current heading -----------
  virtual bool wallFront() = 0;
  virtual bool wallRight() = 0;
  virtual bool wallLeft() = 0;

  // --- Motion primitives (one cell / 90 degrees) ----------------------
  virtual void moveForward() = 0;
  virtual void turnRight() = 0;
  virtual void turnLeft() = 0;

  // --- Optional visualization hooks (no-op on real hardware) ----------
  virtual void showWall(int x, int y, int dir) {
    (void)x;
    (void)y;
    (void)dir;
  }
  virtual void showText(int x, int y, const char* text) {
    (void)x;
    (void)y;
    (void)text;
  }

  // --- Optional crash/reset handling (only the simulator uses this) ---
  virtual bool resetRequested() { return false; }
  virtual void resetAck() {}

protected:
  // Adapters override this to emit a message somewhere.
  virtual void emitLog(LogLevel level, const char* message) {
    (void)level;
    (void)message;
  }

private:
  LogLevel logLevel_ = LogLevel::Info;
};

// Hardware-agnostic flood-fill maze solver.
class Mouse {
public:
  static constexpr int MAZE_SIZE = 16;
  static constexpr int TARGET_X = 7;
  static constexpr int TARGET_Y = 7;

  explicit Mouse(MouseIO& io);

  // Clear the known maze and return to (0,0) facing north. Call before a run.
  void reset();

  // Sense -> flood fill -> move one cell. Returns true once at the target.
  bool step();
  // Repeatedly step() until the target is reached.
  void run();

  bool atTarget() const { return posX_ == TARGET_X && posY_ == TARGET_Y; }
  int x() const { return posX_; }
  int y() const { return posY_; }
  int heading() const { return heading_; } // 0=N, 1=E, 2=S, 3=W
  uint8_t distanceAt(int x, int y) const { return distMap_[x][y]; }
  bool hasWall(int x, int y, int dir) const {
    return (wallMap_[x][y] & (1 << dir)) != 0;
  }

  int steps() const { return steps_; }
  int moves() const { return moves_; }
  int turns() const { return turns_; }
  int knownWalls() const;

private:
  void logf(LogLevel level, const char* format, ...);
  void updateWallsFromSensors();
  void addWall(int x, int y, int dir);
  void floodFill();
  void moveOneStep();
  bool inBounds(int x, int y) const;

  MouseIO& io_;
  uint8_t wallMap_[MAZE_SIZE][MAZE_SIZE];
  uint8_t distMap_[MAZE_SIZE][MAZE_SIZE];
  int posX_;
  int posY_;
  int heading_;
  int steps_;
  int moves_;
  int turns_;
};
