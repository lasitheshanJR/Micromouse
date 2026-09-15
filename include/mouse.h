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

// High-level lifecycle of a competition run. The mouse first searches out to
// the goal (optimistically assuming unknown cells are open), then searches
// back to the start mapping fresh corridors, keeps re-exploring while an
// unknown route could still beat the best fully-known path, and finally
// commits to a provably optimal speed run.
enum class Phase : uint8_t {
  ExploreToGoal = 0,
  ExploreToStart = 1,
  SpeedRun = 2,       // timed/counted start -> goal run on the known path
  ReturnToStart = 3,  // drive goal -> start between speed runs (not counted)
  Done = 4,
};

const char* phaseName(Phase phase);

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

  // Look ahead for a wall `cells` cells in front (along the current heading).
  // On the mms simulator this maps to `wallFront N` (half-step aware), letting
  // the core pre-map straight corridors without physically driving them, which
  // trims exploration distance. Defaults to the adjacent-cell reading so
  // bridges without lookahead (hardware) still behave correctly.
  virtual bool wallFrontAt(int cells) {
    (void)cells;
    return wallFront();
  }

  // 45-degree forward-corner sensors (the "-45/+45" pair in the fan).
  // Defaults derive them from the cardinal sensors so a bridge that lacks
  // them (e.g. the mms simulator) still behaves correctly; HardwareIO
  // overrides these with the real diagonal sensors.
  virtual bool wallDiagonalLeft() { return wallFront() || wallLeft(); }
  virtual bool wallDiagonalRight() { return wallFront() || wallRight(); }

  // --- Motion primitives (one cell / 90 degrees) ----------------------
  virtual void moveForward() = 0;
  virtual void turnRight() = 0;
  virtual void turnLeft() = 0;

  // Move forward `cells` cells in a single straight burst. On the mms
  // simulator this maps to `moveForward N`, which earns the effective-distance
  // discount (each cell past the second counts as only half a point), directly
  // lowering the dominant best-run term of the score. The default falls back to
  // repeated single-cell moves so hardware bridges keep working unchanged.
  virtual void moveForward(int cells) {
    for (int i = 0; i < cells; i++) {
      moveForward();
    }
  }

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

  // --- Optional timed pause (used to stage the speed runs) ------------
  // Default no-op so bridges that do not need it still work.
  virtual void delayMs(int ms) { (void)ms; }

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

  // Goal is the central 2x2 block. Sample maze columns/rows 8-9 (1-based)
  // map to 7-8 (0-based), so the four goal cells are (7..8, 7..8).
  static constexpr int GOAL_X0 = 7;
  static constexpr int GOAL_X1 = 8;
  static constexpr int GOAL_Y0 = 7;
  static constexpr int GOAL_Y1 = 8;

  static bool isGoalCell(int x, int y) {
    return x >= GOAL_X0 && x <= GOAL_X1 && y >= GOAL_Y0 && y <= GOAL_Y1;
  }

  // Start cell (bottom-left). The return leg floods toward this cell.
  static constexpr int START_X = 0;
  static constexpr int START_Y = 0;

  static bool isStartCell(int x, int y) {
    return x == START_X && y == START_Y;
  }

  // --- Orientation-aware cost model (integer weights, MCU friendly) -----
  // Moving forward one cell costs STEP_COST; every 90-degree turn adds
  // TURN_COST. Weighting turns makes the flood prefer long straight runs
  // over winding serpentine paths. A 180-degree reversal costs 2*TURN_COST.
  static constexpr uint16_t STEP_COST = 2;
  static constexpr uint16_t TURN_COST = 1;
  // "Infinity" sentinel for the weighted distance map.
  static constexpr uint16_t COST_INF = 0xFFFF;

  // Re-exploration threshold: only keep chasing an unknown route if its
  // optimistic cost beats the best fully-known path by a MEANINGFUL amount.
  //
  // Scoring context: mms score = best_run_turns + best_run_effective_distance
  // + 0.1*(total_turns + total_effective_distance). An extra explore lap adds
  // its full length to the 0.1-weighted total term but can only reduce the
  // (unit-weighted) best term. So a lap is worthwhile only if the *potential*
  // best-run improvement is large relative to the lap's cost. We therefore
  // require the optimistic route to beat the known route by more than a fixed
  // margin AND by more than a fraction of the known path length, and we cap the
  // number of laps outright so a maze full of tempting-but-dead shortcuts can
  // never blow up the total penalty.
  static constexpr uint16_t EXPLORE_MARGIN = 4 * STEP_COST;
  // Additionally require the gain to exceed knownBest / EXPLORE_GAIN_DIVISOR.
  static constexpr int EXPLORE_GAIN_DIVISOR = 8;
  // Hard cap on optimistic explore laps (each lap = out to goal + back).
  // One out-and-back lap almost always discovers a near-optimal corridor; under
  // the 0.1-weighted total penalty, additional laps rarely recoup their cost, so
  // we cap at a single lap by default.
  static constexpr int MAX_EXPLORE_LAPS = 1;

  // How many cells ahead the corridor-lookahead probes for a terminating wall
  // while sensing. Uses `wallFront N`-style queries (mms) so the flood learns
  // where a straight corridor ends without driving every cell, trimming
  // exploration distance. Hardware bridges without lookahead fall back to the
  // adjacent-cell reading, so this simply has no effect there.
  static constexpr int LOOKAHEAD_CELLS = MAZE_SIZE;

  // Number of times to repeat the start->goal speed run at the end, and how
  // long to pause (blocking) before the first one begins.
  //
  // Only ONE speed run is performed: the run is deterministic over the
  // fully-known map, so repeats cannot lower the best-run term of the mms score
  // (best_run_turns + best_run_effective_distance) yet each repeat adds its full
  // distance and turns to the 0.1*(total...) penalty. One run minimises the
  // total penalty while still recording the optimal best run.
  static constexpr int SPEED_RUNS = 1;
  // Pre-run pause before the committed speed run. Does not affect the mms score,
  // but gives the operator a clear "about to run" window in the simulator.
  static constexpr int SPEED_RUN_DELAY_MS = 3000;

  explicit Mouse(MouseIO& io);

  // Clear the known maze and return to (0,0) facing north. Call before a run.
  void reset();

  // Sense -> flood -> move one cell. Returns true when the whole run
  // (explore + optional re-exploration + speed run) is complete.
  bool step();
  // Repeatedly step() until the run is complete.
  void run();

  bool atTarget() const { return isGoalCell(posX_, posY_); }
  bool atStart() const { return isStartCell(posX_, posY_); }
  bool done() const { return phase_ == Phase::Done; }
  Phase phase() const { return phase_; }
  int x() const { return posX_; }
  int y() const { return posY_; }
  int heading() const { return heading_; } // 0=N, 1=E, 2=S, 3=W
  uint16_t distanceAt(int x, int y) const { return cellCost(distMap_, x, y); }
  bool hasWall(int x, int y, int dir) const {
    return (wallMap_[x][y] & (1 << dir)) != 0;
  }
  bool visited(int x, int y) const { return visited_[x][y]; }

  int steps() const { return steps_; }
  int moves() const { return moves_; }
  int turns() const { return turns_; }
  int knownWalls() const;

private:
  void logf(LogLevel level, const char* format, ...);
  void updateWallsFromSensors();
  void addWall(int x, int y, int dir);

  // Orientation-aware weighted flood into `dist` (indexed [x][y][heading]).
  // `optimistic` = treat unvisited cells as fully open (used while exploring);
  // otherwise unknown walls block (used for the committed speed run).
  // Targets the goal region unless `toStart` selects the start cell.
  void flood(uint16_t dist[MAZE_SIZE][MAZE_SIZE][4], bool toStart,
             bool optimistic);

  // Best (min over heading) cost of a cell from a flooded 3D map.
  static uint16_t cellCost(const uint16_t dist[MAZE_SIZE][MAZE_SIZE][4], int x,
                           int y);

  // Advance toward the currently flooded target. Returns false if the mouse is
  // boxed in (no reachable neighbour). When `allowBatch` is true (committed
  // speed run / returns over a fully-known map), consecutive straight cells are
  // driven as a single moveForward(N) burst to earn the effective-distance
  // discount. During exploration `allowBatch` is false so the mouse senses at
  // every cell before deciding.
  bool moveOneStep(bool allowBatch);

  // Decide the next phase once a leg reaches its target. Returns true when the
  // whole run is complete.
  bool advancePhase();

  // Stage and start a speed run (pauses before the first one).
  void beginSpeedRun();

  // True if any unexplored route could still beat the best path through
  // fully-known cells (drives active re-exploration thresholding).
  bool worthExploring();

  bool inBounds(int x, int y) const;

  MouseIO& io_;
  uint8_t wallMap_[MAZE_SIZE][MAZE_SIZE];
  bool visited_[MAZE_SIZE][MAZE_SIZE];
  // Orientation-aware distances toward the active target: [x][y][heading].
  uint16_t distMap_[MAZE_SIZE][MAZE_SIZE][4];
  Phase phase_;
  int posX_;
  int posY_;
  int heading_;
  int steps_;
  int moves_;
  int turns_;
  int speedRunsDone_;
  // Cumulative move/turn counts captured when the current leg started, so each
  // speed run can report its own (not the total) move/turn count.
  int legStartMoves_;
  int legStartTurns_;
  // Number of optimistic explore laps completed, used to cap re-exploration so
  // the 0.1-weighted total-penalty term of the score stays bounded.
  int exploreLaps_;
};
