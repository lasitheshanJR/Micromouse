#include "mouse.h"

#include <cstdarg>
#include <cstdio>
#include <cstdlib>

namespace {
const int DX[4] = {0, 1, 0, -1}; // N, E, S, W
const int DY[4] = {1, 0, -1, 0};
const int OPPOSITE[4] = {2, 3, 0, 1};
const char* DIR_NAME[4] = {"N", "E", "S", "W"};

// Human-readable local topology from the three cardinal walls.
const char* junctionName(bool front, bool left, bool right) {
  if (front && left && right)
    return "cross";
  if (front && left)
    return "open-right";
  if (front && right)
    return "open-left";
  if (left && right)
    return "corridor";
  if (front)
    return "dead-end";
  if (left)
    return "open-front-right";
  if (right)
    return "open-front-left";
  return "open";
}
} // namespace

const char* logLevelName(LogLevel level) {
  switch (level) {
  case LogLevel::Debug:
    return "DEBUG";
  case LogLevel::Info:
    return "INFO";
  case LogLevel::Warn:
    return "WARN";
  case LogLevel::Error:
    return "ERROR";
  default:
    return "NONE";
  }
}

const char* phaseName(Phase phase) {
  switch (phase) {
  case Phase::ExploreToGoal:
    return "EXPLORE->GOAL";
  case Phase::ExploreToStart:
    return "EXPLORE->START";
  case Phase::SpeedRun:
    return "SPEED-RUN";
  case Phase::ReturnToStart:
    return "RETURN->START";
  default:
    return "DONE";
  }
}

Mouse::Mouse(MouseIO& io)
    : io_(io), phase_(Phase::ExploreToGoal), posX_(0), posY_(0), heading_(0),
      steps_(0), moves_(0), turns_(0), speedRunsDone_(0), legStartMoves_(0),
      legStartTurns_(0), exploreLaps_(0) {}

void Mouse::logf(LogLevel level, const char* format, ...) {
  if (!io_.logEnabled(level)) {
    return;
  }

  char buffer[160];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);

  io_.log(level, buffer);
}

void Mouse::reset() {
  phase_ = Phase::ExploreToGoal;
  posX_ = START_X;
  posY_ = START_Y;
  heading_ = 0;
  steps_ = 0;
  moves_ = 0;
  turns_ = 0;
  speedRunsDone_ = 0;
  legStartMoves_ = 0;
  legStartTurns_ = 0;
  exploreLaps_ = 0;

  for (int x = 0; x < MAZE_SIZE; x++) {
    for (int y = 0; y < MAZE_SIZE; y++) {
      wallMap_[x][y] = 0;
      visited_[x][y] = false;
      for (int h = 0; h < 4; h++) {
        distMap_[x][y][h] = COST_INF;
      }
    }
  }

  visited_[START_X][START_Y] = true;

  logf(LogLevel::Info, "reset: pos=(%d,%d) heading=N goal=(%d..%d,%d..%d)",
       START_X, START_Y, GOAL_X0, GOAL_X1, GOAL_Y0, GOAL_Y1);
}

bool Mouse::inBounds(int x, int y) const {
  return x >= 0 && x < MAZE_SIZE && y >= 0 && y < MAZE_SIZE;
}

int Mouse::knownWalls() const {
  int count = 0;
  for (int x = 0; x < MAZE_SIZE; x++) {
    for (int y = 0; y < MAZE_SIZE; y++) {
      for (int dir = 0; dir < 4; dir++) {
        if (wallMap_[x][y] & (1 << dir)) {
          count++;
        }
      }
    }
  }
  return count;
}

void Mouse::addWall(int x, int y, int dir) {
  wallMap_[x][y] |= (1 << dir);
  io_.showWall(x, y, dir);
  logf(LogLevel::Debug, "wall added: (%d,%d) %s", x, y, DIR_NAME[dir]);

  int nx = x + DX[dir];
  int ny = y + DY[dir];
  if (inBounds(nx, ny)) {
    wallMap_[nx][ny] |= (1 << OPPOSITE[dir]);
    io_.showWall(nx, ny, OPPOSITE[dir]);
    logf(LogLevel::Debug, "wall added: (%d,%d) %s", nx, ny,
         DIR_NAME[OPPOSITE[dir]]);
  }
}

void Mouse::updateWallsFromSensors() {
  struct Observation {
    bool front;
    bool left;
    bool right;
    bool diagLeft;
    bool diagRight;
  } obs;

  obs.front = io_.wallFront();
  obs.left = io_.wallLeft();
  obs.right = io_.wallRight();
  obs.diagLeft = io_.wallDiagonalLeft();
  obs.diagRight = io_.wallDiagonalRight();

  logf(LogLevel::Debug,
       "sense: pos=(%d,%d) heading=%s front=%d left=%d right=%d diagL=%d "
       "diagR=%d",
       posX_, posY_, DIR_NAME[heading_], obs.front, obs.left, obs.right,
       obs.diagLeft, obs.diagRight);

  // A 45-degree beam looking at the forward corner must hit either the side
  // wall or something ahead. If a diagonal sees a wall that the side sensor
  // does not, treat it as a front wall: this catches front walls the 0-degree
  // pair missed (misalignment, sensor cone gaps). For bridges that derive the
  // diagonal from the cardinal sensors this reduces to the original reading.
  bool front =
      obs.front || (obs.diagLeft && !obs.left) || (obs.diagRight && !obs.right);
  bool left = obs.left;
  bool right = obs.right;

  logf(LogLevel::Debug, "junction: %s (front=%d left=%d right=%d)",
       junctionName(front, left, right), front, left, right);

  if (front)
    addWall(posX_, posY_, heading_);
  if (right)
    addWall(posX_, posY_, (heading_ + 1) % 4);
  if (left)
    addWall(posX_, posY_, (heading_ + 3) % 4);

  // Corridor lookahead: if nothing blocks straight ahead, probe further cells
  // along the current heading to find where the corridor's forward passage ends
  // and record that far wall now. This lets the flood reason about the straight
  // run without physically driving each cell first, trimming exploration
  // distance (the 0.1*total penalty). We only learn about *forward* passages
  // between cells straight ahead; side walls of those cells are still sensed
  // when (if) the mouse actually visits them, so this never fabricates unknown
  // information. Bounded by LOOKAHEAD_CELLS to keep the query count small.
  if (!front) {
    int lx = posX_;
    int ly = posY_;
    for (int step = 1; step <= LOOKAHEAD_CELLS; step++) {
      int nx = lx + DX[heading_];
      int ny = ly + DY[heading_];
      if (!inBounds(nx, ny)) {
        break;
      }
      bool wallAhead = io_.wallFrontAt(step + 1);
      if (wallAhead) {
        // Wall on the far side of cell (nx,ny) along the heading.
        addWall(nx, ny, heading_);
        break;
      }
      lx = nx;
      ly = ny;
    }
  }
}

uint16_t Mouse::cellCost(const uint16_t dist[MAZE_SIZE][MAZE_SIZE][4], int x,
                         int y) {
  uint16_t best = COST_INF;
  for (int h = 0; h < 4; h++) {
    if (dist[x][y][h] < best) {
      best = dist[x][y][h];
    }
  }
  return best;
}

// Orientation-aware weighted flood over (x,y,heading) states.
// State cost accounts for both travel (STEP_COST per cell) and steering
// (TURN_COST per 90 degrees), so the resulting distance transform rewards
// long straight corridors. A cell's heading encodes the direction the mouse
// is *facing* upon arrival, i.e. the direction it travelled to get there.
//
// The flood runs backwards from the target: dist[x][y][h] is the cost to
// reach the target starting at (x,y) already facing h. Neighbour relaxation
// therefore reasons about the move the mouse would make *from* (x,y): going
// to a neighbour in direction d costs STEP_COST plus the turn from h to d.
void Mouse::flood(uint16_t dist[MAZE_SIZE][MAZE_SIZE][4], bool toStart,
                  bool optimistic) {
  // Label-correcting shortest path (Bellman-Ford style) over (x,y,heading)
  // states with a FIFO work queue. Memory-light: a single ring buffer of
  // packed states, re-enqueued whenever a cheaper cost is found. Terminates
  // because costs are strictly positive and monotonically decreasing per
  // state. dist[x][y][h] = cost to reach the target from (x,y) already facing
  // h (heading = the direction the mouse last travelled).

  // A passage from (x,y) toward direction d is blocked if there is a known
  // wall. In pessimistic (speed-run) mode an unexplored cell is also treated
  // as blocked so the committed path only uses fully-sensed corridors.
  auto blocked = [&](int x, int y, int d) -> bool {
    if (wallMap_[x][y] & (1 << d)) {
      return true;
    }
    if (!optimistic) {
      int nx = x + DX[d];
      int ny = y + DY[d];
      if (!visited_[x][y]) {
        return true;
      }
      if (inBounds(nx, ny) && !visited_[nx][ny]) {
        return true;
      }
    }
    return false;
  };

  for (int x = 0; x < MAZE_SIZE; x++) {
    for (int y = 0; y < MAZE_SIZE; y++) {
      for (int h = 0; h < 4; h++) {
        dist[x][y][h] = COST_INF;
      }
    }
  }

  // Ring buffer of packed states: ((x*MAZE_SIZE + y) << 2) | heading.
  // A per-state "in queue" flag keeps each state present at most once, so the
  // buffer never needs more than the number of distinct states (+1 slack for
  // the ring's empty/full distinction). Static => shared, predictable MCU RAM.
  const int NSTATE = MAZE_SIZE * MAZE_SIZE * 4;
  const int QCAP = NSTATE + 1;
  static uint16_t queue[MAZE_SIZE * MAZE_SIZE * 4 + 1];
  static bool inQueue[MAZE_SIZE * MAZE_SIZE * 4];
  for (int i = 0; i < NSTATE; i++) {
    inQueue[i] = false;
  }
  int head = 0, tail = 0;

  auto push = [&](int x, int y, int h, uint16_t cost) {
    if (cost >= dist[x][y][h]) {
      return;
    }
    dist[x][y][h] = cost;
    int idx = (x * MAZE_SIZE + y) * 4 + h;
    if (inQueue[idx]) {
      return; // already pending; it will be processed with the lowered cost
    }
    inQueue[idx] = true;
    queue[tail] = (uint16_t)(((x * MAZE_SIZE + y) << 2) | h);
    tail = (tail + 1) % QCAP;
  };

  // Seed target cells at cost 0 for every arrival heading.
  if (toStart) {
    for (int h = 0; h < 4; h++) {
      push(START_X, START_Y, h, 0);
    }
  } else {
    for (int gx = GOAL_X0; gx <= GOAL_X1; gx++) {
      for (int gy = GOAL_Y0; gy <= GOAL_Y1; gy++) {
        for (int h = 0; h < 4; h++) {
          push(gx, gy, h, 0);
        }
      }
    }
  }

  while (head != tail) {
    uint16_t packed = queue[head];
    head = (head + 1) % QCAP;
    int h = packed & 3;
    int cell = packed >> 2;
    int cx = cell / MAZE_SIZE;
    int cy = cell % MAZE_SIZE;
    inQueue[(cx * MAZE_SIZE + cy) * 4 + h] = false;
    uint16_t cost = dist[cx][cy][h];

    // Relax predecessors. To arrive at (cx,cy) facing h, the mouse travelled
    // in direction h from the cell one step opposite h. So the predecessor is
    // at (cx - DX[h], cy - DY[h]) and the wall crossed is side h of it.
    int px = cx - DX[h];
    int py = cy - DY[h];
    if (!inBounds(px, py) || blocked(px, py, h)) {
      continue;
    }
    // From that predecessor, whatever heading ph it had before the forward
    // move, the added cost is STEP_COST plus the turn ph->h.
    for (int ph = 0; ph < 4; ph++) {
      int turn = (h - ph + 4) % 4;
      int quarters = (turn == 3) ? 1 : turn; // 1 == right, 3 == left, 2 == U
      uint16_t add = (uint16_t)(STEP_COST + quarters * TURN_COST);
      push(px, py, ph, (uint16_t)(cost + add));
    }
  }

  logf(LogLevel::Debug, "flood(%s,%s): cost(%d,%d)=%u",
       toStart ? "start" : "goal", optimistic ? "optimistic" : "known", posX_,
       posY_, (unsigned)cellCost(dist, posX_, posY_));
}

bool Mouse::moveOneStep(bool allowBatch) {
  int bestDir = -1;
  uint32_t minCost = COST_INF;

  // Choose the neighbour that minimises (turn cost to face it) + STEP_COST +
  // (flooded cost of arriving there facing that direction). Because turning
  // is penalised, continuing straight is preferred on ties, which produces
  // the long straight runs we want.
  for (int i = 0; i < 4; i++) {
    if (wallMap_[posX_][posY_] & (1 << i)) {
      continue;
    }
    int nx = posX_ + DX[i];
    int ny = posY_ + DY[i];
    if (!inBounds(nx, ny)) {
      continue;
    }
    uint16_t arrive = distMap_[nx][ny][i];
    if (arrive == COST_INF) {
      continue;
    }
    int turn = (i - heading_ + 4) % 4;
    int quarters = (turn == 3) ? 1 : turn;
    uint32_t cost = (uint32_t)arrive + STEP_COST + quarters * TURN_COST;
    if (cost < minCost) {
      minCost = cost;
      bestDir = i;
    }
  }

  if (bestDir < 0) {
    logf(LogLevel::Warn, "boxed in at (%d,%d) heading=%s", posX_, posY_,
         DIR_NAME[heading_]);
    return false;
  }

  int dirDiff = (bestDir - heading_ + 4) % 4;
  const char* action = "forward";
  if (dirDiff == 1) {
    io_.turnRight();
    turns_++;
    action = "turn right";
  } else if (dirDiff == 2) {
    io_.turnRight();
    io_.turnRight();
    turns_ += 2;
    action = "turn around";
  } else if (dirDiff == 3) {
    io_.turnLeft();
    turns_++;
    action = "turn left";
  }

  int fromX = posX_;
  int fromY = posY_;
  const char* fromHeading = DIR_NAME[heading_];

  heading_ = bestDir;

  // Always take the first cell in the chosen direction.
  posX_ += DX[heading_];
  posY_ += DY[heading_];
  visited_[posX_][posY_] = true;
  int run = 1;

  // Batch further straight cells into the same burst when driving a fully-known
  // map (speed run / returns). A single moveForward(N) earns the mms
  // effective-distance discount (each cell past the second counts as half a
  // point), shrinking the dominant best-run term of the score. Turns are only
  // counted once, at the segment start.
  //
  // Correctness/safety: we only extend the burst into cells that are already
  // visited_ (walls fully sensed) and only while the flooded optimal path keeps
  // going straight. This guarantees the burst never drives into an unknown or
  // walled cell, so mms cannot crash. During exploration allowBatch is false so
  // the mouse senses at every cell.
  if (allowBatch) {
    while (true) {
      // Stop if a wall blocks straight-ahead from the current cell.
      if (wallMap_[posX_][posY_] & (1 << heading_)) {
        break;
      }
      int nx = posX_ + DX[heading_];
      int ny = posY_ + DY[heading_];
      if (!inBounds(nx, ny) || !visited_[nx][ny]) {
        break;
      }
      // Continue straight only while it is exactly optimal: arriving at the
      // current cell facing `heading_` should cost STEP_COST more than arriving
      // at the next cell facing `heading_` (i.e. a straight step with no turn is
      // on the optimal path). If a turn here would be cheaper, stop so the next
      // moveOneStep performs it.
      uint16_t hereFacing = distMap_[posX_][posY_][heading_];
      uint16_t nextFacing = distMap_[nx][ny][heading_];
      if (hereFacing == COST_INF || nextFacing == COST_INF) {
        break;
      }
      if ((uint32_t)nextFacing + STEP_COST != (uint32_t)hereFacing) {
        break;
      }

      posX_ = nx;
      posY_ = ny;
      run++;
    }
  }

  io_.moveForward(run);
  moves_ += run;

  logf(LogLevel::Info, "move %d: (%d,%d) %s -> (%d,%d) %s [%s x%d, cost=%u]",
       moves_, fromX, fromY, fromHeading, posX_, posY_, DIR_NAME[heading_],
       action, run, (unsigned)minCost);
  return true;
}

// Active re-exploration thresholding.
// Compare the best path that uses ONLY fully-known (visited) cells against the
// most optimistic path that is still allowed to pass through unknown cells. If
// the optimistic route is meaningfully cheaper, an unexplored shortcut might
// exist and is worth chasing; otherwise the known best cannot be beaten and we
// can commit to the speed run.
bool Mouse::worthExploring() {
  static uint16_t known[MAZE_SIZE][MAZE_SIZE][4];
  static uint16_t optimistic[MAZE_SIZE][MAZE_SIZE][4];

  flood(known, /*toStart=*/false, /*optimistic=*/false);
  flood(optimistic, /*toStart=*/false, /*optimistic=*/true);

  uint16_t knownBest = cellCost(known, START_X, START_Y);
  uint16_t optimisticBest = cellCost(optimistic, START_X, START_Y);

  // Hard cap: never exceed MAX_EXPLORE_LAPS optimistic laps. Each lap adds its
  // full length to the 0.1-weighted total-penalty term but can only ever shave
  // the unit-weighted best term, so unbounded chasing hurts the score.
  if (exploreLaps_ >= MAX_EXPLORE_LAPS) {
    logf(LogLevel::Info, "explore-check: lap cap reached (%d) -> commit",
         exploreLaps_);
    return false;
  }

  // A shortcut is worth chasing only if the potential improvement is BOTH above
  // a fixed floor and a fraction of the current known path. This keeps the
  // exploration cost (total penalty) proportional to the achievable best-run
  // gain rather than chasing every marginal theoretical shortcut.
  uint32_t gain =
      (knownBest > optimisticBest) ? (uint32_t)(knownBest - optimisticBest) : 0;
  uint32_t threshold =
      EXPLORE_MARGIN + (uint32_t)(knownBest / EXPLORE_GAIN_DIVISOR);
  bool worth = gain > threshold;
  logf(LogLevel::Info,
       "explore-check: knownBest=%u optimisticBest=%u gain=%u thr=%u lap=%d "
       "-> %s",
       (unsigned)knownBest, (unsigned)optimisticBest, (unsigned)gain,
       (unsigned)threshold, exploreLaps_, worth ? "keep exploring" : "commit");
  return worth;
}

// Begin a fresh speed run: pause (only before the very first one), announce
// it, and snapshot the counters so the run reports its own move/turn totals.
void Mouse::beginSpeedRun() {
  if (speedRunsDone_ == 0) {
    logf(LogLevel::Info, "committing to speed run (optimal known path)");
    logf(LogLevel::Info, "waiting %d ms before speed run 1...",
         SPEED_RUN_DELAY_MS);
    io_.delayMs(SPEED_RUN_DELAY_MS);
  }
  legStartMoves_ = moves_;
  legStartTurns_ = turns_;
  phase_ = Phase::SpeedRun;
  logf(LogLevel::Info, "SPEED RUN %d/%d start", speedRunsDone_ + 1, SPEED_RUNS);
}

// Called after a leg reaches its target. Returns true when the run is done.
bool Mouse::advancePhase() {
  switch (phase_) {
  case Phase::ExploreToGoal:
    logf(LogLevel::Info, "GOAL reached: moves=%d turns=%d walls=%d", moves_,
         turns_, knownWalls());
    // Head back to start, mapping fresh corridors on the return leg.
    phase_ = Phase::ExploreToStart;
    return false;

  case Phase::ExploreToStart:
    logf(LogLevel::Info, "START reached: moves=%d turns=%d walls=%d", moves_,
         turns_, knownWalls());
    // One full optimistic lap (out to goal + back) just completed.
    exploreLaps_++;
    // Decide whether an unknown route could still beat the known best.
    if (worthExploring()) {
      phase_ = Phase::ExploreToGoal; // another optimistic lap
      return false;
    }
    beginSpeedRun();
    return false;

  case Phase::SpeedRun:
    speedRunsDone_++;
    logf(LogLevel::Info, "SPEED RUN %d/%d complete: moves=%d turns=%d",
         speedRunsDone_, SPEED_RUNS, moves_ - legStartMoves_,
         turns_ - legStartTurns_);
    if (speedRunsDone_ >= SPEED_RUNS) {
      logf(LogLevel::Info, "all %d speed runs complete", SPEED_RUNS);
      phase_ = Phase::Done;
      return true;
    }
    // Drive back to the start along the known path, then run again.
    legStartMoves_ = moves_;
    legStartTurns_ = turns_;
    phase_ = Phase::ReturnToStart;
    logf(LogLevel::Info, "returning to start for speed run %d/%d",
         speedRunsDone_ + 1, SPEED_RUNS);
    return false;

  case Phase::ReturnToStart:
    logf(LogLevel::Info, "back at start (return: moves=%d turns=%d)",
         moves_ - legStartMoves_, turns_ - legStartTurns_);
    beginSpeedRun();
    return false;

  default:
    return true;
  }
}

bool Mouse::step() {
  if (phase_ == Phase::Done) {
    return true;
  }

  steps_++;

  bool toStart =
      (phase_ == Phase::ExploreToStart || phase_ == Phase::ReturnToStart);
  // Exploration legs flood optimistically (unknown == open); the committed
  // speed run and the returns between runs use only fully-known corridors.
  bool optimistic =
      (phase_ == Phase::ExploreToGoal || phase_ == Phase::ExploreToStart);

  // Check leg completion before moving (covers the very first step too).
  bool legDone = toStart ? atStart() : atTarget();
  if (legDone) {
    return advancePhase();
  }

  logf(LogLevel::Debug, "step %d [%s]: pos=(%d,%d) heading=%s", steps_,
       phaseName(phase_), posX_, posY_, DIR_NAME[heading_]);

  updateWallsFromSensors();
  flood(distMap_, toStart, optimistic);

  // Straight-batching is only safe on a fully-known map: the committed speed run
  // and the (uncounted) returns between runs. Exploration must sense every cell,
  // so it moves one cell at a time.
  bool allowBatch = !optimistic;
  if (!moveOneStep(allowBatch)) {
    // Boxed in (only possible in the pessimistic speed run if the known map is
    // inconsistent). Bail out rather than drive into a wall.
    logf(LogLevel::Error, "no reachable move; aborting run");
    phase_ = Phase::Done;
    return true;
  }

  // Reaching the leg target ends this step's leg handling on the next call.
  if (toStart ? atStart() : atTarget()) {
    return advancePhase();
  }
  return false;
}

void Mouse::run() {
  while (!step()) {
  }
}
