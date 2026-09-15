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

  // Clear any leftover visualization from a previous run.
  io_.clearAllColor();
  io_.clearAllText();

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

// -----------------------------------------------------------------------
// Visualization helpers
// -----------------------------------------------------------------------

void Mouse::visualizeExploration() {
  for (int x = 0; x < MAZE_SIZE; x++) {
    for (int y = 0; y < MAZE_SIZE; y++) {
      if (x == posX_ && y == posY_) {
        io_.showColor(x, y, 'c'); // Cyan = current position
      } else if (isGoalCell(x, y)) {
        io_.showColor(x, y, 'y'); // Yellow = goal
      } else if (visited_[x][y]) {
        io_.showColor(x, y, 'G'); // Dark Green = visited
      } else {
        io_.showColor(x, y, 'A'); // Dark Gray = unvisited
      }
    }
  }
}

void Mouse::visualizeFloodDistances() {
  for (int x = 0; x < MAZE_SIZE; x++) {
    for (int y = 0; y < MAZE_SIZE; y++) {
      uint16_t cost = cellCost(distMap_, x, y);
      if (cost == COST_INF) {
        io_.showText(x, y, "inf");
      } else {
        char buf[8];
        snprintf(buf, sizeof(buf), "%u", (unsigned)cost);
        io_.showText(x, y, buf);
      }
    }
  }
}

void Mouse::visualizeSpeedRunPath() {
  io_.clearAllColor();
  // Trace the optimal path from start to goal using the flood map and color it.
  int tx = START_X, ty = START_Y, th = 0;

  // First seed start cell
  io_.showColor(tx, ty, 'g'); // Green = speed run path

  for (int safety = 0; safety < MAZE_SIZE * MAZE_SIZE; safety++) {
    if (isGoalCell(tx, ty))
      break;

    int bestDir = -1;
    uint32_t minCost = COST_INF;
    for (int i = 0; i < 4; i++) {
      if (wallMap_[tx][ty] & (1 << i))
        continue;
      int nx = tx + DX[i];
      int ny = ty + DY[i];
      if (!inBounds(nx, ny))
        continue;
      uint16_t arrive = distMap_[nx][ny][i];
      if (arrive == COST_INF)
        continue;
      int turn = (i - th + 4) % 4;
      int quarters = (turn == 3) ? 1 : turn;
      uint32_t cost = (uint32_t)arrive + STEP_COST + quarters * TURN_COST;
      if (cost < minCost) {
        minCost = cost;
        bestDir = i;
      }
    }
    if (bestDir < 0)
      break;
    th = bestDir;
    tx += DX[bestDir];
    ty += DY[bestDir];
    io_.showColor(tx, ty, 'g');
  }

  // Mark goal cells
  for (int gx = GOAL_X0; gx <= GOAL_X1; gx++) {
    for (int gy = GOAL_Y0; gy <= GOAL_Y1; gy++) {
      io_.showColor(gx, gy, 'y');
    }
  }
}

// -----------------------------------------------------------------------
// Wall sensing with enhanced lookahead
// -----------------------------------------------------------------------

void Mouse::updateWallsFromSensors() {
  struct Observation {
    bool front;
    bool left;
    bool right;
    bool back;
    bool diagLeft;
    bool diagRight;
  } obs;

  obs.front = io_.wallFront();
  obs.left = io_.wallLeft();
  obs.right = io_.wallRight();
  obs.back = io_.wallBack();
  obs.diagLeft = io_.wallDiagonalLeft();
  obs.diagRight = io_.wallDiagonalRight();

  logf(LogLevel::Debug,
       "sense: pos=(%d,%d) heading=%s front=%d left=%d right=%d back=%d "
       "diagL=%d diagR=%d",
       posX_, posY_, DIR_NAME[heading_], obs.front, obs.left, obs.right,
       obs.back, obs.diagLeft, obs.diagRight);

  // A 45-degree beam looking at the forward corner must hit either the side
  // wall or something ahead. If a diagonal sees a wall that the side sensor
  // does not, treat it as a front wall: this catches front walls the 0-degree
  // pair missed (misalignment, sensor cone gaps). For bridges that derive the
  // diagonal from the cardinal sensors this reduces to the original reading.
  bool front =
      obs.front || (obs.diagLeft && !obs.left) || (obs.diagRight && !obs.right);
  bool left = obs.left;
  bool right = obs.right;
  bool back = obs.back;

  logf(LogLevel::Debug, "junction: %s (front=%d left=%d right=%d back=%d)",
       junctionName(front, left, right), front, left, right, back);

  if (front)
    addWall(posX_, posY_, heading_);
  if (right)
    addWall(posX_, posY_, (heading_ + 1) % 4);
  if (left)
    addWall(posX_, posY_, (heading_ + 3) % 4);
  // Back-wall sensing: fully map all 4 walls of every cell on first visit.
  // This gives the flood perfect information sooner, reducing the chance of
  // needing re-exploration. On hardware wallBack() defaults to false (no-op).
  if (back)
    addWall(posX_, posY_, (heading_ + 2) % 4);

  // Corridor lookahead: if nothing blocks straight ahead, probe further cells
  // along the current heading to find where the corridor's forward passage ends
  // and record that far wall now. This lets the flood reason about the straight
  // run without physically driving each cell first, trimming exploration
  // distance (the 0.1*total penalty). Bounded by LOOKAHEAD_CELLS.
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
      return;
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

    int px = cx - DX[h];
    int py = cy - DY[h];
    if (!inBounds(px, py) || blocked(px, py, h)) {
      continue;
    }
    for (int ph = 0; ph < 4; ph++) {
      int turn = (h - ph + 4) % 4;
      int quarters = (turn == 3) ? 1 : turn;
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
    // U-turn: pick direction based on which side is open (minor optimization).
    // If both sides are equally open, left-left is equivalent to right-right.
    io_.turnLeft();
    io_.turnLeft();
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

  posX_ += DX[heading_];
  posY_ += DY[heading_];
  visited_[posX_][posY_] = true;
  int run = 1;

  if (allowBatch) {
    while (true) {
      if (wallMap_[posX_][posY_] & (1 << heading_)) {
        break;
      }
      int nx = posX_ + DX[heading_];
      int ny = posY_ + DY[heading_];
      if (!inBounds(nx, ny) || !visited_[nx][ny]) {
        break;
      }
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

// -----------------------------------------------------------------------
// Diagonal speed-run path planner
// -----------------------------------------------------------------------

// Build the cardinal optimal path from the START to the GOAL using the
// pessimistic flood map. Returns the length (number of steps).
int Mouse::buildCardinalPath(int* pathDirs, int* pathXs, int* pathYs) {
  // Flood pessimistically toward goal from current map knowledge.
  static uint16_t pathDist[MAZE_SIZE][MAZE_SIZE][4];
  flood(pathDist, /*toStart=*/false, /*optimistic=*/false);

  int cx = START_X, cy = START_Y, ch = 0; // start facing north
  int len = 0;

  for (int safety = 0; safety < MAX_PATH_LEN; safety++) {
    if (isGoalCell(cx, cy))
      break;

    int bestDir = -1;
    uint32_t minCost = COST_INF;
    for (int i = 0; i < 4; i++) {
      if (wallMap_[cx][cy] & (1 << i))
        continue;
      int nx = cx + DX[i];
      int ny = cy + DY[i];
      if (!inBounds(nx, ny))
        continue;
      uint16_t arrive = pathDist[nx][ny][i];
      if (arrive == COST_INF)
        continue;
      int turn = (i - ch + 4) % 4;
      int quarters = (turn == 3) ? 1 : turn;
      uint32_t cost = (uint32_t)arrive + STEP_COST + quarters * TURN_COST;
      if (cost < minCost) {
        minCost = cost;
        bestDir = i;
      }
    }
    if (bestDir < 0)
      break;

    pathDirs[len] = bestDir;
    pathXs[len] = cx;
    pathYs[len] = cy;
    len++;
    ch = bestDir;
    cx += DX[bestDir];
    cy += DY[bestDir];
  }

  logf(LogLevel::Info, "cardinal path: %d steps from (%d,%d) to goal", len,
       START_X, START_Y);
  return len;
}

// Execute the speed run using diagonal shortcuts where possible.
//
// The diagonal optimization works by scanning the cardinal path for patterns:
//   - Straight segments: batch into moveForward(N)
//   - Turn-90 patterns (straight, turn, straight): convert to
//     turn45-in, diagonal half-steps, turn45-out
//
// A diagonal move cuts the corner of a 90-degree turn. Instead of:
//   moveForward(1) + turnRight90 + moveForward(1)  = 2 distance + 1 turn
// We do:
//   turnRight45 + moveForwardHalf(2) + turnLeft45   = ~1.41 distance + 0 net turn
//
// The entry half-step goes from cell center to the diagonal crossing point,
// then the exit half-step goes from diagonal to the next cell center.
// This saves both effective distance and turns.
void Mouse::executeDiagonalPath(const int* pathDirs, int pathLen) {
  if (pathLen == 0)
    return;

  // The mouse starts at (START_X, START_Y) facing North (heading 0).
  // We need to face the first direction in the path.
  int curHeading = 0; // facing North at start

  int i = 0;
  while (i < pathLen) {
    // Look ahead to find a straight segment (consecutive same-direction moves).
    int segDir = pathDirs[i];
    int segLen = 0;
    int j = i;
    while (j < pathLen && pathDirs[j] == segDir) {
      segLen++;
      j++;
    }

    // Check if we can do a diagonal: need a turn followed by at least one
    // straight in the new direction. Pattern: ...current straight... TURN ...next straight...
    // A diagonal is beneficial when: straight(>=1) -> turn90 -> straight(>=1)
    //
    // We convert: last cell of current straight + turn90 + first cell of next straight
    // into: turn45 + 2 diagonal half-steps + turn45-out
    //
    // However, diagonal movement in mms places the mouse on cell edges (half-step
    // positions), not cell centers. The sequence is:
    //   1. Drive straight segment minus last cell: moveForward(segLen-1) if segLen > 1
    //   2. Enter diagonal: turnRight45 or turnLeft45
    //   3. Drive diagonal: moveForwardHalf(N) for N consecutive diagonal cells
    //   4. Exit diagonal: turnLeft45 or turnRight45 (opposite of entry)
    //   5. Continue with remaining straight
    //
    // For the diagonal to work, the cells on the diagonal path must be passable
    // (no walls blocking the diagonal traversal).

    // Can we start a diagonal at the end of this segment?
    bool canDiag = false;
    int diagTurn = 0; // +1 = right, -1 = left
    int diagCount = 0;

    if (j < pathLen) {
      int nextDir = pathDirs[j];
      int turnDiff = (nextDir - segDir + 4) % 4;

      if (turnDiff == 1 || turnDiff == 3) {
        // There's a 90-degree turn. Check how many consecutive diagonals we can chain.
        // Each diagonal = one turn90 in the path that we convert.
        // Pattern: straight(a) turn straight(b) turn straight(c) ...
        // becomes: straight(a-1) diag_enter diag(a+b-1) ... diag_exit straight(last)
        //
        // For simplicity and safety, convert one turn at a time:
        // Take the last cell of the current straight + first cell of next straight.
        diagTurn = (turnDiff == 1) ? 1 : -1;
        canDiag = true;
        diagCount = 1; // At minimum, one diagonal crossing

        // Count how many alternating diagonals we can chain.
        // After the first turn, if the next segment also turns in the SAME relative
        // direction, we can extend the diagonal run.
        int dk = j;
        int prevDir = segDir;
        while (dk < pathLen) {
          int nd = pathDirs[dk];
          int dt = (nd - prevDir + 4) % 4;
          int expectedTurn = (diagTurn == 1) ? 1 : 3;
          if (dt != expectedTurn)
            break;

          // Count how many straight cells in this next segment
          int nextSegLen = 0;
          int dk2 = dk;
          while (dk2 < pathLen && pathDirs[dk2] == nd) {
            nextSegLen++;
            dk2++;
          }

          // Each cell in the intermediate segment becomes a diagonal half-step pair.
          // But the first and last cells of the diagonal are entry/exit half-steps.
          // For a simple single-turn diagonal:
          //   entry half-step (from last cell of prev straight to edge)
          //   + exit half-step (from edge to first cell of next straight)
          //   = 2 half-steps total = moveForwardHalf(2)
          // For chained diagonals (same direction turns):
          //   We can extend: each additional turn adds 2 more half-steps to the diagonal.

          diagCount++;
          prevDir = nd;

          // If next segment has only 1 cell and continues turning same way, keep chaining.
          if (nextSegLen == 1 && dk2 < pathLen) {
            dk = dk2;
            continue;
          }
          // Otherwise stop extending the diagonal here.
          break;
        }
      }
    }

    if (canDiag && segLen >= 1) {
      // Execute the straight portion before the diagonal entry.
      // Turn to face segDir first.
      int turnToSeg = (segDir - curHeading + 4) % 4;
      if (turnToSeg == 1) {
        io_.turnRight();
        turns_++;
      } else if (turnToSeg == 2) {
        io_.turnLeft();
        io_.turnLeft();
        turns_ += 2;
      } else if (turnToSeg == 3) {
        io_.turnLeft();
        turns_++;
      }
      curHeading = segDir;

      // Drive all but the last cell of the straight (the last cell becomes
      // the diagonal entry half-step).
      if (segLen > 1) {
        io_.moveForward(segLen - 1);
        moves_ += (segLen - 1);
      }

      // Enter diagonal: half-step forward into the last cell's far edge.
      io_.moveForwardHalf(1);
      moves_++; // count as one move (half-step)

      // Turn 45 degrees into the diagonal.
      if (diagTurn == 1) {
        io_.turnRight45();
      } else {
        io_.turnLeft45();
      }
      turns_++;

      // Count how many half-steps we travel diagonally.
      // For a single turn: 1 diagonal half-step to cross the corner,
      //   then 1 half-step to exit = but we enter with 1 half-step above.
      // Actually in mms, after turning 45 degrees and being on a cell edge,
      // each moveForwardHalf moves along the diagonal.
      //
      // For each turn we converted, we need 1 diagonal half-step.
      // But a sequence of chained same-direction turns means we stay diagonal
      // for longer. The number of diagonal half-steps = number of cells we skip.
      //
      // Simple model: for `diagCount` turns being converted, we travel
      // `diagCount` diagonal half-steps through cell corners.
      // But we must also account for any straight cells between the turns.
      //
      // Let's use a simpler, more reliable approach: consume the path entries
      // one at a time and count how many we converted.
      int consumed = 0;
      int dk = j; // j = start of next segment after current straight

      // We've already consumed the current straight (i to j-1).
      // Now consume the alternating segments that form the diagonal.
      int diagHalfSteps = 0;

      for (int d = 0; d < diagCount && dk < pathLen; d++) {
        int nd = pathDirs[dk];
        // Count cells in this next segment
        int nextSegLen = 0;
        int dk2 = dk;
        while (dk2 < pathLen && pathDirs[dk2] == nd) {
          nextSegLen++;
          dk2++;
        }

        if (d < diagCount - 1 && nextSegLen == 1) {
          // Intermediate diagonal segment: one cell consumed as diagonal
          diagHalfSteps += 2; // each intermediate cell = 2 half-steps diagonal
          consumed += nextSegLen;
          dk = dk2;
        } else {
          // Last diagonal segment: consume one cell for diagonal exit
          diagHalfSteps += 1; // exit half-step
          consumed += 1;
          dk = dk + 1;
          break;
        }
      }

      if (diagHalfSteps < 1)
        diagHalfSteps = 1;

      // Drive the diagonal half-steps.
      io_.moveForwardHalf(diagHalfSteps);
      moves_ += diagHalfSteps;

      // Exit diagonal: turn 45 degrees back to cardinal.
      if (diagTurn == 1) {
        io_.turnLeft45();
      } else {
        io_.turnRight45();
      }
      turns_++;

      // Drive the exit half-step back to cell center.
      io_.moveForwardHalf(1);
      moves_++;

      // Update heading: after the diagonal, we're now facing the last
      // direction we were moving in.
      if (diagTurn == 1) {
        curHeading = (segDir + 1) % 4;
      } else {
        curHeading = (segDir + 3) % 4;
      }

      // Update the path index past all consumed entries.
      // We consumed: segLen (current straight) + consumed (diagonal segments)
      i = j + consumed;

      // If there are remaining cells in the last segment after the diagonal exit,
      // they'll be handled by the next iteration.
    } else {
      // No diagonal possible: just drive straight.
      int turnToSeg = (segDir - curHeading + 4) % 4;
      if (turnToSeg == 1) {
        io_.turnRight();
        turns_++;
      } else if (turnToSeg == 2) {
        io_.turnLeft();
        io_.turnLeft();
        turns_ += 2;
      } else if (turnToSeg == 3) {
        io_.turnLeft();
        turns_++;
      }
      curHeading = segDir;

      io_.moveForward(segLen);
      moves_ += segLen;
      i = j;
    }
  }
}

bool Mouse::runDiagonalSpeedRun() {
  static int pathDirs[MAX_PATH_LEN];
  static int pathXs[MAX_PATH_LEN];
  static int pathYs[MAX_PATH_LEN];

  int pathLen = buildCardinalPath(pathDirs, pathXs, pathYs);
  if (pathLen == 0) {
    logf(LogLevel::Error, "diagonal speed run: no path found");
    return false;
  }

  logf(LogLevel::Info, "diagonal speed run: executing %d-step path", pathLen);

  // Count how many turns the path contains to decide if diagonal is worth it.
  int turnCount = 0;
  for (int i = 1; i < pathLen; i++) {
    if (pathDirs[i] != pathDirs[i - 1])
      turnCount++;
  }

  logf(LogLevel::Info, "path has %d turns in %d steps", turnCount, pathLen);

  if (turnCount == 0) {
    // Pure straight path: no diagonals possible, just batch it.
    // Turn to face the path direction.
    int dirDiff = (pathDirs[0] - heading_ + 4) % 4;
    if (dirDiff == 1) {
      io_.turnRight();
      turns_++;
    } else if (dirDiff == 2) {
      io_.turnLeft();
      io_.turnLeft();
      turns_ += 2;
    } else if (dirDiff == 3) {
      io_.turnLeft();
      turns_++;
    }
    heading_ = pathDirs[0];
    io_.moveForward(pathLen);
    moves_ += pathLen;
    posX_ = pathXs[pathLen - 1] + DX[pathDirs[pathLen - 1]];
    posY_ = pathYs[pathLen - 1] + DY[pathDirs[pathLen - 1]];
    return isGoalCell(posX_, posY_);
  }

  // Use the cardinal batched approach (which already earns effective-distance
  // discount) rather than risk a diagonal crash on complex paths.
  // The diagonal optimizer works on simpler patterns.
  //
  // For now, fall back to the standard batched cardinal speed run which is
  // well-tested and safe. The diagonal execution is available but needs
  // careful per-cell wall checking for the diagonal half-step positions.
  //
  // TODO: Enable full diagonal execution once validated on test mazes.
  // executeDiagonalPath(pathDirs, pathLen);
  // return isGoalCell(posX_, posY_);

  return false; // Signal caller to use the standard cardinal speed run.
}

// Active re-exploration thresholding.
bool Mouse::worthExploring() {
  static uint16_t known[MAZE_SIZE][MAZE_SIZE][4];
  static uint16_t optimistic[MAZE_SIZE][MAZE_SIZE][4];

  flood(known, /*toStart=*/false, /*optimistic=*/false);
  flood(optimistic, /*toStart=*/false, /*optimistic=*/true);

  uint16_t knownBest = cellCost(known, START_X, START_Y);
  uint16_t optimisticBest = cellCost(optimistic, START_X, START_Y);

  if (exploreLaps_ >= MAX_EXPLORE_LAPS) {
    logf(LogLevel::Info, "explore-check: lap cap reached (%d) -> commit",
         exploreLaps_);
    return false;
  }

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

void Mouse::beginSpeedRun() {
  if (speedRunsDone_ == 0) {
    logf(LogLevel::Info, "committing to speed run (optimal known path)");
    // Visualize the planned speed-run path before starting.
    flood(distMap_, /*toStart=*/false, /*optimistic=*/false);
    visualizeSpeedRunPath();
    logf(LogLevel::Info, "waiting %d ms before speed run 1...",
         SPEED_RUN_DELAY_MS);
    io_.delayMs(SPEED_RUN_DELAY_MS);
  }
  legStartMoves_ = moves_;
  legStartTurns_ = turns_;
  phase_ = Phase::SpeedRun;
  logf(LogLevel::Info, "SPEED RUN %d/%d start", speedRunsDone_ + 1, SPEED_RUNS);
}

bool Mouse::advancePhase() {
  switch (phase_) {
  case Phase::ExploreToGoal:
    logf(LogLevel::Info, "GOAL reached: moves=%d turns=%d walls=%d", moves_,
         turns_, knownWalls());
    phase_ = Phase::ExploreToStart;
    return false;

  case Phase::ExploreToStart:
    logf(LogLevel::Info, "START reached: moves=%d turns=%d walls=%d", moves_,
         turns_, knownWalls());
    exploreLaps_++;
    if (worthExploring()) {
      phase_ = Phase::ExploreToGoal;
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
  bool optimistic =
      (phase_ == Phase::ExploreToGoal || phase_ == Phase::ExploreToStart);

  bool legDone = toStart ? atStart() : atTarget();
  if (legDone) {
    return advancePhase();
  }

  logf(LogLevel::Debug, "step %d [%s]: pos=(%d,%d) heading=%s", steps_,
       phaseName(phase_), posX_, posY_, DIR_NAME[heading_]);

  updateWallsFromSensors();
  flood(distMap_, toStart, optimistic);

  // Visualize during exploration phases (every 3 steps to reduce overhead).
  if (optimistic && (steps_ % 3) == 0) {
    visualizeExploration();
    visualizeFloodDistances();
  }

  // Early commit check: during the return leg, periodically check if the
  // known path is already provably optimal. If the optimistic and pessimistic
  // floods agree, there's no point exploring further — mark exploreLaps_ at
  // the cap so worthExploring() returns false immediately on arrival at start.
  if (phase_ == Phase::ExploreToStart && (steps_ % 4) == 0) {
    static uint16_t earlyKnown[MAZE_SIZE][MAZE_SIZE][4];
    static uint16_t earlyOpt[MAZE_SIZE][MAZE_SIZE][4];
    flood(earlyKnown, /*toStart=*/false, /*optimistic=*/false);
    flood(earlyOpt, /*toStart=*/false, /*optimistic=*/true);

    uint16_t knownBest = cellCost(earlyKnown, START_X, START_Y);
    uint16_t optBest = cellCost(earlyOpt, START_X, START_Y);

    if (knownBest != COST_INF && optBest != COST_INF) {
      uint32_t gap =
          (knownBest > optBest) ? (uint32_t)(knownBest - optBest) : 0;
      uint32_t thr =
          EXPLORE_MARGIN + (uint32_t)(knownBest / EXPLORE_GAIN_DIVISOR);
      if (gap <= thr) {
        // Path is provably optimal (or within threshold). Force commit on
        // arrival by saturating the explore-lap counter.
        if (exploreLaps_ < MAX_EXPLORE_LAPS) {
          logf(LogLevel::Info,
               "early commit: gap=%u <= thr=%u, forcing explore cap",
               (unsigned)gap, (unsigned)thr);
          exploreLaps_ = MAX_EXPLORE_LAPS;
        }
      }
    }
  }

  bool allowBatch = !optimistic;
  if (!moveOneStep(allowBatch)) {
    logf(LogLevel::Error, "no reachable move; aborting run");
    phase_ = Phase::Done;
    return true;
  }

  if (toStart ? atStart() : atTarget()) {
    return advancePhase();
  }
  return false;
}

void Mouse::run() {
  while (!step()) {
  }
}
