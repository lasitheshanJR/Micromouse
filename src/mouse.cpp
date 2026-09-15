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

Mouse::Mouse(MouseIO& io)
    : io_(io), posX_(0), posY_(0), heading_(0), steps_(0), moves_(0),
      turns_(0) {}

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
  posX_ = 0;
  posY_ = 0;
  heading_ = 0;
  steps_ = 0;
  moves_ = 0;
  turns_ = 0;

  for (int x = 0; x < MAZE_SIZE; x++) {
    for (int y = 0; y < MAZE_SIZE; y++) {
      wallMap_[x][y] = 0;

      // Seed the initial estimate with the distance to the nearest goal cell.
      int best = MAZE_SIZE * 2;
      for (int gx = GOAL_X0; gx <= GOAL_X1; gx++) {
        for (int gy = GOAL_Y0; gy <= GOAL_Y1; gy++) {
          int d = abs(x - gx) + abs(y - gy);
          if (d < best) {
            best = d;
          }
        }
      }
      distMap_[x][y] = (uint8_t)best;
    }
  }

  logf(LogLevel::Info, "reset: pos=(0,0) heading=N goal=(%d..%d,%d..%d)",
       GOAL_X0, GOAL_X1, GOAL_Y0, GOAL_Y1);
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
}

void Mouse::floodFill() {
  uint8_t queueX[MAZE_SIZE * MAZE_SIZE];
  uint8_t queueY[MAZE_SIZE * MAZE_SIZE];
  int head = 0, tail = 0;

  for (int x = 0; x < MAZE_SIZE; x++) {
    for (int y = 0; y < MAZE_SIZE; y++) {
      distMap_[x][y] = 255;
    }
  }

  // All four goal cells are distance 0, so the flood fill targets the region.
  for (int gx = GOAL_X0; gx <= GOAL_X1; gx++) {
    for (int gy = GOAL_Y0; gy <= GOAL_Y1; gy++) {
      distMap_[gx][gy] = 0;
      queueX[tail] = (uint8_t)gx;
      queueY[tail] = (uint8_t)gy;
      tail++;
    }
  }

  while (head < tail) {
    uint8_t cx = queueX[head];
    uint8_t cy = queueY[head];
    head++;

    uint8_t currentDist = distMap_[cx][cy];

    for (int i = 0; i < 4; i++) {
      if (!(wallMap_[cx][cy] & (1 << i))) {
        int nx = cx + DX[i];
        int ny = cy + DY[i];

        if (inBounds(nx, ny) && distMap_[nx][ny] == 255) {
          distMap_[nx][ny] = (uint8_t)(currentDist + 1);
          queueX[tail] = (uint8_t)nx;
          queueY[tail] = (uint8_t)ny;
          tail++;
        }
      }
    }
  }

  logf(LogLevel::Debug, "floodFill: dist(%d,%d)=%d", posX_, posY_,
       distMap_[posX_][posY_]);
}

void Mouse::moveOneStep() {
  int bestDir = heading_;
  int minDist = 255;

  for (int i = 0; i < 4; i++) {
    if (!(wallMap_[posX_][posY_] & (1 << i))) {
      int nx = posX_ + DX[i];
      int ny = posY_ + DY[i];

      if (inBounds(nx, ny) && distMap_[nx][ny] < minDist) {
        minDist = distMap_[nx][ny];
        bestDir = i;
      }
    }
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

  io_.moveForward();
  moves_++;

  heading_ = bestDir;
  posX_ += DX[heading_];
  posY_ += DY[heading_];

  logf(LogLevel::Info,
       "move %d: (%d,%d) %s -> (%d,%d) %s [%s, dist=%d]", moves_, fromX, fromY,
       fromHeading, posX_, posY_, DIR_NAME[heading_], action, minDist);
}

bool Mouse::step() {
  if (atTarget()) {
    return true;
  }

  steps_++;
  logf(LogLevel::Debug, "step %d: pos=(%d,%d) heading=%s", steps_, posX_, posY_,
       DIR_NAME[heading_]);

  updateWallsFromSensors();
  floodFill();
  moveOneStep();

  if (atTarget()) {
    logf(LogLevel::Info, "GOAL reached: steps=%d moves=%d turns=%d walls=%d",
         steps_, moves_, turns_, knownWalls());
    return true;
  }

  return false;
}

void Mouse::run() {
  while (!step()) {
  }
}
