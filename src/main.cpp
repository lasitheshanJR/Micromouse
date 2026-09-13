/*
  Micromouse Firmware - PlatformIO C++
*/

#include <Arduino.h>

// --- MAZE CONFIGURATION ---
#define MAZE_SIZE 16        
#define TARGET_X 7          
#define TARGET_Y 7
#define WALL_THRESHOLD_CM 18.0

// --- PINOUT DEFINITIONS ---
#define USER_LED PC13

// Motors (L293D Driver)
#define L293D_IN1 PB6
#define L293D_IN2 PA2
#define L293D_IN3 PB7
#define L293D_IN4 PA3

// Encoders
#define LEFT_ENC_A  PB3
#define LEFT_ENC_B  PB15
#define RIGHT_ENC_A PA8
#define RIGHT_ENC_B PA9

// 6x Sharp Analog IR Distance Sensors
#define SENSOR_FAR_LEFT   PA0
#define SENSOR_LEFT       PA5
#define SENSOR_FRONT_L    PA6
#define SENSOR_FRONT_R    PA7
#define SENSOR_RIGHT      PB0
#define SENSOR_FAR_RIGHT  PB1

// --- GLOBAL STATE ---
volatile long leftTicks = 0;
volatile long rightTicks = 0;

uint8_t heading = 0; // 0=NORTH, 1=EAST, 2=SOUTH, 3=WEST
uint8_t posX = 0, posY = 0;

uint8_t wallMap[MAZE_SIZE][MAZE_SIZE]; 
uint8_t distMap[MAZE_SIZE][MAZE_SIZE];

float filteredVal[6] = {0};
const float EMA_ALPHA = 0.3;

// --- ISR ENCODER HANDLERS ---
void readLeftEncoder() {
  if (digitalRead(LEFT_ENC_B) == HIGH) leftTicks++;
  else leftTicks--;
}

void readRightEncoder() {
  if (digitalRead(RIGHT_ENC_B) == HIGH) rightTicks++;
  else rightTicks--;
}

// --- SENSOR PROCESSING ---
float readRawDistanceCM(uint32_t pin) {
  int rawADC = analogRead(pin);
  if (rawADC < 100) return 80.0;
  
  float voltage = (rawADC / 4095.0) * 3.3; 
  if (voltage < 0.4) return 80.0;
  
  float distance = 27.619 * pow(voltage, -1.173);
  return constrain(distance, 10.0, 80.0);
}

float getFilteredDistanceCM(uint8_t sensorIndex, uint32_t pin) {
  float currentRead = readRawDistanceCM(pin);
  if (filteredVal[sensorIndex] == 0) {
    filteredVal[sensorIndex] = currentRead;
  } else {
    filteredVal[sensorIndex] = (EMA_ALPHA * currentRead) + ((1.0 - EMA_ALPHA) * filteredVal[sensorIndex]);
  }
  return filteredVal[sensorIndex];
}

// --- MOTOR CONTROL ---
void setMotors(int leftSpeed, int rightSpeed) {
  digitalWrite(L293D_IN1, leftSpeed > 0 ? HIGH : LOW);
  digitalWrite(L293D_IN2, leftSpeed < 0 ? HIGH : LOW);
  digitalWrite(L293D_IN3, rightSpeed > 0 ? HIGH : LOW);
  digitalWrite(L293D_IN4, rightSpeed < 0 ? HIGH : LOW);
}

void moveOneCellForward() {
  leftTicks = 0;
  rightTicks = 0;
  setMotors(1, 1);
  delay(400); 
  setMotors(0, 0);
  delay(100);

  if (heading == 0) posY++;
  else if (heading == 1) posX++;
  else if (heading == 2) posY--;
  else if (heading == 3) posX--;
}

void turnLeft() {
  setMotors(-1, 1);
  delay(200);
  setMotors(0, 0);
  heading = (heading + 3) % 4;
}

void turnRight() {
  setMotors(1, -1);
  delay(200);
  setMotors(0, 0);
  heading = (heading + 1) % 4;
}

void turn180() {
  turnRight();
  turnRight();
}

// --- NAVIGATION & FLOOD FILL ---
void initializeDistanceMap() {
  for (int x = 0; x < MAZE_SIZE; x++) {
    for (int y = 0; y < MAZE_SIZE; y++) {
      distMap[x][y] = abs(x - TARGET_X) + abs(y - TARGET_Y);
      wallMap[x][y] = 0;
    }
  }
}

void updateWallsFromSensors() {
  float distFL     = getFilteredDistanceCM(0, SENSOR_FAR_LEFT);
  float distL      = getFilteredDistanceCM(1, SENSOR_LEFT);
  float distFrontL = getFilteredDistanceCM(2, SENSOR_FRONT_L);
  float distFrontR = getFilteredDistanceCM(3, SENSOR_FRONT_R);
  float distR      = getFilteredDistanceCM(4, SENSOR_RIGHT);
  float distFR     = getFilteredDistanceCM(5, SENSOR_FAR_RIGHT);

  bool frontWall = ((distFrontL + distFrontR) / 2.0) < WALL_THRESHOLD_CM;
  bool leftWall  = (distL < WALL_THRESHOLD_CM) || (distFL < (WALL_THRESHOLD_CM - 3.0));
  bool rightWall = (distR < WALL_THRESHOLD_CM) || (distFR < (WALL_THRESHOLD_CM - 3.0));

  uint8_t absFront = heading;
  uint8_t absRight = (heading + 1) % 4;
  uint8_t absLeft  = (heading + 3) % 4;

  if (frontWall) wallMap[posX][posY] |= (1 << absFront);
  if (rightWall) wallMap[posX][posY] |= (1 << absRight);
  if (leftWall)  wallMap[posX][posY] |= (1 << absLeft);
}

void floodFill() {
  uint8_t queueX[MAZE_SIZE * MAZE_SIZE];
  uint8_t queueY[MAZE_SIZE * MAZE_SIZE];
  int head = 0, tail = 0;

  for (int x = 0; x < MAZE_SIZE; x++) {
    for (int y = 0; y < MAZE_SIZE; y++) {
      distMap[x][y] = 255;
    }
  }

  distMap[TARGET_X][TARGET_Y] = 0;
  queueX[tail] = TARGET_X;
  queueY[tail] = TARGET_Y;
  tail++;

  while (head < tail) {
    uint8_t cx = queueX[head];
    uint8_t cy = queueY[head];
    head++;

    uint8_t currentDist = distMap[cx][cy];
    int dx[] = {0, 1, 0, -1};
    int dy[] = {1, 0, -1, 0};

    for (int i = 0; i < 4; i++) {
      if (!(wallMap[cx][cy] & (1 << i))) {
        int nx = cx + dx[i];
        int ny = cy + dy[i];

        if (nx >= 0 && nx < MAZE_SIZE && ny >= 0 && ny < MAZE_SIZE) {
          if (distMap[nx][ny] == 255) {
            distMap[nx][ny] = currentDist + 1;
            queueX[tail] = nx;
            queueY[tail] = ny;
            tail++;
          }
        }
      }
    }
  }
}

void navigateNextStep() {
  uint8_t bestDir = heading;
  uint8_t minDist = 255;

  for (int i = 0; i < 4; i++) {
    if (!(wallMap[posX][posY] & (1 << i))) {
      int nx = posX + (i == 1 ? 1 : (i == 3 ? -1 : 0));
      int ny = posY + (i == 0 ? 1 : (i == 2 ? -1 : 0));

      if (nx >= 0 && nx < MAZE_SIZE && ny >= 0 && ny < MAZE_SIZE) {
        if (distMap[nx][ny] < minDist) {
          minDist = distMap[nx][ny];
          bestDir = i;
        }
      }
    }
  }

  int dirDiff = (bestDir - heading + 4) % 4;
  if (dirDiff == 1) turnRight();
  else if (dirDiff == 2) turn180();
  else if (dirDiff == 3) turnLeft();

  moveOneCellForward();
}

// --- SETUP & MAIN LOOP ---
void setup() {
  Serial.begin(115200);

  // Configure 12-Bit ADC resolution for STM32
  analogReadResolution(12);

  pinMode(USER_LED, OUTPUT);
  digitalWrite(USER_LED, HIGH); // Turn off active-low LED

  pinMode(L293D_IN1, OUTPUT);
  pinMode(L293D_IN2, OUTPUT);
  pinMode(L293D_IN3, OUTPUT);
  pinMode(L293D_IN4, OUTPUT);

  pinMode(LEFT_ENC_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), readLeftEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), readRightEncoder, RISING);

  initializeDistanceMap();
}

void loop() {
  if (posX == TARGET_X && posY == TARGET_Y) {
    setMotors(0, 0);
    digitalWrite(USER_LED, LOW); // Turn on LED when target reached
    while (1);
  }

  updateWallsFromSensors();
  floodFill();
  navigateNextStep();
}
