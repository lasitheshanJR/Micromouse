/*
  Single source of truth for hardware pin mapping and track geometry.
  Included only by the hardware bridge (needs <Arduino.h> for the PA and PB
  macros), never by the shared algorithm core.
*/

#pragma once

#include <cstdint>

namespace config {

// ---------------------------------------------------------------------------
// PIN MAPPING -- confirmed against the real wiring.
//
// Sensor fan (physical left to right). Note the pins are NOT in angle order:
//   far left  (-90) = PB0
//   diag left (-45) = PB1
//   front left (0)  = PA7   ("middle left")
//   front right(0)  = PA6   ("middle right")
//   diag right(+45) = PA0
//   far right (+90) = PA1
//
// Motor pins: PA2..PA5 are the only ADC pins left free, and match Scheme A in
// README.md / testing/src/motor_driver.cpp. CONFIRM before flashing -- driving
// a sensor pin as a motor output can damage hardware.
// ---------------------------------------------------------------------------
constexpr uint8_t kMotorIn1 = PA2; // right motor
constexpr uint8_t kMotorIn2 = PA3; // right motor
constexpr uint8_t kMotorIn3 = PA4; // left motor
constexpr uint8_t kMotorIn4 = PA5; // left motor

// Ordered by kSensorAngles below: far-left, diag-left, front-left, front-right,
// diag-right, far-right.
constexpr uint8_t kSensorPins[6] = {PB0, PB1, PA7, PA6, PA0, PA1};

// ---------------------------------------------------------------------------
// Sensor fan geometry: mounting angle in degrees relative to straight ahead
// (negative = to the left). Matches the 6-sensor array:
//   -90 side, -45 diagonal, 0 front, 0 front, +45 diagonal, +90 side
// ---------------------------------------------------------------------------
constexpr float kSensorAngles[6] = {-90.0f, -45.0f, 0.0f, 0.0f, 45.0f, 90.0f};

enum SensorIndex {
  kFarLeft = 0,  // -90 deg
  kDiagLeft,     // -45 deg
  kFrontLeft,    // 0 deg
  kFrontRight,   // 0 deg
  kDiagRight,    // +45 deg
  kFarRight,     // +90 deg
  kSensorCount
};

// ---------------------------------------------------------------------------
// Track geometry (official micromouse), in centimetres.
//   cell pitch 192 mm, wall/post thickness 12 mm, corridor 180 mm,
//   wall height 50 mm, sensor optical axis 10 mm above the floor.
// Wall tops are red, wall sides white, floor black.
// ---------------------------------------------------------------------------
constexpr float kCellPitchCm = 19.2f;                                  // lattice to lattice
constexpr float kWallThicknessCm = 1.2f;
constexpr float kCorridorWidthCm = kCellPitchCm - kWallThicknessCm;    // 18.0
constexpr float kCentreToWallCm = kCorridorWidthCm / 2.0f;             // 9.0
constexpr float kWallHeightCm = 5.0f;
constexpr float kSensorHeightCm = 1.0f;

// ---------------------------------------------------------------------------
// Detection thresholds (cm). Calibrate on the real track.
//
// From the centre of a cell a wall of that same cell is ~9 cm away, while the
// next cell's wall is one full pitch away (~19.2 cm). The analog sensors top
// out around 15 cm, so thresholds in the 12-15 cm range see only the current
// cell's walls and never "leak" into the neighbouring cell.
// ---------------------------------------------------------------------------
constexpr float kNoWallCm = 40.0f;       // reading >= this means "clear"
constexpr float kSideWallCm = 14.0f;     // -90/+90 sensors
constexpr float kFrontWallCm = 12.0f;    // 0 deg sensors (ahead)
constexpr float kDiagonalWallCm = 15.0f; // -45/+45 sensors (forward corners)

// EMA smoothing factor for the analog sensors.
constexpr float kSensorAlpha = 0.3f;

} // namespace config
