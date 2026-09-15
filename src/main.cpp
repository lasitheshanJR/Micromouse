/*
  Micromouse Firmware - PlatformIO C++
  -----------------------------------------------------------------
  Thin entry point: the maze algorithm lives in the shared Mouse core
  (include/mouse.h, src/mouse.cpp) and the robot is driven through the
  HardwareIO bridge (include/hardware_io.h, src/hardware_io.cpp).
*/

#include <Arduino.h>

#include "hardware_io.h"
#include "mouse.h"

namespace {
constexpr uint8_t kUserLed = PC13;

HardwareIO io;
Mouse mouse(io);
} // namespace

void setup() {
  Serial.begin(115200);

  // Configure 12-Bit ADC resolution for STM32
  analogReadResolution(12);

  pinMode(kUserLed, OUTPUT);
  digitalWrite(kUserLed, HIGH); // Turn off active-low LED

  // Log core events to Serial. Use LogLevel::Debug to also stream every IR
  // reading and the full move trace; LogLevel::None to silence.
  io.setLogLevel(LogLevel::Info);

  io.begin();
  mouse.reset();
}

void loop() {
  if (mouse.atTarget()) {
    digitalWrite(kUserLed, LOW); // Turn on LED when target reached
    return;
  }

  mouse.step();
}
