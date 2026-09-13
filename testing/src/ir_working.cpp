/*
  6-IR Sensor Reader & Diagnostic — STM32F103 "Blue Pill"
  --------------------------------------------------------
  Reads raw analog voltages from ADC pins and outputs to 
  Serial Monitor to test sensor thresholds and surface readings.
*/

#include <Arduino.h>

const uint8_t IR_FL = PA0; // Far Left
const uint8_t IR_ML = PA1; // Mid Left
const uint8_t IR_CL = PA6; // Center Left
const uint8_t IR_CR = PA7; // Center Right
const uint8_t IR_MR = PB0; // Mid Right
const uint8_t IR_FR = PB1; // Far Right

const uint8_t irPins[6] = {IR_FL, IR_ML, IR_CL, IR_CR, IR_MR, IR_FR};
const char* sensorNames[6] = {"FL", "ML", "CL", "CR", "MR", "FR"};

// Adjust threshold based on white/black surface testing (default 2000 out of 4095)
const int RAW_THRESHOLD = 2000; 

void setup() {
  Serial.begin(115200);

  for (uint8_t i = 0; i < 6; i++) {
    pinMode(irPins[i], INPUT_ANALOG);
  }

  pinMode(PC13, OUTPUT);
}

void loop() {
  int rawValues[6];
  bool isLineDetected[6];

  // Read raw 12-bit ADC values (0 to 4095)
  for (uint8_t i = 0; i < 6; i++) {
    rawValues[i] = analogRead(irPins[i]);
    
    // Change '>' to '<' if sensor logic is inverted for your surface
    isLineDetected[i] = (rawValues[i] > RAW_THRESHOLD); 
  }

  // --- Print Raw ADC Values ---
  Serial.print("RAW: ");
  for (uint8_t i = 0; i < 6; i++) {
    Serial.print(sensorNames[i]);
    Serial.print(":");
    Serial.print(rawValues[i]);
    if (i < 5) Serial.print(" | ");
  }
  
  // --- Print Digital Detection Status ---
  Serial.print("  ==>  STATE: [ ");
  for (uint8_t i = 0; i < 6; i++) {
    Serial.print(isLineDetected[i] ? "█ " : "_ ");
  }
  Serial.println("]");

  digitalWrite(PC13, !digitalRead(PC13)); // Blink onboard LED to show active reading
  delay(150); // Read frequency ~6-7 Hz
}
