# 6-Sensor Line & Maze Follower Robot

A custom-built autonomous maze-solving and line-following robot using an **STM32F103C8T6 ("Blue Pill")** microcontroller, an **L293D motor driver**, a 6-sensor analog IR array, and a custom 3D-printed sensor bracket.

---

## Project Overview & Power Architecture

* **Microcontroller:** STM32F103C8T6 ("Blue Pill")
* **Power Source:** 3S LiPo / 3.9 lipo *3 Pack (~11.1V – 11.7V nominal/fully charged)
* **Voltage Regulation:** Step-Down (Buck) Converter tuned to **5.0V output**
* **Motor Driver:** L293D H-Bridge IC
* **Motors:** 2x N20 600 RPM Gear Motors
* **Sensors:** 6x Sharp GP2Y0A51SK0F (or compatible analog IR distance sensors)
* **CAD Design:** Custom SolidWorks bracket to shield and hold IR sensors

---

## Power System & Buck Converter Setup

The system operates on an **11.7V battery supply**, which is too high to power the STM32 and sensor logic directly.

<Callout type="warning" title="High Voltage Hazard">
Do NOT feed 11.7V directly into the STM32 5V or 3.3V pins or the IR sensors. Doing so will permanently destroy the microcontroller and sensors.
</Callout>

### Power Distribution:
1. **11.7V Main Rail:** Connected directly to **L293D Pin 8 (VCC2 / Motor Power)** to give full voltage and performance to the drive motors.
2. **Buck Converter (11.7V → 5.0V):** Steps down the battery voltage to a steady 5V output.
   * **5V Output powers:** STM32 `5V` pin, L293D `Pin 16 (VCC1 / Logic Power)`, and all 6 IR distance sensors.
3. **Common Ground:** All GND connections (Battery, Buck Converter output, STM32, L293D, and Sensors) **MUST** be connected together.

---

## Pinout & Hardware Assignment

### Why Were These Pins Chosen?
* **ADC Requirements (Sensors):** The STM32F103 only has ADC (Analog-to-Digital Converter) functionality on pins `PA0–PA7` and `PB0–PB1`. Pins like `PB5–PB10` are digital-only and **cannot** read analog IR sensors.
* **Motor Control:** Motors require digital output pins capable of PWM logic for speed control. `PA2–PA5` were selected to leave `PA0, PA1, PA6, PA7, PB0, PB1` clear for the 6 analog sensor inputs.

### Pin Mapping Table

| Component | Function | STM32 Pin | Logic Type / Notes |
| :--- | :--- | :--- | :--- |
| **Right Motor** | IN1 | **PA2** | GPIO Output (PWM / Direction) |
| **Right Motor** | IN2 | **PA3** | GPIO Output (PWM / Direction) |
| **Left Motor** | IN3 | **PA4** | GPIO Output (PWM / Direction) |
| **Left Motor** | IN4 | **PA5** | GPIO Output (PWM / Direction) |
| **Far Left IR Sensor** | Sensor 0 | **PA0** | ADC Pin (ADC Channel 0) |
| **Mid Left IR Sensor** | Sensor 1 | **PA1** | ADC Pin (ADC Channel 1) |
| **Center Left IR Sensor** | Sensor 2 | **PA6** | ADC Pin (ADC Channel 6) |
| **Center Right IR Sensor** | Sensor 3 | **PA7** | ADC Pin (ADC Channel 7) |
| **Mid Right IR Sensor** | Sensor 4 | **PB0** | ADC Pin (ADC Channel 8) |
| **Far Right IR Sensor** | Sensor 5 | **PB1** | ADC Pin (ADC Channel 9) |
| **Status LED** | Onboard LED | **PC13** | Diagnostic / Calibration Blink |
| **UART Serial TX** | Telemetry | **PA9** | Serial Output (Connect to FTDI RX) |
| **UART Serial RX** | Telemetry | **PA10** | Serial Input (Connect to FTDI TX) |

---

## Complete Wiring Diagram

## System Wiring Diagram

![Wiring Diagram](./docs/wiring_diagram.png)

### Pinout & Schematic Overview

                     ┌──────────────────────────────────────┐
                     │          11.7V LiPo BATTERY          │
                     └──────────────────┬───────────────────┘
                                        │
                         ┌──────────────┴──────────────┐
                         ▼ (+)                         ▼ (-)
              ┌─────────────────────┐       ┌────────────────────┐
              │ BUCK CONVERTER      │       │                    │
              │ (11.7V In ➔ 5V Out) │       │                    │
              └──────────┬──────────┘       │                    │
                    5V   │                  │                    │
       ┌─────────────────┴─────────┐        │                    │
       ▼                           ▼        ▼                    ▼
┌──────────────┐            ┌──────────────┐          ┌──────────────┐
│  STM32 5V    │            │ L293D Pin 16 │          │ L293D Pin 8  │
│ (Logic VCC)  │            │(Logic VCC1)  │          │(Motor VCC2)  │
└──────┬───────┘            └──────┬───────┘          └──────┬───────┘
       │                           │                         │
       ├─► IR Sensor 5V Power      │                         │
       │                           │                         │
       ▼                           ▼                         ▼
 ───► COMMON GROUND (GND) ◄──────────────────────────────────┴──────

====================================================================
                        DETAILED PIN CONNECTIONS
====================================================================

 ┌─────────────────────────┐               ┌────────────────────────┐
 │   STM32F103 (BLUE PILL) │               │   L293D MOTOR DRIVER   │
 │                         │               │                        │
 │                     PA2 ├──────────────►│ Pin 2  (IN1)           │
 │                     PA3 ├──────────────►│ Pin 7  (IN2)           │
 │                     PA4 ├──────────────►│ Pin 10 (IN3)           │
 │                     PA5 ├──────────────►│ Pin 15 (IN4)           │
 │                         │               │                        │
 │                     5V  ├──────────────►│ Pin 1, 9 (Enable 1 & 2)│
 │                     GND ├──────────────►│ Pin 4, 5, 12, 13 (GND) │
 └─────────────────────────┘               └───────────┬────────────┘
                                                       │
                                   Left Motor  ◄───────┤ Pins 11, 14 (OUT3,4)
                                   Right Motor ◄───────┤ Pins 3, 6   (OUT1,2)

 ┌─────────────────────────┐               ┌────────────────────────┐
 │   STM32 ANALOG PINS     │               │   6x IR SENSOR ARRAY   │
 │                         │               │                        │
 │                     PA0 ├──────────────◄│ Far Left Sensor        │
 │                     PA1 ├──────────────◄│ Mid Left Sensor        │
 │                     PA6 ├──────────────◄│ Center Left Sensor     │
 │                     PA7 ├──────────────◄│ Center Right Sensor    │
 │                     PB0 ├──────────────◄│ Mid Right Sensor       │
 │                     PB1 ├──────────────◄│ Far Right Sensor       │
 └─────────────────────────┘               └────────────────────────┘

### Motor Driver (L293D) Connections
* **Pin 1 (EN1,2):** Connect to `5V` (Always Enabled)
* **Pin 9 (EN3,4):** Connect to `5V` (Always Enabled)
* **Pin 2 (IN1):** STM32 Pin `PA2`
* **Pin 7 (IN2):** STM32 Pin `PA3`
* **Pin 10 (IN3):** STM32 Pin `PA4`
* **Pin 15 (IN4):** STM32 Pin `PA5`
* **Pins 3 & 6 (OUT1, OUT2):** Right N20 Motor
* **Pins 11 & 14 (OUT3, OUT4):** Left N20 Motor
* **Pins 4, 5, 12, 13:** Common Ground (`GND`)

---

## 3D Models & Hardware Cad Files

To prevent optical crosstalk between adjacent infrared sensors and keep alignment rigid relative to the track ground, a custom enclosure and mounting wall was designed in SolidWorks.

* **SolidWorks Design Files:** [`/cad/IR_Sensor_Array_Wall.SLDPRT`](./cad/) *(Update path as needed)*
* **STL Files for 3D Printing:** [`/cad/IR_Sensor_Array_Wall.stl`](./cad/)

---

## Software & PlatformIO Configuration

This project is built using **PlatformIO** with the **Arduino Framework** for the STM32 core.

### `platformio.ini` setup:
```ini
[env:bluepill_f103c8]
platform = ststm32
board = bluepill_f103c8
framework = arduino
upload_protocol = stlink
monitor_speed = 115200
