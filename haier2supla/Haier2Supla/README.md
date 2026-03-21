# Haier2Supla

Arduino library for controlling Haier AC units over the SmartAir2 UART protocol on ESP32 boards.

## Contents
- Core SmartAir2 controller
- SmartAir2 packet definitions
- SUPLA adapter classes for HVAC and relay channels
- Minimal example sketch for controller-only usage

## Installation in Arduino IDE
Copy the `Haier2Supla` directory into your Arduino `libraries` folder.

## Dependencies
- HaierProtocol
- SUPLA device library for the adapter classes
- ESP32 Arduino core

## Project layout
- `src/` contains the reusable library code
- `examples/ControllerOnly/` contains a minimal controller example
- The workspace root sketch `haier2supla-test.ino` remains the full reference application for SUPLA integration and web configuration
