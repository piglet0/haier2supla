# haier2supla
Implementation of HaierProtocol in Supla device. This is code for ESP32 to control Haier AC device via Supla. Current version is compatibile with SmartAir2 devices.
## Base of the project - references
Communication to Haier AC is developed by paveldn. HaierProtocol library is a direct port of his code to Arduino IDE. Communication with Haier AC is based on his other project haier-esphome.
- HaierProtocol https://github.com/paveldn/HaierProtocol
- Haier-esphome https://github.com/paveldn/haier-esphome
## Hardware requirements
### Communication with Haier AC
Haier AC communicate using standard UART serial. Units which uses SmartAir2 app connects UART using standard USB plug. For more information please refer to https://github.com/paveldn/haier-esphome/blob/master/README.rst
### Tested version
Code was tested on ESP32-C6 Super Mini board.
## Current status of the project
Version 
- Support of Haier AC in SmartAir2 version - units that uses SmartAir2 app. Hon protocol is not supported.
- This is a first draft of the code
- Working functions: On/Off, AC Mode, Fan Mode, Room temperature sensor, Setting of room temperature, Health mode, Display On/Off
- UART pins can be set via web config
- Interface can be set via web config to:
-- minimal - Cool/Heat mode only, Fan mode and temperature setting
-- application - all AC modes, Fan modes and temperature setting
-- debug - additional sensor channels showing state of AC functions
# Compilation
Project was compiled in VSCode with Arduino Maker pluggin. It can be also compiled in Arduino IDE.
## Libraries
Project requries following libriaries to be installed:
- Supla-device - from the Arduino catalogue
- HaierProtocol - from the included ZIP file
## Compilation settings

- 
