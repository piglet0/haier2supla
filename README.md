# haier2supla
Implementation of HaierProtocol in Supla device. This is code for ESP32 to control Haier AC device via Supla. Current version is compatibile with SmartAir2 devices.
Main project file is in **example/haier2supla** folder
## Base of the project - references
Communication to Haier AC is developed by paveldn. He preapred a HaierProtocol library. Communication with Haier AC is based on his other project haier-esphome.
- Supla-Device https://github.com/SUPLA/supla-device
- HaierProtocol https://github.com/paveldn/HaierProtocol
- Haier-esphome https://github.com/paveldn/haier-esphome
## Hardware requirements
### Communication with Haier AC
Haier AC communicate using standard UART serial. Units which uses SmartAir2 app connects UART using standard USB plug. For more information please refer to https://github.com/paveldn/haier-esphome/blob/master/README.rst
### Tested version
Code was tested on ESP32-C6 Super Mini board. UART pins used: RX GPIO 14, TX GPIO 20. Those pins are not used for other functions. Status LED GPIO 15. Config button (BOOT) GPIO 9.
All pins can be changed in the Web Config.
## Current status of the project
Version 
- Support of Haier AC in SmartAir2 version - units that uses SmartAir2 app. Hon protocol is not supported.
- Working functions: On/Off, AC Mode, Fan Mode, Room temperature sensor, Setting of room temperature, Health mode, Display On/Off
- UART, Button, LED pins can be set via web config
- Interface can be set via web config to:
  - minimal - Cool/Heat mode only, Fan mode and temperature setting
  - standard - all AC modes, Fan modes and temperature setting
  - debug - additional sensor channels showing state of AC functions
Note that change of interface level requires removing device from supla cloud and registration with new channels.
Interface level can be changed in the project sketch.
## OTA Update
For the first install upload 4MB file haier2supla-v0.0.0-merged.bin
For OTA update (available in web config) use smaller file haier2supla-v0.0.0.bin 
# Compilation
Project was compiled in VSCode with Arduino Maker plugin. It can be also compiled in Arduino IDE.
## Libraries
Project requries following libriaries to be installed:
- Supla-device - install from the Arduino catalogue
- HaierProtocol - install from the Arduino catalogue
## Compilation settings
- USB CDC On Boot: Enabled - to allow Logs displayed in board USB serial (115200 bound rate)
- Partition scheme: Minimal SPIFFS (1.9MB APP with OTA/ 128KB SPIFFS)
## release
Compiled bin file for ESP32-C6 super mini board (LED GPIO15, BOOT button GPIO9)
# Change log
## v0.0.5
- initial release
- all main AC functions working
## v0.2.0
- change in UX by using HVAC channel. AC modes kept in separate switches.
- option of timer and weekly programs
- added humidity reading
- option to change pins of LED and config button in web config
- bug fix
## v0.2.2
- OTA update
- power-saving mode (command combo)
- display value change (comand compo) - not confirmed
- wifi icon on AC
## v0.2.3
- removed display value change - not working
## v0.2.5
- change to Arduino library
- device name base on MAC to avoid same name for simillar boards
## v0.3.0
- checked examples, comments added
- added turbo mode and 10deg mode in debug interface level