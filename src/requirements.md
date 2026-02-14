# Project goal
Library Haier2Supla allowing use of Haier AC in Supla-Device

# Code base
- Supla-device https://github.com/SUPLA/supla-device
- HaierProtocol - subfolder of the project
- Haier2Supla - subfolder with a new library

# Inspiration for the code. Use of the same idea for another iot platform esphome instead of Supla
- HaierProtocol https://github.com/paveldn/HaierProtocol
- Haier-esphome https://github.com/paveldn/haier-esphome
- Esphome https://github.com/esphome/esphome

# Required functions
Library should control all Haier AC functions. Library should also help to establish controll channels in Supla-Device.
AC unit uses SmartAir2 Haier protocol.
Control of AC functions is simillar to use of remote control. HVAC control is done by the AC unit itself.

## Supported AC functions in SmartAir2 unit
- Power ON/OFF
- AC mode - Cool, Heat, Dry, Fan, Auto
- Fan mode - High, Mid, Low, Auto
- Swing vertical ON/OFF
- Swing horizontal ON/OFF
- Health ON/OFF
- Room themperature sensor reading
- Pipe themperature sensor reading (optional)
- Required themperature setting 16oC to 30oC
- Display ON/OFF
- SelfCleaning ON/OFF
- AC test ON/OFF
