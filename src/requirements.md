# Project goal
- Firmware for the ESP32 device working as Supla device which allows control of Haier AC in Supla IoT environment

## Key technical informations
- Haier AC has UART interface which is used by Haier wifi dongle to control AC
- Communication of the wifi dongle and AC unit is using HaierProtocol
- Wifi dongle changes only functions in the same way as remote controll. Whole AC automation and logic is done by the unit itself.
- Wifi dongle can read state of the AC via UART e.g. room temperature, working functions etc.

## Code base
- Supla-device https://github.com/SUPLA/supla-device
- HaierProtocol - https://github.com/paveldn/HaierProtocol

## Additional instructions
- Firmware should utilize HaierProtocol as much as reasonable not to double same functions in the code

# reference information for inspiration how functions may be implemented
## inspiration for use of HVAC channel base on Zigbee to Supla
- Z2S library https://github.com/lsroka76/Z2S_Library/tree/main
Z2S is Zigbee to Supla gateway. It supports use of Zigbee Thermostats. It reads or writhes thermostat via Zigbee and synchronises it with Supla channel of HVAC type. So HVAC channel works as remote control not as a full AC logic. Cooling or heating automation logic is a default functionionality of HVAC supla channel.

## information about use of HVAC channel as remote control from supla forum. Use it as instructions or inspiration
1. Termostat oparty o klasę HvacBase. Innego u nas nie ma ;). Ogólnie trzeba zrobić sobie własną implementację klasy Supla::Control::OutputInterface, na której przechowujesz stan "czy grzeje" (czytany z radia) i ustawiasz metodę "isOutputControlledInternally" aby zwracała false.
Potem trzeba pilnować tego co jest na głowicy i w odpowiedniej klasie HVAC i odpowiednio przesyłać dane (czyli zmiana nastawy na głowicy -> propagujemy na klasę hvac; zmiana na hvac -> propagujemy na głowicę).
2. Ja to robiłem tak, że mam klasę do obsługi głowicy i ona ma membera HvacBase oraz trzymam w zmiennych ostanie znane nastawy z głowicy.
Gdy głowica przyśle update, to zapisuję to do tych zmiennych oraz ustawiam na hvac, dzięki czemu dane lecą do Supli.
W iterateAlways czytam sobie nastawy z hvac i jeśli są różne niż te w zmiennych, to wysyłam na głowicę.
To tak w skrócie. Czasem trzeba przez chwilę ignorować wartości z głowicy, aby się jakieś pętle nie porobiły
3. Wrzuciłem przed chwilą do biblioteki klasę RemoteOutputInterface.
Ją ustaw jako output dla HVAC i na niej ustawiaj wartość odczytaną z głowicy (metodą setOutputValueFromRemote() ).
W konstruktorze podaj true (dla on/off) lub false (dla 0-100%).

## Inspiration for the code. Use of the remote control of haier ac by another iot platform esphome instead of Supla
- Haier-esphome implementation of HaierProtocol for EspHome https://github.com/paveldn/haier-esphome
- Esphome as reference for Haier-esphome https://github.com/esphome/esphome

# Required functions
Defice should control all Haier AC functions. Controll is using channels in Supla-Device.
AC unit uses SmartAir2 Haier protocol.
Control of AC functions is simillar to use of remote control. HVAC control is done by the AC unit itself.

## Supported AC functions in SmartAir2 unit
- Power ON/OFF
- AC mode - Cool, Heat, Dry, Fan, Auto
- Fan mode - High, Mid, Low, Auto
- Swing vertical ON/OFF
- Swing horizontal ON/OFF
- Health mode ON/OFF
- Quite mode ON/OFF
- Room themperature and humidity sensor reading
- Required themperature setting 16oC to 30oC
- Display ON/OFF

## functions controled by HVAC supla channel
### first version
- Power ON/OFF
- Required themperature setting 16oC to 30oC despite of chosen AC mode
- AC mode is controlled outside of this channel. Channel should not block use of AC modes settings
### expanded version
- Power ON/OFF
- Required themperature setting 16oC to 30oC despite of chosen AC mode
- Option to choose heat or cool mode
- Choise of the heat/cool mode should not block use of remaining ac modes. If e.g. Dry mode is set outside this channel, channel should still enable setting of required temperature.

## Supla device specific functions
- Device name should be set to Haier2Supla_XXXX where XXXX is last 3 digits of MAC address or anhother unique ESP32 identivicator
- Boot board button should work as config button in supla
- HAIER UART pins should be defined in WEB interface
- Boot button pins shoud be defined in WEB interface
- LED pin should be defined in WEB interface
- Room name should be defined in the WEB interface. Room name is a prefix for all supla channels of the device
- Level of interface should be defined in WEB interface: Minimal, Standard, Debug. Those levels defines what and how many channels/functions are added to the device. Add a note in the WEB interface that change of interface level requires removing and adding device to supla.org cloud
