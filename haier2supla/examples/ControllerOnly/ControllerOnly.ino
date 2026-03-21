#include <Arduino.h>
#include <Haier2Supla.h>

HardwareSerial haierSerial(1);
HaierSmartair2Controller controller(
    haierSerial,
    14,
    20,
    9600,
    false);

void setup() {
  Serial.begin(115200);
  haierSerial.begin(9600, SERIAL_8N1, 14, 20);
  controller.begin();
}

void loop() {
  controller.loop();
}
