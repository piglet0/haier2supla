\
#define STATUS_LED_GPIO 15
#define AC_ROOM "Haier"
#define BUTTON_CFG_GPIO 9


#include <Arduino.h>
#include <cstring>
#include <SuplaDevice.h>
#include <supla/network/esp_wifi.h>
#include <supla/device/status_led.h>
#include <supla/storage/littlefs_config.h>
#include <supla/network/esp_web_server.h>
#include <supla/network/html/device_info.h>
#include <supla/network/html/protocol_parameters.h>
#include <supla/network/html/status_led_parameters.h>
#include <supla/network/html/wifi_parameters.h>
#include <supla/network/html/custom_text_parameter.h>
#include <supla/network/html/custom_parameter.h>
#include <supla/network/html/select_input_parameter.h>
#include <supla/control/button.h>
#include <supla/control/virtual_relay.h>
#include <supla/storage/eeprom.h>
#include <supla/storage/littlefs_config.h>
#include <supla/network/html/text_cmd_input_parameter.h>
#include <supla/network/html/select_cmd_input_parameter.h>
#include <supla/events.h>

#include <supla/sensor/virtual_thermometer.h>
#include <supla/sensor/virtual_binary.h>
#include "HaierSerialStream.h"
#include "HaierSmartair2Controller.h"
#include "HaierAcVirtualRelay.h"
#include "HaierAcHvacChannel.h"
#include "smartair2_packet.h"

// SUPLA components
Supla::Eeprom eeprom;
Supla::ESPWifi wifi;
Supla::EspWebServer suplaServer;
Supla::LittleFsConfig configSupla;
Supla::Device::StatusLed statusLed(STATUS_LED_GPIO, false);  // not reversed state

// Configuration tags for custom parameters
#define CFG_TAG_ROOM_NAME "room_name"
#define CFG_TAG_RX_PIN "uart_rx_pin"
#define CFG_TAG_TX_PIN "uart_tx_pin"
#define CFG_TAG_INTERFACE_LEVEL "iface_level"


// Haier SmartAir2 over UART1
HardwareSerial HaierSerial(1);

// Default UART pin values (can be overridden from web config)
constexpr uint8_t DEFAULT_HAIER_UART_RX_PIN = 14;
constexpr uint8_t DEFAULT_HAIER_UART_TX_PIN = 20;
constexpr uint32_t HAIER_UART_BAUD  = 9600;

// Global variables for configurable pins (will be initialized from config)
uint8_t haierUartRxPin = DEFAULT_HAIER_UART_RX_PIN;
uint8_t haierUartTxPin = DEFAULT_HAIER_UART_TX_PIN;

HaierSmartair2Controller haierController(
    HaierSerial,
    DEFAULT_HAIER_UART_RX_PIN,  // Will be updated in setup()
    DEFAULT_HAIER_UART_TX_PIN,  // Will be updated in setup()
    HAIER_UART_BAUD,
  false   // use_crc = false - CRC is not used by SmartAir2, and enabling it would require changes in the protocol handler to not expect CRC bytes
);

Supla::Control::HaierVirtualRelay *haierPower = nullptr;
Supla::Control::HaierVirtualRelayWithArgOn<Supla::haier::smartair2_protocol::ConditioningMode> *haierCoolRelay = nullptr;
Supla::Control::HaierVirtualRelayWithArgOn<Supla::haier::smartair2_protocol::ConditioningMode> *haierHeatRelay = nullptr;
Supla::Control::HaierVirtualRelayWithArgOn<Supla::haier::smartair2_protocol::ConditioningMode> *haierDryRelay = nullptr;
Supla::Control::HaierVirtualRelayWithArgOn<Supla::haier::smartair2_protocol::ConditioningMode> *haierFanRelay = nullptr;
Supla::Control::HaierVirtualRelayWithArgOn<Supla::haier::smartair2_protocol::ConditioningMode> *haierAutoRelay = nullptr;
Supla::Control::HaierVirtualRelayWithArgOn<Supla::haier::smartair2_protocol::FanMode> *haierFanHighRelay = nullptr;
Supla::Control::HaierVirtualRelayWithArgOn<Supla::haier::smartair2_protocol::FanMode> *haierFanMidRelay = nullptr;
Supla::Control::HaierVirtualRelayWithArgOn<Supla::haier::smartair2_protocol::FanMode> *haierFanLowRelay = nullptr;
Supla::Control::HaierVirtualRelayWithArgOn<Supla::haier::smartair2_protocol::FanMode> *haierFanAutoRelay = nullptr;
Supla::Control::HaierVirtualRelay *haierSwingHorizontalRelay = nullptr;
Supla::Control::HaierVirtualRelay *haierSwingVerticalRelay = nullptr;
Supla::Control::HaierVirtualRelay *haierHealthRelay = nullptr;
Supla::Control::HaierVirtualRelay *haierDisplayRelay = nullptr;
Supla::Control::HaierVirtualRelay *haierQuietRelay = nullptr;
//Supla::Control::HaierAcHvacChannel *haierHvacChannel = nullptr;
Supla::Sensor::VirtualBinary *haierPowerState = nullptr;
Supla::Sensor::VirtualThermometer *haierTemperature = nullptr;
Supla::Control::HaierVirtualRelay *haierTempIncrease = nullptr;
Supla::Sensor::VirtualThermometer *haierSetTemperature = nullptr;
Supla::Sensor::VirtualBinary *haierModeCool = nullptr;
Supla::Sensor::VirtualBinary *haierModeHeat = nullptr;
Supla::Sensor::VirtualBinary *haierModeDry = nullptr;
Supla::Sensor::VirtualBinary *haierModeAuto = nullptr;
Supla::Sensor::VirtualBinary *haierModeFan = nullptr; 
Supla::Sensor::VirtualBinary *haierFanHigh = nullptr;
Supla::Sensor::VirtualBinary *haierFanMid = nullptr;
Supla::Sensor::VirtualBinary *haierFanLow = nullptr;
Supla::Sensor::VirtualBinary *haierFanAuto = nullptr;
Supla::Sensor::VirtualBinary *haierTurbo = nullptr;
Supla::Sensor::VirtualBinary *haierQuiet = nullptr;
Supla::Sensor::VirtualBinary *haierDisplay = nullptr; // 0 -włączony
Supla::Sensor::VirtualBinary *haierLockRemote = nullptr;
Supla::Sensor::VirtualBinary *haierHealth = nullptr;
Supla::Sensor::VirtualBinary *haierCompressor = nullptr;
Supla::Sensor::VirtualBinary *haierTenDegree = nullptr;
Supla::Sensor::VirtualBinary *haierUseSwingBits = nullptr;
Supla::Sensor::VirtualBinary *haierHorizontalSwing = nullptr;
Supla::Sensor::VirtualBinary *haierVerticalSwing = nullptr;
// Supla::Sensor::GeneralPurposeMeasurement *haierModeString = nullptr; 

void setup() {
  Serial.begin(115200);
  delay(100);
// config button to reset WiFi and other settings
  auto buttonCfg = new Supla::Control::Button(BUTTON_CFG_GPIO, true, true);
  buttonCfg->configureAsConfigButton(&SuplaDevice);

  // SUPLA web configuration
  new Supla::Html::DeviceInfo(&SuplaDevice);
  new Supla::Html::WifiParameters;
  new Supla::Html::ProtocolParameters;
  new Supla::Html::StatusLedParameters;
  
  // Custom configuration parameters
  new Supla::Html::CustomTextParameter(CFG_TAG_ROOM_NAME, "Room Name", 10);
  new Supla::Html::CustomParameter(CFG_TAG_RX_PIN, "UART RX Pin", DEFAULT_HAIER_UART_RX_PIN);
  new Supla::Html::CustomParameter(CFG_TAG_TX_PIN, "UART TX Pin", DEFAULT_HAIER_UART_TX_PIN);
  
  // Interface Level dropdown
  auto interfaceLevel = new Supla::Html::SelectInputParameter(CFG_TAG_INTERFACE_LEVEL, "Interface Level");
  interfaceLevel->registerValue("Minimal", 0);
  interfaceLevel->registerValue("Application", 1);
  interfaceLevel->registerValue("Debug", 2);
  
  SuplaDevice.setInitialMode(Supla::InitialMode::StartInCfgMode);
  SuplaDevice.begin();
  
  // Load custom configuration values
  //Supla::Storage::Init(); 
  auto cfg = Supla::Storage::ConfigInstance();
  char roomName[11] = {};
  if (cfg && cfg->getString(CFG_TAG_ROOM_NAME, roomName, sizeof(roomName))) {
    SUPLA_LOG_DEBUG("Loaded room name: %s", roomName);
  } else {
    strncpy(roomName, AC_ROOM, sizeof(roomName) - 1);
  }

  int32_t rxPin = DEFAULT_HAIER_UART_RX_PIN;
  int32_t txPin = DEFAULT_HAIER_UART_TX_PIN;
  if (cfg) {
    cfg->getInt32(CFG_TAG_RX_PIN, &rxPin);
    cfg->getInt32(CFG_TAG_TX_PIN, &txPin);
  }
  haierUartRxPin = static_cast<uint8_t>(rxPin);
  haierUartTxPin = static_cast<uint8_t>(txPin);

  int32_t interfaceLvl = 2; // Default to Application
  if (cfg) {
    cfg->getInt32(CFG_TAG_INTERFACE_LEVEL, &interfaceLvl);
  }

  SUPLA_LOG_DEBUG("Configured interface level: %d", interfaceLvl);

  SUPLA_LOG_DEBUG("UART RX Pin: %d", haierUartRxPin);
  SUPLA_LOG_DEBUG("UART TX Pin: %d", haierUartTxPin);
  SUPLA_LOG_DEBUG("Interface Level: %d", interfaceLvl);

  // Init Haier controller with configured pins
  // Note: The controller was already constructed with default pins
  // For proper reconfiguration, you may need to reinitialize the serial
  HaierSerial.begin(HAIER_UART_BAUD, SERIAL_8N1, haierUartRxPin, haierUartTxPin);
  haierController.begin();

  // Build channel captions using configured room name
  char caption[100];

  // Create channels based on interface level (cumulative)
  // Level 0 (Minimal): Power on/off only
  haierPower = new Supla::Control::HaierVirtualRelay(
    std::bind(&HaierSmartair2Controller::setPower, &haierController, std::placeholders::_1),
    std::bind(&HaierSmartair2Controller::getPower, &haierController)
  );
  haierPower->setDefaultFunction(SUPLA_CHANNELFNC_POWERSWITCH);
  haierPower->setDefaultStateRestore();
  snprintf(caption, sizeof(caption), "%s-ON/OFF", roomName);
  haierPower->setInitialCaption(caption);

    haierCoolRelay = new Supla::Control::HaierVirtualRelayWithArgOn<Supla::haier::smartair2_protocol::ConditioningMode>(
      std::bind(&HaierSmartair2Controller::setACMode, &haierController, std::placeholders::_1),
      std::bind(&HaierSmartair2Controller::getACMode, &haierController),
      Supla::haier::smartair2_protocol::ConditioningMode::COOL
    );
    haierCoolRelay->setDefaultFunction(SUPLA_CHANNELFNC_POWERSWITCH);
    snprintf(caption, sizeof(caption), "%s-Mode:Cool", roomName);
    haierCoolRelay->setInitialCaption(caption);

    haierHeatRelay = new Supla::Control::HaierVirtualRelayWithArgOn<Supla::haier::smartair2_protocol::ConditioningMode>(
      std::bind(&HaierSmartair2Controller::setACMode, &haierController, std::placeholders::_1),
      std::bind(&HaierSmartair2Controller::getACMode, &haierController),
      Supla::haier::smartair2_protocol::ConditioningMode::HEAT
    );
    haierHeatRelay->setDefaultFunction(SUPLA_CHANNELFNC_POWERSWITCH);
    snprintf(caption, sizeof(caption), "%s-Mode:Heat", roomName);
    haierHeatRelay->setInitialCaption(caption);

          // SUPLA channel: room temperature from Haier (SmartAir2 STATUS packet)
      haierTemperature = new Supla::Sensor::VirtualThermometer;
      snprintf(caption, sizeof(caption), "%s-Room Temp", roomName);
      haierTemperature->setInitialCaption(caption);

      // SUPLA channel: target temperature from Haier AC
      haierSetTemperature = new Supla::Sensor::VirtualThermometer;
      snprintf(caption, sizeof(caption), "%s-Set Temp", roomName);
      haierSetTemperature->setInitialCaption(caption);  

      // SUPLA channel: virtual relay to increase/decrease temperature
      haierTempIncrease = new Supla::Control::HaierVirtualRelay(
        std::bind(&HaierSmartair2Controller::increaseTargetTemperatureC, &haierController, std::placeholders::_1),
        []() { return false; }
      );
      haierTempIncrease->setDefaultFunction(SUPLA_CHANNELFNC_POWERSWITCH);
      snprintf(caption, sizeof(caption), " -1°C < %s Set Temp > +1°C", roomName);
      haierTempIncrease->setInitialCaption(caption);


    haierFanHighRelay = new Supla::Control::HaierVirtualRelayWithArgOn<Supla::haier::smartair2_protocol::FanMode>(
      std::bind(&HaierSmartair2Controller::setFanMode, &haierController, std::placeholders::_1),
      std::bind(&HaierSmartair2Controller::getFanMode, &haierController),
      Supla::haier::smartair2_protocol::FanMode::FAN_HIGH
    );
    haierFanHighRelay->setDefaultFunction(SUPLA_CHANNELFNC_POWERSWITCH);
    snprintf(caption, sizeof(caption), "%s-Fan:High", roomName);
    haierFanHighRelay->setInitialCaption(caption);
    
    haierFanMidRelay = new Supla::Control::HaierVirtualRelayWithArgOn<Supla::haier::smartair2_protocol::FanMode>(
      std::bind(&HaierSmartair2Controller::setFanMode, &haierController, std::placeholders::_1),
      std::bind(&HaierSmartair2Controller::getFanMode, &haierController),
      Supla::haier::smartair2_protocol::FanMode::FAN_MID
    );
    haierFanMidRelay->setDefaultFunction(SUPLA_CHANNELFNC_POWERSWITCH);
    snprintf(caption, sizeof(caption), "%s-Fan:Mid", roomName);
    haierFanMidRelay->setInitialCaption(caption);
    
    haierFanLowRelay = new Supla::Control::HaierVirtualRelayWithArgOn<Supla::haier::smartair2_protocol::FanMode>(
      std::bind(&HaierSmartair2Controller::setFanMode, &haierController, std::placeholders::_1),
      std::bind(&HaierSmartair2Controller::getFanMode, &haierController),
      Supla::haier::smartair2_protocol::FanMode::FAN_LOW
    );
    haierFanLowRelay->setDefaultFunction(SUPLA_CHANNELFNC_POWERSWITCH);
    snprintf(caption, sizeof(caption), "%s-Fan:Low", roomName);
    haierFanLowRelay->setInitialCaption(caption);
    
    haierFanAutoRelay = new Supla::Control::HaierVirtualRelayWithArgOn<Supla::haier::smartair2_protocol::FanMode>(
      std::bind(&HaierSmartair2Controller::setFanMode, &haierController, std::placeholders::_1),
      std::bind(&HaierSmartair2Controller::getFanMode, &haierController),
      Supla::haier::smartair2_protocol::FanMode::FAN_AUTO
    );
    haierFanAutoRelay->setDefaultFunction(SUPLA_CHANNELFNC_POWERSWITCH);
    snprintf(caption, sizeof(caption), "%s-Fan:Auto", roomName);
    haierFanAutoRelay->setInitialCaption(caption);


  // Level 1 (Application): Add HVAC channel for temperature control
 
  if (interfaceLvl >= 1) {
 /*
    haierHvacChannel = new Supla::Control::HaierAcHvacChannel(&haierController);
    snprintf(caption, sizeof(caption), "%s-AC Control", roomName);
    haierHvacChannel->setInitialCaption(caption);
 */
    
    haierDryRelay = new Supla::Control::HaierVirtualRelayWithArgOn<Supla::haier::smartair2_protocol::ConditioningMode>(
      std::bind(&HaierSmartair2Controller::setACMode, &haierController, std::placeholders::_1),
      std::bind(&HaierSmartair2Controller::getACMode, &haierController),
      Supla::haier::smartair2_protocol::ConditioningMode::DRY
    );
    haierDryRelay->setDefaultFunction(SUPLA_CHANNELFNC_POWERSWITCH);
    snprintf(caption, sizeof(caption), "%s-Mode:Dry", roomName);
    haierDryRelay->setInitialCaption(caption);
    
    haierFanRelay = new Supla::Control::HaierVirtualRelayWithArgOn<Supla::haier::smartair2_protocol::ConditioningMode>(
      std::bind(&HaierSmartair2Controller::setACMode, &haierController, std::placeholders::_1),
      std::bind(&HaierSmartair2Controller::getACMode, &haierController),
      Supla::haier::smartair2_protocol::ConditioningMode::FAN
    );
    haierFanRelay->setDefaultFunction(SUPLA_CHANNELFNC_POWERSWITCH);
    snprintf(caption, sizeof(caption), "%s-Mode:Fan", roomName);
    haierFanRelay->setInitialCaption(caption);
    
    haierAutoRelay = new Supla::Control::HaierVirtualRelayWithArgOn<Supla::haier::smartair2_protocol::ConditioningMode>(
      std::bind(&HaierSmartair2Controller::setACMode, &haierController, std::placeholders::_1),
      std::bind(&HaierSmartair2Controller::getACMode, &haierController),
      Supla::haier::smartair2_protocol::ConditioningMode::AUTO
    );
    haierAutoRelay->setDefaultFunction(SUPLA_CHANNELFNC_POWERSWITCH);
    snprintf(caption, sizeof(caption), "%s-Mode:Auto", roomName);
    haierAutoRelay->setInitialCaption(caption);
    haierSwingHorizontalRelay = new Supla::Control::HaierVirtualRelay(
      std::bind(&HaierSmartair2Controller::setHorizontalSwing, &haierController, std::placeholders::_1),
      std::bind(&HaierSmartair2Controller::getHorizontalSwing, &haierController)
    );
    haierSwingHorizontalRelay->setDefaultFunction(SUPLA_CHANNELFNC_POWERSWITCH);
    snprintf(caption, sizeof(caption), "%s-Horizontal Swing", roomName);
    haierSwingHorizontalRelay->setInitialCaption(caption);
    
    haierSwingVerticalRelay = new Supla::Control::HaierVirtualRelay(
      std::bind(&HaierSmartair2Controller::setVerticalSwing, &haierController, std::placeholders::_1),
      std::bind(&HaierSmartair2Controller::getVerticalSwing, &haierController)
    );
    haierSwingVerticalRelay->setDefaultFunction(SUPLA_CHANNELFNC_POWERSWITCH);
    snprintf(caption, sizeof(caption), "%s-Vertical Swing", roomName);
    haierSwingVerticalRelay->setInitialCaption(caption);
    
// TODO how to reverse action?    
    haierDisplayRelay = new Supla::Control::HaierVirtualRelay(
      std::bind(&HaierSmartair2Controller::setDisplayStatus, &haierController, std::placeholders::_1),
      std::bind(&HaierSmartair2Controller::getDisplayStatus, &haierController)
    );
    haierDisplayRelay->setDefaultFunction(SUPLA_CHANNELFNC_POWERSWITCH);
    snprintf(caption, sizeof(caption), "%s-Disable Display", roomName);
    haierDisplayRelay->setInitialCaption(caption);
    
    haierQuietRelay = new Supla::Control::HaierVirtualRelay(
      std::bind(&HaierSmartair2Controller::setQuiet, &haierController, std::placeholders::_1),
      std::bind(&HaierSmartair2Controller::getQuiet, &haierController)
    );
    haierQuietRelay->setDefaultFunction(SUPLA_CHANNELFNC_POWERSWITCH);
    snprintf(caption, sizeof(caption), "%s-Quiet Mode", roomName);
    haierQuietRelay->setInitialCaption(caption);
    
    haierHealthRelay = new Supla::Control::HaierVirtualRelay(
        std::bind(&HaierSmartair2Controller::setHealthMode, &haierController, std::placeholders::_1),
        std::bind(&HaierSmartair2Controller::getHealthMode, &haierController)
      );
      haierHealthRelay->setDefaultFunction(SUPLA_CHANNELFNC_POWERSWITCH);
      snprintf(caption, sizeof(caption), "%s-Health Mode", roomName);
      haierHealthRelay->setInitialCaption(caption);

    
  }

  // Level 2 (Debug): Add all remaining channels
  if (interfaceLvl >= 2) {
    SUPLA_LOG_DEBUG("Creating Level 2 (Debug) channels...");

       
      // Supla channel: binary sensor for Haier AC power state
      haierPowerState = new Supla::Sensor::VirtualBinary;
      haierPowerState->setDefaultFunction(SUPLA_CHANNELFNC_BINARY_SENSOR);
      snprintf(caption, sizeof(caption), "%s-Power", roomName);
      haierPowerState->setInitialCaption(caption);


      // SUPLA channel: binary sensor indicating AC is in COOL mode
      haierModeCool = new Supla::Sensor::VirtualBinary;
      haierModeCool->setDefaultFunction(SUPLA_CHANNELFNC_BINARY_SENSOR);
      snprintf(caption, sizeof(caption), "%s-Mode:Cool", roomName);
      haierModeCool->setInitialCaption(caption);
      // SUPLA channel: binary sensor indicating AC is in HEAT mode
      haierModeHeat = new Supla::Sensor::VirtualBinary;
      haierModeHeat->setDefaultFunction(SUPLA_CHANNELFNC_BINARY_SENSOR);
      snprintf(caption, sizeof(caption), "%s-Mode:Heat", roomName);
      haierModeHeat->setInitialCaption(caption);
      // SUPLA channel: binary sensor indicating AC is in DRY mode
      haierModeDry = new Supla::Sensor::VirtualBinary;
      haierModeDry->setDefaultFunction(SUPLA_CHANNELFNC_BINARY_SENSOR);
      snprintf(caption, sizeof(caption), "%s-Mode:Dry", roomName);
      haierModeDry->setInitialCaption(caption);
      // SUPLA channel: binary sensor indicating AC is in AUTO mode
      haierModeAuto = new Supla::Sensor::VirtualBinary;
      haierModeAuto->setDefaultFunction(SUPLA_CHANNELFNC_BINARY_SENSOR);
      snprintf(caption, sizeof(caption), "%s-Mode:Auto", roomName);
      haierModeAuto->setInitialCaption(caption);
      // SUPLA channel: binary sensor indicating AC is in FAN mode
      haierModeFan = new Supla::Sensor::VirtualBinary;
      haierModeFan->setDefaultFunction(SUPLA_CHANNELFNC_BINARY_SENSOR);
      snprintf(caption, sizeof(caption), "%s-Mode:Fan", roomName);
      haierModeFan->setInitialCaption(caption); 
      // SUPLA channel: binary sensor indicating AC fan is in HIGH mode
      haierFanHigh = new Supla::Sensor::VirtualBinary;
      haierFanHigh->setDefaultFunction(SUPLA_CHANNELFNC_BINARY_SENSOR);
      snprintf(caption, sizeof(caption), "%s-Fan:High", roomName);
      haierFanHigh->setInitialCaption(caption);
      // SUPLA channel: binary sensor indicating AC fan is in MID mode
      haierFanMid = new Supla::Sensor::VirtualBinary;
      haierFanMid->setDefaultFunction(SUPLA_CHANNELFNC_BINARY_SENSOR);
      snprintf(caption, sizeof(caption), "%s-Fan:Mid", roomName);
      haierFanMid->setInitialCaption(caption);
      // SUPLA channel: binary sensor indicating AC fan is in LOW mode
      haierFanLow = new Supla::Sensor::VirtualBinary;
      haierFanLow->setDefaultFunction(SUPLA_CHANNELFNC_BINARY_SENSOR);
      snprintf(caption, sizeof(caption), "%s-Fan:Low", roomName);
      haierFanLow->setInitialCaption(caption);
      // SUPLA channel: binary sensor indicating AC fan is in AUTO mode
      haierFanAuto = new Supla::Sensor::VirtualBinary;
      haierFanAuto->setDefaultFunction(SUPLA_CHANNELFNC_BINARY_SENSOR);
      snprintf(caption, sizeof(caption), "%s-Fan:Auto", roomName);
      haierFanAuto->setInitialCaption(caption);
      // SUPLA channel: binary sensor indicating Turbo mode
      haierTurbo = new Supla::Sensor::VirtualBinary;
      haierTurbo->setDefaultFunction(SUPLA_CHANNELFNC_BINARY_SENSOR);
      snprintf(caption, sizeof(caption), "%s-Turbo", roomName);
      haierTurbo->setInitialCaption(caption);
      // SUPLA channel: binary sensor indicating Quiet mode
      haierQuiet = new Supla::Sensor::VirtualBinary;
      haierQuiet->setDefaultFunction(SUPLA_CHANNELFNC_BINARY_SENSOR);
      snprintf(caption, sizeof(caption), "%s-Quiet", roomName);
      haierQuiet->setInitialCaption(caption);
      // SUPLA channel: binary sensor indicating Display status
      haierDisplay = new Supla::Sensor::VirtualBinary;
      haierDisplay->setDefaultFunction(SUPLA_CHANNELFNC_BINARY_SENSOR);
      snprintf(caption, sizeof(caption), "%s-Display", roomName);
      haierDisplay->setInitialCaption(caption); // 0 -włączony
      // SUPLA channel: binary sensor indicating Lock Remote
      haierLockRemote = new Supla::Sensor::VirtualBinary;
      haierLockRemote->setDefaultFunction(SUPLA_CHANNELFNC_BINARY_SENSOR);
      snprintf(caption, sizeof(caption), "%s-Lock Remote", roomName);
      haierLockRemote->setInitialCaption(caption);
      // SUPLA channel: binary sensor indicating Health Mode
      haierHealth = new Supla::Sensor::VirtualBinary;
      haierHealth->setDefaultFunction(SUPLA_CHANNELFNC_BINARY_SENSOR);
      snprintf(caption, sizeof(caption), "%s-Health Mode", roomName);
      haierHealth->setInitialCaption(caption);
      // SUPLA channel: binary sensor indicating Compressor state
      haierCompressor = new Supla::Sensor::VirtualBinary;
      haierCompressor->setDefaultFunction(SUPLA_CHANNELFNC_BINARY_SENSOR);
      snprintf(caption, sizeof(caption), "%s-Compressor", roomName);
      haierCompressor->setInitialCaption(caption);
      // SUPLA channel: binary sensor indicating 10°C Mode
      haierTenDegree = new Supla::Sensor::VirtualBinary;
      haierTenDegree->setDefaultFunction(SUPLA_CHANNELFNC_BINARY_SENSOR);
      snprintf(caption, sizeof(caption), "%s-10°C Mode", roomName);
      haierTenDegree->setInitialCaption(caption);
      // SUPLA channel: binary sensor indicating Use Swing Bits
      haierUseSwingBits = new Supla::Sensor::VirtualBinary;
      haierUseSwingBits->setDefaultFunction(SUPLA_CHANNELFNC_BINARY_SENSOR);
      snprintf(caption, sizeof(caption), "%s-Swing Enabled", roomName);
      haierUseSwingBits->setInitialCaption(caption);
      // SUPLA channel: binary sensor indicating Horizontal Swing
      haierHorizontalSwing = new Supla::Sensor::VirtualBinary;
      haierHorizontalSwing->setDefaultFunction(SUPLA_CHANNELFNC_BINARY_SENSOR);
      snprintf(caption, sizeof(caption), "%s-Horizontal Swing", roomName);
      haierHorizontalSwing->setInitialCaption(caption);
      // SUPLA channel: binary sensor indicating Vertical Swing
      haierVerticalSwing = new Supla::Sensor::VirtualBinary;
      haierVerticalSwing->setDefaultFunction(SUPLA_CHANNELFNC_BINARY_SENSOR);
      snprintf(caption, sizeof(caption), "%s-Vertical Swing", roomName);
      haierVerticalSwing->setInitialCaption(caption);
      
  }

  
}

void loop() {
  SuplaDevice.iterate();
  haierController.loop();

  // Update channels based on availability (nullptr check handles interface level)
  
  // Minimal level: Power only
  if (haierPower != nullptr) {
    haierPower->syncState();
  }

  // Application level: Power + Mode relays
  if (haierCoolRelay != nullptr) {
    haierCoolRelay->syncState();
  }
  if (haierHeatRelay != nullptr) {
    haierHeatRelay->syncState();
  }
  if (haierDryRelay != nullptr) {
    haierDryRelay->syncState();
  }
  if (haierFanRelay != nullptr) {
    haierFanRelay->syncState();
  }
  if (haierAutoRelay != nullptr) {
    haierAutoRelay->syncState();
  }
  if (haierFanHighRelay != nullptr) {
    haierFanHighRelay->syncState();
  }
  if (haierFanMidRelay != nullptr) {
    haierFanMidRelay->syncState();
  }
  if (haierFanLowRelay != nullptr) {
    haierFanLowRelay->syncState();
  }
  if (haierFanAutoRelay != nullptr) {
    haierFanAutoRelay->syncState();
  }
  if (haierDisplayRelay != nullptr) {
    haierDisplayRelay->syncState();
  }
  if (haierQuietRelay != nullptr) {
    haierQuietRelay->syncState();
  }
  if (haierSwingHorizontalRelay != nullptr) {
    haierSwingHorizontalRelay->syncState();
  }
  if (haierSwingVerticalRelay != nullptr) {
    haierSwingVerticalRelay->syncState();
  }

  if (haierHealthRelay != nullptr) {
    haierHealthRelay->syncState();
  }
  
  if (haierPowerState != nullptr) {
    haierController.getPower() ? haierPowerState->set() : haierPowerState->clear();
  }
  
  if (haierTemperature != nullptr) {
    haierTemperature->setValue(haierController.getRoomTemperatureC());
  }
  if (haierSetTemperature != nullptr) {
    haierSetTemperature->setValue(haierController.getTargetTemperatureC());
  }

  if (haierModeCool != nullptr) {
    haierController.getModeCool() ? haierModeCool->set() : haierModeCool->clear();
  }
  if (haierModeHeat != nullptr) {
    haierController.getModeHeat() ? haierModeHeat->set() : haierModeHeat->clear();
  }
  if (haierModeDry != nullptr) {
    haierController.getModeDry() ? haierModeDry->set() : haierModeDry->clear();
  }
  if (haierModeAuto != nullptr) {
    haierController.getModeAuto() ? haierModeAuto->set() : haierModeAuto->clear();
  }
  if (haierModeFan != nullptr) {
    haierController.getModeFan() ? haierModeFan->set() : haierModeFan->clear();
  }
  if (haierFanHigh != nullptr) {
    haierController.getFanHigh() ? haierFanHigh->set() : haierFanHigh->clear();
  }
  if (haierFanMid != nullptr) {
    haierController.getFanMid() ? haierFanMid->set() : haierFanMid->clear();
  }
  if (haierFanLow != nullptr) {
    haierController.getFanLow() ? haierFanLow->set() : haierFanLow->clear();
  }
  if (haierFanAuto != nullptr) {
    haierController.getFanAuto() ? haierFanAuto->set() : haierFanAuto->clear();
  } 
  
  if (haierTurbo != nullptr) {
    haierController.getTurbo() ? haierTurbo->set() : haierTurbo->clear();
  }
  if (haierQuiet != nullptr) {
    haierController.getQuiet() ? haierQuiet->set() : haierQuiet->clear();
  }
  if (haierDisplay != nullptr) {
    haierController.getDisplayStatus() ? haierDisplay->set() : haierDisplay->clear();
  }
  if (haierLockRemote != nullptr) {
    haierController.getLockRemote() ? haierLockRemote->set() : haierLockRemote->clear();
  }
  if (haierHealth != nullptr) {
    haierController.getHealthMode() ? haierHealth->set() : haierHealth->clear();
  }
  if (haierCompressor != nullptr) {
    haierController.getCompressor() ? haierCompressor->set() : haierCompressor->clear();
  }
  if (haierTenDegree != nullptr) {
    haierController.getTenDegree() ? haierTenDegree->set() : haierTenDegree->clear();
  }
  if (haierUseSwingBits != nullptr) {
    haierController.getUseSwingBits() ? haierUseSwingBits->set() : haierUseSwingBits->clear();
  }
  if (haierHorizontalSwing != nullptr) {
    haierController.getHorizontalSwing() ? haierHorizontalSwing->set() : haierHorizontalSwing->clear();
  }
  if (haierVerticalSwing != nullptr) {
    haierController.getVerticalSwing() ? haierVerticalSwing->set() : haierVerticalSwing->clear();
  }

}
