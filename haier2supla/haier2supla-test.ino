#define AC_ROOM "Haier"
#define FW_VERSION "0.2.3"
#define FW_BUILD_INFO __DATE__ " " __TIME__

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
#include <supla/network/html/button_update.h>
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
#include <supla/sensor/virtual_therm_hygro_meter.h>
#include <supla/sensor/virtual_binary.h>
#include <Haier2Supla.h>

// SUPLA components
Supla::Eeprom eeprom;
Supla::ESPWifi wifi;
Supla::EspWebServer suplaServer;
Supla::LittleFsConfig configSupla;
Supla::Device::StatusLed *statusLed = nullptr;
Supla::Control::Button *buttonCfg = nullptr;

// Configuration tags for custom parameters
#define CFG_TAG_ROOM_NAME "room_name"
#define CFG_TAG_RX_PIN "uart_rx_pin"
#define CFG_TAG_TX_PIN "uart_tx_pin"
#define CFG_TAG_INTERFACE_LEVEL "iface_level"
#define CFG_TAG_LED_PIN "status_led_pin"
#define CFG_TAG_BUTTON_PIN "cfg_button_pin"


// Haier SmartAir2 over UART1
HardwareSerial HaierSerial(1);

// Default UART pin values (can be overridden from web config)
constexpr uint8_t DEFAULT_STATUS_LED_PIN = 15;
constexpr uint8_t DEFAULT_BUTTON_CFG_PIN = 9;
constexpr uint8_t DEFAULT_HAIER_UART_RX_PIN = 14;
constexpr uint8_t DEFAULT_HAIER_UART_TX_PIN = 20;
constexpr int32_t DEFAULT_INTERFACE_LEVEL = 1;
constexpr uint32_t HAIER_UART_BAUD  = 9600;

// Global variables for configurable pins (will be initialized from config)
uint8_t statusLedPin = DEFAULT_STATUS_LED_PIN;
uint8_t buttonCfgPin = DEFAULT_BUTTON_CFG_PIN;
uint8_t haierUartRxPin = DEFAULT_HAIER_UART_RX_PIN;
uint8_t haierUartTxPin = DEFAULT_HAIER_UART_TX_PIN;

HaierSmartair2Controller haierController(
    HaierSerial,
    DEFAULT_HAIER_UART_RX_PIN,  // Will be updated in setup()
    DEFAULT_HAIER_UART_TX_PIN,  // Will be updated in setup()
    HAIER_UART_BAUD,
  false   // use_crc = false - CRC is not used by SmartAir2, and enabling it would require changes in the protocol handler to not expect CRC bytes
);
HaierSuplaStateSync haierStateSync(&haierController);

// Supla::Control::HaierVirtualRelay *haierPower = nullptr;
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
Supla::Control::HaierActionVirtualRelay *haierPowerSavingRelay = nullptr;
// Supla::Control::HaierActionVirtualRelay *haierDisplayTemperatureRelay = nullptr;
Supla::Control::HaierVirtualRelay *haierDisplayRelay = nullptr;
Supla::Control::HaierVirtualRelay *haierQuietRelay = nullptr;
HaierAcHvacChannel *haierHvac = nullptr;
Supla::Sensor::VirtualBinary *haierPowerState = nullptr;
Supla::Sensor::VirtualThermHygroMeter *haierTemperatureHumidity = nullptr;
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

static void onHaierTargetTemperatureChanged(float targetTemp) {
  if (haierSetTemperature != nullptr && isfinite(targetTemp)) {
    haierSetTemperature->setValue(targetTemp);
  }
}

static void buildMacSuffix(char *suffix, size_t suffixSize) {
  const unsigned long long mac = static_cast<unsigned long long>(ESP.getEfuseMac());
  snprintf(suffix, suffixSize, "%012llX", mac & 0xFFFFFFFFFFFFULL);
}

static void buildDefaultDeviceName(char *deviceName, size_t deviceNameSize) {
  char macSuffix[13] = {};
  buildMacSuffix(macSuffix, sizeof(macSuffix));
  snprintf(deviceName, deviceNameSize, "Supla2Haier-%s", macSuffix);
}

static uint8_t parseBuildMonth(const char *buildDate) {
  static const char *const months[] = {"Jan", "Feb", "Mar", "Apr",
                                       "May", "Jun", "Jul", "Aug",
                                       "Sep", "Oct", "Nov", "Dec"};

  for (uint8_t index = 0; index < 12; ++index) {
    if (strncmp(buildDate, months[index], 3) == 0) {
      return index + 1;
    }
  }

  return 0;
}

static void buildCompactSwVersion(char *swVersion, size_t swVersionSize) {
  const char *buildDate = __DATE__;
  const char *buildTime = __TIME__;
  const uint8_t month = parseBuildMonth(buildDate);
  const uint8_t day = static_cast<uint8_t>(atoi(buildDate + 4));
  const uint8_t year = static_cast<uint8_t>(atoi(buildDate + 9));
  const uint8_t hour = static_cast<uint8_t>(atoi(buildTime));
  const uint8_t minute = static_cast<uint8_t>(atoi(buildTime + 3));

  snprintf(swVersion, swVersionSize, "%s-%02u%02u%02u%02u%02u", FW_VERSION,
           year, month, day, hour, minute);
}

static bool isLegacyAutoDeviceName(const char *deviceName) {
  return strncmp(deviceName, "Haier2Supla_", 12) == 0;
}

void setup() {
  Serial.begin(115200);
  delay(2000); // Wait for serial to initialize

  Supla::Storage::Init();
  auto cfg = Supla::Storage::ConfigInstance();

  char roomName[11] = {};
  const bool hasStoredRoomName =
      cfg && cfg->getString(CFG_TAG_ROOM_NAME, roomName, sizeof(roomName));
  if (!hasStoredRoomName) {
    strncpy(roomName, AC_ROOM, sizeof(roomName) - 1);
  }

  int32_t rxPin = DEFAULT_HAIER_UART_RX_PIN;
  int32_t txPin = DEFAULT_HAIER_UART_TX_PIN;
  int32_t ledPin = DEFAULT_STATUS_LED_PIN;
  int32_t cfgPin = DEFAULT_BUTTON_CFG_PIN;
  int32_t interfaceLvl = DEFAULT_INTERFACE_LEVEL;
  bool hasStoredRxPin = false;
  bool hasStoredTxPin = false;
  bool hasStoredLedPin = false;
  bool hasStoredCfgPin = false;
  bool hasStoredInterfaceLvl = false;
  if (cfg) {
    hasStoredRxPin = cfg->getInt32(CFG_TAG_RX_PIN, &rxPin);
    hasStoredTxPin = cfg->getInt32(CFG_TAG_TX_PIN, &txPin);
    hasStoredLedPin = cfg->getInt32(CFG_TAG_LED_PIN, &ledPin);
    hasStoredCfgPin = cfg->getInt32(CFG_TAG_BUTTON_PIN, &cfgPin);
    hasStoredInterfaceLvl = cfg->getInt32(CFG_TAG_INTERFACE_LEVEL, &interfaceLvl);
    if (!hasStoredInterfaceLvl) {
      interfaceLvl = DEFAULT_INTERFACE_LEVEL;
      cfg->setInt32(CFG_TAG_INTERFACE_LEVEL, interfaceLvl);
      cfg->commit();
      hasStoredInterfaceLvl = true;
    }
  }

  statusLedPin = static_cast<uint8_t>(ledPin);
  buttonCfgPin = static_cast<uint8_t>(cfgPin);
  haierUartRxPin = static_cast<uint8_t>(rxPin);
  haierUartTxPin = static_cast<uint8_t>(txPin);

  char deviceName[SUPLA_DEVICE_NAME_MAXSIZE] = {};
  const bool hasStoredDeviceName = cfg && cfg->getDeviceName(deviceName);
  if (!hasStoredDeviceName || isLegacyAutoDeviceName(deviceName)) {
    buildDefaultDeviceName(deviceName, sizeof(deviceName));
    if (cfg) {
      cfg->setDeviceName(deviceName);
      cfg->commit();
    }
  }

  SUPLA_LOG_DEBUG(
      "Startup config: storage=%s room_name=%s (%s) device_name=%s (%s) "
      "interface_level=%d (%s) status_led_pin=%d (%s) cfg_button_pin=%d (%s) "
      "uart_rx_pin=%d (%s) uart_tx_pin=%d (%s)",
      cfg ? "ready" : "missing",
      roomName,
      hasStoredRoomName ? "stored" : "default",
      deviceName,
      hasStoredDeviceName ? "stored" : "generated",
      interfaceLvl,
      hasStoredInterfaceLvl ? "stored" : "default",
      statusLedPin,
      hasStoredLedPin ? "stored" : "default",
      buttonCfgPin,
      hasStoredCfgPin ? "stored" : "default",
      haierUartRxPin,
      hasStoredRxPin ? "stored" : "default",
      haierUartTxPin,
      hasStoredTxPin ? "stored" : "default");

  SuplaDevice.setName(deviceName);
  SuplaDevice.setCustomHostnamePrefix("SUPLA2HAIER");
  char swVersion[18] = {};
  buildCompactSwVersion(swVersion, sizeof(swVersion));
  SuplaDevice.setSwVersion(swVersion);

  SUPLA_LOG_DEBUG("Firmware version: %s, build: %s", FW_VERSION,
                  FW_BUILD_INFO);

  statusLed = new Supla::Device::StatusLed(statusLedPin, false);

  // config button to reset WiFi and other settings
  buttonCfg = new Supla::Control::Button(buttonCfgPin, true, true);
  buttonCfg->configureAsConfigButton(&SuplaDevice);

  // SUPLA web configuration
  new Supla::Html::DeviceInfo(&SuplaDevice);
  new Supla::Html::WifiParameters;
  new Supla::Html::ProtocolParameters;
  new Supla::Html::StatusLedParameters;
  new Supla::Html::ButtonUpdate(&suplaServer);
  
  // Custom configuration parameters
  new Supla::Html::CustomTextParameter(CFG_TAG_ROOM_NAME, "Room Name", 10);
  new Supla::Html::CustomParameter(CFG_TAG_RX_PIN, "UART RX Pin", DEFAULT_HAIER_UART_RX_PIN);
  new Supla::Html::CustomParameter(CFG_TAG_TX_PIN, "UART TX Pin", DEFAULT_HAIER_UART_TX_PIN);
  new Supla::Html::CustomParameter(CFG_TAG_LED_PIN, "Status LED Pin", DEFAULT_STATUS_LED_PIN);
  new Supla::Html::CustomParameter(CFG_TAG_BUTTON_PIN, "Config Button Pin", DEFAULT_BUTTON_CFG_PIN);
  
  // Interface Level dropdown
  auto interfaceLevel = new Supla::Html::SelectInputParameter(
      CFG_TAG_INTERFACE_LEVEL,
      "Interface Level (changing this requires removing and re-adding the device in Supla Cloud)");
  interfaceLevel->registerValue("Minimal", 1);
  interfaceLevel->registerValue("Standard", 2);
  interfaceLevel->registerValue("Debug", 3);

  SuplaDevice.setInitialMode(Supla::InitialMode::StartInCfgMode);
  SuplaDevice.begin();

  // Init Haier controller with configured pins
  // Note: The controller was already constructed with default pins
  // For proper reconfiguration, you may need to reinitialize the serial
  HaierSerial.begin(HAIER_UART_BAUD, SERIAL_8N1, haierUartRxPin, haierUartTxPin);
  haierController.setTargetTemperatureChangedCallback(onHaierTargetTemperatureChanged);
  haierController.begin();

  // Build channel captions using configured room name
  char caption[100];

  // Create channels based on interface level (cumulative)
  // Level 1 (Minimal): basic functions only
  SUPLA_LOG_DEBUG("Creating Level 1 (Minimal) channels...");

  // haierPower channel: turn AC on/off - this is now handled by the HVAC channel, so we won't create a separate power channel to avoid confusion. The HVAC channel will control power state based on the selected mode (e.g., COOL, HEAT, AUTO, etc).
  /* haierPower = new Supla::Control::HaierVirtualRelay(
        std::bind(&HaierSmartair2Controller::setPower, &haierController, std::placeholders::_1),
        std::bind(&HaierSmartair2Controller::getPower, &haierController)
      );
      haierPower->setDefaultFunction(SUPLA_CHANNELFNC_POWERSWITCH);
      haierPower->setDefaultStateRestore();
      snprintf(caption, sizeof(caption), "%s-ON/OFF", roomName);
      haierPower->setInitialCaption(caption);
    */

    // haierHvac channel: combined HVAC control channel that will manage power and setpoint. Mode and Fun mode is set by other relays
    haierHvac = new HaierAcHvacChannel(&haierController);
    snprintf(caption, sizeof(caption), "%s-Temp Setting", roomName);
    haierHvac->setInitialCaption(caption);
    // SUPLA channel: combined room temperature and humidity from Haier
    // Created at level 0 so HVAC thermostat always has a temperature source
    haierTemperatureHumidity = new Supla::Sensor::VirtualThermHygroMeter;
    snprintf(caption, sizeof(caption), "%s-Temp&Humi", roomName);
    haierTemperatureHumidity->setInitialCaption(caption);
    if (haierHvac != nullptr && haierTemperatureHumidity->getChannel() != nullptr) {
      haierHvac->setMainThermometerChannelNo(
          haierTemperatureHumidity->getChannel()->getChannelNumber());
    }
    // AC mode Cool channel
      haierCoolRelay = new Supla::Control::HaierVirtualRelayWithArgOn<Supla::haier::smartair2_protocol::ConditioningMode>(
      std::bind(&HaierSmartair2Controller::setACMode, &haierController, std::placeholders::_1),
      std::bind(&HaierSmartair2Controller::getACMode, &haierController),
      Supla::haier::smartair2_protocol::ConditioningMode::COOL
    );
    haierCoolRelay->setDefaultFunction(SUPLA_CHANNELFNC_POWERSWITCH);
    snprintf(caption, sizeof(caption), "%s-Mode:Cool", roomName);
    haierCoolRelay->setInitialCaption(caption);
    // AC mode Heat channel
    haierHeatRelay = new Supla::Control::HaierVirtualRelayWithArgOn<Supla::haier::smartair2_protocol::ConditioningMode>(
      std::bind(&HaierSmartair2Controller::setACMode, &haierController, std::placeholders::_1),
      std::bind(&HaierSmartair2Controller::getACMode, &haierController),
      Supla::haier::smartair2_protocol::ConditioningMode::HEAT
    );
    haierHeatRelay->setDefaultFunction(SUPLA_CHANNELFNC_POWERSWITCH);
    snprintf(caption, sizeof(caption), "%s-Mode:Heat", roomName);
    haierHeatRelay->setInitialCaption(caption);
    // AC Auto mode channel
    haierAutoRelay = new Supla::Control::HaierVirtualRelayWithArgOn<Supla::haier::smartair2_protocol::ConditioningMode>(
      std::bind(&HaierSmartair2Controller::setACMode, &haierController, std::placeholders::_1),
      std::bind(&HaierSmartair2Controller::getACMode, &haierController),
      Supla::haier::smartair2_protocol::ConditioningMode::AUTO
    );
    haierAutoRelay->setDefaultFunction(SUPLA_CHANNELFNC_POWERSWITCH);
    snprintf(caption, sizeof(caption), "%s-Mode:Auto", roomName);
    haierAutoRelay->setInitialCaption(caption);
    // AC Dry mode channel
    haierDryRelay = new Supla::Control::HaierVirtualRelayWithArgOn<Supla::haier::smartair2_protocol::ConditioningMode>(
      std::bind(&HaierSmartair2Controller::setACMode, &haierController, std::placeholders::_1),
      std::bind(&HaierSmartair2Controller::getACMode, &haierController),
      Supla::haier::smartair2_protocol::ConditioningMode::DRY
    );
    haierDryRelay->setDefaultFunction(SUPLA_CHANNELFNC_POWERSWITCH);
    snprintf(caption, sizeof(caption), "%s-Mode:Dry", roomName);
    haierDryRelay->setInitialCaption(caption);
    // AC Fan only mode channel
    haierFanRelay = new Supla::Control::HaierVirtualRelayWithArgOn<Supla::haier::smartair2_protocol::ConditioningMode>(
      std::bind(&HaierSmartair2Controller::setACMode, &haierController, std::placeholders::_1),
      std::bind(&HaierSmartair2Controller::getACMode, &haierController),
      Supla::haier::smartair2_protocol::ConditioningMode::FAN
    );
    haierFanRelay->setDefaultFunction(SUPLA_CHANNELFNC_POWERSWITCH);
    snprintf(caption, sizeof(caption), "%s-Mode:Fan", roomName);
    haierFanRelay->setInitialCaption(caption);
    // Fan Auto mode channel
    haierFanAutoRelay = new Supla::Control::HaierVirtualRelayWithArgOn<Supla::haier::smartair2_protocol::FanMode>(
      std::bind(&HaierSmartair2Controller::setFanMode, &haierController, std::placeholders::_1),
      std::bind(&HaierSmartair2Controller::getFanMode, &haierController),
      Supla::haier::smartair2_protocol::FanMode::FAN_AUTO
    );
    haierFanAutoRelay->setDefaultFunction(SUPLA_CHANNELFNC_POWERSWITCH);
    snprintf(caption, sizeof(caption), "%s-Fan:Auto", roomName);
    haierFanAutoRelay->setInitialCaption(caption);
    // Fan High channel
    haierFanHighRelay = new Supla::Control::HaierVirtualRelayWithArgOn<Supla::haier::smartair2_protocol::FanMode>(
      std::bind(&HaierSmartair2Controller::setFanMode, &haierController, std::placeholders::_1),
      std::bind(&HaierSmartair2Controller::getFanMode, &haierController),
      Supla::haier::smartair2_protocol::FanMode::FAN_HIGH
    );
    haierFanHighRelay->setDefaultFunction(SUPLA_CHANNELFNC_POWERSWITCH);
    snprintf(caption, sizeof(caption), "%s-Fan:High", roomName);
    haierFanHighRelay->setInitialCaption(caption);
    // Fan Mid mode channel
    haierFanMidRelay = new Supla::Control::HaierVirtualRelayWithArgOn<Supla::haier::smartair2_protocol::FanMode>(
      std::bind(&HaierSmartair2Controller::setFanMode, &haierController, std::placeholders::_1),
      std::bind(&HaierSmartair2Controller::getFanMode, &haierController),
      Supla::haier::smartair2_protocol::FanMode::FAN_MID
    );
    haierFanMidRelay->setDefaultFunction(SUPLA_CHANNELFNC_POWERSWITCH);
    snprintf(caption, sizeof(caption), "%s-Fan:Mid", roomName);
    haierFanMidRelay->setInitialCaption(caption);
    // Fan Low channel
    haierFanLowRelay = new Supla::Control::HaierVirtualRelayWithArgOn<Supla::haier::smartair2_protocol::FanMode>(
      std::bind(&HaierSmartair2Controller::setFanMode, &haierController, std::placeholders::_1),
      std::bind(&HaierSmartair2Controller::getFanMode, &haierController),
      Supla::haier::smartair2_protocol::FanMode::FAN_LOW
    );
    haierFanLowRelay->setDefaultFunction(SUPLA_CHANNELFNC_POWERSWITCH);
    snprintf(caption, sizeof(caption), "%s-Fan:Low", roomName);
    haierFanLowRelay->setInitialCaption(caption);
    // Swing Horzontal channel
    haierSwingHorizontalRelay = new Supla::Control::HaierVirtualRelay(
      std::bind(&HaierSmartair2Controller::setHorizontalSwing, &haierController, std::placeholders::_1),
      std::bind(&HaierSmartair2Controller::getHorizontalSwing, &haierController)
    );
    haierSwingHorizontalRelay->setDefaultFunction(SUPLA_CHANNELFNC_POWERSWITCH);
    snprintf(caption, sizeof(caption), "%s-Horizontal Swing", roomName);
    haierSwingHorizontalRelay->setInitialCaption(caption);
    // Swing Vertical channel
    haierSwingVerticalRelay = new Supla::Control::HaierVirtualRelay(
      std::bind(&HaierSmartair2Controller::setVerticalSwing, &haierController, std::placeholders::_1),
      std::bind(&HaierSmartair2Controller::getVerticalSwing, &haierController)
    );
    haierSwingVerticalRelay->setDefaultFunction(SUPLA_CHANNELFNC_POWERSWITCH);
    snprintf(caption, sizeof(caption), "%s-Vertical Swing", roomName);
    haierSwingVerticalRelay->setInitialCaption(caption);

    haierStateSync.registerLevel1(
      haierCoolRelay,
      haierHeatRelay,
      haierDryRelay,
      haierFanRelay,
      haierAutoRelay,
      haierFanHighRelay,
      haierFanMidRelay,
      haierFanLowRelay,
      haierFanAutoRelay,
      haierSwingHorizontalRelay,
      haierSwingVerticalRelay,
      haierTemperatureHumidity);


// interface level 2 adds mode and fan controls, temperature setpoint control, and combined temperature/humidity sensor

if (interfaceLvl >= 2) {
    SUPLA_LOG_DEBUG("Creating Level 2 (Standard) channels...");

    // Health mode channel   
    haierHealthRelay = new Supla::Control::HaierVirtualRelay(
        std::bind(&HaierSmartair2Controller::setHealthMode, &haierController, std::placeholders::_1),
        std::bind(&HaierSmartair2Controller::getHealthMode, &haierController)
      );
      haierHealthRelay->setDefaultFunction(SUPLA_CHANNELFNC_POWERSWITCH);
      snprintf(caption, sizeof(caption), "%s-Health Mode", roomName);
      haierHealthRelay->setInitialCaption(caption);
    haierPowerSavingRelay = new Supla::Control::HaierActionVirtualRelay(
      std::bind(&HaierSmartair2Controller::triggerPowerSavingModeToggleSequence, &haierController),
      std::bind(&HaierSmartair2Controller::isPowerSavingModeToggleSequenceActive, &haierController)
    );
    haierPowerSavingRelay->setDefaultFunction(SUPLA_CHANNELFNC_POWERSWITCH);
    snprintf(caption, sizeof(caption), "%s-Power Saving Toggle", roomName);
    haierPowerSavingRelay->setInitialCaption(caption);
    /*
    haierDisplayTemperatureRelay = new Supla::Control::HaierActionVirtualRelay(
      std::bind(&HaierSmartair2Controller::triggerDisplayTemperatureToggleSequence, &haierController),
      std::bind(&HaierSmartair2Controller::isDisplayTemperatureToggleSequenceActive, &haierController)
    );
    haierDisplayTemperatureRelay->setDefaultFunction(SUPLA_CHANNELFNC_POWERSWITCH);
    snprintf(caption, sizeof(caption), "%s-Display Temp Toggle", roomName);
    haierDisplayTemperatureRelay->setInitialCaption(caption);
    */
    // Quiet mode channel
    haierQuietRelay = new Supla::Control::HaierVirtualRelay(
      std::bind(&HaierSmartair2Controller::setQuiet, &haierController, std::placeholders::_1),
      std::bind(&HaierSmartair2Controller::getQuiet, &haierController)
    );
    haierQuietRelay->setDefaultFunction(SUPLA_CHANNELFNC_POWERSWITCH);
    snprintf(caption, sizeof(caption), "%s-Quiet Mode", roomName);
    haierQuietRelay->setInitialCaption(caption);
    // Disable display channel    
    haierDisplayRelay = new Supla::Control::HaierVirtualRelay(
      std::bind(&HaierSmartair2Controller::setDisplayStatus, &haierController, std::placeholders::_1),
      std::bind(&HaierSmartair2Controller::getDisplayStatus, &haierController)
    );
    haierDisplayRelay->setDefaultFunction(SUPLA_CHANNELFNC_POWERSWITCH);
    snprintf(caption, sizeof(caption), "%s-Disable Display", roomName);
    haierDisplayRelay->setInitialCaption(caption);

    haierStateSync.registerLevel2(
      haierHealthRelay,
      haierPowerSavingRelay,
      nullptr,
      haierDisplayRelay,
      haierQuietRelay);
  }

  // Level 3 (Debug): Add all remaining channels
  if (interfaceLvl >= 3) {
    SUPLA_LOG_DEBUG("Creating Level 3 (Debug) channels...");

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
      snprintf(caption, sizeof(caption), "%s-Disable Display", roomName);
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

      haierStateSync.registerLevel3(
        haierPowerState,
        haierSetTemperature,
        haierModeCool,
        haierModeHeat,
        haierModeDry,
        haierModeAuto,
        haierModeFan,
        haierFanHigh,
        haierFanMid,
        haierFanLow,
        haierFanAuto,
        haierTurbo,
        haierQuiet,
        haierDisplay,
        haierLockRemote,
        haierHealth,
        haierCompressor,
        haierTenDegree,
        haierUseSwingBits,
        haierHorizontalSwing,
        haierVerticalSwing);
  }
}

void loop() {
  haierController.loop();
  SuplaDevice.iterate();

  haierStateSync.iterate();

}
