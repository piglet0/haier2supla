#define AC_ROOM "Haier"
#define FW_VERSION "0.2.3"
#define FW_BUILD_INFO __DATE__ " " __TIME__

#include <Arduino.h>
#include <cstring>
#if defined(ESP32) || defined(SUPLA_DEVICE_ESP32)
#include <esp_mac.h>
#endif
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
#include <supla/storage/eeprom.h>
#include <supla/events.h>

#include <Haier2Supla.h>

Supla::Eeprom eeprom;
Supla::ESPWifi wifi;
Supla::EspWebServer suplaServer;
Supla::LittleFsConfig configSupla;
Supla::Device::StatusLed *statusLed = nullptr;
Supla::Control::Button *buttonCfg = nullptr;

#define CFG_TAG_ROOM_NAME "room_name"
#define CFG_TAG_RX_PIN "uart_rx_pin"
#define CFG_TAG_TX_PIN "uart_tx_pin"
#define CFG_TAG_INTERFACE_LEVEL "iface_level"
#define CFG_TAG_LED_PIN "status_led_pin"
#define CFG_TAG_BUTTON_PIN "cfg_button_pin"

HardwareSerial haierSerial(1);

constexpr uint8_t DEFAULT_STATUS_LED_PIN = 15;
constexpr uint8_t DEFAULT_BUTTON_CFG_PIN = 9;
constexpr uint8_t DEFAULT_HAIER_UART_RX_PIN = 14;
constexpr uint8_t DEFAULT_HAIER_UART_TX_PIN = 20;
constexpr int32_t DEFAULT_INTERFACE_LEVEL = 1;
constexpr uint32_t HAIER_UART_BAUD = 9600;

uint8_t statusLedPin = DEFAULT_STATUS_LED_PIN;
uint8_t buttonCfgPin = DEFAULT_BUTTON_CFG_PIN;
uint8_t haierUartRxPin = DEFAULT_HAIER_UART_RX_PIN;
uint8_t haierUartTxPin = DEFAULT_HAIER_UART_TX_PIN;

HaierSmartair2Controller haierController(
    haierSerial,
    DEFAULT_HAIER_UART_RX_PIN,
    DEFAULT_HAIER_UART_TX_PIN,
    HAIER_UART_BAUD,
    false);
HaierSuplaChannels haierSupla(&haierController);

// Captions below are suffixes. When roomName is set, the final caption becomes
// <roomName>-<caption>.
static const HaierSuplaChannelDefinition kLevel1Channels[] = {
    {HaierSuplaChannelId::Hvac, "HVAC"},
    {HaierSuplaChannelId::TemperatureHumidity, "Room Temp & Humidity"},
    {HaierSuplaChannelId::ModeCoolRelay, "Mode Cool"},
    {HaierSuplaChannelId::ModeHeatRelay, "Mode Heat"},
    {HaierSuplaChannelId::ModeAutoRelay, "Mode Auto"},
    {HaierSuplaChannelId::ModeDryRelay, "Mode Dry"},
    {HaierSuplaChannelId::ModeFanRelay, "Mode Fan"},
    {HaierSuplaChannelId::FanAutoRelay, "Fan Auto"},
    {HaierSuplaChannelId::FanHighRelay, "Fan High"},
    {HaierSuplaChannelId::FanMidRelay, "Fan Mid"},
    {HaierSuplaChannelId::FanLowRelay, "Fan Low"},
    {HaierSuplaChannelId::SwingHorizontalRelay, "Swing Horizontal"},
    {HaierSuplaChannelId::SwingVerticalRelay, "Swing Vertical"},
};

static const HaierSuplaChannelDefinition kLevel2Channels[] = {
    {HaierSuplaChannelId::HealthRelay, "Health Mode"},
    {HaierSuplaChannelId::PowerSavingRelay, "Power Saving Toggle"},
    {HaierSuplaChannelId::QuietRelay, "Quiet Mode"},
    {HaierSuplaChannelId::DisplayRelay, "Display Disable"},
};

static const HaierSuplaChannelDefinition kLevel3Channels[] = {
    {HaierSuplaChannelId::PowerStateSensor, "Power State"},
    {HaierSuplaChannelId::ModeCoolSensor, "State Cool"},
    {HaierSuplaChannelId::ModeHeatSensor, "State Heat"},
    {HaierSuplaChannelId::ModeDrySensor, "State Dry"},
    {HaierSuplaChannelId::ModeAutoSensor, "State Auto"},
    {HaierSuplaChannelId::ModeFanSensor, "State Fan"},
    {HaierSuplaChannelId::FanHighSensor, "State Fan High"},
    {HaierSuplaChannelId::FanMidSensor, "State Fan Mid"},
    {HaierSuplaChannelId::FanLowSensor, "State Fan Low"},
    {HaierSuplaChannelId::FanAutoSensor, "State Fan Auto"},
    {HaierSuplaChannelId::TurboSensor, "State Turbo"},
    {HaierSuplaChannelId::QuietSensor, "State Quiet"},
    {HaierSuplaChannelId::DisplaySensor, "State Display Disabled"},
    {HaierSuplaChannelId::LockRemoteSensor, "State Remote Lock"},
    {HaierSuplaChannelId::HealthSensor, "State Health"},
    {HaierSuplaChannelId::CompressorSensor, "State Compressor"},
    {HaierSuplaChannelId::TenDegreeSensor, "State 10C Mode"},
    {HaierSuplaChannelId::UseSwingBitsSensor, "State Swing Enabled"},
    {HaierSuplaChannelId::HorizontalSwingSensor, "State Horizontal Swing"},
    {HaierSuplaChannelId::VerticalSwingSensor, "State Vertical Swing"},
    {HaierSuplaChannelId::SetTemperatureSensor, "Set Temperature"},
    {HaierSuplaChannelId::TargetTemperatureStepRelay, "Adjust Set Temperature"},
};

static HaierSuplaChannelConfig buildHaierChannelConfig() {
  HaierSuplaChannelConfig config = makeHaierSuplaChannelConfig();

  // The helper supports additional levels as well, but this example exposes
  // only levels 1-3 through the web configuration selector below.
  setHaierSuplaChannelLevel(
    config, 1, kLevel1Channels,
    sizeof(kLevel1Channels) / sizeof(kLevel1Channels[0]));
  setHaierSuplaChannelLevel(
    config, 2, kLevel2Channels,
    sizeof(kLevel2Channels) / sizeof(kLevel2Channels[0]));
  setHaierSuplaChannelLevel(
    config, 3, kLevel3Channels,
    sizeof(kLevel3Channels) / sizeof(kLevel3Channels[0]));
  // Example for extra library-only levels:
  // setHaierSuplaChannelLevel(config, 4, kLevel4Channels,
  //                          sizeof(kLevel4Channels) / sizeof(kLevel4Channels[0]));

  return config;
}

static const HaierSuplaChannelConfig kHaierChannelConfig =
  buildHaierChannelConfig();

static void buildLegacyEfuseMacSuffix(char *suffix, size_t suffixSize) {
  const unsigned long long efuseMac =
      static_cast<unsigned long long>(ESP.getEfuseMac());
  snprintf(suffix, suffixSize, "%012llX", efuseMac & 0xFFFFFFFFFFFFULL);
}

static void buildMacSuffix(char *suffix, size_t suffixSize) {
#if defined(ESP32) || defined(SUPLA_DEVICE_ESP32)
  uint8_t baseMac[6] = {};
  if (esp_read_mac(baseMac, ESP_MAC_BASE) == ESP_OK) {
    snprintf(suffix, suffixSize, "%02X%02X%02X%02X%02X%02X", baseMac[0],
             baseMac[1], baseMac[2], baseMac[3], baseMac[4], baseMac[5]);
    return;
  }
#endif
  const unsigned long long efuseMac =
      static_cast<unsigned long long>(ESP.getEfuseMac());
  snprintf(suffix, suffixSize, "%012llX", efuseMac & 0xFFFFFFFFFFFFULL);
}

static void buildDefaultDeviceName(char *deviceName, size_t deviceNameSize) {
  char macSuffix[13] = {};
  buildMacSuffix(macSuffix, sizeof(macSuffix));
  snprintf(deviceName, deviceNameSize, "Supla2Haier-%s", macSuffix);
}

static void buildLegacyEfuseDeviceName(char *deviceName,
                                       size_t deviceNameSize) {
  char macSuffix[13] = {};
  buildLegacyEfuseMacSuffix(macSuffix, sizeof(macSuffix));
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
  if (strncmp(deviceName, "Haier2Supla_", 12) == 0) {
    return true;
  }

  char legacyDeviceName[SUPLA_DEVICE_NAME_MAXSIZE] = {};
  buildLegacyEfuseDeviceName(legacyDeviceName, sizeof(legacyDeviceName));
  return strcmp(deviceName, legacyDeviceName) == 0;
}

void setup() {
  Serial.begin(115200);
  delay(2000);

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
  if (cfg) {
    cfg->getInt32(CFG_TAG_RX_PIN, &rxPin);
    cfg->getInt32(CFG_TAG_TX_PIN, &txPin);
    cfg->getInt32(CFG_TAG_LED_PIN, &ledPin);
    cfg->getInt32(CFG_TAG_BUTTON_PIN, &cfgPin);
    if (!cfg->getInt32(CFG_TAG_INTERFACE_LEVEL, &interfaceLvl)) {
      cfg->setInt32(CFG_TAG_INTERFACE_LEVEL, interfaceLvl);
      cfg->commit();
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

  SuplaDevice.setName(deviceName);
  SuplaDevice.setCustomHostnamePrefix("SUPLA2HAIER");
  char swVersion[18] = {};
  buildCompactSwVersion(swVersion, sizeof(swVersion));
  SuplaDevice.setSwVersion(swVersion);

  statusLed = new Supla::Device::StatusLed(statusLedPin, false);
  buttonCfg = new Supla::Control::Button(buttonCfgPin, true, true);
  buttonCfg->configureAsConfigButton(&SuplaDevice);

  new Supla::Html::DeviceInfo(&SuplaDevice);
  new Supla::Html::WifiParameters;
  new Supla::Html::ProtocolParameters;
  new Supla::Html::StatusLedParameters;
  new Supla::Html::ButtonUpdate(&suplaServer);
  new Supla::Html::CustomTextParameter(CFG_TAG_ROOM_NAME, "Room Name", 10);
  new Supla::Html::CustomParameter(CFG_TAG_RX_PIN, "UART RX Pin",
                                   DEFAULT_HAIER_UART_RX_PIN);
  new Supla::Html::CustomParameter(CFG_TAG_TX_PIN, "UART TX Pin",
                                   DEFAULT_HAIER_UART_TX_PIN);
  new Supla::Html::CustomParameter(CFG_TAG_LED_PIN, "Status LED Pin",
                                   DEFAULT_STATUS_LED_PIN);
  new Supla::Html::CustomParameter(CFG_TAG_BUTTON_PIN, "Config Button Pin",
                                   DEFAULT_BUTTON_CFG_PIN);

  auto interfaceLevel = new Supla::Html::SelectInputParameter(
      CFG_TAG_INTERFACE_LEVEL,
      "Interface Level (changing this requires removing and re-adding the device in Supla Cloud)");
  interfaceLevel->registerValue("Minimal", 1);
  interfaceLevel->registerValue("Standard", 2);
  interfaceLevel->registerValue("Debug", 3);

  SuplaDevice.setInitialMode(Supla::InitialMode::StartInCfgMode);
  SuplaDevice.begin();

  haierSerial.begin(HAIER_UART_BAUD, SERIAL_8N1, haierUartRxPin, haierUartTxPin);
  haierController.begin();
  haierSupla.begin(roomName, interfaceLvl, kHaierChannelConfig);
}

void loop() {
  haierController.loop();
  SuplaDevice.iterate();
  haierSupla.iterate();
}