\
#include "HaierSmartair2Controller.h"
#include <cstring>

#include <cmath>
#include <supla/log_wrapper.h> // required for logging in the controller
#include <cstdio>
#include <sstream>

using namespace haier_protocol;
using namespace Supla::haier::smartair2_protocol;

HaierSmartair2Controller::HaierSmartair2Controller(HardwareSerial &serial,
                                                   uint8_t rx_pin,
                                                   uint8_t tx_pin,
                                                   uint32_t baud,
                                                   bool use_crc)
    : serial_(serial),
      rx_pin_(rx_pin),
      tx_pin_(tx_pin),
      baud_(baud),
      use_crc_(use_crc),
      stream_(serial_),
      protocol_(stream_) {}

void HaierSmartair2Controller::begin() {
  // UART is configured by the application (sketch) before controller begin.
  // Avoid reconfiguring here to preserve pins set from web config.
  SUPLA_LOG_DEBUG("Haier controller begin: baud=%u rx=%d tx=%d", (unsigned)baud_, rx_pin_, tx_pin_);

  // Answer handler for CONTROL -> STATUS messages
  protocol_.set_answer_handler(
      FrameType::CONTROL,
      std::bind(&HaierSmartair2Controller::handleStatusAnswer_, this,
                std::placeholders::_1,
                std::placeholders::_2,
                std::placeholders::_3,
                std::placeholders::_4));

    // Timeout handler for CONTROL messages: allow controller-level retries
    protocol_.set_timeout_handler(
      FrameType::CONTROL,
      std::bind(&HaierSmartair2Controller::controlTimeoutHandler_, this, std::placeholders::_1));

    last_status_request_ = std::chrono::steady_clock::now();
}

HandlerError HaierSmartair2Controller::controlTimeoutHandler_(FrameType request_type) {
  // Called when protocol-level retries for CONTROL are exhausted for one send.
  if (!pending_settings_.valid) {
    return HandlerError::HANDLER_OK;
  }

  if (control_attempts_ < max_control_attempts_) {
    // Try sending control again (controller-level retry)
    sendControlNow();
  } else {
    // Exhausted attempts — give up and clear pending
    pending_settings_.valid = false;
    control_attempts_ = 0;
  }
  return HandlerError::HANDLER_OK;
}

void HaierSmartair2Controller::loop() {
  protocol_.loop();

  auto now = std::chrono::steady_clock::now();
  auto elapsed =
      std::chrono::duration_cast<std::chrono::milliseconds>(now - last_status_request_);

  // Request AC status every 1 seconds
  if (elapsed.count() > 1000) {
    sendStatusRequest_();
    last_status_request_ = now;
  }
}

void HaierSmartair2Controller::sendStatusRequest_() {
  static const HaierMessage STATUS_REQUEST(FrameType::CONTROL, 0x4D01);
  protocol_.send_message(STATUS_REQUEST, use_crc_);
}

void HaierSmartair2Controller::setPower(bool on) {
  if (on == power_state_) {
    return;
  }
  power_state_ = on;
  const uint16_t subcommand = on ? 0x4D02 : 0x4D03;
  HaierMessage msg(FrameType::CONTROL, subcommand);
  protocol_.send_message(msg, use_crc_);
}

float HaierSmartair2Controller::getRoomTemperatureC() const {
  return room_temperature_c_;
}

HandlerError HaierSmartair2Controller::handleStatusAnswer_(
    FrameType request_type,
    FrameType message_type,
    const uint8_t *data,
    size_t data_size) {
  if (request_type != FrameType::CONTROL ||
      message_type != FrameType::STATUS ||
      data == nullptr ||
      data_size < sizeof(HaierStatus)) {
    return HandlerError::WRONG_MESSAGE_STRUCTURE;
  }

  HaierStatus status;
  memcpy(&status, data, sizeof(HaierStatus));

  // Update cached values from AC. Always update live measurements (room temp,
  // power) but avoid overriding user-requested pending settings.
  room_temperature_c_ = static_cast<float>(status.control.room_temperature);
  power_state_ = (status.control.ac_power != 0);
  compressor_ = (status.control.compressor != 0);
  room_humidity_ = status.control.room_humidity;
  // Only update reported control fields if there is no pending user change.
  if (!pending_settings_.valid) {
    ac_mode_ = static_cast<ConditioningMode>(status.control.ac_mode);
    set_point_ = static_cast<float>(status.control.set_point) + 16.0f; // 16°C offset
    fan_mode_ = static_cast<FanMode>(status.control.fan_mode);
    swing_both_ = (status.control.swing_both != 0);
    use_swing_bits_ = (status.control.use_swing_bits != 0);
    horizontal_swing_ = (status.control.horizontal_swing != 0);
    vertical_swing_ = (status.control.vertical_swing != 0);
    turbo_mode_ = (status.control.turbo_mode != 0);
    quiet_mode_ = (status.control.quiet_mode != 0);
    display_status_ = (status.control.display_status != 0);
    use_fahrenheit_ = (status.control.use_fahrenheit != 0);
    lock_remote_ = (status.control.lock_remote != 0);
    health_mode_ = (status.control.health_mode != 0);
    ten_degree_ = (status.control.ten_degree != 0);
  }
  // Save last status control block for building control messages
  static_assert(sizeof(last_status_message_) == sizeof(status.control), "size mismatch");
  memcpy(last_status_message_.data(), &status.control, sizeof(status.control));
  has_last_status_message_ = true;

  // Log raw packet and diffs compared to previous packet
  logPacketDiff(data, data_size);

  // If we have a pending user change, verify whether the reported status
  // matches the desired values; if so, clear pending and reset attempts.
  if (pending_settings_.valid) {
    bool applied = true;
    if (status.control.ac_mode != static_cast<uint8_t>(ac_mode_)) applied = false;
    if (status.control.fan_mode != static_cast<uint8_t>(fan_mode_)) applied = false;
    if (((status.control.turbo_mode != 0) != turbo_mode_)) applied = false;
    if (((status.control.quiet_mode != 0) != quiet_mode_)) applied = false;
    if (((status.control.display_status != 0) != display_status_)) applied = false;
    if (((status.control.health_mode != 0) != health_mode_)) applied = false;
    if (((status.control.ten_degree != 0) != ten_degree_)) applied = false;
    if (((status.control.use_swing_bits != 0) != use_swing_bits_)) applied = false;
    if (((status.control.swing_both != 0) != swing_both_)) applied = false;
    if (((status.control.horizontal_swing != 0) != horizontal_swing_)) applied = false;
    if (((status.control.vertical_swing != 0) != vertical_swing_)) applied = false;

    // Compare set-point (normalize packet offset + half-degree)
    float reported_set = static_cast<float>(status.control.set_point) + 16.0f;
    if (!std::isnan(set_point_)) {
      if (std::fabs(reported_set - set_point_) > 0.25f) applied = false;
    }

    if (applied) {
      pending_settings_.valid = false;
      control_attempts_ = 0;
    }
  }

  return HandlerError::HANDLER_OK;
}

// Setters — update cached fields directly (no pending_settings_)
void HaierSmartair2Controller::setTargetTemperatureC(float temp) {
  if (temp < 16.0f) {
    temp = 16.0f;
  } else if (temp > 30.0f) {
    temp = 30.0f;
  }
  // store in real degrees (e.g. 21.5). getControlMessage will convert to packet offset.
  // Request latest status and construct baseline control packet first
  sendStatusRequest_();
  getControlMessage();
  set_point_ = temp;
  // mark pending and send control for confirmation
  pending_settings_.valid = true;
  control_attempts_ = 0;
  sendControlNow();
}

void HaierSmartair2Controller::increaseTargetTemperatureC(bool increase) {
  float currentTemp = getTargetTemperatureC();
  float newTemp = increase ? currentTemp + 1.0f : currentTemp - 1.0f;
  setTargetTemperatureC(newTemp);
}

void HaierSmartair2Controller::setACMode(Supla::haier::smartair2_protocol::ConditioningMode mode) {
  sendStatusRequest_();
  getControlMessage();
  ac_mode_ = mode;
  pending_settings_.valid = true;
  control_attempts_ = 0;
  sendControlNow();
}
void HaierSmartair2Controller::setFanMode(Supla::haier::smartair2_protocol::FanMode mode) {
  sendStatusRequest_();
  getControlMessage();
  fan_mode_ = mode;
  pending_settings_.valid = true;
  control_attempts_ = 0;
  sendControlNow();
}
void HaierSmartair2Controller::setSwingBoth(bool both) {
  sendStatusRequest_();
  getControlMessage();
  swing_both_ = both;
  pending_settings_.valid = true;
  control_attempts_ = 0;
  sendControlNow();
}
void HaierSmartair2Controller::setUseSwingBits(bool use_bits) {
  sendStatusRequest_();
  getControlMessage();
  use_swing_bits_ = use_bits;
  pending_settings_.valid = true;
  control_attempts_ = 0;
  sendControlNow();
}
void HaierSmartair2Controller::setHorizontalSwing(bool h) {
  sendStatusRequest_();
  getControlMessage();
  horizontal_swing_ = h;
  pending_settings_.valid = true;
  control_attempts_ = 0;
  sendControlNow();
}
void HaierSmartair2Controller::setVerticalSwing(bool v) {
  sendStatusRequest_();
  getControlMessage();
  vertical_swing_ = v;
  pending_settings_.valid = true;
  control_attempts_ = 0;
  sendControlNow();
}
void HaierSmartair2Controller::setTurbo(bool on) {
  sendStatusRequest_();
  getControlMessage();
  turbo_mode_ = on;
  pending_settings_.valid = true;
  control_attempts_ = 0;
  sendControlNow();
}
void HaierSmartair2Controller::setQuiet(bool on) {
  sendStatusRequest_();
  getControlMessage();
  quiet_mode_ = on;
  pending_settings_.valid = true;
  control_attempts_ = 0;
  sendControlNow();
}
void HaierSmartair2Controller::setDisplayStatus(bool on) {
  sendStatusRequest_();
  getControlMessage();
  display_status_ = on;
  pending_settings_.valid = true;
  control_attempts_ = 0;
  sendControlNow();
}
void HaierSmartair2Controller::setUseFahrenheit(bool on) {
  sendStatusRequest_();
  getControlMessage();
  use_fahrenheit_ = on;
  pending_settings_.valid = true;
  control_attempts_ = 0;
  sendControlNow();
}
void HaierSmartair2Controller::setLockRemote(bool on) {
  sendStatusRequest_();
  getControlMessage();
  lock_remote_ = on;
  pending_settings_.valid = true;
  control_attempts_ = 0;
  sendControlNow();
}
void HaierSmartair2Controller::setHealthMode(bool on) {
  sendStatusRequest_();
  getControlMessage();
  health_mode_ = on;
  pending_settings_.valid = true;
  control_attempts_ = 0;
  sendControlNow();
}
void HaierSmartair2Controller::setTenDegree(bool on) {
  sendStatusRequest_();
  getControlMessage();
  ten_degree_ = on;
  pending_settings_.valid = true;
  control_attempts_ = 0;
  sendControlNow();
}

haier_protocol::HaierMessage HaierSmartair2Controller::getControlMessage() {
  // If we don't have a baseline status message, request it and return an empty control
  if (!has_last_status_message_) {
    sendStatusRequest_();
    return HaierMessage(FrameType::CONTROL, 0x4D5F);
  }

  uint8_t control_out_buffer[sizeof(Supla::haier::smartair2_protocol::HaierPacketControl)];
  memcpy(control_out_buffer, last_status_message_.data(), sizeof(control_out_buffer));
  auto *out_data = reinterpret_cast<Supla::haier::smartair2_protocol::HaierPacketControl *>(control_out_buffer);

  // Mark packet as from ESP -> AC
  out_data->cntrl = 0x00;

  // Power
  out_data->ac_power = power_state_ ? 1 : 0;

  // Mode and fan
  out_data->ac_mode = static_cast<uint8_t>(ac_mode_);
  out_data->fan_mode = static_cast<uint8_t>(fan_mode_);

  // Swing and swing bits
  out_data->swing_both = swing_both_ ? 1 : 0;
  out_data->use_swing_bits = use_swing_bits_ ? 1 : 0;
  out_data->horizontal_swing = horizontal_swing_ ? 1 : 0;
  out_data->vertical_swing = vertical_swing_ ? 1 : 0;

  // Temperature set point (packet uses 16°C offset)
  if (!std::isnan(set_point_)) {
    int sp = static_cast<int>(set_point_ + 0.5f);
    out_data->set_point = static_cast<uint8_t>(sp - 16);
  }

  // Presets / modes
  out_data->turbo_mode = turbo_mode_ ? 1 : 0;
  out_data->quiet_mode = quiet_mode_ ? 1 : 0;
  out_data->ten_degree = ten_degree_ ? 1 : 0;

  // Display / misc
  out_data->display_status = display_status_ ? 1 : 0;
  out_data->use_fahrenheit = use_fahrenheit_ ? 1 : 0;
  out_data->lock_remote = lock_remote_ ? 1 : 0;
  out_data->health_mode = health_mode_ ? 1 : 0;

  return HaierMessage(FrameType::CONTROL, 0x4D5F, control_out_buffer, sizeof(control_out_buffer));
}

void HaierSmartair2Controller::sendControlNow() {
  // Build CONTROL message from last status + pending settings and send it.
  if (!has_last_status_message_) {
    // Request status and return; caller should retry after status received.
    sendStatusRequest_();
    return;
  }

  haier_protocol::HaierMessage msg = getControlMessage();
  // If there is a pending user change, keep it pending and track attempts.
  if (pending_settings_.valid) {
    control_attempts_++;
    protocol_.send_message(msg, use_crc_, /*num_retries=*/2, control_retry_interval_);
  } else {
    // No confirmation required — send normally.
    protocol_.send_message(msg, use_crc_);
  }
}

bool HaierSmartair2Controller::getModeCool() const {
  return ac_mode_ == ConditioningMode::COOL;
}

bool HaierSmartair2Controller::getModeHeat() const {
  return ac_mode_ == ConditioningMode::HEAT;
}
bool HaierSmartair2Controller::getModeAuto() const {
  return ac_mode_ == ConditioningMode::AUTO;
}
bool HaierSmartair2Controller::getModeFan() const {
  return ac_mode_ == ConditioningMode::FAN;
}
bool HaierSmartair2Controller::getModeDry() const {
  return ac_mode_ == ConditioningMode::DRY;
}
bool HaierSmartair2Controller::getFanHigh() const {
  return fan_mode_ == FanMode::FAN_HIGH;
}
bool HaierSmartair2Controller::getFanMid() const {
  return fan_mode_ == FanMode::FAN_MID;
}
bool HaierSmartair2Controller::getFanLow() const {
  return fan_mode_ == FanMode::FAN_LOW;
}
bool HaierSmartair2Controller::getFanAuto() const {
  return fan_mode_ == FanMode::FAN_AUTO;
}

float HaierSmartair2Controller::getTargetTemperatureC() const {
  // `set_point_` uses a 16°C offset in the packet format (see smartair2_packet.h)
  return set_point_;
}

Supla::haier::smartair2_protocol::FanMode HaierSmartair2Controller::getFanMode() const {
  return fan_mode_;
}

// Log a received packet (hex) and a second line showing only changed bytes
void HaierSmartair2Controller::logPacketDiff(const uint8_t *pkt, size_t len) {
  if (pkt == nullptr || len == 0) return;

  bool different = false;
  if (last_raw_packet_.size() != len) {
    different = true;
  } else {
    for (size_t i = 0; i < len; ++i) {
      if (last_raw_packet_[i] != pkt[i]) { different = true; break; }
    }
  }
  if (!different) return; // nothing changed

  // Build full hex string
  std::string full;
  full.reserve(len * 3 + 32);
  char tmp[8];
  for (size_t i = 0; i < len; ++i) {
    snprintf(tmp, sizeof(tmp), "%02X ", pkt[i]);
    full.append(tmp);
  }
  SUPLA_LOG_DEBUG("HAIER PKT: %s", full.c_str());

  // Build diff line: '.' for same, hex for changed
  std::string diff;
  diff.reserve(len * 3 + 32);
  for (size_t i = 0; i < len; ++i) {
    bool same = (i < last_raw_packet_.size() && last_raw_packet_[i] == pkt[i]);
    if (same) {
      diff.append(".  ");
    } else {
      snprintf(tmp, sizeof(tmp), "%02X ", pkt[i]);
      diff.append(tmp);
    }
  }
  SUPLA_LOG_DEBUG("HAIER DIFF: %s", diff.c_str());

  // save
  last_raw_packet_.assign(pkt, pkt + len);
}

Supla::haier::smartair2_protocol::ConditioningMode HaierSmartair2Controller::getACMode() const {
  return ac_mode_;
} 

uint8_t HaierSmartair2Controller::getRoomHumidity() const { return room_humidity_; }

bool HaierSmartair2Controller::getSwingBoth() const { return swing_both_; }

bool HaierSmartair2Controller::getUseSwingBits() const { return use_swing_bits_; }

bool HaierSmartair2Controller::getHorizontalSwing() const { return horizontal_swing_; }

bool HaierSmartair2Controller::getVerticalSwing() const { return vertical_swing_; }

bool HaierSmartair2Controller::getTurbo() const { return turbo_mode_; }

bool HaierSmartair2Controller::getQuiet() const { return quiet_mode_; }

bool HaierSmartair2Controller::getDisplayStatus() const { return display_status_; }

bool HaierSmartair2Controller::getUseFahrenheit() const { return use_fahrenheit_; }

bool HaierSmartair2Controller::getLockRemote() const { return lock_remote_; }

bool HaierSmartair2Controller::getHealthMode() const { return health_mode_; }

bool HaierSmartair2Controller::getCompressor() const { return compressor_; }

bool HaierSmartair2Controller::getTenDegree() const { return ten_degree_; }
