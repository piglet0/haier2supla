\
#include "HaierSmartair2Controller.h"
#include <cstring>

#include <cmath>
#include <WiFi.h>
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
  protocol_.set_answer_handler(
      FrameType::REPORT_NETWORK_STATUS,
      std::bind(&HaierSmartair2Controller::handleNetworkStatusAnswer_, this,
                std::placeholders::_1,
                std::placeholders::_2,
                std::placeholders::_3,
                std::placeholders::_4));

  // Fetch first status immediately to avoid stale default mode/fan states.
  sendStatusRequest_();
  last_status_request_ = std::chrono::steady_clock::now();
  last_network_status_report_ = std::chrono::steady_clock::now();
}

void HaierSmartair2Controller::loop() {
  protocol_.loop();
  processPowerSavingModeToggleSequence_();
  processDisplayTemperatureToggleSequence_();

  auto now = std::chrono::steady_clock::now();
  const bool toggle_sequence_active = isAnyToggleSequenceActive_();

  const auto network_status_elapsed =
      std::chrono::duration_cast<std::chrono::milliseconds>(
          now - last_network_status_report_);
  if (!toggle_sequence_active &&
      network_status_elapsed >= network_status_report_interval_ &&
      !protocol_.is_waiting_for_answer() &&
      protocol_.get_outgoing_queue_size() == 0) {
    sendNetworkStatusReport_();
    last_network_status_report_ = now;
    return;
  }

  if (pending_settings_.valid && !control_send_requested_ &&
      !protocol_.is_waiting_for_answer() &&
      protocol_.get_outgoing_queue_size() == 0) {
    if (has_last_status_message_) {
      sendControlNow();
      control_send_requested_ = true;
    } else {
      sendStatusRequest_();
      last_status_request_ = std::chrono::steady_clock::now();
    }
  }

  auto elapsed =
      std::chrono::duration_cast<std::chrono::milliseconds>(now - last_status_request_);

  // Request AC status every 1 seconds
  if (!toggle_sequence_active &&
      elapsed.count() > 1000 &&
      !protocol_.is_waiting_for_answer() &&
      protocol_.get_outgoing_queue_size() == 0) {
    sendStatusRequest_();
    last_status_request_ = now;
  }
}

void HaierSmartair2Controller::sendNetworkStatusReport_() {
  const uint8_t payload[4] = {0x00, 0x00, 0x00, getWifiSignalStrength_()};
  const HaierMessage network_status(FrameType::REPORT_NETWORK_STATUS, payload,
                                    sizeof(payload));
  protocol_.send_message(network_status, use_crc_);
}

void HaierSmartair2Controller::sendStatusRequest_() {
  static const HaierMessage STATUS_REQUEST(FrameType::CONTROL, 0x4D01);
  protocol_.send_message(STATUS_REQUEST, use_crc_);
}

HandlerError HaierSmartair2Controller::handleNetworkStatusAnswer_(
    FrameType request_type,
    FrameType message_type,
    const uint8_t *data,
    size_t data_size) {
  if (request_type != FrameType::REPORT_NETWORK_STATUS) {
    return HandlerError::UNEXPECTED_MESSAGE;
  }
  if (message_type != FrameType::CONFIRM || data_size != 0) {
    return HandlerError::INVALID_ANSWER;
  }
  return HandlerError::HANDLER_OK;
}

void HaierSmartair2Controller::setPower(bool on) {
  // Always send power command even if local cache already matches.
  // Cache can be stale when packets were missed.
  power_state_ = on;
  const uint16_t subcommand = on ? 0x4D02 : 0x4D03;
  HaierMessage msg(FrameType::CONTROL, subcommand);
  protocol_.send_message(msg, use_crc_);
}

float HaierSmartair2Controller::getRoomTemperatureC() const {
  return room_temperature_c_;
}

uint8_t HaierSmartair2Controller::getWifiSignalStrength_() const {
  const int32_t rssi = WiFi.RSSI();
  if (rssi >= 0) {
    return 0;
  }

  const int32_t magnitude = -rssi;
  if (magnitude > 0xFF) {
    return 0xFF;
  }
  return static_cast<uint8_t>(magnitude);
}

void HaierSmartair2Controller::setTargetTemperatureChangedCallback(
    std::function<void(float)> callback) {
  target_temperature_changed_callback_ = std::move(callback);
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
  // Some units report room temperature with 0.1C scale in this field.
  // Normalize obviously out-of-range values to human-readable Celsius.
  if (room_temperature_c_ > 80.0f) {
    room_temperature_c_ /= 10.0f;
  }
  power_state_ = (status.control.ac_power != 0);
  compressor_ = (status.control.compressor != 0);
  room_humidity_ = status.control.room_humidity;
  // Only update reported control fields if there is no pending user change.
  if (!pending_settings_.valid) {
    const float previous_set_point = set_point_;
    ac_mode_ = static_cast<ConditioningMode>(status.control.ac_mode);
    set_point_ = decodeSetPointC(status.control.set_point);
    fan_mode_ = static_cast<FanMode>(status.control.fan_mode);
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
    if ((std::isnan(previous_set_point) && std::isfinite(set_point_)) ||
        (std::isfinite(previous_set_point) && std::isfinite(set_point_) &&
         std::fabs(previous_set_point - set_point_) > 0.25f)) {
      notifyTargetTemperatureChanged_();
    }
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
    if (((status.control.horizontal_swing != 0) != horizontal_swing_)) applied = false;
    if (((status.control.vertical_swing != 0) != vertical_swing_)) applied = false;

    // Compare set-point (normalize packet offset + half-degree)
    float reported_set = decodeSetPointC(status.control.set_point);
    if (!std::isnan(set_point_)) {
      if (std::fabs(reported_set - set_point_) > 0.25f) applied = false;
    }

    if (applied) {
      pending_settings_.valid = false;
      control_send_requested_ = false;
    } else {
      control_send_requested_ = false;
    }
  }

  return HandlerError::HANDLER_OK;
}

void HaierSmartair2Controller::queueControlUpdate_() {
  pending_settings_.valid = true;
  control_send_requested_ = false;
}

void HaierSmartair2Controller::notifyTargetTemperatureChanged_() {
  if (target_temperature_changed_callback_ && std::isfinite(set_point_)) {
    target_temperature_changed_callback_(set_point_);
  }
}

void HaierSmartair2Controller::triggerPowerSavingModeToggleSequence() {
  power_saving_mode_toggle_sequence_.active = true;
  power_saving_mode_toggle_sequence_.auto_mode_sent = false;
  power_saving_mode_toggle_sequence_.next_health_mode = !health_mode_;
  power_saving_mode_toggle_sequence_.toggles_remaining = 6;
  power_saving_mode_toggle_sequence_.next_step_at = std::chrono::steady_clock::now();
}

bool HaierSmartair2Controller::isPowerSavingModeToggleSequenceActive() const {
  return power_saving_mode_toggle_sequence_.active;
}

void HaierSmartair2Controller::triggerDisplayTemperatureToggleSequence() {
  display_temperature_toggle_sequence_.active = true;
  display_temperature_toggle_sequence_.next_display_status = !display_status_;
  display_temperature_toggle_sequence_.toggles_remaining = 10;
  display_temperature_toggle_sequence_.next_step_at = std::chrono::steady_clock::now();
}

bool HaierSmartair2Controller::isDisplayTemperatureToggleSequenceActive() const {
  return display_temperature_toggle_sequence_.active;
}

bool HaierSmartair2Controller::isAnyToggleSequenceActive_() const {
  return power_saving_mode_toggle_sequence_.active ||
         display_temperature_toggle_sequence_.active;
}

void HaierSmartair2Controller::sendDisplayToggleStep_(bool on) {
  if (!has_last_status_message_) {
    return;
  }

  display_status_ = on;
  sendControlNow();
}

void HaierSmartair2Controller::processPowerSavingModeToggleSequence_() {
  auto &sequence = power_saving_mode_toggle_sequence_;
  if (!sequence.active) {
    return;
  }

  const auto now = std::chrono::steady_clock::now();
  if (now < sequence.next_step_at) {
    return;
  }

  if (pending_settings_.valid || control_send_requested_ ||
      protocol_.is_waiting_for_answer() ||
      protocol_.get_outgoing_queue_size() != 0) {
    return;
  }

  if (!has_last_status_message_) {
    sendStatusRequest_();
    last_status_request_ = now;
    sequence.next_step_at = now + power_saving_status_retry_interval_;
    return;
  }

  if (!sequence.auto_mode_sent) {
    setACMode(ConditioningMode::AUTO);
    sequence.auto_mode_sent = true;
    sequence.next_step_at = now + power_saving_toggle_interval_;
    return;
  }

  if (sequence.toggles_remaining > 0) {
    setHealthMode(sequence.next_health_mode);
    sequence.next_health_mode = !sequence.next_health_mode;
    --sequence.toggles_remaining;
    sequence.next_step_at =
        (sequence.toggles_remaining == 0) ? now : now + power_saving_toggle_interval_;
    return;
  }

  sequence.active = false;
}

void HaierSmartair2Controller::processDisplayTemperatureToggleSequence_() {
  auto &sequence = display_temperature_toggle_sequence_;
  if (!sequence.active) {
    return;
  }

  const auto now = std::chrono::steady_clock::now();
  if (now < sequence.next_step_at) {
    return;
  }

  if (pending_settings_.valid || control_send_requested_ ||
      protocol_.is_waiting_for_answer() ||
      protocol_.get_outgoing_queue_size() != 0) {
    return;
  }

  if (!has_last_status_message_) {
    sendStatusRequest_();
    last_status_request_ = now;
    sequence.next_step_at = now + display_temperature_status_retry_interval_;
    return;
  }

  if (sequence.toggles_remaining > 0) {
    sendDisplayToggleStep_(sequence.next_display_status);
    sequence.next_display_status = !sequence.next_display_status;
    --sequence.toggles_remaining;
    sequence.next_step_at =
        (sequence.toggles_remaining == 0) ? now
                                          : now + display_temperature_toggle_interval_;
    return;
  }

  sequence.active = false;
}

// Setters — update cached fields directly (no pending_settings_)
void HaierSmartair2Controller::setTargetTemperatureC(float temp) {
  if (temp < kSetpointMinC) {
    temp = kSetpointMinC;
  } else if (temp > kSetpointMaxC) {
    temp = kSetpointMaxC;
  }
  // SmartAir2 setpoint has 1C resolution, keep internal cache aligned.
  temp = roundf(temp);
  const bool changed = std::isnan(set_point_) || std::fabs(set_point_ - temp) > 0.25f;
  set_point_ = temp;
  if (changed) {
    notifyTargetTemperatureChanged_();
  }
  queueControlUpdate_();
}

void HaierSmartair2Controller::increaseTargetTemperatureC(bool increase) {
  float currentTemp = getTargetTemperatureC();
  float newTemp = increase ? currentTemp + 1.0f : currentTemp - 1.0f;
  setTargetTemperatureC(newTemp);
}

void HaierSmartair2Controller::setACMode(Supla::haier::smartair2_protocol::ConditioningMode mode) {
  ac_mode_ = mode;
  if (mode == Supla::haier::smartair2_protocol::ConditioningMode::AUTO) {
    fan_mode_ = Supla::haier::smartair2_protocol::FanMode::FAN_AUTO;
  } else if (mode == Supla::haier::smartair2_protocol::ConditioningMode::FAN &&
             fan_mode_ == Supla::haier::smartair2_protocol::FanMode::FAN_AUTO) {
    fan_mode_ = Supla::haier::smartair2_protocol::FanMode::FAN_LOW;
  }
  queueControlUpdate_();
}
void HaierSmartair2Controller::setFanMode(Supla::haier::smartair2_protocol::FanMode mode) {
  if (ac_mode_ == Supla::haier::smartair2_protocol::ConditioningMode::FAN &&
      mode == Supla::haier::smartair2_protocol::FanMode::FAN_AUTO) {
    return;
  }
  fan_mode_ = mode;
  queueControlUpdate_();
}
void HaierSmartair2Controller::setUseSwingBits(bool use_bits) {
  use_swing_bits_ = use_bits;
  queueControlUpdate_();
}
void HaierSmartair2Controller::setHorizontalSwing(bool h) {
  horizontal_swing_ = h;
  queueControlUpdate_();
}
void HaierSmartair2Controller::setVerticalSwing(bool v) {
  vertical_swing_ = v;
  queueControlUpdate_();
}
void HaierSmartair2Controller::setTurbo(bool on) {
  turbo_mode_ = on;
  queueControlUpdate_();
}
void HaierSmartair2Controller::setQuiet(bool on) {
  quiet_mode_ = on;
  queueControlUpdate_();
}
void HaierSmartair2Controller::setDisplayStatus(bool on) {
  display_status_ = on;
  queueControlUpdate_();
}
void HaierSmartair2Controller::setUseFahrenheit(bool on) {
  use_fahrenheit_ = on;
  queueControlUpdate_();
}
void HaierSmartair2Controller::setLockRemote(bool on) {
  lock_remote_ = on;
  queueControlUpdate_();
}
void HaierSmartair2Controller::setHealthMode(bool on) {
  health_mode_ = on;
  queueControlUpdate_();
}
void HaierSmartair2Controller::setTenDegree(bool on) {
  ten_degree_ = on;
  queueControlUpdate_();
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
  out_data->use_swing_bits = use_swing_bits_ ? 1 : 0;
  out_data->horizontal_swing = horizontal_swing_ ? 1 : 0;
  out_data->vertical_swing = vertical_swing_ ? 1 : 0;

  // Temperature set point encoded with SmartAir2 offset.
  if (!std::isnan(set_point_)) {
    out_data->set_point = encodeSetPointC(set_point_);
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
    return;
  }

  haier_protocol::HaierMessage msg = getControlMessage();
  if (pending_settings_.valid) {
    protocol_.send_message(msg, use_crc_, /*num_retries=*/2, control_retry_interval_);
  } else {
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
  // set_point_ is stored in Celsius (decoded from packet format).
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
  SUPLA_LOG_DEBUG("HAIER  PKT: %s", full.c_str());

  // Build diff line: '.' for same, hex for changed
  std::string diff;
  diff.reserve(len * 3 + 32);
  for (size_t i = 0; i < len; ++i) {
    bool same = (i < last_raw_packet_.size() && last_raw_packet_[i] == pkt[i]);
    if (same) {
      diff.append(".. ");
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
