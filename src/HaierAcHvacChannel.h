#ifndef HAIER_AC_HVAC_CHANNEL_H
#define HAIER_AC_HVAC_CHANNEL_H

#include <supla/control/hvac_base.h>
#include <supla/control/output_interface.h>
#include <supla/log_wrapper.h>
#include <cmath>
#include <cstdint>
#include <climits>
#include "HaierSmartair2Controller.h"

namespace Supla {
namespace Control {

class HaierHvacOutput : public OutputInterface {
 public:
  explicit HaierHvacOutput(HaierSmartair2Controller *controller)
      : controller_(controller) {}

  int getOutputValue() const override {
    return outputValue_;
  }

  void setOutputValue(int value) override {
    // Keep HVAC output binary to avoid percentage artifacts like 1%.
    outputValue_ = (value > 0) ? 100 : 0;
  }

  bool isOnOffOnly() const override {
    return true;
  }

  bool isControlledInternally() const override {
    return false;
  }

  // Compatibility with projects expecting this method name.
  bool isOutputControlledInternally() const {
    return false;
  }

  void syncFromDevice() {
    if (controller_) {
      outputValue_ = controller_->getPower() ? 100 : 0;
    }
  }

 private:
  HaierSmartair2Controller *controller_ = nullptr;
  int outputValue_ = 0;
};

class HaierAcHvacChannel : public HvacBase {
 public:
  explicit HaierAcHvacChannel(HaierSmartair2Controller *controller)
      : HvacBase(&output_), output_(controller), haierCtrl(controller) {
    getChannel()->setDefault(SUPLA_CHANNELFNC_HVAC_THERMOSTAT);
    setDefaultTemperatureRoomMin(SUPLA_CHANNELFNC_HVAC_THERMOSTAT, 1600);
    setDefaultTemperatureRoomMax(SUPLA_CHANNELFNC_HVAC_THERMOSTAT, 3000);
    setButtonTemperatureStep(100);
  }

  void onInit() override {
    HvacBase::onInit();

    if (haierCtrl) {
      float targetTempC = haierCtrl->getTargetTemperatureC();
      int16_t currentTemp = 2200;
      if (std::isfinite(targetTempC)) {
        currentTemp = static_cast<int16_t>(lroundf(targetTempC * 100.0f));
      }
      if (currentTemp < 1600 || currentTemp > 3000) {
        currentTemp = 2200;
      }
      setTemperatureSetpointHeat(currentTemp);
      lastSetpoint = currentTemp;
      output_.syncFromDevice();

      // Wait for first valid AC status before syncing HVAC mode/setpoint.
      initialSyncDone = false;
      lastMode = getMode();
    }
  }

  void iterateAlways() override {
    HvacBase::iterateAlways();

    if (!haierCtrl) {
      return;
    }

    output_.syncFromDevice();

    // Do not drive HVAC logic until first valid status is received.
    if (!haierCtrl->hasStatus()) {
      return;
    }

    if (!initialSyncDone) {
      bool modeIsSupported = false;
      int deviceMode = mapDeviceModeToHvac(&modeIsSupported);
      if (modeIsSupported && deviceMode != SUPLA_HVAC_MODE_OFF) {
        lastSupportedMode = deviceMode;
      }
      setTargetMode(deviceMode, false);
      lastMode = deviceMode;

      float acTargetTempC = haierCtrl->getTargetTemperatureC();
      if (std::isfinite(acTargetTempC)) {
        int16_t acTempIn01C =
            static_cast<int16_t>(lroundf(acTargetTempC * 100.0f));
        if (acTempIn01C >= 1600 && acTempIn01C <= 3000) {
          setTemperatureSetpointHeat(acTempIn01C);
          lastSetpoint = acTempIn01C;
        }
      }

      initialSyncDone = true;
      return;
    }

    bool modeIsSupported = false;
    int deviceMode = mapDeviceModeToHvac(&modeIsSupported);
    if (modeIsSupported && deviceMode != SUPLA_HVAC_MODE_OFF) {
      lastSupportedMode = deviceMode;
    }

    // Head-side changes (radio/status packet) -> HVAC class.
    if (deviceMode != getMode()) {
      setTargetMode(deviceMode, false);
      lastMode = deviceMode;
    }

    int mode = getMode();
    if (mode != lastMode) {
      switch (mode) {
        case SUPLA_HVAC_MODE_OFF:
          haierCtrl->setPower(false);
          break;
        case SUPLA_HVAC_MODE_COOL:
          haierCtrl->setPower(true);
          haierCtrl->setACMode(
              Supla::haier::smartair2_protocol::ConditioningMode::COOL);
          lastSupportedMode = SUPLA_HVAC_MODE_COOL;
          break;
        case SUPLA_HVAC_MODE_HEAT:
          haierCtrl->setPower(true);
          haierCtrl->setACMode(
              Supla::haier::smartair2_protocol::ConditioningMode::HEAT);
          lastSupportedMode = SUPLA_HVAC_MODE_HEAT;
          break;
      }
      lastMode = mode;
    }

    int16_t setpointHeat = getTemperatureSetpointHeat();
    if (setpointHeat != INT16_MIN && setpointHeat != lastSetpoint) {
      if (setpointHeat < 1600) {
        setpointHeat = 1600;
      }
      if (setpointHeat > 3000) {
        setpointHeat = 3000;
      }

      // Haier supports integer setpoint in C; HVAC keeps 0.01C scale.
      setpointHeat = static_cast<int16_t>(((setpointHeat + 50) / 100) * 100);

      float targetTemp = static_cast<float>(setpointHeat) / 100.0f;
      SUPLA_LOG_DEBUG("HVAC setpoint changed to %.2fC", targetTemp);
      haierCtrl->setTargetTemperatureC(targetTemp);
      lastSetpoint = setpointHeat;
    }

    float acTargetTempC = haierCtrl->getTargetTemperatureC();
    if (std::isfinite(acTargetTempC)) {
      int16_t acTempIn01C =
          static_cast<int16_t>(lroundf(acTargetTempC * 100.0f));
      if (acTempIn01C >= 1600 && acTempIn01C <= 3000 &&
          acTempIn01C != getTemperatureSetpointHeat()) {
        setTemperatureSetpointHeat(acTempIn01C);
        lastSetpoint = acTempIn01C;
      }
    }
  }

 private:
  int mapDeviceModeToHvac(bool *isSupportedMode) const {
    if (!haierCtrl || !haierCtrl->getPower()) {
      if (isSupportedMode) {
        *isSupportedMode = true;
      }
      return SUPLA_HVAC_MODE_OFF;
    }

    if (haierCtrl->getModeCool()) {
      if (isSupportedMode) {
        *isSupportedMode = true;
      }
      return SUPLA_HVAC_MODE_COOL;
    }

    if (haierCtrl->getModeHeat()) {
      if (isSupportedMode) {
        *isSupportedMode = true;
      }
      return SUPLA_HVAC_MODE_HEAT;
    }

    // DRY/AUTO/FAN -> keep last supported HVAC mode (variant B).
    if (isSupportedMode) {
      *isSupportedMode = false;
    }
    return lastSupportedMode;
  }

  HaierHvacOutput output_;
  HaierSmartair2Controller *haierCtrl = nullptr;
  int16_t lastSetpoint = INT16_MIN;
  int16_t lastMode = INT16_MIN;
  int16_t lastSupportedMode = SUPLA_HVAC_MODE_HEAT;
  bool initialSyncDone = false;
};

}  // namespace Control
}  // namespace Supla

#endif  // HAIER_AC_HVAC_CHANNEL_H
