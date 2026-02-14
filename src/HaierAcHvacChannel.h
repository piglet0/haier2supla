#ifndef HAIER_AC_HVAC_CHANNEL_H
#define HAIER_AC_HVAC_CHANNEL_H

#include <supla/channel_element.h>
#include <supla/actions.h>
#include <supla/log_wrapper.h>
#include "HaierSmartair2Controller.h"

namespace Supla {
namespace Control {

class HaierAcHvacChannel : public ChannelElement {
 public:
  explicit HaierAcHvacChannel(HaierSmartair2Controller* controller) 
    : haierCtrl(controller), lastSetpoint(INT16_MIN) {
    channel.setType(SUPLA_CHANNELTYPE_HVAC);
    channel.setDefault(SUPLA_CHANNELFNC_HVAC_THERMOSTAT);
    channel.setFlag(SUPLA_CHANNEL_FLAG_CHANNELSTATE);
  }

  void onInit() override {
    // Use HEAT_COOL mode to show temperature control without mode preference
    channel.setHvacMode(SUPLA_HVAC_MODE_HEAT_COOL);
    
    // Initialize with current AC temperature
    if (haierCtrl) {
      uint8_t currentTemp = haierCtrl->getTargetTemperatureC();
      if (currentTemp > 0) {
        int16_t tempIn01C = currentTemp * 100;  // Convert to 0.01°C units
        channel.setHvacSetpointTemperatureHeat(tempIn01C);
        lastSetpoint = tempIn01C;
      }
    }
  }

  void iterateAlways() override {
    if (!haierCtrl) {
      return;
    }

    // Read temperature setpoint from Supla channel (set by user via app)
    int16_t setpointHeat = channel.getHvacSetpointTemperatureHeat();
    
    // If user changed the setpoint via Supla app
    if (setpointHeat != INT16_MIN && setpointHeat != lastSetpoint) {
      // Convert from 0.01°C to actual °C
      uint8_t targetTemp = setpointHeat / 100;
      
      // Validate temperature range (16-30°C for most ACs)
      if (targetTemp >= 16 && targetTemp <= 30) {
        SUPLA_LOG_DEBUG("User set temperature to: %d°C", targetTemp);
        
        // Send to Haier AC
        haierCtrl->setTargetTemperatureC(targetTemp);
        lastSetpoint = setpointHeat;
      }
    }

    // Update channel with current AC state (read from Haier)
    uint8_t acTemp = haierCtrl->getTargetTemperatureC();
    int16_t acTempIn01C = acTemp * 100;
    
    // Only update if different to avoid unnecessary channel updates
    if (acTempIn01C != channel.getHvacSetpointTemperatureHeat()) {
      channel.setHvacSetpointTemperatureHeat(acTempIn01C);
      lastSetpoint = acTempIn01C;
    }

    // Update power state only
    channel.setHvacIsOn(haierCtrl->getPower());
  }

  // Temperature changes from server are handled in iterateAlways() via channel.getHvacSetpointTemperatureHeat()
  // No additional server command handling needed since the channel state is already synchronized

 private:
  HaierSmartair2Controller* haierCtrl;
  int16_t lastSetpoint;
};

}  // namespace Control
}  // namespace Supla

#endif  // HAIER_AC_HVAC_CHANNEL_H
