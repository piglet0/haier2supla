#pragma once

#include <Arduino.h>
#include <supla/control/hvac_base.h>
#include <supla/control/remote_output_interface.h>

#include "HaierSmartair2Controller.h"

class HaierAcHvacChannel : public Supla::Control::HvacBase {
 public:
	explicit HaierAcHvacChannel(HaierSmartair2Controller *controller)
			: Supla::Control::HvacBase(&power_output_),
				controller_(controller),
				power_output_(true) {
		channel.setDefaultFunction(SUPLA_CHANNELFNC_HVAC_THERMOSTAT);
		setHeatingAndCoolingSupported(true);
		setHeatCoolSupported(false);
		setTemperatureRoomMin(1600);
		setTemperatureRoomMax(3000);
		setButtonTemperatureStep(100);
		setOutputValueOnError(0);
		setMinOnTimeS(0);
		setMinOffTimeS(0);
		setDefaultSubfunction(SUPLA_HVAC_SUBFUNCTION_HEAT);
		setSubfunction(SUPLA_HVAC_SUBFUNCTION_HEAT);
		setTemperatureSetpointHeat(2300);
		setTemperatureSetpointCool(2300);
		last_requested_setpoint_centi_ = 2300;
		last_synced_setpoint_centi_ = 2300;
	}

	void onInit() override {
		Supla::Control::HvacBase::onInit();
		syncFromController();
	}

	void iterateAlways() override {
		Supla::Control::HvacBase::iterateAlways();

		if (controller_ == nullptr) {
			updateChannelState();
			return;
		}

		normalizeRequestedMode_();

		const bool desired_power = getMode() != SUPLA_HVAC_MODE_OFF;
		if (desired_power != last_requested_power_) {
			last_requested_power_ = desired_power;
			if (controller_->getPower() != desired_power) {
				controller_->setPower(desired_power);
				pending_local_power_change_ = true;
				pending_power_state_ = desired_power;
				pending_power_change_started_ms_ = millis();
			}
		}

		const int16_t desired_setpoint_centi = getDesiredSharedSetpointCenti_();
		if (desired_setpoint_centi != INT16_MIN &&
				desired_setpoint_centi != last_requested_setpoint_centi_) {
			last_requested_setpoint_centi_ = desired_setpoint_centi;
			controller_->setTargetTemperatureC(centiToCelsius_(desired_setpoint_centi));
			pending_local_setpoint_change_ = true;
			pending_setpoint_centi_ = desired_setpoint_centi;
			pending_setpoint_change_started_ms_ = millis();
		}

		syncFromController();
		updateChannelState();
	}

	void syncFromController() {
		if (controller_ == nullptr || !controller_->hasStatus()) {
			return;
		}

		const bool actual_power = controller_->getPower();
		const int16_t actual_setpoint_centi =
				celsiusToCenti_(controller_->getTargetTemperatureC());
		const unsigned long now = millis();

		if (pending_local_power_change_) {
			if (actual_power == pending_power_state_) {
				pending_local_power_change_ = false;
			} else if (now - pending_power_change_started_ms_ <
								 kLocalPowerSyncGuardMs) {
				return;
			} else {
				pending_local_power_change_ = false;
			}
		}

		power_output_.setOutputValueFromRemote(actual_power ? 1 : 0);

		if (!actual_power && getMode() != SUPLA_HVAC_MODE_OFF) {
			setTargetMode(SUPLA_HVAC_MODE_OFF, false);
			last_requested_power_ = false;
		} else if (actual_power && getMode() == SUPLA_HVAC_MODE_OFF) {
			setTargetMode(kHvacActiveMode, false);
			last_requested_power_ = true;
		} else if (actual_power && getMode() != kHvacActiveMode) {
			setTargetMode(kHvacActiveMode, false);
		}

		if (actual_setpoint_centi == INT16_MIN) {
			return;
		}

		if (pending_local_setpoint_change_) {
			if (abs(actual_setpoint_centi - pending_setpoint_centi_) <= 50) {
				pending_local_setpoint_change_ = false;
			} else if (now - pending_setpoint_change_started_ms_ <
						 kLocalSetpointSyncGuardMs) {
				return;
			} else {
				pending_local_setpoint_change_ = false;
			}
		}

		syncSharedSetpointInHvac_(actual_setpoint_centi);
		last_requested_setpoint_centi_ = actual_setpoint_centi;
	}

 private:
	static constexpr unsigned long kLocalPowerSyncGuardMs = 3000;
	static constexpr unsigned long kLocalSetpointSyncGuardMs = 3000;
	static constexpr int kHvacActiveMode = SUPLA_HVAC_MODE_HEAT;

	static int16_t celsiusToCenti_(float temperature_c) {
		if (!isfinite(temperature_c)) {
			return INT16_MIN;
		}
		return static_cast<int16_t>(lroundf(temperature_c * 100.0f));
	}

	static float centiToCelsius_(int16_t temperature_centi) {
		return static_cast<float>(temperature_centi) / 100.0f;
	}

	int16_t getDesiredSharedSetpointCenti_() {
		const int16_t heat_setpoint = getTemperatureSetpointHeat();
		const int16_t cool_setpoint = getTemperatureSetpointCool();

		if (heat_setpoint == INT16_MIN) {
			return cool_setpoint;
		}
		if (cool_setpoint == INT16_MIN) {
			return heat_setpoint;
		}
		if (heat_setpoint == cool_setpoint) {
			return heat_setpoint;
		}

		if (last_synced_setpoint_centi_ == heat_setpoint) {
			return cool_setpoint;
		}
		return heat_setpoint;
	}

	void syncSharedSetpointInHvac_(int16_t shared_setpoint_centi) {
		if (shared_setpoint_centi == INT16_MIN) {
			return;
		}

		if (getTemperatureSetpointHeat() != shared_setpoint_centi) {
			setTemperatureSetpointHeat(shared_setpoint_centi);
		}
		if (getTemperatureSetpointCool() != shared_setpoint_centi) {
			setTemperatureSetpointCool(shared_setpoint_centi);
		}
		last_synced_setpoint_centi_ = shared_setpoint_centi;
	}

	void normalizeRequestedMode_() {
		if (getMode() != SUPLA_HVAC_MODE_OFF && getMode() != kHvacActiveMode) {
			setTargetMode(kHvacActiveMode, false);
		}
	}

	HaierSmartair2Controller *controller_ = nullptr;
	Supla::Control::RemoteOutputInterface power_output_;
	bool pending_local_power_change_ = false;
	bool pending_local_setpoint_change_ = false;
	bool pending_power_state_ = false;
	bool last_requested_power_ = false;
	int16_t pending_setpoint_centi_ = INT16_MIN;
	int16_t last_requested_setpoint_centi_ = INT16_MIN;
	int16_t last_synced_setpoint_centi_ = INT16_MIN;
	unsigned long pending_power_change_started_ms_ = 0;
	unsigned long pending_setpoint_change_started_ms_ = 0;
};
