\
#pragma once

#include <supla/control/virtual_relay.h>
#include "HaierSmartair2Controller.h"
#include <functional>

namespace Supla {
namespace Control {

// Virtual relay channel in SUPLA mapped to Haier AC power state.
// obecnie nieużywany
class HaierAcVirtualRelay : public VirtualRelay {
 public:
  explicit HaierAcVirtualRelay(HaierSmartair2Controller *controller,
                               _supla_int_t functions =
                                   (0xFF ^ SUPLA_BIT_FUNC_CONTROLLINGTHEROLLERSHUTTER));

  void turnOn(_supla_int_t duration = 0) override;
  void turnOff(_supla_int_t duration = 0) override;
  bool isOn() override;
  void syncState();

 private:
  HaierSmartair2Controller *controller_;
};

// Virtual relay that switches AC to COOL when turned on, and to AUTO when turned off.
class HaierAcCoolVirtualRelay : public VirtualRelay {
 public:
  explicit HaierAcCoolVirtualRelay(HaierSmartair2Controller *controller,
                                   _supla_int_t functions =
                                       (0xFF ^ SUPLA_BIT_FUNC_CONTROLLINGTHEROLLERSHUTTER));

  void turnOn(_supla_int_t duration = 0) override;
  void turnOff(_supla_int_t duration = 0) override;
  bool isOn() override;
  void syncState();

 private:
  HaierSmartair2Controller *controller_;
};

// Generic virtual relay that calls user-supplied getter/setter functions.
// T is the argument type accepted by the setter. The relay stores the
// values to use for on/off (onValue/offValue).
template <typename T>
class HaierVirtualRelayWithArg : public VirtualRelay {
 public:
  HaierVirtualRelayWithArg(std::function<void(T)> setter,
                          std::function<T()> getter,
                          T onValue,
                          T offValue,
                          _supla_int_t functions = (0xFF ^ SUPLA_BIT_FUNC_CONTROLLINGTHEROLLERSHUTTER))
      : VirtualRelay(functions), setter_(std::move(setter)), getter_(std::move(getter)),
        onValue_(onValue), offValue_(offValue) {}

  void turnOn(_supla_int_t duration = 0) override {
    VirtualRelay::turnOn(duration);
    if (setter_) setter_(onValue_);
  }

  void turnOff(_supla_int_t duration = 0) override {
    VirtualRelay::turnOff(duration);
    if (setter_) setter_(offValue_);
  }

  bool isOn() override {
    if (getter_) return getter_() == onValue_;
    return VirtualRelay::isOn();
  }

  void syncState() {
    if (!getter_) return;
    bool on = getter_() == onValue_;
    if (on && !VirtualRelay::isOn()) {
      VirtualRelay::turnOn(0);
    } else if (!on && VirtualRelay::isOn()) {
      VirtualRelay::turnOff(0);
    }
  }

 private:
  std::function<void(T)> setter_;
  std::function<T()> getter_;
  T onValue_;
  T offValue_;
};

// Virtual relay that only triggers setter on turnOn with onValue.
// turnOff does nothing (no setter call).
template <typename T>
class HaierVirtualRelayWithArgOn : public VirtualRelay {
 public:
  HaierVirtualRelayWithArgOn(std::function<void(T)> setter,
                            std::function<T()> getter,
                            T onValue,
                            _supla_int_t functions = (0xFF ^ SUPLA_BIT_FUNC_CONTROLLINGTHEROLLERSHUTTER))
      : VirtualRelay(functions), setter_(std::move(setter)), getter_(std::move(getter)),
        onValue_(onValue) {}

  void turnOn(_supla_int_t duration = 0) override {
    VirtualRelay::turnOn(duration);
    if (setter_) setter_(onValue_);
  }

  void turnOff(_supla_int_t duration = 0) override {
    VirtualRelay::turnOff(duration);
    // Do nothing - no setter call
  }

  bool isOn() override {
    if (getter_) return getter_() == onValue_;
    return VirtualRelay::isOn();
  }

  void syncState() {
    if (!getter_) return;
    bool on = getter_() == onValue_;
    if (on && !VirtualRelay::isOn()) {
      VirtualRelay::turnOn(0);
    } else if (!on && VirtualRelay::isOn()) {
      VirtualRelay::turnOff(0);
    }
  }

 private:
  std::function<void(T)> setter_;
  std::function<T()> getter_;
  T onValue_;
};

// Convenience bool specialization implemented in .cpp
class HaierVirtualRelay : public VirtualRelay {
 public:
  HaierVirtualRelay(std::function<void(bool)> setter,
                    std::function<bool()> getter,
                    _supla_int_t functions = (0xFF ^ SUPLA_BIT_FUNC_CONTROLLINGTHEROLLERSHUTTER));

  void turnOn(_supla_int_t duration = 0) override;
  void turnOff(_supla_int_t duration = 0) override;
  bool isOn() override;
  void syncState();

 private:
  std::function<void(bool)> setter_;
  std::function<bool()> getter_;
};


}  // namespace Control
}  // namespace Supla
