#pragma once

#include "Device/RangefinderDevice.hpp"

namespace Espfc::Device {

class Hcsr04Rangefinder : public RangefinderDevice
{
public:
  int begin(const RangefinderConfig& config) override;
  int update() override;
  bool present() const override;
  RangefinderMeasurement measurement() const override;
  DeviceType getType() const override;

private:
  enum class State : uint8_t
  {
    DISABLED,
    WAIT_TRIGGER,
    TRIGGER_HIGH,
    WAIT_ECHO_RISE,
    WAIT_ECHO_FALL,
  };

  static bool reached(uint32_t now, uint32_t deadline);

  State _state{State::DISABLED};
  int8_t _triggerPin{-1};
  int8_t _echoPin{-1};
  uint16_t _minDistanceCm{2};
  uint16_t _maxDistanceCm{400};
  uint16_t _updateIntervalMs{60};
  uint32_t _timeoutUs{30000};
  uint32_t _nextTriggerUs{0};
  uint32_t _triggerStartUs{0};
  uint32_t _echoStartUs{0};
  uint32_t _deadlineUs{0};

  bool _present{false};
  bool _measurementValid{false};
  float _distance{0.0f};
  uint8_t _quality{0};
};

} // namespace Espfc::Device
