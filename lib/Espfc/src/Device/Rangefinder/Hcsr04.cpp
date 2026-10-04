#include "Device/Rangefinder/Hcsr04.hpp"

#include "Hal/Gpio.hpp"
#include "Hal/Time.hpp"

#include <algorithm>
#include <cstdint>

namespace Espfc::Device {

namespace {

constexpr uint32_t HC_SR04_TRIGGER_US = 10;
constexpr uint32_t HC_SR04_CM_TO_US = 58;

}

bool Hcsr04Rangefinder::reached(
    uint32_t now,
    uint32_t deadline)
{
  return static_cast<int32_t>(
             now - deadline) >= 0;
}

int Hcsr04Rangefinder::begin(
    const RangefinderConfig& config)
{
  _state = State::HCSR04_DISABLED;
  _triggerPin = config.triggerPin;
  _echoPin = config.echoPin;
  _minDistanceCm = config.minDistanceCm;
  _maxDistanceCm = config.maxDistanceCm;
  _updateIntervalMs = config.updateIntervalMs;

  _present = false;
  _measurementValid = false;
  _distance = 0.0f;
  _quality = 0;

  if (config.type != RANGEFINDER_HCSR04 ||
      _triggerPin < 0 ||
      _echoPin < 0 ||
      _triggerPin == _echoPin)
  {
    return 0;
  }

  _timeoutUs =
      std::clamp(
          static_cast<uint32_t>(_maxDistanceCm) *
              HC_SR04_CM_TO_US +
              2000u,
          5000u,
          35000u);

  Hal::Gpio::pinMode(
      static_cast<uint8_t>(_triggerPin),
      Hal::Gpio::Output);
  Hal::Gpio::pinMode(
      static_cast<uint8_t>(_echoPin),
      Hal::Gpio::Input);
  Hal::Gpio::digitalWrite(
      static_cast<uint8_t>(_triggerPin),
      Hal::Gpio::Low);

  _nextTriggerUs = micros();
  _state = State::WAIT_TRIGGER;
  _present = true;

  return 1;
}

int Hcsr04Rangefinder::update()
{
  if (!_present)
    return 0;

  const uint32_t now = micros();

  switch (_state)
  {
    case State::WAIT_TRIGGER:
      if (reached(now, _nextTriggerUs))
      {
        Hal::Gpio::digitalWrite(
            static_cast<uint8_t>(_triggerPin),
            Hal::Gpio::High);
        _triggerStartUs = now;
        _state = State::TRIGGER_HIGH;
      }
      break;

    case State::TRIGGER_HIGH:
      if (static_cast<uint32_t>(
              now - _triggerStartUs) >=
          HC_SR04_TRIGGER_US)
      {
        Hal::Gpio::digitalWrite(
            static_cast<uint8_t>(_triggerPin),
            Hal::Gpio::Low);
        _deadlineUs = now + _timeoutUs;
        _state = State::WAIT_ECHO_RISE;
      }
      break;

    case State::WAIT_ECHO_RISE:
      if (Hal::Gpio::digitalRead(
              static_cast<uint8_t>(_echoPin)) ==
          Hal::Gpio::High)
      {
        _echoStartUs = now;
        _state = State::WAIT_ECHO_FALL;
      }
      else if (reached(now, _deadlineUs))
      {
        _measurementValid = false;
        _quality = 0;
        _nextTriggerUs =
            now +
            static_cast<uint32_t>(_updateIntervalMs) *
                1000u;
        _state = State::WAIT_TRIGGER;
      }
      break;

    case State::WAIT_ECHO_FALL:
      if (Hal::Gpio::digitalRead(
              static_cast<uint8_t>(_echoPin)) ==
          Hal::Gpio::Low)
      {
        const uint32_t widthUs =
            static_cast<uint32_t>(
                now - _echoStartUs);
        const float distanceCm =
            static_cast<float>(widthUs) /
            static_cast<float>(HC_SR04_CM_TO_US);

        const bool valid =
            widthUs >=
                static_cast<uint32_t>(_minDistanceCm) *
                    HC_SR04_CM_TO_US &&
            distanceCm <=
                static_cast<float>(_maxDistanceCm);

        _measurementValid = valid;

        if (valid)
        {
          _distance = distanceCm;
          // HC-SR04 has no native quality field. A valid in-range
          // echo gets full quality; estimator innovation gating remains
          // the consistency check.
          _quality = 100;
        }
        else
        {
          _quality = 0;
        }

        _nextTriggerUs =
            now +
            static_cast<uint32_t>(_updateIntervalMs) *
                1000u;
        _state = State::WAIT_TRIGGER;
      }
      else if (reached(now, _deadlineUs))
      {
        _measurementValid = false;
        _quality = 0;
        _nextTriggerUs =
            now +
            static_cast<uint32_t>(_updateIntervalMs) *
                1000u;
        _state = State::WAIT_TRIGGER;
      }
      break;

    case State::HCSR04_DISABLED:
    default:
      break;
  }

  return _measurementValid ? 1 : 0;
}

bool Hcsr04Rangefinder::present() const
{
  return _present;
}

RangefinderMeasurement Hcsr04Rangefinder::measurement() const
{
  return {
      _measurementValid,
      _distance,
      _quality};
}

RangefinderDevice::DeviceType
Hcsr04Rangefinder::getType() const
{
  return RANGEFINDER_HCSR04;
}

} // namespace Espfc::Device
