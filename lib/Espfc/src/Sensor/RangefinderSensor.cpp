#include "Sensor/RangefinderSensor.hpp"

#include "Hal/Time.hpp"
#include <cmath>

namespace Espfc::Sensor {

RangefinderSensor::RangefinderSensor(Model& model):
    _model(model)
{
}

int RangefinderSensor::activateBackend()
{
  _device = nullptr;
  _type = static_cast<RangefinderType>(
      _model.config.rangefinder.type);

  switch (_type)
  {
    case RANGEFINDER_HCSR04:
      // This is only the FC's *direct GPIO* trigger/echo driver.
      // There is no Arduino companion receiver in this firmware.
      _device = &_hcsr04;
      break;

    case RANGEFINDER_NONE:
    default:
      // Unknown/unsupported companion types are not aliased to
      // direct GPIO sonar. Leave state.present=false.
      break;
  }

  auto& state = _model.state.rangefinder;
  state.present = false;
  state.sampleValid = false;
  state.timestampValid = false;
  state.lastUpdateUs = 0;
  state.distance = 0.0f;
  state.quality = 0;

  if (!_device)
    return 0;

  const int ok =
      _device->begin(
          _model.config.rangefinder);

  state.present =
      ok != 0 &&
      _device->present();

  return state.present ? 1 : 0;
}

int RangefinderSensor::begin()
{
  return activateBackend();
}

int RangefinderSensor::reload(
    ModelChangeEvent event)
{
  if (event != MODEL_CHANGE_FILTER)
  {
    return 1;
  }

  return activateBackend();
}

int RangefinderSensor::update()
{
  if (!_device)
    return 0;

  const int status = _device->update();
  const auto measurement = _device->measurement();
  return applyMeasurement(_model.state.rangefinder, status,
                          measurement, micros());
}

int RangefinderSensor::applyMeasurement(
    RangefinderState& state, int status,
    const Device::RangefinderMeasurement& measurement,
    uint32_t nowUs)
{
  constexpr uint32_t MAX_SAMPLE_AGE_US = 100000u;
  if (!state.present)
  {
    state.sampleValid = false;
    state.timestampValid = false;
    state.quality = 0;
    return 0;
  }

  if (status > 0 && measurement.valid &&
      measurement.quality > 0 &&
      std::isfinite(measurement.distance) &&
      measurement.distance >= 0.0f &&
      static_cast<uint32_t>(nowUs - measurement.capturedUs) <
          MAX_SAMPLE_AGE_US)
  {
    state.sampleValid = true;
    state.timestampValid = true;
    state.distance = measurement.distance;
    state.quality = measurement.quality;
    state.lastUpdateUs = measurement.capturedUs;
    return 1;
  }

  // An explicit backend-invalid result (timeout or rejected echo) must
  // invalidate the prior measurement even inside its freshness window.
  if (!measurement.valid || status > 0)
  {
    state.sampleValid = false;
    state.timestampValid = false;
    state.quality = 0;
    return 0;
  }

  // An in-flight echo is not a new measurement. The last capture may
  // remain usable only for the original capture-age window.
  state.sampleValid =
      state.timestampValid &&
      static_cast<uint32_t>(nowUs - state.lastUpdateUs) <
          MAX_SAMPLE_AGE_US;
  if (!state.sampleValid)
  {
    state.quality = 0;
    state.timestampValid = false;
  }
  return 0;
}

} // namespace Espfc::Sensor
