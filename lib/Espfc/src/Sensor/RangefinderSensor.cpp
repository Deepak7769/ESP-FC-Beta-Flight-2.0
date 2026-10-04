#include "Sensor/RangefinderSensor.hpp"

#include "Hal/Time.hpp"

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
      _device = &_hcsr04;
      break;

    case RANGEFINDER_NONE:
    default:
      break;
  }

  auto& state = _model.state.rangefinder;
  state.present = false;
  state.sampleValid = false;
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

  const int status =
      _device->update();

  const auto measurement =
      _device->measurement();

  auto& state =
      _model.state.rangefinder;

  if (status > 0 &&
      measurement.valid)
  {
    state.sampleValid = true;
    state.distance = measurement.distance;
    state.quality = measurement.quality;
    state.lastUpdateUs = micros();
    return 1;
  }

  // Keep the last valid distance available during the backend's
  // conversion/echo window. The estimator independently applies a
  // freshness timeout, so a missed echo cannot be mistaken for a fresh
  // measurement.
  if (state.present &&
      state.lastUpdateUs != 0)
  {
    state.sampleValid =
        static_cast<uint32_t>(
            micros() - state.lastUpdateUs) <
        100000u;
  }
  else
  {
    state.sampleValid = false;
  }

  return 0;
}

} // namespace Espfc::Sensor
