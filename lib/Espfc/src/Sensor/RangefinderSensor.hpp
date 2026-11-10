#pragma once

#include "Device/Rangefinder/Hcsr04.hpp"
#include "Model.h"

namespace Espfc::Sensor {

class RangefinderSensor
{
public:
  explicit RangefinderSensor(Model& model);

  int begin();
  int update();
  int reload(ModelChangeEvent event);

  // Pure sample publication/expiry rule to test delayed, invalid and
  // rollover captures without depending on particular GPIO hardware.
  static int applyMeasurement(RangefinderState& state, int status,
                              const Device::RangefinderMeasurement& measurement,
                              uint32_t nowUs);

private:
  int activateBackend();

  Model& _model;
  Device::Hcsr04Rangefinder _hcsr04;
  Device::RangefinderDevice* _device{nullptr};
  RangefinderType _type{RANGEFINDER_NONE};
};

} // namespace Espfc::Sensor
