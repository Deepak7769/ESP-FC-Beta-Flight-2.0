#pragma once

#include "BaseSensor.hpp"
#include "Model.h"
#include "Utils/Filter.h"

namespace Espfc::Sensor {

class VoltageSensor : public BaseSensor
{
public:
  enum State
  {
    VBAT,
    IBAT
  };
  VoltageSensor(Model& model);
  int begin();
  int reload(ModelChangeEvent event);
  int update();
  int readVbat();
  int readIbat();

  // Betaflight ADC current-meter calibration: scale is mV / 10 A;
  // offset is mA. Keep the conversion independent of ADC hardware for tests.
  static float adcCurrentAmps(float millivolts, int16_t scale, int16_t offsetMilliAmps)
  {
    return scale > 0
        ? millivolts * 10.0f / static_cast<float>(scale) +
              static_cast<float>(offsetMilliAmps) * 0.001f
        : 0.0f;
  }

private:
  Model& _model;
  Utils::Filter _vFilterFast;
  Utils::Filter _vFilter;
  Utils::Filter _iFilterFast;
  Utils::Filter _iFilter;
  State _state;
};

} // namespace Espfc::Sensor
