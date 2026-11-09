#pragma once

#include "BaseSensor.hpp"
#include "Model.h"
#include "Utils/Filter.h"
#include <cmath>

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

  // Purely voltage-driven pack lifecycle; callable from native tests.
  // A confirmed disarmed disconnect is required before resetting cell count.
  void updatePackDetection();
  int readIbat();

  // A gap without fresh ADC data must not be integrated as real discharge.
  static float integrateConsumedMah(float previousMah, float amps, uint32_t elapsedUs)
  {
    if (!std::isfinite(previousMah) || previousMah < 0.0f)
    {
      previousMah = 0.0f;
    }
    if (!std::isfinite(amps) || amps <= 0.0f ||
        elapsedUs == 0 || elapsedUs > 250000u)
    {
      return previousMah;
    }
    return previousMah + amps * static_cast<float>(elapsedUs) / 3600000.0f;
  }

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
  uint32_t _lastAdcSampleUs{0};
  int8_t _lastConsumedSource{CURRENT_METER_NONE};
  uint8_t _packAbsentReads{0};
};

} // namespace Espfc::Sensor
