#pragma once

#include "Device/BaroDevice.hpp"
#include "Model.h"
#include "Sensor/BaseSensor.hpp"
#include "Utils/Filter.h"
#if defined(ESPFC_SPI_SLOW_SENSOR_WORKER) && defined(ESPFC_MULTI_CORE) && defined(ESPFC_FREE_RTOS)
#include <atomic>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#endif

namespace Espfc::Sensor {

class BaroSensor : public BaseSensor
{
public:
  enum BaroState
  {
    BARO_STATE_INIT,
    BARO_STATE_TEMP_GET,
    BARO_STATE_PRESS_GET,
  };

  BaroSensor(Model& model);

  int begin();
  int update();
  int read();
  int reload(ModelChangeEvent event);
  bool enableBackground();
  void disableBackground();
  int updateBackground();

#ifndef UNIT_TEST
private:
#endif
  void readTemperature();
  bool readPressure();
  void updateAltitude();
  int readSample();
  int reloadFilter();
  Espfc::BaroState& runtimeState();

  Model& _model;

  Device::BaroDevice* _baro{nullptr};

  BaroState _state{BARO_STATE_INIT};

  Utils::Filter _temperatureFilter;
  Utils::Filter _pressureFilter;
  Utils::Filter _altitudeFilter;
  Utils::Filter _varioFilter;

  uint32_t _wait{0};

  int32_t _counter{0};

  float _biasAlpha{0.0f};

  bool _first{true};

bool _filtersInitialized{false};
bool _pressurePrimed{false};

  uint32_t _lastAltitudeUs{0};
#if defined(ESPFC_SPI_SLOW_SENSOR_WORKER) && defined(ESPFC_MULTI_CORE) && defined(ESPFC_FREE_RTOS)
  struct BackgroundSample {
    Espfc::BaroState baro{};
    uint32_t elapsedUs{0};
  };
  Espfc::BaroState _backgroundState{};
  QueueHandle_t _backgroundQueue{nullptr};
  bool _backgroundEnabled{false};
  std::atomic<bool> _backgroundReloadPending{false};
#endif
};

} // namespace Espfc::Sensor
