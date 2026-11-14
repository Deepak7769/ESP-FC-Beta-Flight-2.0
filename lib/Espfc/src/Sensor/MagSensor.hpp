#pragma once

#include "BaseSensor.hpp"
#include "Device/MagDevice.hpp"
#include "Model.h"
#if defined(ESPFC_SPI_SLOW_SENSOR_WORKER) && defined(ESPFC_MULTI_CORE) && defined(ESPFC_FREE_RTOS)
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#endif

namespace Espfc::Sensor {

class MagSensor : public BaseSensor
{
public:
  MagSensor(Model& model);

  int begin();
  int reload(ModelChangeEvent event);
  int update();
  int read();
  int filter();
  bool enableBackground();
  void disableBackground();
  int updateBackground();

private:
  void calibrate();
  void resetCalibration();
  void updateCalibration();
  void applyCalibration();

  Model& _model;
  Device::MagDevice* _mag{nullptr};
#if defined(ESPFC_SPI_SLOW_SENSOR_WORKER) && defined(ESPFC_MULTI_CORE) && defined(ESPFC_FREE_RTOS)
  struct BackgroundSample {
    VectorInt16 raw{};
    uint32_t elapsedUs{0};
    bool valid{false};
  };
  QueueHandle_t _backgroundQueue{nullptr};
  bool _backgroundEnabled{false};
#endif
};

} // namespace Espfc::Sensor
