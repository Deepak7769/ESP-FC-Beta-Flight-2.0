#include "SensorManager.h"

namespace Espfc {

SensorManager::SensorManager(Model& model)
    : _model(model), _gyro(model), _accel(model), _mag(model), _baro(model), _voltage(model), _fusion(model),
      _altitude(model), _fusionUpdate(false)
{
}


int SensorManager::begin()
{
  _gyro.begin();
  _accel.begin();
  _mag.begin();
  _baro.begin();
  _voltage.begin();
  _fusion.begin();
  _altitude.begin();
  _button.begin(_model.config.pin[PIN_BUTTON]);

  return 1;
}

int SensorManager::reload(ModelChangeEvent event)
{
  _gyro.reload(event);
  _accel.reload(event);
  _fusion.reload(event);
  _baro.reload(event);
  _mag.reload(event);
  _altitude.reload(event);
  _voltage.reload(event);
  return 1;
}

int FAST_CODE_ATTR SensorManager::read()
{
  int flags = SENSOR_READ_NONE;

  const bool gyroReadOk = _gyro.read();

  // Always advance the scheduler divider.
  // If the gyro transaction failed, that control cycle is skipped
  // instead of reusing an old gyro value.
  const bool controlDue =
      _model.state.loopTimer.syncTo(
          _model.state.gyro.timer);

  if (gyroReadOk && controlDue)
  {
    flags |= SENSOR_READ_CONTROL;
  }

  if (_model.state.accel.timer.syncTo(
          _model.state.gyro.timer))
  {
const bool accelReadOk =
    _accel.update();

_model.state.mode.button =
    _button.update();

if (accelReadOk)
{
  flags |= SENSOR_READ_ACCEL;
}

    return flags;
  }

  if (_mag.update())
  {
    return flags;
  }

  if (_baro.update())
  {
    return flags;
  }

  if (_voltage.update())
  {
    return flags;
  }

  return flags;
}

int FAST_CODE_ATTR SensorManager::preLoop()
{
  _gyro.filter();
  if (_model.state.gyro.biasSamples == 0)
  {
    _model.state.gyro.biasSamples = -1;
    _fusion.restoreGain();
  }
  return 1;
}

int SensorManager::postLoop()
{
  _gyro.postLoop();
  return 1;
}

int FAST_CODE_ATTR SensorManager::fusion()
{
  const bool fusionValid =
      _fusion.update() != 0;

  // Altitude estimation may still consume a new
  // barometer observation after a rejected AHRS cycle,
  // but it must not integrate stale world-frame
  // acceleration from that rejected cycle.
  _altitude.update(
      fusionValid);

  return
      fusionValid ? 1 : 0;
}

// main task
int FAST_CODE_ATTR SensorManager::update()
{
  if (!_gyro.read())
  {
    return 0;
  }

  return preLoop();
}

// sub task
int SensorManager::updateDelayed()
{
  _gyro.postLoop();

  // update at most one sensor besides gyro
  int status = 0;
  if (_model.state.accel.timer.syncTo(_model.state.gyro.timer))
  {
    _accel.update();
    _model.state.mode.button = _button.update();
    status = 1;
  }

  // delay imu update to next cycle
  if (_fusionUpdate)
  {
    _fusionUpdate = false;
    fusion();
  }
  _fusionUpdate = status;

  if (status) return 1;

  if (_mag.update()) return 1;

  if (_baro.update()) return 1;

  if (_voltage.update()) return 0;

  return 0;
}

} // namespace Espfc
