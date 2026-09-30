#include "Espfc.h"
#include "Debug_Espfc.h"

namespace Espfc {

Espfc::Espfc()
    : _hardware{_model}, _controller{_model}, _telemetry{_model}, _input{_model, _telemetry}, _actuator{_model},
      _sensor{_model}, _mixer{_model}, _blackbox{_model}, _buzzer{_model}, _serial{_model, _telemetry}
{
}

int Espfc::load()
{
  PIN_DEBUG_INIT();
  _model.load();
  _model.state.appQueue.begin();
  return 1;
}

int Espfc::begin()
{
  _model.state.led.begin(_model.config.pin[PIN_LED_BLINK], _model.config.led.type, _model.config.led.invert);

  _serial.begin();   // requires _model.load()
  _hardware.begin(); // requires _model.load()
  _model.begin();    // requires _hardware.begin()
  _mixer.begin();
  _sensor.begin();   // requires _hardware.begin()
  _input.begin();    // requires _serial.begin()
  _actuator.begin(); // requires _model.begin()
  _controller.begin();
  _blackbox.begin(); // requires _serial.begin(), _actuator.begin()
  _buzzer.begin();

  _model.state.buzzer.push(BUZZER_SYSTEM_INIT);

  _model.setConfigChangeListener([this](ModelChangeEvent event) {
    _serial.reload(event);
    _sensor.reload(event);
    _input.reload(event);
    _controller.reload(event);
  });

  return 1;
}

int FAST_CODE_ATTR Espfc::update(bool externalTrigger)
{
  if (externalTrigger)
  {
    _model.state.gyro.timer.update();
  }
  else
  {
    if (!_model.state.gyro.timer.check()) return 0;
  }
  Utils::Stats::Measure measure(_model.state.stats, COUNTER_CPU_0);
#if defined(ESPFC_MULTI_CORE)

  const int sensorFlags =
      _sensor.read();

  // Receiver update first so a control iteration uses
  // the newest available setpoint.
  if (_model.state.input.timer.syncTo(
          _model.state.gyro.timer,
          1u))
  {
    const bool wasArmedBeforeInput =
        _model.isModeActive(
            MODE_ARMED);

    _input.update();

    // RX Stage-2 DROP can disarm outside the normal mixer cadence. Push the
    // configured disarmed outputs immediately instead of leaving the previous
    // motor command active until the next scheduled mixer tick.
    if (wasArmedBeforeInput &&
        !_model.isModeActive(
            MODE_ARMED))
    {
      _mixer.writeDisarmed();
    }
  }

  // Only preprocess gyro data when a valid gyro sample
  // actually triggered this PID cycle.
  if (sensorFlags & SENSOR_READ_CONTROL)
  {
    _sensor.preLoop();
  }

  // Fusion is executed in the same deterministic task.
  if (sensorFlags & SENSOR_READ_ACCEL)
  {
    _sensor.fusion();
  }

  if (sensorFlags & SENSOR_READ_CONTROL)
  {
    _controller.update();

    if (_model.state.mixer.timer.syncTo(
            _model.state.loopTimer))
    {
      _mixer.update();
    }

    _blackbox.update();

    _sensor.postLoop();
  }

if (_model.state.actuatorTimer.check())
{
  const bool wasArmed =
      _model.isModeActive(
          MODE_ARMED);

  _actuator.update();

  const bool isArmed =
      _model.isModeActive(
          MODE_ARMED);

  if (wasArmed &&
      !isArmed)
  {
    _mixer.writeDisarmed();
  }
}

#else


  const bool gyroSampleValid =
      _sensor.update() != 0;

  const bool controlDue =
      _model.state.loopTimer.syncTo(
          _model.state.gyro.timer);

  // Receiver supervision must not depend on a successful gyro transaction.
  // Otherwise a gyro-bus fault can also stop RX-loss qualification/failsafe
  // from advancing and leave the last actuator command resident longer than
  // intended. Keep RX handling on its own scheduler cadence.
  if (_model.state.input.timer.syncTo(
          _model.state.gyro.timer,
          1u))
  {
    const bool wasArmedBeforeInput =
        _model.isModeActive(
            MODE_ARMED);

    _input.update();

    if (wasArmedBeforeInput &&
        !_model.isModeActive(
            MODE_ARMED))
    {
      _mixer.writeDisarmed();
    }
  }

  // Advance the loop divider even on a failed gyro transaction, but never run
  // the controller from a stale sample.
  if (gyroSampleValid &&
      controlDue)
  {
    _controller.update();

    if (_model.state.mixer.timer.syncTo(
            _model.state.loopTimer))
    {
      _mixer.update();
    }

    _blackbox.update();
  }

  // Arming/failsafe supervision is likewise independent of the PID sample.
  if (_model.state.actuatorTimer.check())
  {
    const bool wasArmed =
        _model.isModeActive(
            MODE_ARMED);

    _actuator.update();

    if (wasArmed &&
        !_model.isModeActive(
            MODE_ARMED))
    {
      _mixer.writeDisarmed();
    }
  }

  _sensor.updateDelayed();

#endif

  _serial.update();
  _buzzer.update();
  _model.state.led.update();
  _model.state.stats.update();

  return 1;
}

// other task
int FAST_CODE_ATTR Espfc::updateOther()
{
#if defined(ESPFC_MULTI_CORE)

  if (_model.state.appQueue.isEmpty())
  {
    return 0;
  }

  // Gyro/PID/IMU processing is no longer driven by this queue.
  // Drain only legacy/non-flight-critical events.
  (void)_model.state.appQueue.receive();

  return 1;

#else

  return 0;

#endif
}

} // namespace Espfc
