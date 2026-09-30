#include "VoltageSensor.hpp"
#include "Hal/Time.hpp"

#include <algorithm>

namespace Espfc::Sensor {

VoltageSensor::VoltageSensor(Model& model): _model(model) {}

int VoltageSensor::begin()
{
  _model.state.battery.timer.setRate(100);
  _model.state.battery.samples = 50;

  _model.state.battery.mspCurrentCentiAmps = 0;
  _model.state.battery.mspMahDrawn = 0;
  _model.state.battery.mspCurrentLastUpdateUs = 0;
  _model.state.battery.mspCurrentValid = false;

  reload(MODEL_CHANGE_FILTER);

  _state = VBAT;

  return 1;
}

int VoltageSensor::reload(ModelChangeEvent event)
{
  switch (event)
  {
    case MODEL_CHANGE_FILTER:
      _vFilterFast.begin(FilterConfig(FILTER_PT1, 20), _model.state.battery.timer.rate);
      _vFilter.begin(FilterConfig(FILTER_PT2, 2), _model.state.battery.timer.rate);
      _iFilterFast.begin(FilterConfig(FILTER_PT1, 20), _model.state.battery.timer.rate);
      _iFilter.begin(FilterConfig(FILTER_PT2, 2), _model.state.battery.timer.rate);
      break;
    default:
      break;
  }
  return 1;
}

int VoltageSensor::update()
{
  if (!_model.state.battery.timer.check()) return 0;

  Utils::Stats::Measure measure(_model.state.stats, COUNTER_BATTERY);

  switch (_state)
  {
    case VBAT:
      _state = IBAT;
      return readVbat();
    case IBAT:
      _state = VBAT;
      return readIbat();
  }

  return 0;
}

int VoltageSensor::readVbat()
{
#ifdef ESPFC_ADC_0
  if (_model.config.vbat.source != 1 || _model.config.pin[PIN_INPUT_ADC_0] == -1) return 0;
  // wemos d1 mini has divider 3.2:1 (220k:100k)
  // additionaly I've used divider 5.7:1 (4k7:1k)
  // total should equals ~18.24:1, 73:4 resDiv:resMult should be ideal,
  // but ~52:1 is real, did I miss something?
  _model.state.battery.rawVoltage = analogRead(_model.config.pin[PIN_INPUT_ADC_0]);
  float volts = _vFilterFast.update(_model.state.battery.rawVoltage * ESPFC_ADC_SCALE);

  volts *= _model.config.vbat.scale * 0.1f;
  volts *= _model.config.vbat.resMult;
  volts /= _model.config.vbat.resDiv;

  _model.state.battery.voltageUnfiltered = volts;
  _model.state.battery.voltage = _vFilter.update(_model.state.battery.voltageUnfiltered);

  // cell count detection
  if (_model.state.battery.samples > 0)
  {
    _model.state.battery.cells = std::ceil(_model.state.battery.voltage / 4.2f);
    _model.state.battery.samples--;
  }

  _model.state.battery.cellVoltage = _model.state.battery.voltage / std::clamp<int>(_model.state.battery.cells, 1, 6);
  _model.state.battery.percentage =
      std::clamp(Utils::map(_model.state.battery.cellVoltage, 3.4f, 4.2f, 0.0f, 100.0f), 0.0f, 100.0f);

  if (_model.config.debug.mode == DEBUG_BATTERY)
  {
    _model.state.debug[0] = std::clamp<long>(lrintf(_model.state.battery.voltageUnfiltered * 100.0f), 0L, 32000L);
    _model.state.debug[1] = std::clamp<long>(lrintf(_model.state.battery.voltage * 100.0f), 0L, 32000L);
  }
  return 1;
#else
  return 0;
#endif
}

int VoltageSensor::readIbat()
{
  auto& battery =
      _model.state.battery;

  if (_model.config.ibat.source ==
      CURRENT_METER_MSP)
  {
    constexpr uint32_t MSP_CURRENT_STALE_US =
        500000;

    const uint32_t now =
        micros();

    const bool fresh =
        battery.mspCurrentValid &&
        static_cast<uint32_t>(
            now -
            battery.mspCurrentLastUpdateUs) <
            MSP_CURRENT_STALE_US;

    if (!fresh)
    {
      // Never keep publishing an old companion current indefinitely. Current
      // telemetry is deliberately fail-silent and has no control authority.
      battery.mspCurrentValid =
          false;

      battery.currentUnfiltered =
          0.0f;

      battery.current =
          0.0f;

      _iFilter.prime(
          0.0f);

      return 0;
    }

    const float amps =
        static_cast<float>(
            battery.mspCurrentCentiAmps) *
        0.01f;

    battery.rawCurrent =
        battery.mspCurrentCentiAmps;

    battery.currentUnfiltered =
        amps;

    battery.current =
        _iFilter.update(
            amps);

    if (_model.config.debug.mode ==
        DEBUG_CURRENT_SENSOR)
    {
      _model.state.debug[0] =
          battery.mspCurrentCentiAmps;

      _model.state.debug[1] =
          std::clamp<long>(
              lrintf(
                  battery.current *
                  100.0f),
              -32000L,
              32000L);

      _model.state.debug[2] =
          static_cast<uint16_t>(
              battery.mspMahDrawn);
    }

    return 1;
  }

#ifdef ESPFC_ADC_1
  if (_model.config.ibat.source !=
          CURRENT_METER_ADC ||
      _model.config.pin[
          PIN_INPUT_ADC_1] == -1)
  {
    return 0;
  }

  battery.rawCurrent =
      analogRead(
          _model.config.pin[
              PIN_INPUT_ADC_1]);

  float volts =
      _iFilterFast.update(
          battery.rawCurrent *
          ESPFC_ADC_SCALE);

  const float milivolts =
      volts *
      1000.0f;

  volts +=
      _model.config.ibat.offset *
      0.001f;

  volts *=
      _model.config.ibat.scale *
      0.1f;

  battery.currentUnfiltered =
      volts;

  battery.current =
      _iFilter.update(
          battery.currentUnfiltered);

  if (_model.config.debug.mode ==
      DEBUG_CURRENT_SENSOR)
  {
    _model.state.debug[0] =
        lrintf(
            milivolts);

    _model.state.debug[1] =
        std::clamp<long>(
            lrintf(
                battery.currentUnfiltered *
                100.0f),
            -32000L,
            32000L);

    _model.state.debug[2] =
        battery.rawCurrent;
  }

  return 1;
#else
  return 0;
#endif
}

} // namespace Espfc::Sensor
