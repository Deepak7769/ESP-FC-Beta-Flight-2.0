#include "VoltageSensor.hpp"
#include "Hal/Time.hpp"

#include <algorithm>

namespace Espfc::Sensor {

VoltageSensor::VoltageSensor(Model& model): _model(model) {}

int VoltageSensor::begin()
{
  _model.state.battery.timer.setRate(100);
  _model.state.battery.samples = 50;
  _model.state.battery.cells = 0;
  _packAbsentReads = _packStableReads = 0;
  _packCandidateCells = 0;
  _packReferenceVoltage = 0.0f;

  _model.state.battery.mspCurrentCentiAmps = 0;
  _model.state.battery.mspMahDrawn = 0;
  _model.state.battery.mspCurrentLastUpdateUs = 0;
  _model.state.battery.mspCurrentValid = false;
  _model.state.battery.consumedMah = 0.0f;
  _model.state.battery.consumedMahValid = false;
  _lastAdcSampleUs = 0;
  _lastConsumedSource = _model.config.ibat.source;

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

  // Keep looking for the main pack even when USB powered at boot.
  updatePackDetection();

  _model.state.battery.cellVoltage = _model.state.battery.voltage / std::clamp<int>(_model.state.battery.cells, 1, 6);
  _model.state.battery.percentage =
      std::clamp(Utils::map(_model.state.battery.cellVoltage,
          _model.config.extension.cellMinimum * 0.01f,
          _model.config.extension.cellMaximum * 0.01f, 0.0f, 100.0f), 0.0f, 100.0f);
  if (_model.config.extension.capacityMah && _model.state.battery.consumedMahValid)
    _model.state.battery.percentage = std::clamp(100.0f *
        (1.0f - _model.state.battery.consumedMah / _model.config.extension.capacityMah), 0.0f, 100.0f);

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


void VoltageSensor::updatePackDetection()
{
  auto& battery = _model.state.battery;
  const float volts = battery.voltageUnfiltered;

  if (!std::isfinite(volts) || volts < 0.0f)
  {
    _packAbsentReads = 0;
    return;
  }

  // Once armed, never revise the selected cell count even if the initial
  // disarmed detection window has not yet completed.
  if (_model.isModeActive(MODE_ARMED))
  {
    _packAbsentReads = 0;
    return;
  }

  if (volts >= 2.5f)
  {
    _packAbsentReads = 0;
    if (battery.cells > 0) return; // fixed until a confirmed disconnect
    const int8_t candidate = static_cast<int8_t>(std::clamp<int>(
        static_cast<int>(std::ceil(volts / (_model.config.extension.cellMaximum * 0.01f))), 1, 6));
    if (candidate != _packCandidateCells || std::fabs(volts - _packReferenceVoltage) > 0.15f)
    {
      _packCandidateCells = candidate;
      _packReferenceVoltage = volts;
      _packStableReads = 1;
    }
    else if (_packStableReads < 8) ++_packStableReads;
    if (_packStableReads >= 8)
    {
      battery.cells = candidate;
      battery.samples = 0;
    }
    return;
  }
  _packStableReads = 0;
  _packCandidateCells = 0;

  // Never interpret an in-flight voltage sag as a pack replacement.
  if (battery.cells == 0 || volts >= 2.0f)
  {
    _packAbsentReads = 0;
    return;
  }

  // 50 consecutive VBAT observations (~1 s at the 50 Hz VBAT cadence)
  // below 2 V while disarmed are required before clearing pack state.
  if (++_packAbsentReads < 50u)
  {
    return;
  }

  _packAbsentReads = 0;
  battery.cells = 0;
  battery.samples = 50;
  battery.consumedMah = 0.0f;
  battery.consumedMahValid = false;
  battery.mspCurrentValid = false;
  battery.mspCurrentLastUpdateUs = 0;
  battery.mspMahDrawn = 0;
  battery.currentUnfiltered = 0.0f;
  battery.current = 0.0f;
  _lastAdcSampleUs = 0;
  _iFilter.prime(0.0f);
}

int VoltageSensor::readIbat()
{
  auto& battery =
      _model.state.battery;

  // Do not merge consumption accumulated by different meter sources.
  if (_lastConsumedSource != _model.config.ibat.source)
  {
    _lastConsumedSource = _model.config.ibat.source;
    _lastAdcSampleUs = 0;
    battery.consumedMah = 0.0f;
    battery.consumedMahValid = false;
  }

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
      battery.consumedMahValid = false;

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
    battery.consumedMah = static_cast<float>(battery.mspMahDrawn);
    battery.consumedMahValid = true;

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
#if defined(ESPFC_ADC_0)
  // With an ADC-voltage meter, an absent pack cannot produce valid
  // consumption even if the current ADC has an offset/noise reading.
  if (_model.config.vbat.source == 1 &&
      _model.config.pin[PIN_INPUT_ADC_0] != -1 &&
      battery.cells == 0)
  {
    battery.consumedMahValid = false;
    _lastAdcSampleUs = 0;
    battery.currentUnfiltered = 0.0f;
    battery.current = 0.0f;
    _iFilter.prime(0.0f);
    return 0;
  }
#endif
  if (_model.config.ibat.source !=
          CURRENT_METER_ADC ||
      _model.config.pin[
          PIN_INPUT_ADC_1] == -1)
  {
    battery.consumedMahValid = false;
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

  battery.currentUnfiltered =
      adcCurrentAmps(
          milivolts,
          _model.config.ibat.scale,
          _model.config.ibat.offset);

  battery.current =
      _iFilter.update(
          battery.currentUnfiltered);

  const uint32_t sampleUs = micros();
  if (_lastAdcSampleUs != 0)
  {
    battery.consumedMah = integrateConsumedMah(
        battery.consumedMah,
        battery.currentUnfiltered,
        static_cast<uint32_t>(sampleUs - _lastAdcSampleUs));
  }
  _lastAdcSampleUs = sampleUs;
  battery.consumedMahValid = true;

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
  battery.consumedMahValid = false;
  return 0;
#endif
}

} // namespace Espfc::Sensor
