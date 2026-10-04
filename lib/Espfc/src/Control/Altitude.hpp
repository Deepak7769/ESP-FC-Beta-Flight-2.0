#pragma once

#include "Model.h"
#include "Utils/Filter.h"
#include "Hal/Time.hpp"




#include <algorithm>
#include <cmath>

namespace Espfc::Control {

class Altitude
{
public:
Altitude(Model& model):
    _model(model),
    _heightInitialized(false),
    _filteredBaroValid(false),
    _filtersInitialized(false),
    _estimatorTimeValid(false),
    _baroTimeValid(false),
    _acceptedBaroTimeValid(false),
    _lastEstimatorUpdateUs(0),
    _lastBaroUpdateUs(0),
    _lastAcceptedBaroUs(0),
    _filteredBaroAlt(0.0f),
    _filteredBaroVario(0.0f)
{
}
  int begin()
  {
    auto& altitude =
        _model.state.altitude;

    altitude.height =
        0.0f;

    altitude.vario =
        0.0f;

    altitude.acceleration =
        0.0f;

    altitude.baroInnovation =
        0.0f;

    altitude.healthy =
        false;

    altitude.lastUpdateUs =
    0;

    altitude.baroAccepted =
        false;

    altitude.accelerationBias = 0.0f;
    altitude.baroBias = 0.0f;
    altitude.baroRateInnovation = 0.0f;
    altitude.rangefinderInnovation = 0.0f;
    altitude.estimatorState =
        ALTITUDE_ESTIMATOR_UNINITIALIZED;
    altitude.faultFlags = ALTITUDE_FAULT_NONE;
    altitude.baroAcceptedSamples = 0;
    altitude.baroRejectedSamples = 0;
    altitude.baroRateRejectedSamples = 0;
    altitude.baroConsecutiveRejects = 0;

    _heightInitialized =
        false;

    _filteredBaroValid =
        false;
    _estimatorTimeValid =
    false;

_baroTimeValid =
    false;
_filtersInitialized =
    false;

_acceptedBaroTimeValid =
    false;

_lastEstimatorUpdateUs =
    0;
    _lastBaroUpdateUs =
        0;

    _lastAcceptedBaroUs =
        0;

    _filteredBaroAlt =
        0.0f;

    _filteredBaroVario =
        0.0f;

    _accelBias = 0.0f;
    _baroBias = 0.0f;
    _rangefinderReference = 0.0f;
    _rangefinderReferenceValid = false;

    reload(
        MODEL_CHANGE_FILTER);

    return 1;
  }

int reload(ModelChangeEvent event)
{
  if (event != MODEL_CHANGE_FILTER)
  {
    return 1;
  }

  const int baroRate =
      std::max<int>(
          _model.state.baro.rate,
          1);

  const FilterConfig altitudeFilterConfig(
      FILTER_PT3,
      5);

  // begin() clears filter history, so only use it
  // for the first initialization.
  if (!_filtersInitialized)
  {
    _altitudeFilter.begin(
        altitudeFilterConfig,
        baroRate);

    _varioFilter.begin(
        altitudeFilterConfig,
        baroRate);

    _filtersInitialized =
        true;
  }
  else
  {
    // Reconfigure coefficients without destroying the
    // existing estimator history.
    _altitudeFilter.reconfigure(
        altitudeFilterConfig,
        baroRate);

    _varioFilter.reconfigure(
        altitudeFilterConfig,
        baroRate);
  }

  return 1;
}

int update(
    bool fusionValid = true)
  {
    Utils::Stats::Measure measure(
        _model.state.stats,
        COUNTER_IMU_FUSION2);

    auto& altitude = _model.state.altitude;
    const auto& baro = _model.state.baro;
    const auto& attitude = _model.state.attitude;
    const auto& range = _model.state.rangefinder;

    const int accelRate =
        std::max<int>(_model.state.accel.timer.rate, 1);
    const float nominalDt =
        1.0f / static_cast<float>(accelRate);
    const uint32_t now = micros();

    float dt = nominalDt;
    if (_estimatorTimeValid)
    {
      const uint32_t elapsedUs =
          static_cast<uint32_t>(
              now - _lastEstimatorUpdateUs);
      if (elapsedUs > 0)
      {
        dt = std::clamp(
            static_cast<float>(elapsedUs) * 0.000001f,
            nominalDt * 0.25f,
            nominalDt * 4.0f);
      }
    }

    _lastEstimatorUpdateUs = now;
    _estimatorTimeValid = true;
    altitude.lastUpdateUs = now;
    altitude.baroAccepted = false;
    altitude.baroRateAccepted = false;
    altitude.rangefinderUsed = false;
    altitude.rangefinderInnovation = 0.0f;
    altitude.faultFlags = ALTITUDE_FAULT_NONE;

    constexpr uint32_t BARO_STALE_US = 350000;
    constexpr uint32_t ACCEPTED_BARO_STALE_US = 500000;
    constexpr uint32_t ATTITUDE_STALE_US = 100000;
    constexpr uint32_t ACCEL_STALE_US = 50000;
    constexpr uint32_t RANGEFINDER_STALE_US = 100000;

    const bool baroFresh =
        baro.sampleValid &&
        static_cast<uint32_t>(
            now - baro.lastUpdateUs) <
        BARO_STALE_US;

    const bool attitudeFresh =
        attitude.healthy &&
        static_cast<uint32_t>(
            now - attitude.lastUpdateUs) <
        ATTITUDE_STALE_US;

    const bool accelFresh =
        _model.state.accel.present &&
        _model.state.accel.sampleValid &&
        static_cast<uint32_t>(
            now - _model.state.accel.lastUpdateUs) <
        ACCEL_STALE_US;

    const bool baroBiasReady =
        baro.altitudeBiasSamples < 0;

    const bool newBaroSample =
        baro.sampleValid &&
        (!_baroTimeValid ||
         baro.lastUpdateUs != _lastBaroUpdateUs);

    float baroDt =
        1.0f /
        static_cast<float>(
            std::max<int>(baro.rate, 1));

    if (newBaroSample)
    {
      if (_baroTimeValid)
      {
        baroDt = std::clamp(
            static_cast<float>(
                static_cast<uint32_t>(
                    baro.lastUpdateUs -
                    _lastBaroUpdateUs)) *
                0.000001f,
            0.001f,
            0.250f);
      }

      _lastBaroUpdateUs = baro.lastUpdateUs;
      _baroTimeValid = true;

      if (std::isfinite(baro.altitudeGround) &&
          std::isfinite(baro.vario))
      {
        _filteredBaroAlt =
            _altitudeFilter.update(baro.altitudeGround);
        _filteredBaroVario =
            _varioFilter.update(baro.vario);
        _filteredBaroValid =
            std::isfinite(_filteredBaroAlt) &&
            std::isfinite(_filteredBaroVario);
      }
      else
      {
        _filteredBaroValid = false;
        altitude.faultFlags |= ALTITUDE_FAULT_BARO_SAMPLE;
      }
    }

    const float accZ =
        _model.state.accel.world.z;
    const bool accelFinite =
        std::isfinite(accZ);

    const uint32_t projectionMaxAgeUs =
        std::max<uint32_t>(
            static_cast<uint32_t>(
                nominalDt * 3.0f * 1000000.0f),
            5000u);

    const bool accelProjectionFresh =
        attitudeFresh &&
        static_cast<uint32_t>(
            now - attitude.lastUpdateUs) <=
        projectionMaxAgeUs;

    const float safeAccZ =
        (fusionValid &&
         accelFinite &&
         accelProjectionFresh &&
         accelFresh)
            ? accZ - _accelBias
            : 0.0f;

    if (!fusionValid)
      altitude.faultFlags |= ALTITUDE_FAULT_FUSION;
    if (!attitudeFresh)
      altitude.faultFlags |= ALTITUDE_FAULT_ATTITUDE_STALE;
    if (!accelFresh)
      altitude.faultFlags |= ALTITUDE_FAULT_ACCEL_STALE;
    if (!baroFresh)
      altitude.faultFlags |= ALTITUDE_FAULT_BARO_STALE;

    const float previousVario =
        std::isfinite(altitude.vario)
            ? altitude.vario
            : 0.0f;

    const float predictedVario =
        previousVario +
        safeAccZ * dt;

    bool acceptedThisSample = false;
    bool initializedThisCycle = false;

    if (!_heightInitialized &&
        newBaroSample &&
        baroFresh &&
        baroBiasReady &&
        _filteredBaroValid)
    {
      altitude.height =
          _filteredBaroAlt - _baroBias;
      altitude.baroInnovation = 0.0f;
      altitude.baroRateInnovation = 0.0f;

      _heightInitialized = true;
      _lastAcceptedBaroUs = baro.lastUpdateUs;
      _acceptedBaroTimeValid = true;

      altitude.baroAccepted = true;
      altitude.baroRateAccepted = true;
      altitude.baroAcceptedSamples++;
      altitude.vario =
          std::isfinite(_filteredBaroVario)
              ? _filteredBaroVario
              : 0.0f;
      initializedThisCycle = true;
    }

    if (_heightInitialized &&
        !initializedThisCycle)
    {
      const float predictedHeight =
          altitude.height +
          previousVario * dt +
          0.5f * safeAccZ * dt * dt;

      if (newBaroSample &&
          baroFresh &&
          baroBiasReady &&
          _filteredBaroValid)
      {
        altitude.baroInnovation =
            _filteredBaroAlt -
            _baroBias -
            predictedHeight;

        altitude.baroRateInnovation =
            _filteredBaroVario -
            predictedVario;

        const float heightGate =
            std::clamp(
                static_cast<float>(
                    _model.config.altHold.baroInnovationGate) *
                0.1f,
                0.5f,
                5.0f);

        const float rateGate =
            std::clamp(
                static_cast<float>(
                    _model.config.altHold.baroRateInnovationGate) *
                0.1f,
                0.5f,
                10.0f);

        const bool heightAccepted =
            std::isfinite(altitude.baroInnovation) &&
            std::fabs(altitude.baroInnovation) <
            heightGate;

        const bool rateAccepted =
            std::isfinite(altitude.baroRateInnovation) &&
            std::fabs(altitude.baroRateInnovation) <
            rateGate;

        altitude.baroRateAccepted = rateAccepted;
        acceptedThisSample =
            heightAccepted &&
            rateAccepted;

        if (!heightAccepted)
          altitude.faultFlags |= ALTITUDE_FAULT_HEIGHT_REJECT;

        if (!rateAccepted)
        {
          altitude.faultFlags |= ALTITUDE_FAULT_RATE_REJECT;
          altitude.baroRateRejectedSamples++;
        }

        if (acceptedThisSample)
        {
          altitude.baroAcceptedSamples++;
          altitude.baroConsecutiveRejects = 0;
          altitude.baroAccepted = true;

          _lastAcceptedBaroUs = baro.lastUpdateUs;
          _acceptedBaroTimeValid = true;

          const float configuredTau =
              std::max(
                  static_cast<float>(
                      _model.config.altHold.baroTau) *
                  0.1f,
                  0.05f);

          const float alpha =
              std::clamp(
                  baroDt /
                  (configuredTau + baroDt),
                  0.0f,
                  1.0f);

          const bool propWashProtected =
              altitude.height <
                  static_cast<float>(
                      _model.config.altHold.groundEffectHeight) *
                  0.1f &&
              std::fabs(safeAccZ) >
                  static_cast<float>(
                      _model.config.altHold.propWashAccelThreshold) *
                  0.1f;

          altitude.height =
              predictedHeight +
              (propWashProtected ? alpha * 0.25f : alpha) *
              altitude.baroInnovation;

          const float beta =
              std::clamp(
                  baroDt /
                  (configuredTau + baroDt),
                  0.0f,
                  1.0f);

          if (!propWashProtected)
          {
            altitude.vario =
                predictedVario +
                beta * altitude.baroRateInnovation;
          }

          if (!propWashProtected &&
              std::fabs(previousVario) < 0.20f &&
              std::fabs(_filteredBaroVario) < 0.20f &&
              std::fabs(accZ) < 1.0f &&
              std::fabs(attitude.euler[AXIS_ROLL]) <
                  Utils::toRad(15.0f) &&
              std::fabs(attitude.euler[AXIS_PITCH]) <
                  Utils::toRad(15.0f))
          {
            const float learnRate =
                std::clamp(
                    static_cast<float>(
                        _model.config.altHold.hoverLearnRate) *
                    0.01f,
                    0.0f,
                    1.0f);

            const float biasAlpha =
                std::clamp(
                    dt * 0.10f * learnRate,
                    0.0f,
                    0.001f);

            _accelBias +=
                (accZ - _accelBias) *
                biasAlpha;

            _accelBias =
                std::clamp(
                    _accelBias,
                    -0.50f,
                    0.50f);

            const float baroBiasAlpha =
                std::clamp(
                    dt * 0.02f,
                    0.0f,
                    0.0005f);

            _baroBias +=
                std::clamp(
                    altitude.baroInnovation,
                    -0.25f,
                    0.25f) *
                baroBiasAlpha;

            _baroBias =
                std::clamp(
                    _baroBias,
                    -2.0f,
                    2.0f);
          }
        }
        else
        {
          altitude.baroRejectedSamples++;
          altitude.baroConsecutiveRejects =
              static_cast<uint16_t>(
                  std::min<uint32_t>(
                      static_cast<uint32_t>(
                          altitude.baroConsecutiveRejects) + 1u,
                      65535u));
          altitude.height = predictedHeight;
        }
      }
      else
      {
        altitude.height = predictedHeight;
      }
    }

    const bool rangeFresh =
        range.present &&
        range.sampleValid &&
        static_cast<uint32_t>(
            now - range.lastUpdateUs) <
        RANGEFINDER_STALE_US;

    const float rangeTiltCos =
        std::clamp(
            attitude.cosTheta,
            0.50f,
            1.0f);

    const bool rangeValid =
        rangeFresh &&
        range.quality > 0 &&
        std::isfinite(range.distance) &&
        range.distance >= 0.05f &&
        range.distance <= 10.0f &&
        attitudeFresh &&
        rangeTiltCos >= 0.50f;

    if (range.present &&
        (!range.sampleValid || !rangeFresh))
    {
      altitude.faultFlags |= ALTITUDE_FAULT_RANGEFINDER;
    }

    if (rangeValid &&
        !_rangefinderReferenceValid)
    {
      _rangefinderReference =
          range.distance * rangeTiltCos;
      _rangefinderReferenceValid = true;
    }

    if (rangeValid &&
        _rangefinderReferenceValid &&
        altitude.height <
            static_cast<float>(
                _model.config.altHold.groundEffectHeight) *
            0.1f + 0.5f)
    {
      const float rangeHeight =
          range.distance * rangeTiltCos -
          _rangefinderReference;

      const float rangePrediction =
          altitude.height +
          previousVario * dt +
          0.5f * safeAccZ * dt * dt;

      altitude.rangefinderInnovation =
          rangeHeight - rangePrediction;

      if (std::isfinite(altitude.rangefinderInnovation) &&
          std::fabs(altitude.rangefinderInnovation) <
          1.0f)
      {
        const float rangeAlpha =
            std::clamp(
                dt / (0.30f + dt),
                0.0f,
                0.50f);

        altitude.height +=
            rangeAlpha * altitude.rangefinderInnovation;
        altitude.rangefinderUsed = true;
      }
    }

    if (!acceptedThisSample &&
        !initializedThisCycle)
    {
      altitude.vario = predictedVario;
    }

    const bool acceptedBaroFresh =
        _acceptedBaroTimeValid &&
        static_cast<uint32_t>(
            now - _lastAcceptedBaroUs) <
        ACCEPTED_BARO_STALE_US;

    if (_heightInitialized &&
        !acceptedBaroFresh &&
        baroFresh &&
        baroBiasReady &&
        newBaroSample &&
        _filteredBaroValid &&
        !_model.isModeActive(MODE_ARMED))
    {
      altitude.height =
          _filteredBaroAlt - _baroBias;
      altitude.vario = 0.0f;
      altitude.baroInnovation = 0.0f;
      altitude.baroRateInnovation = 0.0f;
      altitude.baroAccepted = true;
      altitude.baroRateAccepted = true;
      _lastAcceptedBaroUs = baro.lastUpdateUs;
      _acceptedBaroTimeValid = true;
    }

    altitude.acceleration = safeAccZ;
    altitude.accelerationBias = _accelBias;
    altitude.baroBias = _baroBias;

    const bool numericHealthy =
        std::isfinite(altitude.height) &&
        std::isfinite(altitude.vario) &&
        std::isfinite(safeAccZ);

    if (!numericHealthy)
      altitude.faultFlags |= ALTITUDE_FAULT_NUMERIC;

    altitude.healthy =
        _heightInitialized &&
        _filteredBaroValid &&
        baroBiasReady &&
        baroFresh &&
        acceptedBaroFresh &&
        attitudeFresh &&
        accelProjectionFresh &&
        accelFresh &&
        numericHealthy &&
        fusionValid;

    if (!_heightInitialized)
      altitude.estimatorState = ALTITUDE_ESTIMATOR_UNINITIALIZED;
    else if (!numericHealthy || !fusionValid)
      altitude.estimatorState = ALTITUDE_ESTIMATOR_FAULT;
    else if (altitude.healthy &&
             altitude.baroConsecutiveRejects < 3)
      altitude.estimatorState = ALTITUDE_ESTIMATOR_TRACKING;
    else
      altitude.estimatorState = ALTITUDE_ESTIMATOR_DEGRADED;

    if (altitude.baroConsecutiveRejects >= 10)
      altitude.faultFlags |= ALTITUDE_FAULT_HEIGHT_REJECT;

    if (_model.config.debug.mode == DEBUG_ALTITUDE)
    {
      _model.state.debug[0] =
          std::clamp(lrintf(baro.altitudeGround * 100.0f), -32000l, 32000l);
      _model.state.debug[1] =
          std::clamp(lrintf(baro.vario * 100.0f), -32000l, 32000l);
      _model.state.debug[2] =
          std::clamp(lrintf(altitude.height * 100.0f), -32000l, 32000l);
      _model.state.debug[3] =
          std::clamp(lrintf(altitude.vario * 100.0f), -32000l, 32000l);
      _model.state.debug[4] =
          std::clamp(lrintf(altitude.baroInnovation * 100.0f), -32000l, 32000l);
      _model.state.debug[5] =
          static_cast<int16_t>(altitude.estimatorState);
      _model.state.debug[6] =
          static_cast<int16_t>(
              std::min<uint32_t>(altitude.baroRejectedSamples, 32000u));
      _model.state.debug[7] =
          static_cast<int16_t>(
              std::min<uint32_t>(altitude.baroRateRejectedSamples, 32000u));
    }

    return 1;
  }

private:
  Model& _model;

  Utils::Filter _altitudeFilter;
  Utils::Filter _varioFilter;



  // Estimator state
bool _heightInitialized;
bool _filteredBaroValid;
bool _filtersInitialized;

  // Timestamp validity flags.
  // These avoid treating micros()==0 as "invalid"
  // after the 32-bit timer wraps.
  bool _estimatorTimeValid;
  bool _baroTimeValid;
  bool _acceptedBaroTimeValid;

  // Timing
  uint32_t _lastEstimatorUpdateUs;
  uint32_t _lastBaroUpdateUs;
  uint32_t _lastAcceptedBaroUs;

  // Filtered barometer state
  float _filteredBaroAlt;
  float _filteredBaroVario;

  float _accelBias{0.0f};
  float _baroBias{0.0f};
  float _rangefinderReference{0.0f};
  bool _rangefinderReferenceValid{false};
};

} // namespace Espfc::Control
