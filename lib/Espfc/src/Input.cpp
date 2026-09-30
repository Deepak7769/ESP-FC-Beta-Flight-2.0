
#include "Input.h"
#include "Control/AssistedModeV2.h"
#include "Hal/Time.hpp"
#include "ModelConfig.h"
#include "Utils/Filter.h"
#include "Utils/Math.hpp"
#include "Utils/MemoryHelper.h"

namespace Espfc {


Input::Input(Model& model, TelemetryManager& telemetry): _model(model), _telemetry(telemetry) {}

int Input::begin()
{
  _device = getInputDevice();
  _model.state.input.channelCount =
    std::min<size_t>(
        _device
            ? _device->getChannelCount()
            : 0u,
        INPUT_CHANNELS);

  _model.state.input.channelsValid =
      false;
  _model.state.input.frameTime =
      0;

  _model.state.input.frameTimeValid =
      false;

  _model.state.input.frameDelta =
      FRAME_TIME_DEFAULT_US;

  _model.state.input.frameRate =
      1000000ul /
      _model.state.input.frameDelta;

  _model.state.input.frameCount = 0;
  // -----------------------------------------------------
  // RECEIVER STARTUP STATE
  //
  // At boot the receiver has not yet been qualified.
  // This blocks arming but must NOT start Stage-2
  // failsafe or LAND.
  // -----------------------------------------------------

  _model.state.failsafe.phase =
      FC_FAILSAFE_IDLE;

  _model.state.failsafe.timeout =
      0;

  _model.state.failsafe.rxEverValid =
      false;

  _model.state.failsafe.recoveryActive =
      false;

  _model.state.failsafe.recoveryStartedUs =
      0;

    _model.state.failsafe.landingRequested =
      false;

  _model.state.failsafe.landingEligible =
      false;

  _model.state.failsafe.landingEstimatorHealthy =
      false;

  _model.state.failsafe.landingRequestedUs =
      0;

  _model.state.failsafe.landingEntryHeight =
      0.0f;

  _model.state.failsafe.landingEntryVario =
      0.0f;

  _model.state.failsafe.landingEntryThrust =
      0.0f;

    _model.state.failsafe.landingActive =
      false;

  _model.state.failsafe.landingLevelRequested =
      false;

  _model.state.failsafe.landingDescentRequested =
      false;

  _model.state.failsafe.landingFault =
      false;

  _model.state.failsafe.landingOutputBlocked =
      true;

  _model.state.failsafe.landingLastUpdateUs =
      0;

  _model.state.failsafe.landingTouchdownCandidate =
      false;

  _model.state.failsafe.landingTouchdownStartedUs =
      0;

  _model.state.input.rxLoss =
      true;

  _model.state.input.rxFailSafe =
      false;

  _model.state.input.lossTime =
      0;
  reload(MODEL_CHANGE_INPUT);

  for (size_t c = 0; c < INPUT_CHANNELS; ++c)
  {
    const int16_t v = c == AXIS_THRUST ? PWM_RANGE_MIN : PWM_RANGE_MID;
    _model.state.input.raw[c] = v;
    _model.state.input.buffer[c] = v;
    _model.state.input.bufferPrevious[c] = v;
    setInput((Axis)c, v, true, true);
  }
  return 1;
}

int Input::reload(ModelChangeEvent event)
{
  switch (event)
  {
    case MODEL_CHANGE_INPUT: {
      _model.state.input.autoFactor = 1.f / (2.f + _model.config.input.filterAutoFactor * 0.1f);
      _model.state.input.autoThrottleFactor = 1.f / (2.f + _model.config.input.filterAutoThrottleFactor * 0.1f);
      const FilterConfig rxFilter{_device && _device->needAverage() ? FILTER_FIR2 : FILTER_NONE, 1};
      const FilterConfig inputFilter{
    _model.config.input.filterEnable
        ? _model.config.input.filter
        : FilterConfig(FILTER_NONE, 0)
};

const FilterConfig throtleFilter{
    _model.config.input.filterEnable
        ? _model.config.input.filterThrottle
        : FilterConfig(FILTER_NONE, 0)
};
      for (size_t i = 0; i < AXIS_COUNT_RPYT; i++)
      {
        _filter[i].begin(rxFilter, 100); // rx filter uses FIR2 on NONE, sample rate doesn't really matter here
        if (i == AXIS_THRUST)
        {
          _model.state.input.filter[i].begin(throtleFilter, _model.state.input.timer.rate);
        }
        else
        {
          _model.state.input.filter[i].begin(inputFilter, _model.state.input.timer.rate);
        }
      }
      break;
    }
    default:
      break;
  }
  return 1;
}

int16_t FAST_CODE_ATTR Input::getFailsafeValue(uint8_t c)
{
  const InputChannelConfig& ich = _model.config.input.channel[c];
  switch (ich.fsMode)
  {
    case FAILSAFE_MODE_AUTO:
      return c == AXIS_THRUST ? PWM_RANGE_MIN : PWM_RANGE_MID;
    case FAILSAFE_MODE_SET:
      return ich.fsValue;
    case FAILSAFE_MODE_INVALID:
    case FAILSAFE_MODE_HOLD:
    default:
      return _model.state.input.buffer[c];
  }
}

void FAST_CODE_ATTR Input::setInput(
    Axis i,
    float v,
    bool newFrame,
    bool noFilter)
{
  if (i <= AXIS_THRUST)
  {
    const float nv =
        noFilter
            ? v
            : _model.state.input.filter[i].update(v);

    _model.state.input.us[i] = nv;

    // processInputs() converts receiver calibration to the
    // canonical 1000..2000 range.
    _model.state.input.ch[i] =
        Utils::map(
            nv,
            PWM_RANGE_MIN,
            PWM_RANGE_MAX,
            -1.f,
            1.f);
  }
  else if (newFrame)
  {
    _model.state.input.us[i] = v;

    _model.state.input.ch[i] =
        Utils::map(
            v,
            PWM_RANGE_MIN,
            PWM_RANGE_MAX,
            -1.f,
            1.f);
  }
}

int FAST_CODE_ATTR Input::update()
{
  if (!_device) return 0;

  uint32_t startTime = micros();

  InputStatus status = readInputs();

  if (!failsafe(status))
  {
    filterInputs(status);
  }

  if (_model.config.debug.mode == DEBUG_PIDLOOP)
  {
    _model.state.debug[1] = micros() - startTime;
  }

  return 1;
}

InputStatus FAST_CODE_ATTR Input::readInputs()
{
  Utils::Stats::Measure measure(_model.state.stats, COUNTER_INPUT_READ);
  uint32_t startTime = micros();

  InputStatus status = _device->update();

  if (_model.config.debug.mode == DEBUG_RX_TIMING)
  {
    _model.state.debug[0] = micros() - startTime;
  }

  if (status == INPUT_IDLE) return status;

  // Some receiver formats (notably PPM) discover their actual channel count
  // only after a complete, stable frame has been observed.
  _model.state.input.channelCount =
      std::min<size_t>(
          _device->getChannelCount(),
          INPUT_CHANNELS);

  _model.state.input.rxLoss = (status == INPUT_LOST || status == INPUT_FAILSAFE);
  _model.state.input.rxFailSafe = (status == INPUT_FAILSAFE);
  _model.state.input.frameCount++;

  processInputs();

  // frameTime represents the time of the most recent
  // structurally valid receiver frame.
  if (status == INPUT_RECEIVED &&
      _model.state.input.channelsValid)
  {
    updateFrameRate();
  }

  if (_model.config.debug.mode == DEBUG_RX_SIGNAL_LOSS)
  {
    _model.state.debug[0] = !_model.state.input.rxLoss;
    _model.state.debug[1] = _model.state.input.rxFailSafe;
    _model.state.debug[2] = _model.state.input.channelsValid;
    _model.state.debug[3] = _model.state.input.lossTime / (100 * 1000);
  }

  return status;
}

void FAST_CODE_ATTR Input::processInputs()
{
  if (_model.state.input.frameCount < 5) return; // ignore few first frames that might be garbage

  uint32_t startTime = micros();

 uint16_t channels[INPUT_CHANNELS] = {};

const size_t channelCount =
    std::min<size_t>(
        _model.state.input.channelCount,
        INPUT_CHANNELS);

_device->get(channels, channelCount);

bool channelsValid =
    channelCount >= AXIS_COUNT_RPYT;

for (size_t c = 0; c < channelCount; c++)
{
  const InputChannelConfig& ich =
      _model.config.input.channel[c];

  if (ich.map < 0 ||
      static_cast<size_t>(ich.map) >= channelCount)
  {
    const int16_t fallback =
        getFailsafeValue(c);

    _model.state.input.raw[c] =
        fallback;

    _model.state.input.bufferPrevious[c] =
        _model.state.input.buffer[c];

    _model.state.input.buffer[c] =
        fallback;

    channelsValid = false;
    continue;
  }

  int16_t v =
      _model.state.input.raw[c] =
          static_cast<int16_t>(
              channels[ich.map]);

// Preserve the existing global mid-RC correction.
v -=
    _model.config.input.midRc -
    PWM_RANGE_MID;

float t = PWM_RANGE_MID;

if (c == AXIS_THRUST)
{
  // Throttle is endpoint-calibrated; it has no center point.
  if (ich.min < ich.max)
  {
    t = Utils::map(
        (float)v,
        (float)ich.min,
        (float)ich.max,
        (float)PWM_RANGE_MIN,
        (float)PWM_RANGE_MAX);
  }
  else
  {
    t = PWM_RANGE_MIN;
    channelsValid = false;
  }
}
else
{
  // Roll/Pitch/Yaw/AUX use min-neutral-max calibration.
  if (ich.min < ich.neutral &&
      ich.neutral < ich.max)
  {
    t = Utils::map3(
        (float)v,
        (float)ich.min,
        (float)ich.neutral,
        (float)ich.max,
        (float)PWM_RANGE_MIN,
        (float)PWM_RANGE_MID,
        (float)PWM_RANGE_MAX);
  }
  else
  {
    t = PWM_RANGE_MID;
    channelsValid = false;
  }
}
    // filter if required
    t = _filter[c].update(t);
    v = lrintf(t);

    // apply deadband
    if (c < AXIS_THRUST)
    {
      v = Utils::deadband(v - PWM_RANGE_MID, (int)_model.config.input.deadband) + PWM_RANGE_MID;
    }

    // check if inputs are valid, apply failsafe value otherwise
    if (v < _model.config.input.minRc || v > _model.config.input.maxRc)
    {
      v = getFailsafeValue(c);
      if (c <= AXIS_THRUST) channelsValid = false;
    }

    // update input buffer
    _model.state.input.bufferPrevious[c] = _model.state.input.buffer[c];
    _model.state.input.buffer[c] = v;
  }
  _model.state.input.channelsValid = channelsValid;

  if (_model.config.debug.mode == DEBUG_RX_TIMING)
  {
    _model.state.debug[2] = micros() - startTime;
  }
}

bool FAST_CODE_ATTR Input::failsafe(
    InputStatus status)
{
  Utils::Stats::Measure measure(
      _model.state.stats,
      COUNTER_FAILSAFE);

  auto& failsafe =
      _model.state.failsafe;

  auto& input =
      _model.state.input;

  const uint32_t now =
      micros();

    // =====================================================
  // MANUALLY REQUESTED FAILSAFE / BOXFAILSAFE
  //
  // This check MUST happen before the valid-frame path.
  // Otherwise INPUT_RECEIVED returns before BOXFAILSAFE
  // is ever processed.
  // =====================================================

  if (_model.isSwitchActive(
          MODE_FAILSAFE) &&
      failsafe.rxEverValid)
  {
    failsafe.recoveryActive =
        false;

    failsafeStage2();

    // A BOXFAILSAFE request is authoritative even when the RF link itself is
    // still healthy. Do not continue filtering pilot sticks into the control
    // path after Stage 2 has taken ownership (especially AUTO_LAND).
    return true;
  }

  const bool validFrame =
      status == INPUT_RECEIVED &&
      input.channelsValid;

  // =====================================================
  // VALID RECEIVER FRAME
  // =====================================================

  if (validFrame)
  {
    // Start/restart receiver qualification.
    if (!failsafe.recoveryActive)
    {
      failsafe.recoveryActive =
          true;

      failsafe.recoveryStartedUs =
          now;
    }

    const uint32_t healthyForUs =
        static_cast<uint32_t>(
            now -
            failsafe.recoveryStartedUs);

    // ---------------------------------------------------
    // STARTUP:
    // receiver has never previously been qualified.
    // ---------------------------------------------------

    if (!failsafe.rxEverValid)
    {
      // Keep arming blocked until RX has remained healthy
      // for the complete qualification period.
      input.rxLoss =
          true;

      input.rxFailSafe =
          false;

      input.lossTime =
          0;

      if (healthyForUs <
          RX_RECOVERY_US)
      {
        return true;
      }

      // Receiver has now been continuously healthy long
      // enough to become authoritative.
      failsafe.rxEverValid =
          true;

      failsafe.recoveryActive =
          false;

      input.rxLoss =
          false;

      input.rxFailSafe =
          false;

      failsafeIdle();

      return false;
    }

    // ---------------------------------------------------
    // RECOVERY AFTER A REAL FAILSAFE
    // ---------------------------------------------------

    if (failsafe.phase !=
            FC_FAILSAFE_IDLE ||
        input.rxLoss ||
        input.rxFailSafe)
    {
#if defined(ESPFC_LAND_V2_ACTIVE)
      // -------------------------------------------------
      // RX RECOVERY DURING ACTIVE LAND
      //
      // A failsafe LAND is terminal for the current armed flight. We still
      // qualify the recovered receiver for diagnostics, but we do not hand
      // throttle authority back in mid-descent. This avoids a discontinuity
      // with stateful/accumulated transmitter throttle. LAND stays in control
      // through touchdown and disarm; the pilot can re-arm afterward.
      // -------------------------------------------------
      if (failsafe.phase ==
              FC_FAILSAFE_LANDING &&
          failsafe.landingRequested)
      {
        input.rxLoss =
            true;

        input.rxFailSafe =
            false;

        if (healthyForUs <
            RX_RECOVERY_US)
        {
          return true;
        }

        // Receiver is continuously healthy again, but pilot input remains
        // gated while LAND owns the current flight.
        input.rxLoss =
            false;

        input.rxFailSafe =
            false;

        return true;
      }
#endif

      failsafe.phase =
          FC_FAILSAFE_RX_LOSS_MONITORING;

      // Do not hand control back to the receiver yet.
      input.rxLoss =
          true;

      input.rxFailSafe =
          false;

      if (healthyForUs <
          RX_RECOVERY_US)
      {
        return true;
      }

      failsafe.recoveryActive =
          false;

      failsafe.phase =
          FC_FAILSAFE_RX_LOSS_RECOVERED;

      input.rxLoss =
          false;

      input.rxFailSafe =
          false;

      failsafeIdle();

      return false;
    }

    // Receiver was already healthy and remains healthy.
    failsafe.recoveryActive =
        false;

    input.rxLoss =
        false;

    input.rxFailSafe =
        false;

    input.lossTime =
        0;

    return false;
  }

  // =====================================================
  // RECEIVER QUALIFICATION INTERRUPTION
  // =====================================================

const bool recoveryInterrupted =
    (status == INPUT_RECEIVED &&
     !input.channelsValid) ||
    status == INPUT_DROPPED ||
    status == INPUT_LOST ||
    status == INPUT_FAILSAFE;

if (recoveryInterrupted)
{
  // Break the current continuous-healthy qualification
  // window. The MONITORING phase remains active, so input
  // stays blocked until a new full recovery period passes.
  failsafe.recoveryActive =
      false;
}
  // =====================================================
  // STARTUP WITH NO QUALIFIED RECEIVER
  //
  // This is the case:
  //
  //   drone ON
  //   transmitter OFF
  //
  // Keep motors/arming blocked, but DO NOT call Stage 1
  // or Stage 2. There has been no in-flight RX loss
  // because RX was never acquired in the first place.
  // =====================================================

  if (!failsafe.rxEverValid)
  {
    input.rxLoss =
        true;

    input.rxFailSafe =
        status == INPUT_FAILSAFE;

    input.lossTime =
        0;

    failsafe.phase =
        FC_FAILSAFE_IDLE;

    // One isolated receiver frame must not count toward
    // recovery forever. If no new valid frame has arrived
    // within the Stage-1 window, restart qualification.
    if (failsafe.recoveryActive)
    {
      const uint32_t frameAgeUs =
          static_cast<uint32_t>(
              now -
              input.frameTime);

      if (frameAgeUs >=
          RX_RECOVERY_GAP_US)
      {
        failsafe.recoveryActive =
            false;
      }
    }

    return true;
  }


  // =====================================================
  // RECEIVER-REPORTED FAILSAFE
  // =====================================================

  if (status == INPUT_FAILSAFE)
  {
    failsafe.recoveryActive =
        false;

    failsafeStage2();

    return true;
  }

  // =====================================================
  // NORMAL RX LOSS TIMEOUTS
  // =====================================================

  input.lossTime =
      static_cast<uint32_t>(
          now -
          input.frameTime);

  const uint32_t stage2Timeout =
      std::clamp<uint32_t>(
          _model.config.failsafe.delay,
          2u,
          200u) *
      TENTH_TO_US;

  // Stage 2
  if (input.lossTime >
      stage2Timeout)
  {
    failsafe.recoveryActive =
        false;

    failsafeStage2();

    return true;
  }

  // Stage 1
  if (input.lossTime >=
      RX_RECOVERY_GAP_US)
  {
    failsafe.recoveryActive =
        false;

    failsafeStage1();

    return true;
  }

// =====================================================
// RECOVERY INPUT GATE
//
// FC_FAILSAFE_RX_LOSS_MONITORING means receiver control
// has NOT yet been re-qualified.
//
// recoveryActive only tells us whether a continuous
// healthy-frame timing window is currently accumulating.
// An interrupted recovery deliberately clears that timer,
// but must NOT hand pilot input back to the controller.
// =====================================================

if (failsafe.phase ==
    FC_FAILSAFE_RX_LOSS_MONITORING)
{
  return true;
}

#if defined(ESPFC_LAND_V2_ACTIVE)
if (failsafe.phase ==
        FC_FAILSAFE_LANDING &&
    failsafe.landingRequested)
{
  return true;
}
#endif

return false;
}

void FAST_CODE_ATTR Input::failsafeIdle()
{
  _model.state.failsafe.phase = FC_FAILSAFE_IDLE;
  _model.state.input.lossTime = 0;
}

void FAST_CODE_ATTR Input::failsafeStage1()
{
  _model.state.failsafe.recoveryActive =
      false;

  _model.state.failsafe.phase =
      FC_FAILSAFE_RX_LOSS_DETECTED;

  _model.state.input.rxLoss =
      true;
  for (size_t i = 0; i < _model.state.input.channelCount; i++)
  {
    setInput((Axis)i, getFailsafeValue(i), true, true);
  }
}

void FAST_CODE_ATTR Input::failsafeStage2()
{
  auto& failsafe =
      _model.state.failsafe;

  auto& input =
      _model.state.input;

  failsafe.recoveryActive =
      false;

  input.rxLoss =
      true;

  input.rxFailSafe =
      true;

  // -----------------------------------------------------
  // ALREADY DISARMED
  //
  // Once Stage 2 has completed and entered LANDED,
  // repeated timeout/failsafe processing must not move
  // the state machine backwards.
  // -----------------------------------------------------

  if (!_model.isModeActive(
          MODE_ARMED))
  {
    if (failsafe.phase !=
        FC_FAILSAFE_LANDED)
    {
      failsafe.phase =
          FC_FAILSAFE_RX_LOSS_DETECTED;
    }

    return;
  }

  // Stage 2 is beginning from an armed state.
  failsafe.phase =
      FC_FAILSAFE_RX_LOSS_DETECTED;

  // =====================================================
  // AUTO-LAND REQUEST
  // =====================================================

  if (_model.config.failsafe.procedure ==
      FAILSAFE_PROCEDURE_AUTO_LAND)
  {
    // Edge-trigger the LAND request. Repeated Stage-2
    // processing must not erase supervisor state.
    if (!failsafe.landingRequested)
    {
      failsafe.landingRequested =
          true;

      failsafe.landingRequestedUs =
          micros();

      failsafe.landingEntryHeight =
          _model.state.altitude.height;

      failsafe.landingEntryVario =
          _model.state.altitude.vario;

      failsafe.landingEntryThrust =
          _model.state.output.ch[
              AXIS_THRUST];

      failsafe.landingEstimatorHealthy =
          false;

      failsafe.landingEligible =
          false;

      failsafe.landingActive =
          false;

      failsafe.landingLevelRequested =
          false;

      failsafe.landingDescentRequested =
          false;

      failsafe.landingFault =
          false;

      failsafe.landingOutputBlocked =
          true;

      failsafe.landingLastUpdateUs =
          0;

      failsafe.landingTouchdownCandidate =
          false;

      failsafe.landingTouchdownStartedUs =
          0;
    }

    failsafe.phase =
        FC_FAILSAFE_LANDING;
  }
  else
  {
    // ===================================================
    // DROP
    // ===================================================

    failsafe.landingRequested =
        false;

    failsafe.landingEstimatorHealthy =
        false;

    failsafe.landingEligible =
        false;

    failsafe.landingActive =
        false;

    failsafe.landingLevelRequested =
        false;

    failsafe.landingDescentRequested =
        false;

    failsafe.landingFault =
        false;

    failsafe.landingOutputBlocked =
        true;

    failsafe.landingRequestedUs =
        0;

    failsafe.landingLastUpdateUs =
        0;

    failsafe.landingTouchdownCandidate =
        false;

    failsafe.landingTouchdownStartedUs =
        0;

    failsafe.landingEntryHeight =
        0.0f;

    failsafe.landingEntryVario =
        0.0f;

    failsafe.landingEntryThrust =
        0.0f;
  }

#if defined(ESPFC_LAND_V2_ACTIVE)
  if (_model.config.failsafe.procedure ==
      FAILSAFE_PROCEDURE_AUTO_LAND)
  {
    // Active Assisted V2 keeps the aircraft logically armed so the LAND
    // controller can own attitude and vertical thrust until touchdown. In a
    // safe-bench build Mixer.cpp still prevents physical ESC attachment.
    return;
  }
#endif

  // =====================================================
  // OPERATIONAL FALLBACK
  //
  // Ordinary builds, and DROP in every build, disarm
  // immediately.
  // =====================================================

  failsafe.phase =
      FC_FAILSAFE_LANDED;

  _model.disarm(
      DISARM_REASON_FAILSAFE);
}

void FAST_CODE_ATTR Input::filterInputs(InputStatus status)
{
  Utils::Stats::Measure measure(_model.state.stats, COUNTER_INPUT_FILTER);
  uint32_t startTime = micros();

  const bool newFrame = status != INPUT_IDLE;

  for (size_t c = 0; c < _model.state.input.channelCount; c++)
  {
    const float v = _model.state.input.buffer[c];
    setInput((Axis)c, v, newFrame);
  }

  if (_model.config.debug.mode == DEBUG_RX_TIMING)
  {
    _model.state.debug[3] = micros() - startTime;
  }
}

void FAST_CODE_ATTR Input::updateFrameRate()
{
  auto& input =
      _model.state.input;

  const uint32_t now =
      micros();

  if (!input.frameTimeValid)
  {
    // Do not fold the uptime before the first qualified frame into the RX-rate
    // estimator. This is especially important when the transmitter is powered
    // on long after the flight controller.
    input.frameTime =
        now;

    input.frameTimeValid =
        true;

    return;
  }

  const uint32_t frameDelta =
      static_cast<uint32_t>(
          now -
          input.frameTime);

  input.frameTime =
      now;

  if (frameDelta >=
      RX_RECOVERY_GAP_US)
  {
    // A receiver outage is not a slow frame. Keep the last known frame-rate
    // estimate and use this frame only as the new timing baseline.
    return;
  }

  const int32_t deltaError =
      static_cast<int32_t>(
          frameDelta) -
      static_cast<int32_t>(
          input.frameDelta);

  input.frameDelta =
      std::max<uint32_t>(
          1u,
          static_cast<uint32_t>(
              static_cast<int32_t>(
                  input.frameDelta) +
              deltaError / 8));

  input.frameRate =
      1000000ul /
      input.frameDelta;

  if (_model.config.debug.mode == DEBUG_RC_SMOOTHING_RATE)
  {
    _model.state.debug[0] = input.frameDelta / 10;
    _model.state.debug[1] = input.frameRate;
  }

  // auto cutoff input freq
  float freq = std::clamp(input.frameRate * input.autoFactor, 15.f, 500.f);                 // no lower than 15Hz
  float throttleFreq = std::clamp(input.frameRate * input.autoThrottleFactor, 15.f, 500.f); // no lower than 15Hz
  if (freq > input.autoFreq * 1.1f || freq < input.autoFreq * 0.9f)
  {
    input.autoFreq += 0.25f * (freq - input.autoFreq);                         // lpf
    input.autoThrottleFreq += 0.25f * (throttleFreq - input.autoThrottleFreq); // lpf

    FilterConfig conf{(FilterType)_model.config.input.filter.type, std::clamp<int16_t>(input.autoFreq, 15, 500)};
    FilterConfig confThrottle{(FilterType)_model.config.input.filterThrottle.type,
                              std::clamp<int16_t>(input.autoThrottleFreq, 15, 500)};
    FilterConfig confDerivative{(FilterType)_model.config.input.filterDerivative.type,
                                std::clamp<int16_t>(input.autoFreq, 15, 500)};

for (size_t i = 0;
     i < AXIS_COUNT_RPY;
     i++)
{
  if (_model.config.input.filterEnable &&
      _model.config.input.filter.freq == 0)
  {
    _model.state.input.filter[i]
        .reconfigure(
            conf,
            input.timer.rate);
  }

  // Feed-forward derivative filter actually runs
  // in the PID loop, so loopTimer.rate is correct here.
  if (_model.config.input.filterDerivative.freq == 0)
  {
    _model.state.innerPid[i]
        .ftermFilter
        .reconfigure(
            confDerivative,
            _model.state.loopTimer.rate);
  }
}

if (_model.config.input.filterEnable &&
    _model.config.input.filterThrottle.freq == 0)
{
  _model.state.input
      .filter[AXIS_THRUST]
      .reconfigure(
          confThrottle,
          input.timer.rate);
}

    if (_model.config.debug.mode == DEBUG_RC_SMOOTHING_RATE)
    {
      _model.state.debug[2] = lrintf(freq);
      _model.state.debug[3] = lrintf(input.autoFreq);
      _model.state.debug[4] = lrintf(input.autoThrottleFreq);
    }
  }

  if (_model.config.debug.mode == DEBUG_RX_TIMING)
  {
    _model.state.debug[1] = micros() - now;
  }
}

Device::InputDevice* Input::getInputDevice()
{
  auto* serial = _model.getSerialStream(SERIAL_FUNCTION_RX_SERIAL);
  if (serial && _model.isFeatureActive(FEATURE_RX_SERIAL))
  {
    switch (_model.config.input.serialRxProvider)
    {
      case SERIALRX_IBUS:
        _ibus.begin(serial);
        _model.logger.info().logln("RX IBUS");
        return &_ibus;

      case SERIALRX_SBUS:
        _sbus.begin(serial);
        _model.logger.info().logln("RX SBUS");
        return &_sbus;

      case SERIALRX_CRSF:
        _crsf.begin(serial, _model.isFeatureActive(FEATURE_TELEMETRY) ? &_telemetry : nullptr);
        _model.logger.info().logln("RX CRSF");
        return &_crsf;
    }
  }
  else if (_model.isFeatureActive(FEATURE_RX_PPM) && _model.config.pin[PIN_INPUT_RX] != -1)
  {
    _ppm.begin(_model.config.pin[PIN_INPUT_RX], _model.config.input.ppmMode);
    _model.logger.info().log("RX PPM").log(_model.config.pin[PIN_INPUT_RX]).logln(_model.config.input.ppmMode);
    return &_ppm;
  }
#if defined(ESPFC_ESPNOW)
  else if (_model.isFeatureActive(FEATURE_RX_SPI))
  {
    int status = _espnow.begin();
    _model.logger.info().log("RX ESPNOW").logln(status);
    return &_espnow;
  }
#endif

  return nullptr;
}

} // namespace Espfc
