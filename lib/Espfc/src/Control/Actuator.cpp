#include "Control/Actuator.h"
#include "Control/AssistedModeV2.h"
#include "Hal/Time.hpp"
#include "Input.h"
#include "Utils/Math.hpp"

#include <algorithm>

#include <cmath>

namespace Espfc::Control {

namespace {

bool inputChannelAvailable(
    const Model& model,
    size_t channel)
{
  if (channel >= AXIS_COUNT)
  {
    return false;
  }

  const auto& input =
      model.state.input;

  if (input.channelCount > 0)
  {
    return channel <
        input.channelCount;
  }

  // Before Input::begin() the receiver width is unknown. After Input::begin()
  // a missing receiver is represented by channelCount == 0 and rxLoss == true,
  // so default/stale 1500-us AUX slots must not act like real switches.
  return !input.rxLoss;
}

constexpr uint8_t MODE_LOGIC_OR =
    0;

constexpr uint8_t MODE_LOGIC_AND =
    1;

bool modeConditionHasLink(
    const ActuatorCondition& condition)
{
  return
      condition.linkId != 0;
}

bool modeHasLinkedCondition(
    const Model& model,
    uint8_t modeId)
{
  if (modeId >= MODE_COUNT)
  {
    return false;
  }

  for (size_t i = 0;
       i < ACTUATOR_CONDITIONS;
       ++i)
  {
    const auto& candidate =
        model.config.conditions[i];

    if (candidate.id == modeId &&
        candidate.linkId != 0)
    {
      return true;
    }
  }

  return false;
}

bool modeConditionLinkValid(
    const Model& model,
    const ActuatorCondition& condition)
{
  // Betaflight rejects linked ARM rows and linked-to-linked chains. Keep the
  // same safety invariant here at runtime so malformed/stale configuration
  // cannot turn a linked row into an unexpected mode request.
  if (!modeConditionHasLink(
          condition) ||
      condition.id >= MODE_COUNT ||
      condition.id == MODE_ARMED ||
      condition.linkId >= MODE_COUNT ||
      condition.linkId == condition.id)
  {
    return false;
  }

  return
      !modeHasLinkedCondition(
          model,
          condition.linkId);
}

void updateMasksForCondition(
    const ActuatorCondition& condition,
    bool active,
    uint32_t& andMask,
    uint32_t& newMask)
{
  if (condition.id >= MODE_COUNT)
  {
    return;
  }

  const uint32_t bit =
      uint32_t{1} <<
      condition.id;

  // Match Betaflight's mode-range semantics:
  // OR rows latch a mode active when any OR row is active.
  // AND rows keep the mode active only when every AND row is active.
  if ((andMask & bit) ||
      !(newMask & bit))
  {
    const bool useAnd =
        condition.logicMode ==
        MODE_LOGIC_AND;

    if (!useAnd)
    {
      if (active)
      {
        andMask &=
            ~bit;

        newMask |=
            bit;
      }
    }
    else
    {
      andMask |=
          bit;

      if (!active)
      {
        newMask |=
            bit;
      }
    }
  }
}

#if defined(ESPFC_ALTHOLD_V2_ACTIVE)
bool altHoldPilotStickValid(
    const Model& model)
{
  constexpr size_t PILOT_CHANNEL =
      static_cast<size_t>(
          ESPFC_ALTHOLD_V2_CENTERED_STICK_CHANNEL);

  static_assert(
      PILOT_CHANNEL < AXIS_COUNT,
      "AltHold V2 centered-stick channel exceeds input channel count");

  constexpr float ENTRY_CENTER_WINDOW =
      0.15f;

  const auto& input =
      model.state.input;

  // The dedicated spring-centered vertical command must actually exist in
  // the current receiver frame.  A missing AUX channel otherwise retains
  // its zero-initialized value and can look falsely centered.
  if (!input.channelsValid ||
      input.channelCount <= PILOT_CHANNEL)
  {
    return false;
  }

  const auto& channelConfig =
      model.config.input.channel[
          PILOT_CHANNEL];

  if (channelConfig.map < 0 ||
      static_cast<size_t>(
          channelConfig.map) >=
          input.channelCount)
  {
    return false;
  }

  // processInputs() stores the receiver sample in raw[] before replacing an
  // invalid AUX sample with its configured failsafe value.  Validate that
  // original sample here so a broken/missing spring-stick channel cannot be
  // mistaken for a centered 1500-us fallback.
  const int32_t correctedRaw =
      static_cast<int32_t>(
          input.raw[
              PILOT_CHANNEL]) -
      (model.config.input.midRc -
       PWM_RANGE_MID);

  if (correctedRaw <
          model.config.input.minRc ||
      correctedRaw >
          model.config.input.maxRc)
  {
    return false;
  }

  return
      std::isfinite(
          input.ch[
              PILOT_CHANNEL]);
}

bool altHoldPilotStickCentered(
    const Model& model)
{
  constexpr size_t PILOT_CHANNEL =
      static_cast<size_t>(
          ESPFC_ALTHOLD_V2_CENTERED_STICK_CHANNEL);

  constexpr float ENTRY_CENTER_WINDOW =
      0.15f;

  if (!altHoldPilotStickValid(
          model))
  {
    return false;
  }

  return
      std::fabs(
          model.state.input.ch[
              PILOT_CHANNEL]) <=
      ENTRY_CENTER_WINDOW;
}

bool assistedVerticalControlOwnsThrust(
    const Model& model)
{
  const bool altHold =
      model.isModeActive(
          MODE_ALTHOLD);

#if defined(ESPFC_LAND_V2_ACTIVE)
  const bool land =
      model.state.failsafe.landingRequested &&
      model.state.failsafe.phase ==
          FC_FAILSAFE_LANDING;
#else
  constexpr bool land =
      false;
#endif

  return
      altHold ||
      land;
}
#endif

} // namespace

Actuator::Actuator(Model& model): _model(model) {}

int Actuator::begin()
{
  _model.state.mode.mask = 0;
  _model.state.mode.maskPrev = 0;
  _model.state.mode.maskPresent = 0;
  _model.state.mode.maskSwitch = 0;
  for (size_t i = 0;
       i < ACTUATOR_CONDITIONS;
       ++i)
  {
    const auto& c =
        _model.config.conditions[i];

    if (c.id >= MODE_COUNT)
    {
      continue;
    }

    const bool hasLink =
        modeConditionHasLink(c);

    const bool linked =
        hasLink &&
        modeConditionLinkValid(
            _model,
            c);

    // Invalid link rows are ignored as rows, rather than falling back to
    // their AUX range. This mirrors Betaflight's configuration sanitization
    // for linked ARM targets and linked-to-linked chains.
    if (hasLink &&
        !linked)
    {
      continue;
    }

    bool rangeConfigured =
        c.min < c.max &&
        c.ch >= AXIS_AUX_1 &&
        c.ch < AXIS_COUNT;

    if (rangeConfigured &&
        _model.state.input.channelCount > 0 &&
        static_cast<size_t>(c.ch) >=
            _model.state.input.channelCount)
    {
      rangeConfigured =
          false;
    }

    // Linked-only conditions have no AUX range, but still make the target
    // mode present to the runtime/configurator.
    if (!rangeConfigured &&
        !linked)
    {
      continue;
    }

    _model.state.mode.maskPresent |=
        (uint32_t{1} << c.id);
  }
  _model.state.mode.airmodeAllowed = false;
  _model.state.mode.rescueConfigMode = RESCUE_CONFIG_PENDING;
  _angleFaultLatched =
    false;

_altHoldFaultLatched =
    false;
  return 1;
}

int Actuator::update()
{
  uint32_t startTime = micros();
  Utils::Stats::Measure measure(_model.state.stats, COUNTER_ACTUATOR);
  updateArmingDisabled();
  updateModeMask();
  updateArmed();

  updateFailsafeLand();

  updateAirMode();
  updateScaler();
  updateBuzzer();
  updateDynLpf();
  updateRescueConfig();
  updateLed();

  if (_model.config.debug.mode == DEBUG_PIDLOOP)
  {
    _model.state.debug[4] = micros() - startTime;
  }

  return 1;
}

void Actuator::updateScaler()
{
  // -----------------------------------------------------
  // RESET TRANSIENT RUNTIME SCALERS
  // -----------------------------------------------------

  for (size_t axis = 0;
       axis < AXIS_COUNT_RPYT;
       ++axis)
  {
    auto& inner =
        _model.state.innerPid[axis];

    inner.pScale = 1.0f;
    inner.iScale = 1.0f;
    inner.dScale = 1.0f;
    inner.fScale = 1.0f;
  }

  for (size_t axis = 0;
       axis < AXIS_COUNT_RP;
       ++axis)
  {
    _model.state.angleV2
        .pScale[axis] =
        1.0f;
  }

  // -----------------------------------------------------
  // APPLY CONFIGURED AUX SCALERS
  // -----------------------------------------------------

  for (size_t i = 0;
       i < SCALER_COUNT;
       ++i)
  {
    const uint32_t mode =
        _model.config.scaler[i]
            .dimension;

    if (!mode)
    {
      continue;
    }

    const short channel =
        _model.config.scaler[i]
            .channel;

    if (channel < AXIS_AUX_1 ||
        channel >= AXIS_COUNT)
    {
      continue;
    }

    if (!inputChannelAvailable(
            _model,
            static_cast<size_t>(
                channel)))
    {
      continue;
    }

    const float input =
        _model.state.input.ch[
            channel];

    const float minScale =
        _model.config.scaler[i]
            .minScale *
        0.01f;

    const float maxScale =
        _model.config.scaler[i]
            .maxScale *
        0.01f;

    const float scale =
        Utils::map3(
            input,
            -1.0f,
            0.0f,
            1.0f,
            minScale,
            minScale < 0.0f
                ? 0.0f
                : 1.0f,
            maxScale);

    for (size_t axis = 0;
         axis < AXIS_COUNT_RPYT;
         ++axis)
    {
      const bool selected =
          (axis == AXIS_ROLL &&
           (mode & ACT_AXIS_ROLL)) ||
          (axis == AXIS_PITCH &&
           (mode & ACT_AXIS_PITCH)) ||
          (axis == AXIS_YAW &&
           (mode & ACT_AXIS_YAW)) ||
          (axis == AXIS_THRUST &&
           (mode & ACT_AXIS_THRUST));

      if (!selected)
      {
        continue;
      }

      // Existing rate/vertical PID scaling.
      if (mode & ACT_INNER_P)
      {
        _model.state.innerPid[
            axis].pScale =
            scale;
      }

      if (mode & ACT_INNER_I)
      {
        _model.state.innerPid[
            axis].iScale =
            scale;
      }

      if (mode & ACT_INNER_D)
      {
        _model.state.innerPid[
            axis].dScale =
            scale;
      }

      if (mode & ACT_INNER_F)
      {
        _model.state.innerPid[
            axis].fScale =
            scale;
      }

      // Angle V2 has its own outer-loop gain.
      if (axis < AXIS_COUNT_RP &&
          (mode & ACT_ANGLE_P))
      {
        _model.state.angleV2
            .pScale[axis] =
            scale;
      }
    }
  }
}

void Actuator::updateArmingDisabled()
{
  int errors = _model.state.i2cErrorDelta;
  _model.state.i2cErrorDelta = 0;

  constexpr uint32_t
      GYRO_STALE_US =
          100000;

  const uint32_t now =
      micros();

  const bool gyroFresh =
      _model.state.gyro.present &&
      _model.state.gyro.sampleValid &&
      static_cast<uint32_t>(
          now -
          _model.state.gyro.lastUpdateUs) <
          GYRO_STALE_US;

  _model.setArmingDisabled(
      ARMING_DISABLED_NO_GYRO,
      !gyroFresh ||
      errors);

  // A quadrotor has no safe attitude-control fallback after the sole gyro
  // stops updating. Do not leave the previous motor command resident.
  if (_model.isModeActive(
          MODE_ARMED) &&
      !gyroFresh)
  {
    _model.disarm(
        DISARM_REASON_SYSTEM);
  }
  _model.setArmingDisabled(ARMING_DISABLED_FAILSAFE, _model.state.failsafe.phase != FC_FAILSAFE_IDLE);
  _model.setArmingDisabled(ARMING_DISABLED_RX_FAILSAFE, _model.state.input.rxLoss || _model.state.input.rxFailSafe);
  _model.setArmingDisabled(ARMING_DISABLED_THROTTLE, !_model.isThrottleLow());
  _model.setArmingDisabled(ARMING_DISABLED_CALIBRATING, _model.calibrationActive());
  _model.setArmingDisabled(ARMING_DISABLED_MOTOR_PROTOCOL, _model.config.output.protocol == ESC_PROTOCOL_DISABLED);
  _model.setArmingDisabled(
    ARMING_DISABLED_REBOOT_REQUIRED,
    _model.getRebootRequired() ||
        _model.state.mode.rescueConfigMode == RESCUE_CONFIG_ACTIVE);

  // Check small angle - prevent arming if tilted beyond configured angle
if (_model.config.arming.smallAngle < 180.0f &&
    _model.accelActive())
{
  bool angleUnsafe =
      !attitudeEstimateHealthy();

  if (!angleUnsafe)
  {
    const float maxTiltRad =
        Utils::toRad(
            _model.config.arming.smallAngle);

    const float roll =
        _model.state.attitude.euler[
            AXIS_ROLL];

    const float pitch =
        _model.state.attitude.euler[
            AXIS_PITCH];

    if (!std::isfinite(roll) ||
        !std::isfinite(pitch))
    {
      angleUnsafe =
          true;
    }
    else
    {
      const float currentTilt =
          std::max(
              std::fabs(roll),
              std::fabs(pitch));

      angleUnsafe =
          currentTilt >
          maxTiltRad;
    }
  }

  _model.setArmingDisabled(
      ARMING_DISABLED_ANGLE,
      angleUnsafe);
}
else
{
  _model.setArmingDisabled(
      ARMING_DISABLED_ANGLE,
      false);
}
  if (_model.isFeatureActive(FEATURE_GPS))
  {
    _model.setArmingDisabled(
        ARMING_DISABLED_GPS,
        !_model.state.gps.present ||
        _model.state.gps.numSats <
            _model.config.gps.minSats);
  }
  else
  {
    // Do not retain a stale GPS arming block after the feature is disabled.
    _model.setArmingDisabled(
        ARMING_DISABLED_GPS,
        false);
  }
}

void Actuator::updateModeMask()
{
  uint32_t newMask =
      0;

  uint32_t andMask =
      0;

  // -----------------------------------------------------
  // PASS 1: PHYSICAL AUX-RANGE CONDITIONS
  // -----------------------------------------------------
  //
  // Linked rows are intentionally deferred to pass 2, exactly like
  // Betaflight. This makes a linked row depend on another mode's resolved
  // request rather than on an unrelated/stale AUX sample.
  // -----------------------------------------------------

  for (size_t i = 0;
       i < ACTUATOR_CONDITIONS;
       ++i)
  {
    const ActuatorCondition& condition =
        _model.config.conditions[i];

    if (condition.id >= MODE_COUNT ||
        modeConditionHasLink(
            condition))
    {
      // Linked rows are handled only by pass 2. Invalid links are also
      // ignored here instead of being reinterpreted as physical AUX rows.
      continue;
    }

    if (condition.min >=
        condition.max)
    {
      continue;
    }

    const size_t channel =
        condition.ch;

    if (channel < AXIS_AUX_1 ||
        channel >= AXIS_COUNT)
    {
      continue;
    }

    if (!inputChannelAvailable(
            _model,
            channel))
    {
      // A configured AUX condition must never become active from the default
      // 1500-us contents of a channel the receiver does not actually provide.
      continue;
    }

    const int16_t value =
        _model.state.input.us[
            channel];

    const bool active =
        value > condition.min &&
        value < condition.max;

    updateMasksForCondition(
        condition,
        active,
        andMask,
        newMask);
  }

  // -----------------------------------------------------
  // PASS 2: MODE-LINK CONDITIONS
  // -----------------------------------------------------
  //
  // Example:
  //   target id = MODE_ANGLE
  //   linkId    = MODE_ALTHOLD
  //
  // means "this ANGLE condition is active whenever ALTHOLD is requested".
  // -----------------------------------------------------

  for (size_t i = 0;
       i < ACTUATOR_CONDITIONS;
       ++i)
  {
    const ActuatorCondition& condition =
        _model.config.conditions[i];

    if (!modeConditionLinkValid(
            _model,
            condition))
    {
      continue;
    }

    const uint32_t sourceBit =
        uint32_t{1} <<
        condition.linkId;

    const bool linkedModeActive =
        (andMask & sourceBit) !=
        (newMask & sourceBit);

    updateMasksForCondition(
        condition,
        linkedModeActive,
        andMask,
        newMask);
  }

  // Betaflight keeps AND bookkeeping in a separate mask while all rows are
  // evaluated. XOR resolves the requested mode state after both passes.
  newMask ^=
      andMask;

  _model.updateSwitchActive(
      newMask);
  // -----------------------------------------------------
  // ASSISTED-MODE SUPERVISOR
  // -----------------------------------------------------

  constexpr uint32_t ANGLE_BIT =
      uint32_t{1} <<
      MODE_ANGLE;

  constexpr uint32_t ALTHOLD_BIT =
      uint32_t{1} <<
      MODE_ALTHOLD;

  const bool angleRequested =
      (newMask & ANGLE_BIT) != 0;

  const bool altHoldRequested =
      (newMask & ALTHOLD_BIT) != 0;

  const bool angleHealthy =
      attitudeEstimateHealthy();

const bool altHoldHealthy =
    altitudeEstimateHealthy();

#if defined(ESPFC_ALTHOLD_V2_ACTIVE) || \
    defined(ESPFC_LAND_V2_ACTIVE)
  // If the pilot explicitly requests AltHold, do not allow an ARM transition
  // without a trustworthy vertical estimate. Likewise, an AUTO_LAND
  // failsafe configuration is only meaningful when the aircraft is armed
  // with a healthy altitude estimator.
  const bool assistedAltitudeRequired =
      altHoldRequested
#if defined(ESPFC_LAND_V2_ACTIVE)
      ||
      (_model.config.failsafe.procedure ==
           FAILSAFE_PROCEDURE_AUTO_LAND &&
       (newMask &
        (uint32_t{1} << MODE_ARMED)))
#endif
      ;

  const bool altHoldWasActive =
      _model.isModeActive(
          MODE_ALTHOLD);

  const bool altHoldPilotValid =
      !altHoldRequested ||
      altHoldPilotStickValid(
          _model);

  // Centering is an entry-only gate. Once AltHold is active, stick deflection
  // is the intended climb/descent command and must not be treated as a fault.
  const bool altHoldEntryUnsafe =
      altHoldRequested &&
      !altHoldWasActive &&
      !altHoldPilotStickCentered(
          _model);

  _model.setArmingDisabled(
      ARMING_DISABLED_ALTHOLD,
      (assistedAltitudeRequired &&
       !altHoldHealthy) ||
      !altHoldPilotValid ||
      altHoldEntryUnsafe);
#else
  constexpr bool altHoldPilotValid =
      true;

  _model.setArmingDisabled(
      ARMING_DISABLED_ALTHOLD,
      false);
#endif


  // -----------------------------------------------------
  // ANGLE fault latch
  // -----------------------------------------------------

  if (!angleRequested)
  {
    // Switch OFF clears the previous fault.
    _angleFaultLatched =
        false;
  }
  else if (!angleHealthy)
  {
    // Once the estimator fails, do not automatically
    // re-enter Angle mode while the switch remains ON.
    _angleFaultLatched =
        true;
  }

  if (_angleFaultLatched ||
      !angleHealthy)
  {
    newMask &=
        ~ANGLE_BIT;
  }


  // -----------------------------------------------------
  // ALT HOLD fault latch
  // -----------------------------------------------------

  if (!altHoldRequested)
  {
    // Switch OFF clears the previous fault.
    _altHoldFaultLatched =
        false;
  }
  else if (!altHoldHealthy ||
           !altHoldPilotValid)
  {
    // Sensor/estimator or dedicated vertical-channel failure requires a
    // deliberate OFF -> ON switch cycle before re-entry.
    _altHoldFaultLatched =
        true;
  }

  if (_altHoldFaultLatched ||
      !altHoldHealthy ||
      !altHoldPilotValid)
  {
    newMask &=
        ~ALTHOLD_BIT;
  }
  _model.setArmingDisabled(ARMING_DISABLED_FAILSAFE, _model.state.failsafe.phase != FC_FAILSAFE_IDLE);
  _model.setArmingDisabled(ARMING_DISABLED_BOXFAILSAFE, _model.isSwitchActive(MODE_FAILSAFE));
  _model.setArmingDisabled(ARMING_DISABLED_ARM_SWITCH, _model.armingDisabled() && _model.isSwitchActive(MODE_ARMED));

  if (_model.state.failsafe.phase != FC_FAILSAFE_IDLE)
  {
    newMask |= (1 << MODE_FAILSAFE);
  }

  for (size_t i = 0; i < MODE_COUNT; i++)
  {
    bool newVal = newMask & (1 << i);
    bool oldVal = _model.state.mode.mask & (1 << i);
    if (newVal == oldVal) continue; // mode unchanged
    if (newVal && !canActivateMode((FlightMode)i))
    {
      newMask &= ~(1 << i); // block activation, clear bit
    }
  }

  _model.updateModes(newMask);
}
bool Actuator::attitudeEstimateHealthy() const
{
  const auto& attitude =
      _model.state.attitude;

  if (!_model.gyroActive() ||
      !_model.accelActive() ||
      !attitude.healthy)
  {
    return false;
  }

  constexpr uint32_t
      ATTITUDE_STALE_US =
          100000;

  const uint32_t age =
      static_cast<uint32_t>(
          micros() -
          attitude.lastUpdateUs);

  if (age >= ATTITUDE_STALE_US)
  {
    return false;
  }

  const auto& q =
      attitude.quaternion;

  const bool finite =
      std::isfinite(
          attitude.euler[AXIS_ROLL]) &&
      std::isfinite(
          attitude.euler[AXIS_PITCH]) &&
      std::isfinite(q.w) &&
      std::isfinite(q.x) &&
      std::isfinite(q.y) &&
      std::isfinite(q.z);

  if (!finite)
  {
    return false;
  }

  const float normSq =
      q.w * q.w +
      q.x * q.x +
      q.y * q.y +
      q.z * q.z;

  return
      std::isfinite(normSq) &&
      normSq > 0.5f &&
      normSq < 1.5f;
}
bool Actuator::altitudeEstimateHealthy() const
{
  const auto& altitude =
      _model.state.altitude;

  const auto& baro =
      _model.state.baro;

  if (!_model.baroActive() ||
      !altitude.healthy ||
      !baro.sampleValid)
  {
    return false;
  }

  if (!attitudeEstimateHealthy())
  {
    return false;
  }

  constexpr uint32_t
      ALTITUDE_STALE_US =
          100000;

  constexpr uint32_t
      BARO_STALE_US =
          350000;

  const uint32_t now =
      micros();

  const uint32_t altitudeAge =
      static_cast<uint32_t>(
          now -
          altitude.lastUpdateUs);

  const uint32_t baroAge =
      static_cast<uint32_t>(
          now -
          baro.lastUpdateUs);

  if (altitudeAge >=
          ALTITUDE_STALE_US ||
      baroAge >=
          BARO_STALE_US)
  {
    return false;
  }

  return
      std::isfinite(
          altitude.height) &&
      std::isfinite(
          altitude.vario);
}

void Actuator::updateFailsafeLand()
{
  auto& failsafe =
      _model.state.failsafe;

  const uint32_t now =
      micros();

#if defined(ESPFC_LAND_V2_ACTIVE)
  const auto& altitude =
      _model.state.altitude;

  const bool activeLandRequest =
      failsafe.landingRequested &&
      failsafe.phase ==
          FC_FAILSAFE_LANDING &&
      _model.isModeActive(
          MODE_ARMED);
#endif

  // The logical LAND controller starts blocked. It is cleared only while a
  // healthy active LAND request owns the controller. Safe-bench builds still
  // block physical ESC attachment independently in Mixer.cpp.
  failsafe.landingOutputBlocked =
      true;

  // -----------------------------------------------------
  // NO LAND REQUEST
  // -----------------------------------------------------

  if (!failsafe.landingRequested)
  {
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

    // landingFault is a lifecycle diagnostic latch. It is cleared when
    // a new LAND request begins or when a new arm starts, not merely because
    // the request has just terminated after timeout/estimator failure.
    failsafe.landingLastUpdateUs =
        0;

    failsafe.landingTouchdownCandidate =
        false;

    failsafe.landingTouchdownStartedUs =
        0;
  }
  else
  {
    // ---------------------------------------------------
    // CONTINUOUS ESTIMATOR VALIDATION
    // ---------------------------------------------------

    const bool estimatorHealthy =
        altitudeEstimateHealthy();

    failsafe.landingEstimatorHealthy =
        estimatorHealthy;

    failsafe.landingEligible =
        failsafe.rxEverValid &&
        estimatorHealthy;

    failsafe.landingActive =
        failsafe.landingEligible;

    failsafe.landingLevelRequested =
        failsafe.landingActive;

    failsafe.landingDescentRequested =
        failsafe.landingActive;

    if (!failsafe.landingEligible)
    {
      failsafe.landingFault =
          true;
    }

    failsafe.landingLastUpdateUs =
        now;

#if defined(ESPFC_LAND_V2_ACTIVE)
    if (activeLandRequest)
    {
      if (!failsafe.landingEligible)
      {
        // An automatic descent without a trustworthy
        // attitude/altitude estimate is not allowed.
        // The active validation path falls back to DROP.
        failsafe.landingOutputBlocked =
            true;

        failsafe.landingTouchdownCandidate =
            false;

        failsafe.landingTouchdownStartedUs =
            0;

        failsafe.landingRequested =
            false;

        failsafe.landingActive =
            false;

        failsafe.landingLevelRequested =
            false;

        failsafe.landingDescentRequested =
            false;

        failsafe.phase =
            FC_FAILSAFE_LANDED;

        _model.disarm(
            DISARM_REASON_FAILSAFE);
      }
      else
      {
        // Logical controller authority is enabled. Physical actuation still
        // depends on the selected build policy and Mixer configuration.
        failsafe.landingOutputBlocked =
            false;

        // -------------------------------------------------
        // TOUCHDOWN CONFIRMATION
        //
        // Barometer-only touchdown detection is purposely
        // conservative. Require:
        //   1) some landing time,
        //   2) near-zero vertical speed,
        //   3) evidence that the aircraft descended,
        //   4) the condition to persist for a dwell time.
        // -------------------------------------------------

        constexpr uint32_t
            MIN_LANDING_TIME_US =
                1500000;

        constexpr uint32_t
            TOUCHDOWN_DWELL_US =
                1000000;

        constexpr float
            TOUCHDOWN_VARIO_MS =
                0.15f;

        constexpr float
            MIN_DESCENT_EVIDENCE_M =
                0.20f;

        constexpr float
            LOW_ENTRY_HEIGHT_M =
                0.25f;

        constexpr float
            TOUCHDOWN_HEIGHT_M =
                0.30f;

        const uint32_t landingElapsedUs =
            static_cast<uint32_t>(
                now -
                failsafe.landingRequestedUs);

        // -------------------------------------------------
        // LAND TERMINATION TIMEOUT
        //
        // Touchdown is primarily confirmed from near-ground height + low
        // vertical speed. A bounded timeout prevents a failed/noisy ground
        // detector from leaving the aircraft in LAND forever after RX loss.
        // The timeout scales with the entry height at the selected LAND V2
        // descent rate, then adds a generous ten-second margin.
        // -------------------------------------------------

        constexpr float
            LAND_COMMAND_DESCENT_RATE_MS =
                static_cast<float>(
                    ESPFC_LAND_V2_DESCENT_RATE_MS);

        constexpr float
            LAND_TIMEOUT_MARGIN_S =
                10.0f;

        constexpr float
            LAND_TIMEOUT_MIN_S =
                15.0f;

        constexpr float
            LAND_TIMEOUT_MAX_S =
                60.0f;

        const float nonNegativeEntryHeight =
            std::max(
                failsafe.landingEntryHeight,
                0.0f);

        const float landingTimeoutS =
            std::clamp(
                nonNegativeEntryHeight /
                    LAND_COMMAND_DESCENT_RATE_MS +
                    LAND_TIMEOUT_MARGIN_S,
                LAND_TIMEOUT_MIN_S,
                LAND_TIMEOUT_MAX_S);

        const uint32_t landingTimeoutUs =
            static_cast<uint32_t>(
                landingTimeoutS *
                1000000.0f);

        const bool landingTimedOut =
            landingElapsedUs >=
            landingTimeoutUs;

        const float descendedM =
            failsafe.landingEntryHeight -
            altitude.height;

        const bool slowVerticalMotion =
            std::fabs(
                altitude.vario) <=
            TOUCHDOWN_VARIO_MS;

        const bool descentEvidence =
            descendedM >=
                MIN_DESCENT_EVIDENCE_M ||
            failsafe.landingEntryHeight <=
                LOW_ENTRY_HEIGHT_M;

        const bool nearGround =
            altitude.height <=
            TOUCHDOWN_HEIGHT_M;

        // Mature multicopter landing detectors do not declare touchdown from
        // low vertical speed alone: they also require evidence that thrust has
        // fallen below the hover region. This prevents a near-ground hover or
        // a temporarily stalled descent from being mistaken for contact.
        const float configuredHoverThrust =
            std::clamp(
                -1.0f +
                    2.0f *
                        (static_cast<float>(
                             std::clamp<int>(
                                 _model.config.altHold
                                     .itermCenter,
                                 10,
                                 60)) *
                         0.01f),
                -0.8f,
                0.8f);

        const float entryThrust =
            std::isfinite(
                failsafe.landingEntryThrust)
                ? std::clamp(
                      failsafe.landingEntryThrust,
                      -1.0f,
                      1.0f)
                : configuredHoverThrust;

        // Use the lower of configured hover thrust and the actual command at
        // failsafe entry. This is conservative for aircraft whose real hover
        // command is below the configured center: touchdown must still show a
        // real thrust reduction before disarming.
        const float touchdownReferenceThrust =
            std::min(
                configuredHoverThrust,
                entryThrust);

        constexpr float
            TOUCHDOWN_THRUST_MARGIN =
                0.05f;

        const float commandedThrust =
            _model.state.output.ch[
                AXIS_THRUST];

        const bool lowLandingThrust =
            std::isfinite(
                commandedThrust) &&
            commandedThrust <=
                touchdownReferenceThrust -
                    TOUCHDOWN_THRUST_MARGIN;

        const bool touchdownEvidence =
            landingElapsedUs >=
                MIN_LANDING_TIME_US &&
            slowVerticalMotion &&
            nearGround &&
            descentEvidence &&
            lowLandingThrust;

        // Once strict ground-contact evidence has started the dwell timer,
        // allow a slightly wider hold band. This mirrors staged/hysteretic
        // landing detectors and prevents normal barometer noise from resetting
        // a valid contact candidate every few cycles.
        constexpr float
            TOUCHDOWN_HOLD_VARIO_MS =
                0.25f;

        constexpr float
            TOUCHDOWN_HOLD_HEIGHT_M =
                0.40f;

        constexpr float
            TOUCHDOWN_HOLD_THRUST_MARGIN =
                0.02f;

        const bool touchdownHoldEvidence =
            landingElapsedUs >=
                MIN_LANDING_TIME_US &&
            std::fabs(
                altitude.vario) <=
                TOUCHDOWN_HOLD_VARIO_MS &&
            altitude.height <=
                TOUCHDOWN_HOLD_HEIGHT_M &&
            descentEvidence &&
            std::isfinite(
                commandedThrust) &&
            commandedThrust <=
                touchdownReferenceThrust -
                    TOUCHDOWN_HOLD_THRUST_MARGIN;

        if (landingTimedOut)
        {
          failsafe.landingFault =
              true;

          failsafe.landingRequested =
              false;

          failsafe.landingActive =
              false;

          failsafe.landingLevelRequested =
              false;

          failsafe.landingDescentRequested =
              false;

          failsafe.landingOutputBlocked =
              true;

          failsafe.landingTouchdownCandidate =
              false;

          failsafe.landingTouchdownStartedUs =
              0;

          failsafe.phase =
              FC_FAILSAFE_LANDED;

          _model.disarm(
              DISARM_REASON_FAILSAFE);
        }
        else if (!failsafe.landingTouchdownCandidate)
        {
          if (touchdownEvidence)
          {
            failsafe.landingTouchdownCandidate =
                true;

            failsafe.landingTouchdownStartedUs =
                now;
          }
        }
        else if (touchdownHoldEvidence)
        {
          const uint32_t touchdownDwellUs =
              static_cast<uint32_t>(
                  now -
                  failsafe
                      .landingTouchdownStartedUs);

          if (touchdownDwellUs >=
              TOUCHDOWN_DWELL_US)
          {
            failsafe.landingRequested =
                false;

            failsafe.landingActive =
                false;

            failsafe.landingLevelRequested =
                false;

            failsafe.landingDescentRequested =
                false;

            failsafe.landingOutputBlocked =
                true;

            failsafe.phase =
                FC_FAILSAFE_LANDED;

            _model.disarm(
                DISARM_REASON_FAILSAFE);
          }
        }
        else
        {
          failsafe.landingTouchdownCandidate =
              false;

          failsafe.landingTouchdownStartedUs =
              0;
        }
      }
    }
#endif
  }

  // -----------------------------------------------------
  // FAILSAFE DEBUG
  // -----------------------------------------------------

  if (_model.config.debug.mode ==
      DEBUG_FAILSAFE)
  {
    _model.state.debug[0] =
        static_cast<int16_t>(
            failsafe.phase);

    _model.state.debug[1] =
        static_cast<int16_t>(
            _model.config.failsafe.procedure);

    _model.state.debug[2] =
        failsafe.rxEverValid ? 1 : 0;

    _model.state.debug[3] =
        failsafe.landingRequested ? 1 : 0;

    _model.state.debug[4] =
        failsafe.landingEstimatorHealthy
            ? 1
            : 0;

    _model.state.debug[5] =
        failsafe.landingEligible
            ? 1
            : 0;

    _model.state.debug[6] =
        failsafe.landingActive
            ? 1
            : 0;

    _model.state.debug[7] =
        failsafe.landingOutputBlocked
            ? 1
            : 0;
  }
}

bool Actuator::canActivateMode(
    FlightMode mode)
{
  switch (mode)
  {
    case MODE_ARMED:
      return
          !_model.armingDisabled() &&
          _model.isThrottleLow();

    case MODE_ANGLE:
      return
          attitudeEstimateHealthy() &&
          !_angleFaultLatched;

    case MODE_AIRMODE:
      return
          _model.state.mode.airmodeAllowed;

case MODE_ALTHOLD:
#if defined(ESPFC_ALTHOLD_V2_ACTIVE)
  return
      altitudeEstimateHealthy() &&
      !_altHoldFaultLatched &&
      altHoldPilotStickCentered(
          _model);
#else
  return
      altitudeEstimateHealthy() &&
      !_altHoldFaultLatched;
#endif

    default:
      return true;
  }
}

void Actuator::updateArmed()
{
  if (_model.hasChanged(MODE_ARMED))
  {
    bool armed = _model.isModeActive(MODE_ARMED);
    if (armed)
    {
      _model.state.mode.disarmReason =
          DISARM_REASON_SYSTEM;

      _model.state.mode.rescueConfigMode =
          RESCUE_CONFIG_DISABLED;

      // -------------------------------------------------
      // A deliberate new arm starts a completely new
      // failsafe/LAND lifecycle.
      // -------------------------------------------------

      auto& failsafe =
          _model.state.failsafe;

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
    else if (!armed && _model.state.mode.disarmReason == DISARM_REASON_SYSTEM)
    {
      _model.state.mode.disarmReason = DISARM_REASON_SWITCH;
    }
    if (armed) _model.setGpsHome();
  }
}

void Actuator::updateAirMode()
{
  bool armed = _model.isModeActive(MODE_ARMED);
  if (!armed)
  {
    _model.state.mode.airmodeAllowed = false;
  }
  const int16_t airModeActivateThreshold = _model.config.input.airModeActivateThreshold * 10 + 1000;
  if (armed && !_model.state.mode.airmodeAllowed &&
      _model.state.input.us[AXIS_THRUST] > airModeActivateThreshold) // activate airmode in the air
  {
    _model.state.mode.airmodeAllowed = true;
  }
}

void Actuator::updateBuzzer()
{
  if (_model.isModeActive(MODE_FAILSAFE))
  {
    _model.state.buzzer.play(BUZZER_RX_LOST);
  }
  if (_model.state.battery.warn(_model.config.vbat.cellWarning))
  {
    _model.state.buzzer.play(BUZZER_BAT_LOW);
  }
  if (_model.isModeActive(MODE_BUZZER))
  {
    _model.state.buzzer.play(BUZZER_RX_SET);
  }
  if ((_model.hasChanged(MODE_ARMED)))
  {
    _model.state.buzzer.push(_model.isModeActive(MODE_ARMED) ? BUZZER_ARMING : BUZZER_DISARMING);
  }
  if (!_model.state.gps.wasLocked && _model.state.gps.numSats >= _model.config.gps.minSats)
  {
    _model.state.buzzer.play(BUZZER_READY_BEEP);
    _model.state.gps.wasLocked = true;
  }
}
void Actuator::updateDynLpf()
{
  float throttleUs =
      _model.state.input.us[
          AXIS_THRUST];

#if defined(ESPFC_ALTHOLD_V2_ACTIVE)
  if (assistedVerticalControlOwnsThrust(
          _model))
  {
    const float normalizedThrust =
        std::clamp(
            _model.state.output.ch[
                AXIS_THRUST],
            -1.0f,
            1.0f);

    throttleUs =
        Utils::map(
            normalizedThrust,
            -1.0f,
            1.0f,
            1000.0f,
            2000.0f);
  }
#endif

  const int throttle =
      std::clamp(
          (int)lrintf(
              throttleUs),
          1000,
          2000);

  // Leave some distance below Nyquist.
  const int safeMaxFreq =
      std::max(
          1,
          (int)lrintf(
              _model.state.loopTimer.rate *
              0.45f));

  // -----------------------------
  // GYRO DYNAMIC LOW-PASS
  // -----------------------------
  if (_model.config.gyro
          .dynLpfFilter.cutoff > 0)
  {
    const int low =
        std::clamp(
            (int)_model.config.gyro
                .dynLpfFilter.cutoff,
            1,
            safeMaxFreq);

    const int high =
        std::clamp(
            (int)_model.config.gyro
                .dynLpfFilter.freq,
            low,
            safeMaxFreq);

    const int gyroFreq =
        std::clamp(
            (int)lrintf(
                Utils::map(
                    (float)throttle,
                    1000.f,
                    2000.f,
                    (float)low,
                    (float)high)),
            low,
            high);

    for (size_t i = 0;
         i < AXIS_COUNT_RPY;
         i++)
    {
      _model.state.gyro
          .filter[i]
          .reconfigure(
              gyroFreq);
    }
  }

  // -----------------------------
  // D-TERM DYNAMIC LOW-PASS
  // -----------------------------
  if (_model.config.dterm
          .dynLpfFilter.cutoff > 0)
  {
    const int low =
        std::clamp(
            (int)_model.config.dterm
                .dynLpfFilter.cutoff,
            1,
            safeMaxFreq);

    const int high =
        std::clamp(
            (int)_model.config.dterm
                .dynLpfFilter.freq,
            low,
            safeMaxFreq);

    const int dtermFreq =
        std::clamp(
            (int)lrintf(
                Utils::map(
                    (float)throttle,
                    1000.f,
                    2000.f,
                    (float)low,
                    (float)high)),
            low,
            high);

    for (size_t i = 0;
         i < AXIS_COUNT_RPY;
         i++)
    {
      _model.state.innerPid[i]
          .dtermFilter
          .reconfigure(
              dtermFreq);
    }
  }
}

void Actuator::updateRescueConfig()
{
  switch (_model.state.mode.rescueConfigMode)
  {
    case RESCUE_CONFIG_PENDING:
      // if some rc frames are received, disable to prevent activate later
      if (_model.state.input.frameCount > 100)
      {
        _model.state.mode.rescueConfigMode = RESCUE_CONFIG_DISABLED;
      }
      if (_model.state.failsafe.phase != FC_FAILSAFE_IDLE && _model.config.rescueConfigDelay > 0 &&
          millis() > _model.config.rescueConfigDelay * 1000)
      {
        _model.state.mode.rescueConfigMode = RESCUE_CONFIG_ACTIVE;
      }
      break;
    case RESCUE_CONFIG_ACTIVE:
    case RESCUE_CONFIG_DISABLED:
      // nothing to do here
      break;
  }
}

void Actuator::updateLed()
{
  if (_model.isModeActive(MODE_ARMED) || _model.state.mode.isLongClickActive())
  {
    if (_model.state.mode.isLongClickActive()) _model.setGpsHome();
    _model.state.led.setStatus(Connect::LED_ON);
  }
  else if (_model.armingDisabled())
  {
    _model.state.led.setStatus(Connect::LED_ERROR);
  }
  else
  {
    _model.state.led.setStatus(Connect::LED_OK);
  }
}

} // namespace Espfc::Control
