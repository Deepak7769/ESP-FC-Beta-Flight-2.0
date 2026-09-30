#include "Control/Controller.h"
#include "Control/AssistedModeV2.h"
#include "Hal/Time.hpp"
#include "Utils/Math.hpp"
#include <algorithm>
#include <cmath>
namespace Espfc::Control {
namespace {

// Legacy AltHold remains disconnected. Assisted V2 owns the active vertical
// controller only in builds selected by AssistedModeV2.h; ordinary builds keep
// the legacy output path disabled.
constexpr bool ENABLE_LEGACY_ALTHOLD_OUTPUT =
    false;
#if defined(ESPFC_LAND_V2_ACTIVE)

bool landingV2OwnsControl(
    const Model& model)
{
  const auto& failsafe =
      model.state.failsafe;

  return
      failsafe.landingRequested &&
      failsafe.phase ==
          FC_FAILSAFE_LANDING &&
      failsafe.landingActive &&
      !failsafe.landingOutputBlocked;
}

#endif

bool assistedVerticalControlOwnsThrust(
    const Model& model)
{
#if defined(ESPFC_ALTHOLD_V2_ACTIVE)
  const bool altHold =
      model.isModeActive(
          MODE_ALTHOLD);

  #if defined(ESPFC_LAND_V2_ACTIVE)
  const bool land =
      landingV2OwnsControl(
          model);
  #else
  constexpr bool land =
      false;
  #endif

  return
      altHold ||
      land;
#else
  (void)model;
  return false;
#endif
}

} // namespace

Controller::Controller(Model& model): _model(model), _rates{} {}



int Controller::begin()
{
  reload(MODEL_CHANGE_RATES);
  reload(MODEL_CHANGE_FILTER);
  reload(MODEL_CHANGE_PID);

_angleV2WasActive =
    false;

_altHoldWasActive =
    false;

_altHoldV2OutputWasActive =
    false;

_altHoldAltitudeTarget =
    0.0f;

_altHoldVerticalRateTarget =
    0.0f;

_assistedLastUpdateUs =
    0;

_antiGravityPrimed =
    false;

_antiGravityPrevThrottle =
    0.0f;

_model.state.antiGravity =
    AntiGravityState{};

_model.state.angleV2 =
    AngleV2State{};

_model.state.assistedMode =
    AssistedModeState{};

  return 1;
}

int Controller::reload(ModelChangeEvent event)
{
  switch (event)
  {
    case MODEL_CHANGE_RATES:
      _rates.begin(_model.config.input);
      break;
    case MODEL_CHANGE_FILTER:
      reloadFilter();
      break;
    case MODEL_CHANGE_PID:
      reloadPid();
      break;
    default:
      break;
  }
  return 1;
}

int FAST_CODE_ATTR Controller::update()
{
  uint32_t startTime = 0;
  if (_model.config.debug.mode == DEBUG_PIDLOOP)
  {
    startTime = micros();
    _model.state.debug[0] = startTime - _model.state.loopTimer.last;
  }

{
  Utils::Stats::Measure measure(
      _model.state.stats,
      COUNTER_OUTER_PID);

  resetIterm();

// Update authoritative Angle V2 state plus the
// feature-gated AltHold/LAND assisted-controller state.
updateAssistedModes();

  switch (_model.config.mixer.type)
    {
      case FC_MIXER_GIMBAL:
        outerLoopRobot();
        break;

      default:
        outerLoop();
        break;
    }
  }

  // Betaflight-style Anti-Gravity demand is computed every cycle. Builds with
  // ESPFC_ANTI_GRAVITY_ACTIVE feed that demand into the roll/pitch rate PID
  // when the runtime feature/mode is enabled and manual thrust owns the output.
  // SAFE_BENCH builds use the same controller math while Mixer.cpp blocks
  // physical ESC/servo attachment.
  updateAntiGravity();

  {
    Utils::Stats::Measure measure(_model.state.stats, COUNTER_INNER_PID);
    switch (_model.config.mixer.type)
    {
      case FC_MIXER_GIMBAL:
        innerLoopRobot();
        break;

      default:
        innerLoop();
        break;
    }
  }

  if (_model.config.debug.mode == DEBUG_PIDLOOP)
  {
    _model.state.debug[2] = micros() - startTime;
  }

  return 1;
}

void Controller::outerLoopRobot()
{
  const float speedScale = 2.f;
  const float gyroScale = 0.1f;
  const float speed = _speedFilter.update(_model.state.output.ch[AXIS_PITCH] * speedScale +
                                          _model.state.gyro.adc[AXIS_PITCH] * gyroScale);
const auto& input =
    _model.state.input;

const auto& levelConf =
    _model.config.level;

const float angle =
    input.ch[AXIS_PITCH] *
    Utils::toRad(
        levelConf.angleLimit);
  _model.state.setpoint.angle.set(AXIS_PITCH, angle);
  _model.state.setpoint.rate[AXIS_YAW] = input.ch[AXIS_YAW] * Utils::toRad(levelConf.rateLimit);

  if (_model.config.debug.mode == DEBUG_ANGLERATE)
  {
    _model.state.debug[0] = speed * 1000;
    _model.state.debug[1] = lrintf(Utils::toDeg(angle) * 10);
  }
}

void Controller::innerLoopRobot()
{
  // VectorFloat v(0.f, 0.f, 1.f);
  // v.rotate(_model.state.attitude.quaternion);
  // const float angle = acos(v.z);

  const auto& attitude = _model.state.attitude;
  const auto& setpoint = _model.state.setpoint;

  auto& output = _model.state.output;
  auto& innerPid = _model.state.innerPid;

  const float angle = std::max(abs(attitude.euler[AXIS_PITCH]), abs(attitude.euler[AXIS_ROLL]));
  const bool stabilize = angle < Utils::toRad(_model.config.level.angleLimit);
  if (stabilize)
  {
    output.ch[AXIS_PITCH] = innerPid[AXIS_PITCH].update(setpoint.angle[AXIS_PITCH], attitude.euler[AXIS_PITCH]);
    output.ch[AXIS_YAW] = innerPid[AXIS_YAW].update(setpoint.rate[AXIS_YAW], _model.state.gyro.adc[AXIS_YAW]);
  }
  else
  {
    resetIterm();
    output.ch[AXIS_PITCH] = 0.f;
    output.ch[AXIS_YAW] = 0.f;
  }

  if (_model.config.debug.mode == DEBUG_ANGLERATE)
  {
    _model.state.debug[2] = lrintf(Utils::toDeg(attitude.euler[AXIS_PITCH]) * 10);
    _model.state.debug[3] = lrintf(output.ch[AXIS_PITCH] * 1000);
  }
}

void FAST_CODE_ATTR Controller::outerLoop()
{
#if defined(ESPFC_LAND_V2_ACTIVE)
  const bool landingV2Requested =
      landingV2OwnsControl(
          _model);
#else
  constexpr bool landingV2Requested =
      false;
#endif

  // -----------------------------------------------------
  // ROLL / PITCH
  // -----------------------------------------------------
  //
  // LAND V2 deliberately reuses the same Angle V2
  // controller instead of creating a second leveling loop.
  // This keeps one authoritative attitude path.
  // -----------------------------------------------------


if (_model.isModeActive(MODE_ANGLE) ||
    landingV2Requested)
{
  const auto& angleV2 =
      _model.state.angleV2;

  if (angleV2.active)
  {
    _model.state.setpoint.rate[
        AXIS_ROLL] =
        angleV2.rateTarget[
            AXIS_ROLL];

    _model.state.setpoint.rate[
        AXIS_PITCH] =
        angleV2.rateTarget[
            AXIS_PITCH];
  }
  else
  {
    // Angle/LAND was requested, but Angle V2 cannot
    // provide a valid target. Do not reuse stale data.
    _model.state.setpoint.rate[
        AXIS_ROLL] =
        0.0f;

    _model.state.setpoint.rate[
        AXIS_PITCH] =
        0.0f;
  }
}
else
{
  for (size_t i = 0;
       i < AXIS_COUNT_RP;
       ++i)
  {
    _model.state.setpoint.rate[i] =
        calculateSetpointRate(
            i,
            _model.state.input.ch[i]);
  }
}

  // -----------------------------------------------------
  // YAW
  // -----------------------------------------------------
  //
  // During automatic LAND there is no valid pilot yaw
  // command. Request zero yaw rate and let the existing
  // rate loop damp rotation.
  // -----------------------------------------------------

  if (landingV2Requested)
  {
    _model.state.setpoint.rate[
        AXIS_YAW] =
        0.0f;
  }
  else
  {
    _model.state.setpoint.rate[
        AXIS_YAW] =
        calculateSetpointRate(
            AXIS_YAW,
            _model.state.input.ch[
                AXIS_YAW]);
  }

  // -----------------------------------------------------
  // THRUST / VERTICAL-RATE SETPOINT
  // -----------------------------------------------------

#if defined(ESPFC_ALTHOLD_V2_ACTIVE)
  const bool altHoldV2Active =
      _model.state.assistedMode.altitudeActive &&
      (_model.isModeActive(MODE_ALTHOLD) ||
       landingV2Requested);
#else
  constexpr bool altHoldV2Active =
      false;
#endif

  const bool legacyAltHoldActive =
      ENABLE_LEGACY_ALTHOLD_OUTPUT &&
      _model.isModeActive(MODE_ALTHOLD);

  if (altHoldV2Active)
  {
    _model.state.setpoint.rate[
        AXIS_THRUST] =
        _model.state.assistedMode
            .verticalRateTarget;
  }
#if defined(ESPFC_LAND_V2_ACTIVE)
  else if (landingV2Requested)
  {
    // LAND was requested, but the estimator has not yet
    // been accepted (or just failed). Preserve the last
    // commanded thrust for this supervisor cycle. The
    // Actuator LAND supervisor will either validate the
    // estimator or perform the configured hard fallback.
    _model.state.setpoint.rate[
        AXIS_THRUST] =
        _model.state.output.ch[
            AXIS_THRUST];
  }
#endif
  else if (legacyAltHoldActive)
  {
    _model.state.setpoint.rate[
        AXIS_THRUST] =
        calcualteAltHoldSetpoint();
  }
  else
  {
    _model.state.setpoint.rate[
        AXIS_THRUST] =
        _model.state.input.ch[
            AXIS_THRUST];
  }

  // debug
  if (_model.config.debug.mode ==
      DEBUG_ANGLERATE)
  {
    for (size_t i = 0;
         i < AXIS_COUNT_RPY;
         ++i)
    {
      _model.state.debug[i] =
          lrintf(
              Utils::toDeg(
                  _model.state.setpoint.rate[i]));
    }
  }
}

void FAST_CODE_ATTR Controller::innerLoop()
{
  // Roll/Pitch/Yaw rates control
  const float tpaFactor = getTpaFactor();
  const bool tpaP =
    _model.config.controller.tpaMode == 0;
  const auto& setpoint = _model.state.setpoint;
  const auto& altitude = _model.state.altitude;

  auto& innerPid = _model.state.innerPid;
  auto& output = _model.state.output;
  auto& antiGravity = _model.state.antiGravity;

#if defined(ESPFC_ANTI_GRAVITY_ACTIVE)
  // Active-authority builds report whether Anti-Gravity is actually modifying
  // the roll/pitch rate PID on this controller cycle.
  antiGravity.ratePidApplied =
      antiGravity.active;
#else
  antiGravity.ratePidApplied =
      false;
#endif

  if (_model.config.debug.mode ==
      DEBUG_ANTI_GRAVITY)
  {
    _model.state.debug[4] =
        antiGravity.ratePidApplied
            ? 1
            : 0;

    _model.state.debug[5] =
        std::clamp<long>(
            lrintf(
                antiGravity.scaledDerivative *
                100.0f),
            -32000L,
            32000L);

    _model.state.debug[6] =
        std::clamp<long>(
            lrintf(
                antiGravity.iAccelerator *
                1000.0f),
            -32000L,
            32000L);
  }

 for (size_t i = 0;
     i < AXIS_COUNT_RPY;
     ++i)
{
  auto& pid =
      innerPid[i];

  const float fScale =
      pid.fScale;

  float antiGravityPMultiplier =
      1.0f;

  float antiGravityIAccelerator =
      0.0f;

#if defined(ESPFC_ANTI_GRAVITY_ACTIVE)
  // Match Betaflight's current axis policy: Anti-Gravity boosts P and I on
  // roll/pitch, while yaw receives neither Anti-Gravity I acceleration nor P
  // boost. P boost is attenuated above 50 deg/s of commanded axis rate.
  if (antiGravity.active &&
      i < AXIS_YAW)
  {
    const float axisRateDps =
        std::fabs(
            Utils::toDeg(
                setpoint.rate[i]));

    const float setpointAttenuator =
        std::max(
            axisRateDps /
                50.0f,
            1.0f);

    constexpr float
        ANTIGRAVITY_KP =
            0.0034f;

    const float pGain =
        (static_cast<float>(
             ControllerConfig::ANTI_GRAVITY_P_GAIN) /
         100.0f) *
        ANTIGRAVITY_KP;

    antiGravityPMultiplier =
        1.0f +
        (antiGravity.scaledDerivative /
         setpointAttenuator) *
            pGain;

    antiGravityIAccelerator =
        antiGravity.iAccelerator;
  }
#endif

const bool assistedAttitudeRateOwned =
    _model.state.angleV2.active;

  if (assistedAttitudeRateOwned &&
      i < AXIS_COUNT_RP)
  {
    // Angle V2 and LAND V2 both generate an outer-loop
    // rate target. Do not layer stick/feed-forward on top.
    pid.fScale = 0.f;
  }

  output.ch[i] =
    pid.update(
        setpoint.rate[i],
        _model.state.gyro.adc[i],
        tpaFactor,
        tpaP,
        antiGravityPMultiplier,
        antiGravityIAccelerator);

  
  pid.fScale =
      fScale;
}

// -----------------------------------------------------
// THRUST OUTPUT
// -----------------------------------------------------

#if defined(ESPFC_ALTHOLD_V2_ACTIVE)
  #if defined(ESPFC_LAND_V2_ACTIVE)
const bool landingV2Requested =
      landingV2OwnsControl(
          _model);
  #else
constexpr bool landingV2Requested =
    false;
  #endif

const bool altHoldV2OutputActive =
    _model.state.assistedMode.altitudeActive &&
    (_model.isModeActive(MODE_ALTHOLD) ||
     landingV2Requested);
#else
constexpr bool altHoldV2OutputActive =
    false;
#endif

const bool legacyAltHoldActive =
    ENABLE_LEGACY_ALTHOLD_OUTPUT &&
    _model.isModeActive(MODE_ALTHOLD);

auto& verticalPid =
    innerPid[AXIS_THRUST];

if (altHoldV2OutputActive)
{
  if (!_altHoldV2OutputWasActive)
  {
    // --------------------------------------------------
    // BUMPLESS VERTICAL-CONTROL ENTRY
    //
    // Seed the PID history from the measured state and
    // choose I-term so P+I+F is already close to the
    // thrust that was being commanded.
    //
    // The first authoritative cycle explicitly preserves
    // the previous thrust as an additional hard guarantee
    // against a transfer step if the configured I-term
    // limits cannot exactly reproduce that thrust.
    // --------------------------------------------------

    const float entrySetpoint =
        setpoint.rate[AXIS_THRUST];

    const float entryMeasurement =
        altitude.vario;

    const float entryError =
        entrySetpoint -
        entryMeasurement;

    const float entryP =
        verticalPid.Kp *
        verticalPid.pScale *
        entryError;

    const float entryF =
        verticalPid.Kf *
        verticalPid.fScale *
        entrySetpoint;

    const float existingThrust =
        std::clamp(
            output.ch[AXIS_THRUST],
            verticalPid.oLimitLow,
            verticalPid.oLimitHigh);

    verticalPid.prevMeasurement =
        entryMeasurement;

    verticalPid.prevError =
        entryError;

    verticalPid.prevSetpoint =
        entrySetpoint;

    verticalPid.pTerm =
        entryP;

    verticalPid.dTerm =
        0.0f;

    verticalPid.fTerm =
        entryF;

    verticalPid.iTerm =
        std::clamp(
            existingThrust -
                entryP -
                entryF,
            verticalPid.iLimitLow,
            verticalPid.iLimitHigh);

    // Advance internal PID history once, but preserve the
    // pre-transfer actuator command on this cycle.
    verticalPid.update(
        setpoint.rate[AXIS_THRUST],
        altitude.vario);

    output.ch[AXIS_THRUST] =
        existingThrust;
  }
  else
  {
    output.ch[AXIS_THRUST] =
        verticalPid.update(
            setpoint.rate[AXIS_THRUST],
            altitude.vario);
  }

  _altHoldV2OutputWasActive =
      true;
}
else if (legacyAltHoldActive)
{
  output.ch[AXIS_THRUST] =
      verticalPid.update(
          setpoint.rate[AXIS_THRUST],
          altitude.vario);

  _altHoldV2OutputWasActive =
      false;
}
else
{
  // Keep the vertical PID state synchronized while
  // manual thrust owns the output. This also gives the
  // V2 controller a current derivative history when it
  // is engaged later.
  verticalPid.update(
      0.0f,
      altitude.vario);

  verticalPid.iTerm =
      std::clamp(
          _model.state.input.ch[
              AXIS_THRUST],
          verticalPid.iLimitLow,
          verticalPid.iLimitHigh);

  output.ch[AXIS_THRUST] =
      setpoint.rate[
          AXIS_THRUST];

  _altHoldV2OutputWasActive =
      false;
}

  if (_model.config.debug.mode ==
      DEBUG_AUTOPILOT_PID)
  {
    // Assisted vertical-controller trace. Units are chosen so Blackbox can
    // display the entire AltHold/LAND cause-and-effect chain using int16 debug
    // channels in both the active production build and SAFE_BENCH validation.
    const float verticalError =
        setpoint.rate[AXIS_THRUST] -
        altitude.vario;

    _model.state.debug[0] =
        std::clamp<long>(
            lrintf(
                setpoint.rate[AXIS_THRUST] *
                100.0f),
            -32000L,
            32000L); // vertical-rate target, cm/s

    _model.state.debug[1] =
        std::clamp<long>(
            lrintf(
                altitude.vario *
                100.0f),
            -32000L,
            32000L); // measured vertical rate, cm/s

    _model.state.debug[2] =
        std::clamp<long>(
            lrintf(
                verticalError *
                100.0f),
            -32000L,
            32000L); // vertical-rate error, cm/s

    _model.state.debug[3] =
        std::clamp<long>(
            lrintf(
                verticalPid.pTerm *
                1000.0f),
            -32000L,
            32000L);

    _model.state.debug[4] =
        std::clamp<long>(
            lrintf(
                verticalPid.iTerm *
                1000.0f),
            -32000L,
            32000L);

    _model.state.debug[5] =
        std::clamp<long>(
            lrintf(
                verticalPid.dTerm *
                1000.0f),
            -32000L,
            32000L);

    _model.state.debug[6] =
        std::clamp<long>(
            lrintf(
                output.ch[AXIS_THRUST] *
                1000.0f),
            -32000L,
            32000L);

    uint16_t flags =
        0;

    if (_model.isModeActive(
            MODE_ALTHOLD))
    {
      flags |=
          uint16_t{1} << 0;
    }

    if (_model.state.assistedMode
            .altitudeActive)
    {
      flags |=
          uint16_t{1} << 1;
    }

    if (altitude.healthy)
    {
      flags |=
          uint16_t{1} << 2;
    }

    if (altitude.baroAccepted)
    {
      flags |=
          uint16_t{1} << 3;
    }

    if (_altHoldV2OutputWasActive)
    {
      flags |=
          uint16_t{1} << 4;
    }

#if defined(ESPFC_LAND_V2_ACTIVE)
    if (_model.state.failsafe
            .landingRequested)
    {
      flags |=
          uint16_t{1} << 5;
    }

    if (_model.state.failsafe
            .landingActive)
    {
      flags |=
          uint16_t{1} << 6;
    }

    if (_model.state.failsafe
            .landingOutputBlocked)
    {
      flags |=
          uint16_t{1} << 7;
    }
#endif

    if (_model.state.input.channelsValid)
    {
      flags |=
          uint16_t{1} << 8;
    }

    _model.state.debug[7] =
        static_cast<int16_t>(
            flags);
  }

  if (_model.config.debug.mode == DEBUG_STACK)
  {
    _model.state.debug[0] = std::clamp(lrintf(setpoint.rate[AXIS_THRUST] * 1000.0f), -3000l, 3000l);    // hi mem
    _model.state.debug[1] = std::clamp(lrintf(altitude.vario * 1000.0f), -30000l, 30000l);              // lo mem
    _model.state.debug[2] = std::clamp(lrintf(altitude.height * 100.0f), -30000l, 30000l);              // curr
    _model.state.debug[3] = std::clamp(lrintf(innerPid[AXIS_THRUST].error * 1000.0f), -30000l, 30000l); // p
    _model.state.debug[4] = std::clamp(lrintf(innerPid[AXIS_THRUST].pTerm * 1000.0f), -3000l, 3000l);
    _model.state.debug[5] = std::clamp(lrintf(innerPid[AXIS_THRUST].iTerm * 1000.0f), -3000l, 3000l);
    _model.state.debug[6] = std::clamp(lrintf(innerPid[AXIS_THRUST].dTerm * 1000.0f), -3000l, 3000l);
    _model.state.debug[7] = std::clamp(lrintf(innerPid[AXIS_THRUST].fTerm * 1000.0f), -3000l, 3000l);
  }

  // debug
  if (_model.config.debug.mode == DEBUG_ITERM_RELAX)
  {
    _model.state.debug[0] = lrintf(Utils::toDeg(innerPid[AXIS_ROLL].itermRelaxBase));
    _model.state.debug[1] = lrintf(innerPid[AXIS_ROLL].itermRelaxFactor * 100.0f);
    _model.state.debug[2] = lrintf(Utils::toDeg(innerPid[AXIS_ROLL].iTermError));
    _model.state.debug[3] = lrintf(innerPid[AXIS_ROLL].iTerm * 1000.0f);
  }
}
void Controller::updateAntiGravity()
{
  auto& antiGravity =
      _model.state.antiGravity;

  // Authoritative application is decided later inside innerLoop().
  antiGravity.ratePidApplied =
      false;

  antiGravity.enabled =
      _model.isFeatureActive(
          FEATURE_ANTI_GRAVITY) ||
      _model.isModeActive(
          MODE_ANTI_GRAVITY);

  const auto& input =
      _model.state.input;

  const bool manualThrustOwnsOutput =
      !assistedVerticalControlOwnsThrust(
          _model);

  const bool inputHealthy =
      input.channelsValid &&
      !input.rxLoss &&
      !input.rxFailSafe;

  if (!antiGravity.enabled ||
      !manualThrustOwnsOutput ||
      !inputHealthy)
  {
    antiGravity.active =
        false;

    antiGravity.throttle =
        0.0f;

    antiGravity.derivative =
        0.0f;

    antiGravity.filteredDerivative =
        0.0f;

    antiGravity.scaledDerivative =
        0.0f;

    antiGravity.iAccelerator =
        0.0f;

    antiGravity.iMultiplier =
        1.0f;

    antiGravity.pMultiplier =
        1.0f;

    _antiGravityPrimed =
        false;

    return;
  }

  const float throttle =
      std::clamp(
          (_model.state.input.ch[
               AXIS_THRUST] +
           1.0f) *
              0.5f,
          0.0f,
          1.0f);

  antiGravity.throttle =
      throttle;

  if (!_antiGravityPrimed)
  {
    _antiGravityPrevThrottle =
        throttle;

    _antiGravityFilter.prime(
        0.0f);

    _antiGravityPrimed =
        true;

    antiGravity.active =
        false;

    antiGravity.derivative =
        0.0f;

    antiGravity.filteredDerivative =
        0.0f;

    antiGravity.scaledDerivative =
        0.0f;

    antiGravity.iAccelerator =
        0.0f;

    antiGravity.iMultiplier =
        1.0f;

    antiGravity.pMultiplier =
        1.0f;

    return;
  }

  const float loopRate =
      static_cast<float>(
          std::max<int>(
              _model.state.loopTimer.rate,
              1));

  const float throttleInv =
      1.0f -
      throttle;

  float derivative =
      std::fabs(
          throttle -
          _antiGravityPrevThrottle) *
      loopRate;

  derivative *=
      throttleInv *
      throttleInv;

  if (throttle >
      _antiGravityPrevThrottle)
  {
    derivative *=
        throttleInv *
        0.5f;
  }

  _antiGravityPrevThrottle =
      throttle;

  const float filteredDerivative =
      _antiGravityFilter.update(
          derivative);

  antiGravity.derivative =
      derivative;

  antiGravity.filteredDerivative =
      filteredDerivative;

  const float scaledDerivative =
      filteredDerivative *
      static_cast<float>(
          _model.config
              .antiGravityGain);

  // Betaflight constants define the Anti-Gravity gain demand. When
  // ESPFC_ANTI_GRAVITY_ACTIVE is compiled, innerLoop() applies the demand to
  // roll/pitch while this remains the single detector/calculation path.
  constexpr float ANTIGRAVITY_KI =
      0.34f;

  constexpr float ANTIGRAVITY_KP =
      0.0034f;

  const float pitchKi =
      std::fabs(
          _model.state.innerPid[
              AXIS_PITCH]
              .Ki);

  const float itermAccelerator =
      scaledDerivative *
      ANTIGRAVITY_KI;

  antiGravity.scaledDerivative =
      scaledDerivative;

  antiGravity.iAccelerator =
      itermAccelerator;

  antiGravity.iMultiplier =
      pitchKi > 0.000001f
          ? 1.0f +
                itermAccelerator /
                    pitchKi
          : 1.0f;

  const float pitchRateDps =
      std::fabs(
          Utils::toDeg(
              _model.state.setpoint.rate[
                  AXIS_PITCH]));

  const float setpointAttenuator =
      std::max(
          pitchRateDps /
              50.0f,
          1.0f);

  const float pGain =
      (static_cast<float>(
           ControllerConfig::ANTI_GRAVITY_P_GAIN) /
       100.0f) *
      ANTIGRAVITY_KP;

  antiGravity.pMultiplier =
      1.0f +
      (scaledDerivative /
       setpointAttenuator) *
          pGain;

  antiGravity.active =
      scaledDerivative >
      0.01f;

  if (_model.config.debug.mode ==
      DEBUG_ANTI_GRAVITY)
  {
    _model.state.debug[0] =
        std::clamp<long>(
            lrintf(
                derivative *
                100.0f),
            -32000L,
            32000L);

    _model.state.debug[1] =
        std::clamp<long>(
            lrintf(
                filteredDerivative *
                100.0f),
            -32000L,
            32000L);

    _model.state.debug[2] =
        std::clamp<long>(
            lrintf(
                antiGravity.iMultiplier *
                1000.0f),
            -32000L,
            32000L);

    _model.state.debug[3] =
        std::clamp<long>(
            lrintf(
                antiGravity.pMultiplier *
                1000.0f),
            -32000L,
            32000L);
  }
}


float Controller::calculatePilotClimbRate() const
{
  constexpr float DEADBAND =
      0.10f;

  constexpr float MAX_DESCENT_MS =
      1.0f;

  constexpr float MAX_CLIMB_MS =
      1.5f;

  constexpr size_t PILOT_CHANNEL =
      static_cast<size_t>(
          ESPFC_ALTHOLD_V2_CENTERED_STICK_CHANNEL);

  static_assert(
      PILOT_CHANNEL < AXIS_COUNT,
      "AltHold V2 centered-stick channel exceeds input channel count");

  float stick =
      std::clamp(
          _model.state.input.ch[
              PILOT_CHANNEL],
          -1.0f,
          1.0f);

  stick =
      Utils::deadband(
          stick,
          DEADBAND);

  // Utils::deadband() removes 0.10 from the magnitude.
  // Renormalize so full stick is still +/-1.0.
  if (stick != 0.0f)
  {
    stick /=
        (1.0f -
         DEADBAND);
  }

  stick =
      std::clamp(
          stick,
          -1.0f,
          1.0f);

  if (stick > 0.0f)
  {
    return
        stick *
        MAX_CLIMB_MS;
  }

  return
      stick *
      MAX_DESCENT_MS;
}

// Shared assisted-controller update.
//
// Angle V2 is always authoritative for MODE_ANGLE.
// AltHold and failsafe LAND remain feature-gated by
// AssistedModeV2.h.
void Controller::updateAssistedModes()
{
auto& angleV2 =
    _model.state.angleV2;

auto& assisted =
    _model.state.assistedMode;

  const auto& attitude =
      _model.state.attitude;

  const auto& altitude =
      _model.state.altitude;

  const auto& input =
      _model.state.input;

#if defined(ESPFC_LAND_V2_ACTIVE)
  const auto& failsafe =
      _model.state.failsafe;

  const bool landingV2Requested =
      landingV2OwnsControl(
          _model);
#else
  constexpr bool landingV2Requested =
      false;
#endif

const float nominalDt =
    1.0f /
    static_cast<float>(
        std::max<int>(
            _model.state.loopTimer.rate,
            1));

const uint32_t now =
    micros();
    
constexpr uint32_t
    ATTITUDE_STALE_US =
        100000;

constexpr uint32_t
    BARO_STALE_US =
        350000;

const bool attitudeFresh =
    attitude.healthy &&
    static_cast<uint32_t>(
        now -
        attitude.lastUpdateUs) <
        ATTITUDE_STALE_US;

const auto& baro =
    _model.state.baro;

const bool assistedBaroFresh =
    baro.sampleValid &&
    static_cast<uint32_t>(
        now -
        baro.lastUpdateUs) <
        BARO_STALE_US;

float dt =
    nominalDt;

if (_assistedLastUpdateUs != 0)
{
  const uint32_t elapsedUs =
      static_cast<uint32_t>(
          now -
          _assistedLastUpdateUs);

  if (elapsedUs > 0)
  {
    const float measuredDt =
        static_cast<float>(
            elapsedUs) *
        0.000001f;

    // Prevent an interrupted/debug-stalled loop from
    // creating a huge one-cycle target jump.
    dt =
        std::clamp(
            measuredDt,
            0.00025f,
            0.050f);
  }
}

_assistedLastUpdateUs =
    now;

// =====================================================
// ANGLE MODE V2
// =====================================================

const bool angleActive =
    (_model.isModeActive(
         MODE_ANGLE) ||
     landingV2Requested) &&
        attitudeFresh;

if (angleActive &&
    !_angleV2WasActive)
{
  // Bumpless transfer: begin from measured attitude.
  angleV2.angleTarget[
      AXIS_ROLL] =
      attitude.euler[
          AXIS_ROLL];

  angleV2.angleTarget[
      AXIS_PITCH] =
      attitude.euler[
          AXIS_PITCH];
}

if (angleActive)
{
  constexpr float ANGLE_SLEW_DPS =
      120.0f;

  const float maxAngleStep =
      Utils::toRad(
          ANGLE_SLEW_DPS) *
      dt;

  const float maxRate =
      Utils::toRad(
          _model.config.level
              .rateLimit);

  const float baseLevelKp =
      static_cast<float>(
          _model.config.pid[
              FC_PID_LEVEL].P) *
      LEVEL_PTERM_SCALE;

  for (size_t axis = 0;
       axis < AXIS_COUNT_RP;
       ++axis)
  {
    const float requestedAngle =
        landingV2Requested
            ? 0.0f
            : Utils::toRad(
                  _model.config.level
                      .angleLimit) *
                  input.ch[axis];

    const float change =
        std::clamp(
            requestedAngle -
                angleV2.angleTarget[
                    axis],
            -maxAngleStep,
            maxAngleStep);

    angleV2.angleTarget[
        axis] +=
        change;

    const float angleError =
        angleV2.angleTarget[
            axis] -
        attitude.euler[
            axis];

    const float levelKp =
        baseLevelKp *
        angleV2.pScale[
            axis];

    angleV2.rateTarget[
        axis] =
        std::clamp(
            levelKp *
                angleError,
            -maxRate,
            maxRate);
  }
}
else
{
  // Keep targets synchronized while Angle V2 is inactive.
  // This preserves bumpless re-entry.
  for (size_t axis = 0;
       axis < AXIS_COUNT_RP;
       ++axis)
  {
    angleV2.angleTarget[
        axis] =
        attitude.euler[
            axis];

    angleV2.rateTarget[
        axis] =
        0.0f;
  }
}

angleV2.active =
    angleActive;

_angleV2WasActive =
    angleActive;


  // =====================================================
  // ALTITUDE HOLD V2
  // =====================================================

const bool altActive =
    (_model.isModeActive(MODE_ALTHOLD) ||
     landingV2Requested) &&
    altitude.healthy &&
    attitudeFresh &&
    assistedBaroFresh;

  constexpr float LAND_DESCENT_RATE_MS =
      -static_cast<float>(
          ESPFC_LAND_V2_DESCENT_RATE_MS);

  const float pilotVz =
      landingV2Requested
          ? LAND_DESCENT_RATE_MS
          : calculatePilotClimbRate();

  if (altActive &&
      !_altHoldWasActive)
  {
    // Capture current estimated altitude.
    _altHoldAltitudeTarget =
        altitude.height;

    // Begin from current vertical velocity.
    _altHoldVerticalRateTarget =
        altitude.vario;

    assisted.altitudeTargetValid =
        true;
  }

 if (altActive)
{
  // --------------------------------------------------
  // ALTITUDE TARGET ANTI-WINDUP
  //
  // The altitude-position controller saturates at
  // +/-1.0 m/s with Kp = 0.50, therefore an altitude
  // error larger than 2.0 m cannot produce any more
  // correction authority.
  //
  // Do not allow pilot target integration to build an
  // unreachable -20 m / -60 m / +60 m backlog.
  // --------------------------------------------------

  constexpr float ALTITUDE_KP =
      0.50f;

  constexpr float MAX_POSITION_CORRECTION_MS =
      1.0f;

  constexpr float MAX_TARGET_ERROR_M =
      MAX_POSITION_CORRECTION_MS /
      ALTITUDE_KP; // 2.0 m

  // Integrate pilot climb/descent command.
  _altHoldAltitudeTarget +=
      pilotVz * dt;

  // Keep the requested altitude inside the useful
  // position-control window around the current
  // estimated altitude.
  const float minAltitudeTarget =
      altitude.height -
      MAX_TARGET_ERROR_M;

  const float maxAltitudeTarget =
      altitude.height +
      MAX_TARGET_ERROR_M;

  _altHoldAltitudeTarget =
      std::clamp(
          _altHoldAltitudeTarget,
          minAltitudeTarget,
          maxAltitudeTarget);

  const float altitudeError =
      _altHoldAltitudeTarget -
      altitude.height;

  const float velocityCorrection =
      std::clamp(
          ALTITUDE_KP *
              altitudeError,
          -MAX_POSITION_CORRECTION_MS,
          MAX_POSITION_CORRECTION_MS);

    constexpr float MAX_DESCENT_MS =
        1.0f;

    constexpr float MAX_CLIMB_MS =
        1.5f;

    const float requestedVz =
        std::clamp(
            pilotVz +
                velocityCorrection,
            -MAX_DESCENT_MS,
            MAX_CLIMB_MS);

    // Smooth vertical acceleration.
    constexpr float VERTICAL_ACCEL_LIMIT_MSS =
        1.0f;

    const float maxVzStep =
        VERTICAL_ACCEL_LIMIT_MSS *
        dt;

    _altHoldVerticalRateTarget +=
        std::clamp(
            requestedVz -
                _altHoldVerticalRateTarget,
            -maxVzStep,
            maxVzStep);

    assisted.altitudeTarget =
        _altHoldAltitudeTarget;

    assisted.verticalRatePilot =
        pilotVz;

    assisted.verticalRateCorrection =
        velocityCorrection;

    assisted.verticalRateTarget =
        _altHoldVerticalRateTarget;
  }
  else
  {
    assisted.altitudeTarget =
        altitude.height;

    assisted.verticalRatePilot =
        0.0f;

    assisted.verticalRateCorrection =
        0.0f;

    assisted.verticalRateTarget =
        altitude.vario;

    assisted.altitudeTargetValid =
        false;

    _altHoldAltitudeTarget =
        altitude.height;

    _altHoldVerticalRateTarget =
        altitude.vario;
  }

  assisted.altitudeActive =
      altActive;

  _altHoldWasActive =
      altActive;


  // =====================================================
  // ALTITUDE DEBUG
  // =====================================================

  if (_model.config.debug.mode ==
      DEBUG_AUTOPILOT_ALTITUDE)
  {
    _model.state.debug[0] =
        std::clamp(
            lrintf(
                altitude.height *
                100.0f),
            -32000l,
            32000l);

    _model.state.debug[1] =
        std::clamp(
            lrintf(
                assisted.altitudeTarget *
                100.0f),
            -32000l,
            32000l);

    _model.state.debug[2] =
        std::clamp(
            lrintf(
                altitude.vario *
                100.0f),
            -32000l,
            32000l);

    _model.state.debug[3] =
        std::clamp(
            lrintf(
                assisted.verticalRateTarget *
                100.0f),
            -32000l,
            32000l);

    _model.state.debug[4] =
        std::clamp(
            lrintf(
                assisted.verticalRatePilot *
                100.0f),
            -32000l,
            32000l);

    _model.state.debug[5] =
        std::clamp(
            lrintf(
                altitude.baroInnovation *
                100.0f),
            -32000l,
            32000l);

    _model.state.debug[6] =
        altitude.healthy ? 1 : 0;

    _model.state.debug[7] =
        altitude.baroAccepted ? 1 : 0;
  }


  // =====================================================
  // ANGLE DEBUG
  // =====================================================

if (_model.config.debug.mode ==
    DEBUG_ANGLE_TARGET)
{
  _model.state.debug[0] =
      lrintf(
          Utils::toDeg(
              angleV2.angleTarget[
                  AXIS_ROLL]) *
          10.0f);

  _model.state.debug[1] =
      lrintf(
          Utils::toDeg(
              attitude.euler[
                  AXIS_ROLL]) *
          10.0f);

  _model.state.debug[2] =
      lrintf(
          Utils::toDeg(
              angleV2.rateTarget[
                  AXIS_ROLL]));

  _model.state.debug[3] =
      lrintf(
          Utils::toDeg(
              angleV2.angleTarget[
                  AXIS_PITCH]) *
          10.0f);

  _model.state.debug[4] =
      lrintf(
          Utils::toDeg(
              attitude.euler[
                  AXIS_PITCH]) *
          10.0f);

  _model.state.debug[5] =
      lrintf(
          Utils::toDeg(
              angleV2.rateTarget[
                  AXIS_PITCH]));
}
}
float Controller::calcualteAltHoldSetpoint() const
{
  float thrust = _model.state.input.ch[AXIS_THRUST];

  // if(_model.isThrottleLow()) thrust = 0.0f; // stick below min check, no command

  thrust = Utils::deadband(thrust, 0.1f); // +/- 12.5% deadband

  return Utils::map3(thrust, -1.f, 0.f, 1.f, -2.0f, 0.f, 4.f); // climb rate 5ms, descend rate 2 m/s
}

float Controller::getTpaFactor() const
{
  const float scale =
      std::clamp(
          (float)_model.config.controller.tpaScale,
          0.f,
          100.f);

  if (scale <= 0.f)
  {
    return 1.f;
  }

  const float breakpoint =
      std::clamp(
          (float)_model.config.controller.tpaBreakpoint,
          1000.f,
          1999.f);

  float throttleUs =
      _model.state.input.us[
          AXIS_THRUST];

  if (assistedVerticalControlOwnsThrust(
          _model))
  {
    throttleUs =
        Utils::map(
            std::clamp(
                _model.state.output.ch[
                    AXIS_THRUST],
                -1.0f,
                1.0f),
            -1.0f,
            1.0f,
            1000.0f,
            2000.0f);
  }

  const float throttle =
      std::clamp(
          throttleUs,
          breakpoint,
          2000.f);

  const float factor =
      Utils::map(
          throttle,
          breakpoint,
          2000.f,
          1.f,
          1.f - scale * 0.01f);

  return std::isfinite(factor)
      ? std::clamp(factor, 0.f, 1.f)
      : 1.f;
}

void Controller::resetIterm()
{
  const bool assistedVerticalThrust =
      assistedVerticalControlOwnsThrust(
          _model);

  if (!_model.isModeActive(MODE_ARMED) // when not armed
      || (!assistedVerticalThrust &&
          !_model.isAirModeActive() &&
          _model.config.iterm.lowThrottleZeroIterm &&
          _model.isThrottleLow()) // low manual throttle only when manual thrust owns output
  )
  {
    for (size_t i = 0; i < AXIS_COUNT_RPY; i++)
    {
      _model.state.innerPid[i].resetIterm();
    
    }
  }
  if (!_model.isModeActive(MODE_ARMED))
  {
    //_model.state.innerPid[AXIS_THRUST].resetIterm();
  }
}

float Controller::calculateSetpointRate(int axis, float input) const
{
  return _rates.getSetpoint(axis, axis == AXIS_YAW ? -input : input);
}

void Controller::reloadPid()
{
  const int pidFilterRate = _model.state.loopTimer.rate;

  float pidScale[] = {1.f, 1.f, 1.f};
  if (_model.config.mixer.type == FC_MIXER_GIMBAL)
  {
    pidScale[AXIS_YAW] = 0.2f;   // ROBOT
    pidScale[AXIS_PITCH] = 20.f; // ROBOT
  }

  // inner loop
  for (size_t axis = 0; axis < AXIS_COUNT_RPY; axis++)
  {
    const auto& pc = _model.config.pid[axis];
    auto& pid = _model.state.innerPid[axis];
    pid.Kp = (float)pc.P * PTERM_SCALE * pidScale[axis];
    pid.Ki = (float)pc.I * ITERM_SCALE * pidScale[axis];
    pid.Kd = (float)pc.D * DTERM_SCALE * pidScale[axis];
    pid.Kf = (float)pc.F * FTERM_SCALE * pidScale[axis];
    pid.iLimitLow = -_model.config.iterm.limit * 0.01f;
    pid.iLimitHigh = _model.config.iterm.limit * 0.01f;
    pid.oLimitLow = -0.66f;
    pid.oLimitHigh = 0.66f;
    pid.rate = pidFilterRate;
    if (axis == AXIS_YAW)
    {
      pid.itermRelax =
          (_model.config.iterm.relax == ITERM_RELAX_RPY || _model.config.iterm.relax == ITERM_RELAX_RPY_INC)
              ? _model.config.iterm.relax
              : ITERM_RELAX_OFF;
    }
    else
    {
      pid.itermRelax = _model.config.iterm.relax;
    }
    pid.begin();
  }


  // alt hold pid
  float itermCenter = std::clamp((int)_model.config.altHold.itermCenter, 10, 60) * 0.01f;
  float itermRange = itermCenter * std::clamp((int)_model.config.altHold.itermRange, 10, 60) * 0.01f;
  const auto& pc = _model.config.pid[FC_PID_VEL];

  auto& pid = _model.state.innerPid[AXIS_THRUST];
  pid.Kp = (float)pc.P * VEL_PTERM_SCALE;
  pid.Ki = (float)pc.I * VEL_ITERM_SCALE;
  pid.Kd = (float)pc.D * VEL_DTERM_SCALE;
  pid.Kf = (float)pc.F * VEL_FTERM_SCALE;
  pid.iLimitLow = -1.0f + 2.0f * (itermCenter - itermRange);
  pid.iLimitHigh = -1.0f + 2.0f * (itermCenter + itermRange);
  pid.iReset = pid.iLimitLow;
  pid.rate = _model.state.loopTimer.rate;
  pid.begin();
}

void Controller::reloadFilter()
{
  _speedFilter.begin(FilterConfig(FILTER_BIQUAD, 10), _model.state.loopTimer.rate);

  const int pidFilterRate =
      std::max<int>(
          _model.state.loopTimer.rate,
          1);

  _antiGravityFilter.begin(
      FilterConfig(
          FILTER_PT2,
          ControllerConfig::ANTI_GRAVITY_CUTOFF_HZ),
      pidFilterRate);

  _antiGravityPrimed =
      false;

  // inner loop
  const auto& dtermConf = _model.config.dterm;
  for (size_t axis = 0; axis < AXIS_COUNT_RPY; axis++)
  {
    auto& pid = _model.state.innerPid[axis];
    pid.rate = pidFilterRate;
    pid.dtermNotchFilter.begin(dtermConf.notchFilter, pidFilterRate);
    if (dtermConf.dynLpfFilter.cutoff > 0)
    {
      pid.dtermFilter.begin(FilterConfig((FilterType)dtermConf.filter.type, dtermConf.dynLpfFilter.cutoff),
                            pidFilterRate);
    }
    else
    {
      pid.dtermFilter.begin(dtermConf.filter, pidFilterRate);
    }
    pid.dtermFilter2.begin(dtermConf.filter2, pidFilterRate);
    pid.ftermFilter.begin(_model.config.input.filterDerivative, pidFilterRate);
    pid.itermRelaxFilter.begin(FilterConfig(FILTER_PT1, _model.config.iterm.relaxCutoff), pidFilterRate);
    if (axis == AXIS_YAW)
    {
      pid.ptermFilter.begin(_model.config.yaw.filter, pidFilterRate);
    }
    pid.begin();
  }


  // alt hold pid
  auto& pid = _model.state.innerPid[AXIS_THRUST];
  pid.rate = _model.state.loopTimer.rate;
  pid.dtermFilter.begin(FilterConfig(FILTER_PT1, 10), _model.state.loopTimer.rate);
  pid.ftermDerivative = false;
  pid.begin();
}

} // namespace Espfc::Control
