#include "Control/Fusion.h"
#include "Utils/MemoryHelper.h"
#include "Hal/Time.hpp"
#include <algorithm>

#include <cmath>

namespace Espfc::Control {

Fusion::Fusion(Model& model): _model(model), _madgwick(), _mahony(), _rtqf(), _useMag(false) {}

void Fusion::resetAlgorithms(
    bool startupGain)
{
  const float rate =
      static_cast<float>(
          std::max<int>(
              _model.state.accel.timer.rate,
              1));

  _madgwick =
      Madgwick{};

  _mahony =
      Mahony{};

  _rtqf =
      Rtqf{};

  _madgwick.begin(
      rate);

  _mahony.begin(
      rate);

  _rtqf.begin(
      rate);

  if (startupGain)
  {
    _madgwick.setKp(
        _model.config.fusion.gain *
        0.05f);

    _mahony.setKp(
        _model.config.fusion.gain *
        0.1f);

    _mahony.setKi(
        _model.config.fusion.gainI *
        0.1f);

    _rtqf.setKp(
        _model.config.fusion.gain *
        0.0002f);
  }
  else
  {
    restoreGain();
  }

  _invalidOutputCount =
      0;

  reload(
      MODEL_CHANGE_FILTER);
}


void Fusion::noteInvalidOutput()
{
  if (_invalidOutputCount < 255u)
  {
    ++_invalidOutputCount;
  }

  constexpr uint8_t
      INVALID_OUTPUT_LIMIT =
          3;

  // Never re-seed the recursive attitude estimator while
  // armed; that could create an attitude discontinuity.
  if (_invalidOutputCount >=
          INVALID_OUTPUT_LIMIT &&
      !_model.isModeActive(
          MODE_ARMED))
  {
    resetAlgorithms(
        false);

    _model.state.attitude.healthy =
        false;
  }
}
int Fusion::begin()
{
  _model.state.attitude.healthy =
    false;

_model.state.attitude.lastUpdateUs =
    0;
  _useMag =
      _model.config.fusion.useMag;

  resetAlgorithms(
      true);

  _model.logger.info()
      .log("FUSION")
      .log(FusionConfig::getModeName((FusionMode)_model.config.fusion.mode))
      .logln(_model.config.fusion.gain);

  return 1;
}

int Fusion::reload(ModelChangeEvent event)
{
    _useMag =
    _model.config.fusion.useMag;
  switch (event)
  {
case MODEL_CHANGE_FILTER: {
  _useMag =
      _model.config.fusion.useMag;

  const int accelRate =
      std::max<int>(
          _model.state.accel.timer.rate,
          1);

  const int loopRate =
      std::max<int>(
          _model.state.loopTimer.rate,
          1);

  const int cutoff =
      std::max<int>(
          accelRate /
              GYRO_FUSION_LPF_DIV,
          1);

  for (size_t i = 0;
       i < AXIS_COUNT_RPY;
       ++i)
  {
    _model.state.attitude
        .filter[i]
        .begin(
            FilterConfig(
                FILTER_PT1,
                cutoff),
            loopRate);
  }

  for (size_t i = 0;
       i < 4;
       ++i)
  {
    _qFilter[i].begin(
        FilterConfig(
            FILTER_BIQUAD,
            20),
        accelRate);
  }

  break;
}
    default:
      break;
  }
  return 1;
}

void Fusion::restoreGain()
{
  _madgwick.setKp(_model.config.fusion.gain * 0.005f);
  _mahony.setKp(_model.config.fusion.gain * 0.02f);
  _mahony.setKi(_model.config.fusion.gainI * 0.02f);
  _rtqf.setKp(_model.config.fusion.gain * 0.00005f);
}

int FAST_CODE_ATTR Fusion::update()
{
  Utils::Stats::Measure measure(
      _model.state.stats,
      COUNTER_IMU_FUSION);

 auto& attitude =
    _model.state.attitude;

const uint32_t now =
    micros();

constexpr uint32_t
    ATTITUDE_STALE_US =
        100000;

// A single rejected AHRS sample should not instantly
// destroy the previous valid attitude solution.
// It becomes unhealthy only if no valid solution has
// arrived within the freshness window.
const bool previousSolutionFresh =
    attitude.healthy &&
    static_cast<uint32_t>(
        now -
        attitude.lastUpdateUs) <
        ATTITUDE_STALE_US;

if (!previousSolutionFresh)
{
  attitude.healthy =
      false;
}

  if (!_model.accelActive() ||
      !_model.gyroActive())
  {
    return 0;
  }

const auto g =
    attitude.rate;

const auto a =
    _model.state.accel.adc.fetch();

auto m =
    _model.state.mag.adc;


// -----------------------------------------------------
// INPUT VALIDATION
//
// Validate sensor values BEFORE they enter the AHRS.
// Once NaN/Inf reaches a recursive AHRS state, checking
// the quaternion afterward can be too late.
// -----------------------------------------------------

const bool gyroFinite =
    std::isfinite(g.x) &&
    std::isfinite(g.y) &&
    std::isfinite(g.z);

const bool accelFinite =
    std::isfinite(a.x) &&
    std::isfinite(a.y) &&
    std::isfinite(a.z);

const float accelNormSq =
    a.x * a.x +
    a.y * a.y +
    a.z * a.z;

// Reject corrupt or effectively zero accelerometer
// vectors. The zero-vector case is especially important
// for RTQF attitude construction.
if (!gyroFinite ||
    !accelFinite ||
    !std::isfinite(accelNormSq) ||
    accelNormSq <= 1.0e-6f)
{
  return 0;
}


// -----------------------------------------------------
// MAGNETOMETER FALLBACK
// -----------------------------------------------------

const bool magFinite =
    std::isfinite(m.x) &&
    std::isfinite(m.y) &&
    std::isfinite(m.z);

const float magNormSq =
    m.x * m.x +
    m.y * m.y +
    m.z * m.z;

if (!magFinite ||
    !std::isfinite(magNormSq) ||
    magNormSq <= 1.0e-12f)
{
  // A bad magnetometer must degrade gracefully to
  // six-axis gyro+accelerometer fusion rather than
  // poisoning the AHRS state.
  m =
      VectorFloat{
          0.0f,
          0.0f,
          0.0f};
}

  Quaternion q;

  bool fusionProduced =
      true;

  switch (_model.config.fusion.mode)
  {
    case FUSION_MADGWICK:
      q =
          madgwickFusion(
              g,
              a,
              m);
      break;

    case FUSION_MAHONY:
      q =
          mahonyFusion(
              g,
              a,
              m);
      break;

    case FUSION_RTQF:
      q =
          rtqfFusion(
              g,
              a,
              m);
      break;

    case FUSION_NONE:
    default:
      fusionProduced =
          false;
      break;
  }

  if (!fusionProduced)
  {
    return 0;
  }

  const bool quaternionFinite =
      std::isfinite(q.w) &&
      std::isfinite(q.x) &&
      std::isfinite(q.y) &&
      std::isfinite(q.z);

if (!quaternionFinite)
{
  noteInvalidOutput();
  return 0;
}

  const float qNormSq =
      q.w * q.w +
      q.x * q.x +
      q.y * q.y +
      q.z * q.z;

if (!std::isfinite(qNormSq) ||
    qNormSq < 0.25f ||
    qNormSq > 2.25f)
{
  noteInvalidOutput();
  return 0;
}

  q.normalize();

  const Quaternion signedQ =
      Quaternion::ensureSign(
          q,
          attitude.quaternion);

  VectorFloat euler;

  euler.eulerFromQuaternion(
      signedQ);

if (!std::isfinite(euler.x) ||
    !std::isfinite(euler.y) ||
    !std::isfinite(euler.z))
{
  noteInvalidOutput();
  return 0;
}

  const auto fq =
      filterQuaternion(
          signedQ)
          .getNormalized();

if (!std::isfinite(fq.w) ||
    !std::isfinite(fq.x) ||
    !std::isfinite(fq.y) ||
    !std::isfinite(fq.z))
{
  noteInvalidOutput();
  return 0;
}

  auto world =
      a.getRotated(fq);

  world.z -=
      ACCEL_G;

if (!std::isfinite(world.x) ||
    !std::isfinite(world.y) ||
    !std::isfinite(world.z))
{
  noteInvalidOutput();
  return 0;
}

  const float cosTheta =
      1.0f -
      2.0f *
          (fq.x * fq.x +
           fq.y * fq.y);

if (!std::isfinite(cosTheta))
{
  noteInvalidOutput();
  return 0;
}

  // Commit the complete state only after every check
  // has succeeded.
  attitude.quaternion =
      signedQ;

  attitude.euler =
      euler;

  attitude.cosTheta =
      cosTheta;

  _model.state.accel.world =
      world;

  attitude.healthy =
      true;

  attitude.lastUpdateUs =
      now;
_invalidOutputCount =
    0;
    
  if (_model.config.debug.mode ==
      DEBUG_AC_ERROR)
  {
    _model.state.debug[0] =
        lrintf(
            world[0] *
            ACCEL_G_INV *
            1000);

    _model.state.debug[1] =
        lrintf(
            world[1] *
            ACCEL_G_INV *
            1000);

    _model.state.debug[2] =
        lrintf(
            world[2] *
            ACCEL_G_INV *
            1000);

    _model.state.debug[3] =
        lrintf(
            attitude.cosTheta *
            1000.f);
  }

  if (_model.config.debug.mode ==
      DEBUG_AC_CORRECTION)
  {
    _model.state.debug[0] =
        lrintf(
            Utils::toDeg(
                attitude.euler[0]) *
            10);

    _model.state.debug[1] =
        lrintf(
            Utils::toDeg(
                attitude.euler[1]) *
            10);

    _model.state.debug[2] =
        lrintf(
            Utils::toDeg(
                attitude.euler[2]) *
            10);
  }

  return 1;
}

Quaternion Fusion::filterQuaternion(const Quaternion& q)
{
  return {_qFilter[0].update(q.w), _qFilter[1].update(q.x), _qFilter[2].update(q.y), _qFilter[3].update(q.z)};
}

Quaternion FAST_CODE_ATTR Fusion::madgwickFusion(VectorFloat g, VectorFloat a, VectorFloat m)
{
  if (_useMag && _model.magActive())
  {
    _madgwick.update(g.x, g.y, g.z, a.x, a.y, a.z, m.x, m.y, m.z);
  }
  else
  {
    _madgwick.update(g.x, g.y, g.z, a.x, a.y, a.z);
  }
  return _madgwick.getQuaternion();
}

Quaternion FAST_CODE_ATTR Fusion::mahonyFusion(VectorFloat g, VectorFloat a, VectorFloat m)
{
  if (_useMag && _model.magActive())
  {
    _mahony.update(g.x, g.y, g.z, a.x, a.y, a.z, m.x, m.y, m.z);
  }
  else
  {
    _mahony.update(g.x, g.y, g.z, a.x, a.y, a.z);
  }
  return _mahony.getQuaternion();
}

Quaternion FAST_CODE_ATTR Fusion::rtqfFusion(VectorFloat g, VectorFloat a, VectorFloat m)
{
  if (_useMag && _model.magActive())
  {
    _rtqf.update(g.x, g.y, g.z, a.x, a.y, a.z, m.x, m.y, m.z);
  }
  else
  {
    _rtqf.update(g.x, g.y, g.z, a.x, a.y, a.z);
  }
  return _rtqf.getQuaternion();
}

} // namespace Espfc::Control
