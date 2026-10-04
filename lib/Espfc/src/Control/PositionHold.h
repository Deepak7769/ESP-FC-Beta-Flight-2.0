
#pragma once
 
// -----------------------------------------------------------------------------
// Position hold (multirotor) - pure math, no Model dependency, unit-testable.
//
// Structure follows the usual multirotor position cascade used by ArduPilot's
// AC_PosControl / PosHold and by Betaflight 2025.12 Position Hold:
//
//   position error --P--> velocity target --PI--> acceleration --atan(a/g)--> lean angle
//
// plus Betaflight-style pilot handling: roll/pitch stick beyond a deadband
// takes over and moves the target; on release the aircraft brakes to a stop and
// the hold point is re-latched where it stopped.
//
// Sign convention (important): the returned angles use the SAME sign as the
// normalised roll/pitch stick fed to Angle mode.
//   pitch > 0  ==  stick forward  ==  accelerate toward the nose
//   roll  > 0  ==  stick right    ==  accelerate toward the right
// This is true for any working Angle mode regardless of the IMU euler sign.
//
// Heading is compass heading in radians, clockwise from north (0 = north).
// -----------------------------------------------------------------------------
 
#include <algorithm>
#include <cmath>
#include <cstdint>
 
namespace Espfc::Control {
 
#ifndef ESPFC_POSHOLD_POS_P
#define ESPFC_POSHOLD_POS_P 0.4f // 1/s: velocity target per metre of error
#endif
#ifndef ESPFC_POSHOLD_MAX_SPEED_MS
#define ESPFC_POSHOLD_MAX_SPEED_MS 1.5f // m/s speed limit while correcting
#endif
#ifndef ESPFC_POSHOLD_VEL_P
#define ESPFC_POSHOLD_VEL_P 0.8f // (m/s^2) per (m/s) velocity error
#endif
#ifndef ESPFC_POSHOLD_VEL_I
#define ESPFC_POSHOLD_VEL_I 0.15f // (m/s^2) per (m/s*s)
#endif
#ifndef ESPFC_POSHOLD_I_LIMIT_MSS
#define ESPFC_POSHOLD_I_LIMIT_MSS 1.0f // wind/trim authority of the I term
#endif
#ifndef ESPFC_POSHOLD_MAX_ACCEL_MSS
#define ESPFC_POSHOLD_MAX_ACCEL_MSS 2.0f // horizontal acceleration limit
#endif
#ifndef ESPFC_POSHOLD_MAX_ANGLE_DEG
#define ESPFC_POSHOLD_MAX_ANGLE_DEG 12.0f // hard cap on lean angle
#endif
#ifndef ESPFC_POSHOLD_DEADBAND
#define ESPFC_POSHOLD_DEADBAND 0.15f // stick deflection that takes over
#endif
#ifndef ESPFC_POSHOLD_BRAKE_SPEED_MS
#define ESPFC_POSHOLD_BRAKE_SPEED_MS 0.4f // re-latch below this ground speed
#endif
#ifndef ESPFC_POSHOLD_BRAKE_TIMEOUT_S
#define ESPFC_POSHOLD_BRAKE_TIMEOUT_S 3.0f
#endif
#ifndef ESPFC_POSHOLD_MAX_ERROR_M
#define ESPFC_POSHOLD_MAX_ERROR_M 30.0f // farther than this: re-latch, never chase
#endif
#ifndef ESPFC_POSHOLD_FILTER_TAU_S
#define ESPFC_POSHOLD_FILTER_TAU_S 0.60f // position estimate time constant
#endif
#ifndef ESPFC_POSHOLD_DEADBAND_M
#define ESPFC_POSHOLD_DEADBAND_M 0.75f // do not chase sub-metre GPS noise
#endif
#ifndef ESPFC_POSHOLD_MAX_INNOVATION_M
#define ESPFC_POSHOLD_MAX_INNOVATION_M 8.0f // reject implausible GPS jumps
#endif
 
struct PositionHoldParams
{
  float posP = ESPFC_POSHOLD_POS_P;
  float maxSpeed = ESPFC_POSHOLD_MAX_SPEED_MS;
  float velP = ESPFC_POSHOLD_VEL_P;
  float velI = ESPFC_POSHOLD_VEL_I;
  float iLimit = ESPFC_POSHOLD_I_LIMIT_MSS;
  float maxAccel = ESPFC_POSHOLD_MAX_ACCEL_MSS;
  float maxAngle = ESPFC_POSHOLD_MAX_ANGLE_DEG * 0.017453292519943295f; // rad
  float deadband = ESPFC_POSHOLD_DEADBAND;
  float brakeSpeed = ESPFC_POSHOLD_BRAKE_SPEED_MS;
  float brakeTimeout = ESPFC_POSHOLD_BRAKE_TIMEOUT_S;
  float maxError = ESPFC_POSHOLD_MAX_ERROR_M;
  float filterTau = ESPFC_POSHOLD_FILTER_TAU_S;
  float positionDeadband = ESPFC_POSHOLD_DEADBAND_M;
  float maxInnovation = ESPFC_POSHOLD_MAX_INNOVATION_M;
};
 
enum class PosHoldPhase : uint8_t
{
  OFF = 0,
  HOLD = 1,    // holding latched position
  PILOT = 2,   // pilot stick override
  BRAKE = 3,   // stick released, stopping
};
 
struct PositionHoldInput
{
  int32_t lat = 0;       // deg * 1e7
  int32_t lon = 0;       // deg * 1e7
  float velNorth = 0.0f; // m/s
  float velEast = 0.0f;  // m/s
  float horizontalAccuracy = 0.0f; // m; 0 means unavailable
  uint32_t gpsTimestampUs = 0; // changes only when a new GPS solution arrives
  float heading = 0.0f;  // rad, clockwise from north
  float stickRoll = 0.0f;  // -1..1 normalised
  float stickPitch = 0.0f; // -1..1 normalised
  float dt = 0.02f;        // s
};
 
struct PositionHoldOutput
{
  float rollAngle = 0.0f;  // rad, same sign as roll stick
  float pitchAngle = 0.0f; // rad, same sign as pitch stick
  bool controlling = false; // true: angles replace the pilot's angle request
  PosHoldPhase phase = PosHoldPhase::OFF;
  float errNorth = 0.0f;   // m, target - current
  float errEast = 0.0f;    // m
  float velTargetN = 0.0f; // m/s
  float velTargetE = 0.0f; // m/s
  float accelN = 0.0f;     // m/s^2
  float accelE = 0.0f;     // m/s^2
};
 
class PositionHold
{
public:
  static constexpr float GRAVITY = 9.80665f;
 
  explicit PositionHold(const PositionHoldParams& p = PositionHoldParams{}): _p(p) {}
 
  void reset()
  {
    _phase = PosHoldPhase::OFF;
    _iN = _iE = 0.0f;
    _brakeTime = 0.0f;
    _latched = false;
    _filterInitialized = false;
    _filterLastGpsTs = 0;
    _filteredLat = 0.0;
    _filteredLon = 0.0;
  }
 
  PosHoldPhase phase() const { return _phase; }
  const PositionHoldParams& params() const { return _p; }
 
  // metres north/east from (lat, lon) to the target; flat-earth, fine for < few hundred m
  static void deltaMeters(int32_t targetLat, int32_t targetLon, int32_t lat, int32_t lon, float& north, float& east)
  {
    constexpr float M_PER_UNIT = 1.113195e-2f; // 111319.5 m/deg / 1e7
    int64_t dlon = static_cast<int64_t>(targetLon) - static_cast<int64_t>(lon);
    if (dlon > 1800000000LL) dlon -= 3600000000LL;
    else if (dlon < -1800000000LL) dlon += 3600000000LL;
    const float latRad = static_cast<float>(lat) * 1e-7f * 0.017453292519943295f;
    north = static_cast<float>(static_cast<int64_t>(targetLat) - static_cast<int64_t>(lat)) * M_PER_UNIT;
    east = static_cast<float>(dlon) * M_PER_UNIT * std::cos(latRad);
  }
 
  PositionHoldOutput update(const PositionHoldInput& in)
  {
    PositionHoldOutput out;
    const float dt = std::clamp(in.dt, 0.001f, 0.25f);
    const float speed = std::hypot(in.velNorth, in.velEast);
 
    const bool stickActive = std::fabs(in.stickRoll) > _p.deadband || std::fabs(in.stickPitch) > _p.deadband;
 
    // ---- phase machine ----------------------------------------------------
    if (!_latched)
    {
      latch(in);
      _phase = PosHoldPhase::HOLD;
    }
 
    if (stickActive)
    {
      _phase = PosHoldPhase::PILOT;
      _iN = _iE = 0.0f;
    }
    else if (_phase == PosHoldPhase::PILOT)
    {
      _phase = PosHoldPhase::BRAKE;
      _brakeTime = 0.0f;
    }
 
    if (_phase == PosHoldPhase::BRAKE)
    {
      _brakeTime += dt;
      if (speed < _p.brakeSpeed || _brakeTime > _p.brakeTimeout)
      {
        latch(in);
        _phase = PosHoldPhase::HOLD;
      }
    }
 
    out.phase = _phase;
 
    if (_phase == PosHoldPhase::PILOT)
    {
      latch(in); // target follows the aircraft while the pilot flies
      out.controlling = false;
      return out;
    }
 
    // ---- GPS position estimate ---------------------------------------------
    // Keep the raw receiver position untouched. Position Hold uses a local
    // constant-velocity prediction plus a low-pass correction from each new
    // GPS solution. This suppresses normal NEO-6M wander without turning a
    // genuine aircraft displacement into a permanent bias.
    const double METERS_PER_DEG = 111319.49079327357;
    if (!_filterInitialized)
    {
      _filteredLat = static_cast<double>(in.lat) * 1e-7;
      _filteredLon = static_cast<double>(in.lon) * 1e-7;
      _filterLastGpsTs = in.gpsTimestampUs;
      _filterInitialized = true;
    }
    else
    {
      const double latRad = _filteredLat * 0.017453292519943295;
      const double cosLat = std::max(0.1, std::fabs(std::cos(latRad)));
      _filteredLat += static_cast<double>(in.velNorth) * dt / METERS_PER_DEG;
      _filteredLon += static_cast<double>(in.velEast) * dt / (METERS_PER_DEG * cosLat);

      const bool newGps = (in.gpsTimestampUs == 0) || (in.gpsTimestampUs != _filterLastGpsTs);
      if (newGps)
      {
        float measurementDt = dt;
        if (in.gpsTimestampUs != 0 && _filterLastGpsTs != 0)
        {
          const uint32_t deltaUs = in.gpsTimestampUs - _filterLastGpsTs;
          measurementDt = std::clamp(static_cast<float>(deltaUs) * 1e-6f, 0.02f, 1.0f);
        }

        float rawN = 0.0f, rawE = 0.0f;
        deltaMeters(
            static_cast<int32_t>(std::lrint(_filteredLat * 1e7)),
            static_cast<int32_t>(std::lrint(_filteredLon * 1e7)),
            in.lat, in.lon, rawN, rawE);

        const float innovation = std::hypot(rawN, rawE);
        const float accuracyGate =
            in.horizontalAccuracy > 0.0f
                ? std::max(_p.maxInnovation, 3.0f * in.horizontalAccuracy)
                : _p.maxInnovation;

        if (innovation <= accuracyGate)
        {
          const float alpha =
              std::clamp(
                  1.0f - std::exp(-measurementDt / std::max(_p.filterTau, 0.05f)),
                  0.15f,
                  0.75f);

          const double latScale = METERS_PER_DEG;
          const double lonScale = METERS_PER_DEG * cosLat;
          _filteredLat += static_cast<double>(alpha * rawN) / latScale;
          _filteredLon += static_cast<double>(alpha * rawE) / lonScale;
        }

        _filterLastGpsTs = in.gpsTimestampUs;
      }
    }

    const int32_t filteredLat =
        static_cast<int32_t>(std::lrint(_filteredLat * 1e7));
    const int32_t filteredLon =
        static_cast<int32_t>(std::lrint(_filteredLon * 1e7));

    // ---- position error -> velocity target --------------------------------
    float eN = 0.0f, eE = 0.0f;
    deltaMeters(_targetLat, _targetLon, filteredLat, filteredLon, eN, eE);

    const float errorMagnitude = std::hypot(eN, eE);
    if (errorMagnitude < _p.positionDeadband)
    {
      eN = 0.0f;
      eE = 0.0f;
    }
 
    if (std::hypot(eN, eE) > _p.maxError)
    {
      // Too far from the hold point (GPS jump, long drift): never chase it.
      latch(in);
      eN = eE = 0.0f;
    }
 
    float vtN = 0.0f, vtE = 0.0f;
    if (_phase == PosHoldPhase::HOLD)
    {
      vtN = _p.posP * eN;
      vtE = _p.posP * eE;
      const float vt = std::hypot(vtN, vtE);
      if (vt > _p.maxSpeed)
      {
        const float k = _p.maxSpeed / vt;
        vtN *= k;
        vtE *= k;
      }
    }
    // BRAKE: velocity target stays zero, position P is not applied.
 
    // ---- velocity PI -> acceleration (north/east) -------------------------
    const float evN = vtN - in.velNorth;
    const float evE = vtE - in.velEast;
 
    float aN = _p.velP * evN + _iN;
    float aE = _p.velP * evE + _iE;
 
    float a = std::hypot(aN, aE);
    const bool saturated = a > _p.maxAccel;
    if (saturated)
    {
      const float k = _p.maxAccel / a;
      aN *= k;
      aE *= k;
    }
 
    // Integrate only while not saturated (anti-windup), and only while holding.
    if (!saturated && _phase == PosHoldPhase::HOLD)
    {
      _iN = std::clamp(_iN + _p.velI * evN * dt, -_p.iLimit, _p.iLimit);
      _iE = std::clamp(_iE + _p.velI * evE * dt, -_p.iLimit, _p.iLimit);
    }
 
    // ---- rotate north/east acceleration into body forward/right -----------
    const float c = std::cos(in.heading);
    const float s = std::sin(in.heading);
    const float aForward = aN * c + aE * s;
    const float aRight = -aN * s + aE * c;
 
    out.pitchAngle = std::clamp(std::atan(aForward / GRAVITY), -_p.maxAngle, _p.maxAngle);
    out.rollAngle = std::clamp(std::atan(aRight / GRAVITY), -_p.maxAngle, _p.maxAngle);
    out.controlling = true;
    out.errNorth = eN;
    out.errEast = eE;
    out.velTargetN = vtN;
    out.velTargetE = vtE;
    out.accelN = aN;
    out.accelE = aE;
    return out;
  }
 
private:
  void latch(const PositionHoldInput& in)
  {
    _targetLat = in.lat;
    _targetLon = in.lon;
    _latched = true;
  }
 
  PositionHoldParams _p;
  PosHoldPhase _phase = PosHoldPhase::OFF;
  int32_t _targetLat = 0;
  int32_t _targetLon = 0;
  float _iN = 0.0f;
  float _iE = 0.0f;
  float _brakeTime = 0.0f;
  bool _latched = false;

  bool _filterInitialized = false;
  uint32_t _filterLastGpsTs = 0;
  double _filteredLat = 0.0;
  double _filteredLon = 0.0;
};
 
} // namespace Espfc::Control
 



