
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
#define ESPFC_POSHOLD_FILTER_TAU_S 0.60f
#endif

#ifndef ESPFC_POSHOLD_MAX_INNOVATION_M
#define ESPFC_POSHOLD_MAX_INNOVATION_M 8.0f
#endif

#ifndef ESPFC_POSHOLD_FILTER_MIN_ALPHA
#define ESPFC_POSHOLD_FILTER_MIN_ALPHA 0.15f
#endif

#ifndef ESPFC_POSHOLD_FILTER_MAX_ALPHA
#define ESPFC_POSHOLD_FILTER_MAX_ALPHA 0.75f
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
  float maxInnovation = ESPFC_POSHOLD_MAX_INNOVATION_M;
  float filterMinAlpha = ESPFC_POSHOLD_FILTER_MIN_ALPHA;
  float filterMaxAlpha = ESPFC_POSHOLD_FILTER_MAX_ALPHA;
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
  // RAW GPS position.
  int32_t lat = 0;       // deg * 1e7
  int32_t lon = 0;       // deg * 1e7

  // Receiver velocity from NAV-VELNED / NAV-PVT.
  float velNorth = 0.0f; // m/s
  float velEast = 0.0f;  // m/s

  float heading = 0.0f;  // rad, clockwise from north

  float stickRoll = 0.0f;
  float stickPitch = 0.0f;

  float dt = 0.02f;

  // GPS solution timestamp.
  // NEO-6M NAV-POSLLH iTOW is milliseconds.
  uint32_t gpsTimestampMs = 0;

  // Horizontal accuracy from receiver.
  // Input units: metres.
  float horizontalAccuracy = 0.0f;
};
 
struct PositionHoldOutput
{
  float rollAngle = 0.0f;  // rad, same sign as roll stick
  float pitchAngle = 0.0f; // rad, same sign as pitch stick
  bool controlling = false; // true: angles replace the pilot's angle request
  PosHoldPhase phase = PosHoldPhase::OFF;
  // Final filtered GPS position used by Position Hold.
  int32_t filteredLat = 0;
  int32_t filteredLon = 0;

  // Raw-to-filtered separation.
  float rawFilteredDistance = 0.0f;

  // Whether the latest RAW GPS solution was accepted.
  bool gpsFilterAccepted = false;

  // Filtered position-derived velocity.
  float filteredVelNorth = 0.0f;
  float filteredVelEast = 0.0f;
  float filteredGroundSpeed = 0.0f;
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
  _filterLastGpsTimestampMs = 0;

  _filteredLat = 0.0;
  _filteredLon = 0.0;

  _filterLastLat = 0;
  _filterLastLon = 0;

  _previousFilteredLat = 0;
  _previousFilteredLon = 0;

  _filteredVelNorth = 0.0f;
  _filteredVelEast = 0.0f;

  _filteredVelocityInitialized = false;

  _acceptedSamples = 0;
  _rejectedSamples = 0;
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
       // -----------------------------------------------------------------
    // GPS position filter.
    //
    // RAW GPS is never modified.
    // _filteredLat/_filteredLon are the final coordinates used by
    // Position Hold.
    //
    // The filter only consumes a GPS sample when gpsTimestampMs
    // changes. This prevents the 200 Hz controller loop from treating
    // the same GPS sample as new data.
    // -----------------------------------------------------------------

    if (!_filterInitialized)
    {
      _filterInitialized = true;

      _filteredLat =
          static_cast<double>(in.lat) * 1e-7;

      _filteredLon =
          static_cast<double>(in.lon) * 1e-7;

      _previousFilteredLat = in.lat;
      _previousFilteredLon = in.lon;

      _filterLastGpsTimestampMs =
          in.gpsTimestampMs;

      _filterLastLat = in.lat;
      _filterLastLon = in.lon;

      out.gpsFilterAccepted = true;
      _acceptedSamples++;

      _filteredVelocityInitialized = false;
    }
    else if (in.gpsTimestampMs !=
             _filterLastGpsTimestampMs)
    {
      const uint32_t dtMs =
          in.gpsTimestampMs -
          _filterLastGpsTimestampMs;

      const float gpsDt =
          std::clamp(
              static_cast<float>(dtMs) * 0.001f,
              0.02f,
              2.0f);

      constexpr double METERS_PER_DEG =
          111319.49079327357;

      const double latRad =
          static_cast<double>(in.lat) *
          1e-7 *
          0.017453292519943295;

      const double cosLat =
          std::max(
              0.1,
              std::fabs(std::cos(latRad)));

      const double rawLat =
          static_cast<double>(in.lat) * 1e-7;

      const double rawLon =
          static_cast<double>(in.lon) * 1e-7;

      const double innovationNorth =
          (rawLat - _filteredLat) *
          METERS_PER_DEG;

      const double innovationEast =
          (rawLon - _filteredLon) *
          METERS_PER_DEG *
          cosLat;

      const float innovation =
          static_cast<float>(
              std::hypot(
                  innovationNorth,
                  innovationEast));

      float innovationLimit =
          _p.maxInnovation;

      if (in.horizontalAccuracy > 0.0f)
      {
        innovationLimit =
            std::max(
                innovationLimit,
                3.0f * in.horizontalAccuracy);
      }

      const bool accepted =
          innovation <= innovationLimit;

      out.gpsFilterAccepted = accepted;

      if (accepted)
      {
        const float alpha =
            std::clamp(
                1.0f -
                std::exp(
                    -gpsDt /
                    std::max(
                        0.05f,
                        _p.filterTau)),
                _p.filterMinAlpha,
                _p.filterMaxAlpha);

        _filteredLat +=
            static_cast<double>(
                alpha) *
            innovationNorth /
            METERS_PER_DEG;

        _filteredLon +=
            static_cast<double>(
                alpha) *
            innovationEast /
            (METERS_PER_DEG * cosLat);

        _acceptedSamples++;
      }
      else
      {
        _rejectedSamples++;
      }

      _filterLastGpsTimestampMs =
          in.gpsTimestampMs;

      // Filtered position-derived velocity.
      const int32_t filteredLat =
          static_cast<int32_t>(
              std::lrint(
                  _filteredLat * 1e7));

      const int32_t filteredLon =
          static_cast<int32_t>(
              std::lrint(
                  _filteredLon * 1e7));

      const double filteredNorth =
          (static_cast<double>(
              filteredLat) -
           static_cast<double>(
              _previousFilteredLat)) *
          1e-7 *
          METERS_PER_DEG;

      const double filteredEast =
          (static_cast<double>(
              filteredLon) -
           static_cast<double>(
              _previousFilteredLon)) *
          1e-7 *
          METERS_PER_DEG *
          cosLat;

      _filteredVelNorth =
          static_cast<float>(
              filteredNorth /
              gpsDt);

      _filteredVelEast =
          static_cast<float>(
              filteredEast /
              gpsDt);

      _previousFilteredLat =
          filteredLat;

      _previousFilteredLon =
          filteredLon;

      _filterLastLat = in.lat;
      _filterLastLon = in.lon;
    }

    const int32_t filteredLat =
        static_cast<int32_t>(
            std::lrint(
                _filteredLat * 1e7));

    const int32_t filteredLon =
        static_cast<int32_t>(
            std::lrint(
                _filteredLon * 1e7));

    out.filteredLat = filteredLat;
    out.filteredLon = filteredLon;

    out.filteredVelNorth =
        _filteredVelNorth;

    out.filteredVelEast =
        _filteredVelEast;

    out.filteredGroundSpeed =
        std::hypot(
            _filteredVelNorth,
            _filteredVelEast);

    float rawFilteredNorth = 0.0f;
    float rawFilteredEast = 0.0f;

    deltaMeters(
        filteredLat,
        filteredLon,
        in.lat,
        in.lon,
        rawFilteredNorth,
        rawFilteredEast);

    out.rawFilteredDistance =
        std::hypot(
            rawFilteredNorth,
            rawFilteredEast);
   
    const float dt = std::clamp(in.dt, 0.001f, 0.25f);
    const float speed = std::hypot(in.velNorth, in.velEast);
 
    const bool stickActive = std::fabs(in.stickRoll) > _p.deadband || std::fabs(in.stickPitch) > _p.deadband;
 
    // ---- phase machine ----------------------------------------------------
    if (!_latched)
    {
      latch(
          out.filteredLat,
          out.filteredLon);

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
        latch(
            out.filteredLat,
            out.filteredLon);

        _phase = PosHoldPhase::HOLD;
      }
    }
 
    out.phase = _phase;
 
    if (_phase == PosHoldPhase::PILOT)
    {
      latch(
          out.filteredLat,
          out.filteredLon); // target follows the filtered aircraft position

      out.controlling = false;
      return out;
    }
 
    // ---- position error -> velocity target --------------------------------
    float eN = 0.0f, eE = 0.0f;
    deltaMeters(
    _targetLat,
    _targetLon,
    out.filteredLat,
    out.filteredLon,
    eN,
    eE);
 
    if (std::hypot(eN, eE) > _p.maxError)
    {
      // Too far from the hold point (GPS jump, long drift): never chase it.
      // Re-latch to the FILTERED position, never the raw GPS position.
      latch(
          out.filteredLat,
          out.filteredLon);

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

out.filteredLat =
    static_cast<int32_t>(
        std::lrint(
            _filteredLat * 1e7));

out.filteredLon =
    static_cast<int32_t>(
        std::lrint(
            _filteredLon * 1e7));

out.filteredVelNorth =
    _filteredVelNorth;

out.filteredVelEast =
    _filteredVelEast;

out.filteredGroundSpeed =
    std::hypot(
        _filteredVelNorth,
        _filteredVelEast);

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

  // GPS filter state.
  bool _filterInitialized = false;

  double _filteredLat = 0.0;
  double _filteredLon = 0.0;

  uint32_t _filterLastGpsTimestampMs = 0;

  int32_t _filterLastLat = 0;
  int32_t _filterLastLon = 0;

  int32_t _previousFilteredLat = 0;
  int32_t _previousFilteredLon = 0;

  float _filteredVelNorth = 0.0f;
  float _filteredVelEast = 0.0f;

  bool _filteredVelocityInitialized = false;

  uint32_t _acceptedSamples = 0;
  uint32_t _rejectedSamples = 0;
};
 
} // namespace Espfc::Control
 



