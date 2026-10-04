#pragma once
#include "Control/Navigation.h"
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace Espfc::Control {

enum class GpsRescuePhase : uint8_t
{
  IDLE = 0,
  INITIALIZE,
  CLIMB,
  ALIGN,
  RETURN,
  APPROACH,
  DESCEND,
  LAND,
  COMPLETE,
  ABORT
};

enum GpsRescueFault : uint16_t
{
  GPS_RESCUE_FAULT_NONE       = 0,
  GPS_RESCUE_FAULT_HOME       = 1u << 0,
  GPS_RESCUE_FAULT_GPS        = 1u << 1,
  GPS_RESCUE_FAULT_STALE      = 1u << 2,
  GPS_RESCUE_FAULT_SATS       = 1u << 3,
  GPS_RESCUE_FAULT_ACCURACY   = 1u << 4,
  GPS_RESCUE_FAULT_ATTITUDE   = 1u << 5,
  GPS_RESCUE_FAULT_ALTITUDE   = 1u << 6,
  GPS_RESCUE_FAULT_POSITION    = 1u << 7,
  GPS_RESCUE_FAULT_VELOCITY    = 1u << 8,
  GPS_RESCUE_FAULT_NUMERIC     = 1u << 9,
  GPS_RESCUE_FAULT_NO_PROGRESS = 1u << 10
};

enum GpsRescueAltitudeMode : uint8_t
{
  GPS_RESCUE_ALT_CURRENT_MARGIN = 0,
  GPS_RESCUE_ALT_FIXED = 1,
  GPS_RESCUE_ALT_MAX = 2,
  GPS_RESCUE_ALT_AT_LEAST = 3,
  GPS_RESCUE_ALTITUDE_MODE_COUNT
};

struct GpsRescueParams
{
  uint8_t altitudeMode{GPS_RESCUE_ALT_CURRENT_MARGIN};
  float altitudeMargin{10.0f};
  float fixedAltitude{20.0f};
  float maxAltitude{50.0f};
  float climbRate{2.0f};
  float descentRate{0.6f};
  float maxSpeed{8.0f};
  float approachSpeed{2.0f};
  float maxAcceleration{3.0f};
  float maxAngle{25.0f * Navigation::DEG_TO_RAD};
  float approachDistance{15.0f};
  float landDistance{4.0f};
  float minDistance{10.0f};
  float alignTolerance{15.0f * Navigation::DEG_TO_RAD};
  float minLandingAltitude{0.8f};
  float gpsStaleS{0.5f};
  float maxHorizontalAccuracy{10.0f};
  float positionGain{0.8f};
  float velocityGain{1.5f};
  float yawGain{2.0f};
  float maxYawRate{1.2f};
  float altitudeTolerance{0.5f};
  float headingBlend{0.06f};
};

struct GpsRescueInput
{
  bool requested{false};
  bool armed{false};
  bool homeValid{false};
  bool gpsValid{false};
  bool attitudeHealthy{false};
  bool altitudeHealthy{false};
  bool positionValid{false};
  bool velocityValid{false};
  uint8_t sats{0};
  float horizontalAccuracy{0.0f};
  float gpsAgeS{0.0f};
  float north{0.0f};
  float east{0.0f};
  float velocityNorth{0.0f};
  float velocityEast{0.0f};
  float altitudeAboveHome{0.0f};
  float verticalRate{0.0f};
  float yawHeading{0.0f};
  float courseOverGround{0.0f};
  bool courseValid{false};
  float groundSpeed{0.0f};
  float dt{0.005f};
};

struct GpsRescueOutput
{
  bool active{false};
  bool controlling{false};
  bool requestLand{false};
  GpsRescuePhase phase{GpsRescuePhase::IDLE};
  uint16_t faultFlags{GPS_RESCUE_FAULT_NONE};
  float rollAngle{0.0f};
  float pitchAngle{0.0f};
  float yawRate{0.0f};
  float verticalRate{0.0f};
  float targetAltitude{0.0f};
  float targetVelocityNorth{0.0f};
  float targetVelocityEast{0.0f};
  float distanceToHome{0.0f};
  float bearingToHome{0.0f};
  float navigationHeading{0.0f};
};

class GpsRescue
{
public:
  void reset()
  {
    _phase = GpsRescuePhase::IDLE;
    _targetAltitude = 0.0f;
    _phaseElapsed = 0.0f;
    _returnStartDistance = 0.0f;
    _bestReturnDistance = 0.0f;
    _noProgressElapsed = 0.0f;
    _headingOffset = 0.0f;
    _headingOffsetValid = false;
  }

  GpsRescueOutput update(
      const GpsRescueInput& in,
      const GpsRescueParams& p)
  {
    GpsRescueOutput out{};
    out.active = in.requested && in.armed;

    if (!out.active)
    {
      reset();
      return out;
    }

    const uint16_t faults =
        validate(in, p);

    if (faults != GPS_RESCUE_FAULT_NONE)
    {
      _phase = GpsRescuePhase::ABORT;
      out.active = true;
      out.controlling = false;
      out.requestLand = in.altitudeHealthy;
      out.phase = _phase;
      out.faultFlags = faults;
      out.targetAltitude = _targetAltitude;
      return out;
    }

    updateHeadingEstimate(in, p);
    _phaseElapsed += std::max(in.dt, 0.001f);

    if (_phase == GpsRescuePhase::IDLE)
    {
      initialize(in, p);
    }

    const float distance =
        std::hypot(in.north, in.east);
    const float altitude =
        std::max(0.0f, in.altitudeAboveHome);

    out.distanceToHome = distance;
    out.bearingToHome =
        Navigation::wrapRadians(
            std::atan2(-in.east, -in.north));
    if (out.bearingToHome < 0.0f)
    {
      out.bearingToHome += 2.0f * Navigation::PI;
    }
    out.navigationHeading = navigationHeading(in);
    out.targetAltitude = _targetAltitude;

    switch (_phase)
    {
      case GpsRescuePhase::CLIMB:
        out.controlling = true;
        out.verticalRate =
            std::clamp(
                (_targetAltitude - altitude) * 1.2f,
                0.0f,
                p.climbRate);

        // Establish safe altitude before starting the horizontal return leg.
        // Existing horizontal velocity is still damped during the climb.
        horizontalHold(
            in,
            p,
            out);
        yawControl(
            out.bearingToHome,
            out.navigationHeading,
            p,
            out);

        if (distance <= p.landDistance)
        {
          _phase =
              altitude <= p.minLandingAltitude
                  ? GpsRescuePhase::LAND
                  : GpsRescuePhase::DESCEND;
          _phaseElapsed = 0.0f;
        }
        else if (
            altitude >=
            _targetAltitude - p.altitudeTolerance)
        {
          _phase = GpsRescuePhase::ALIGN;
          _phaseElapsed = 0.0f;
        }
        break;

      case GpsRescuePhase::ALIGN:
        out.controlling = true;
        out.verticalRate =
            std::clamp(
                (_targetAltitude - altitude) * 1.2f,
                -p.descentRate,
                p.climbRate);

        horizontalHold(
            in,
            p,
            out);
        yawControl(
            out.bearingToHome,
            out.navigationHeading,
            p,
            out);

        if (
            std::fabs(
                Navigation::wrapRadians(
                    out.bearingToHome -
                    out.navigationHeading)) <=
                p.alignTolerance ||
            _phaseElapsed >= 4.0f)
        {
          _phase = GpsRescuePhase::RETURN;
          _phaseElapsed = 0.0f;
        }
        break;

      case GpsRescuePhase::RETURN:
      {
        out.controlling = true;
        out.verticalRate =
            verticalReturnRate(
                altitude,
                p);

        // Taper horizontal speed as home is approached.
        const float taperDistance =
            std::max(
                p.approachDistance * 2.0f,
                p.approachDistance + 1.0f);
        const float speedFraction =
            std::clamp(
                distance / taperDistance,
                0.0f,
                1.0f);
        const float returnSpeed =
            std::max(
                p.approachSpeed,
                p.maxSpeed * speedFraction);

        horizontalControl(
            in,
            p,
            0.0f,
            0.0f,
            returnSpeed,
            out);
        yawControl(
            out.bearingToHome,
            out.navigationHeading,
            p,
            out);

        if (distance < _bestReturnDistance - 0.5f)
        {
          _bestReturnDistance = distance;
          _noProgressElapsed = 0.0f;
        }
        else
        {
          _noProgressElapsed +=
              std::max(
                  in.dt,
                  0.001f);
        }

        const float progressTimeout =
            std::clamp(
                (_returnStartDistance /
                     std::max(
                         p.maxSpeed,
                         0.5f)) *
                    3.0f +
                    15.0f,
                20.0f,
                120.0f);

        // Never allow an apparently healthy but non-progressing return leg
        // to continue indefinitely. The existing supervisor will land.
        if (_noProgressElapsed >= 8.0f ||
            _phaseElapsed >= progressTimeout)
        {
          _phase = GpsRescuePhase::ABORT;
          out.controlling = false;
          out.requestLand = in.altitudeHealthy;
          out.faultFlags |=
              GPS_RESCUE_FAULT_NO_PROGRESS;
          break;
        }

        if (distance <= p.approachDistance)
        {
          _phase = GpsRescuePhase::APPROACH;
          _phaseElapsed = 0.0f;
          _noProgressElapsed = 0.0f;
        }
        break;
      }

      case GpsRescuePhase::APPROACH:
        out.controlling = true;
        out.verticalRate =
            altitude >
                p.minLandingAltitude +
                p.altitudeTolerance
                ? 0.0f
                : -p.descentRate;

        horizontalControl(
            in,
            p,
            0.0f,
            0.0f,
            p.approachSpeed,
            out);
        yawControl(
            out.bearingToHome,
            out.navigationHeading,
            p,
            out);

        if (distance <= p.landDistance)
        {
          _phase =
              altitude <=
                      p.minLandingAltitude + 0.5f
                  ? GpsRescuePhase::LAND
                  : GpsRescuePhase::DESCEND;
          _phaseElapsed = 0.0f;
        }
        break;

      case GpsRescuePhase::DESCEND:
        out.controlling = true;
        out.verticalRate = -p.descentRate;

        horizontalControl(
            in,
            p,
            0.0f,
            0.0f,
            p.approachSpeed,
            out);
        yawControl(
            out.bearingToHome,
            out.navigationHeading,
            p,
            out);

        if (
            distance <= p.landDistance + 0.5f &&
            altitude <= p.minLandingAltitude)
        {
          _phase = GpsRescuePhase::LAND;
          _phaseElapsed = 0.0f;
        }
        break;

      case GpsRescuePhase::LAND:
        out.controlling = false;
        out.requestLand = true;
        out.verticalRate = 0.0f;
        out.yawRate = 0.0f;
        break;

      case GpsRescuePhase::COMPLETE:
      case GpsRescuePhase::ABORT:
        out.controlling = false;
        break;

      default:
        break;
    }

    out.phase = _phase;
    out.targetAltitude = _targetAltitude;
    return out;
  }

private:
  static void horizontalControl(
      const GpsRescueInput& in,
      const GpsRescueParams& p,
      float targetNorth,
      float targetEast,
      float maxSpeed,
      GpsRescueOutput& out);

  static void horizontalHold(
      const GpsRescueInput& in,
      const GpsRescueParams& p,
      GpsRescueOutput& out)
  {
    horizontalControl(
        in,
        p,
        in.north,
        in.east,
        0.0f,
        out);
  }

  static uint16_t validate(
      const GpsRescueInput& in,
      const GpsRescueParams& p)
  {
    uint16_t f = GPS_RESCUE_FAULT_NONE;
    if (!in.homeValid) f |= GPS_RESCUE_FAULT_HOME;
    if (!in.gpsValid) f |= GPS_RESCUE_FAULT_GPS;
    if (in.gpsAgeS > p.gpsStaleS) f |= GPS_RESCUE_FAULT_STALE;
    if (in.sats < 4) f |= GPS_RESCUE_FAULT_SATS;
    if (in.horizontalAccuracy <= 0.0f ||
        in.horizontalAccuracy > p.maxHorizontalAccuracy)
      f |= GPS_RESCUE_FAULT_ACCURACY;
    if (!in.attitudeHealthy) f |= GPS_RESCUE_FAULT_ATTITUDE;
    if (!in.altitudeHealthy) f |= GPS_RESCUE_FAULT_ALTITUDE;
    if (!in.positionValid) f |= GPS_RESCUE_FAULT_POSITION;
    if (!in.velocityValid) f |= GPS_RESCUE_FAULT_VELOCITY;

    const bool finite =
        std::isfinite(in.north) &&
        std::isfinite(in.east) &&
        std::isfinite(in.velocityNorth) &&
        std::isfinite(in.velocityEast) &&
        std::isfinite(in.altitudeAboveHome) &&
        std::isfinite(in.verticalRate) &&
        std::isfinite(in.yawHeading);

    if (!finite)
      f |= GPS_RESCUE_FAULT_NUMERIC;
    return f;
  }

  void initialize(
      const GpsRescueInput& in,
      const GpsRescueParams& p)
  {
    const float altitude =
        std::max(0.0f, in.altitudeAboveHome);

    switch (p.altitudeMode)
    {
      case GPS_RESCUE_ALT_FIXED:
        _targetAltitude = p.fixedAltitude;
        break;
      case GPS_RESCUE_ALT_MAX:
        _targetAltitude = p.maxAltitude;
        break;
      case GPS_RESCUE_ALT_AT_LEAST:
        _targetAltitude =
            std::max(
                p.fixedAltitude,
                altitude + p.altitudeMargin);
        break;
      case GPS_RESCUE_ALT_CURRENT_MARGIN:
      default:
        _targetAltitude =
            altitude + p.altitudeMargin;
        break;
    }

    _targetAltitude =
        std::max(
            _targetAltitude,
            altitude);
    _targetAltitude =
        std::min(
            _targetAltitude,
            std::max(
                p.maxAltitude,
                altitude));
    _phaseElapsed = 0.0f;

    const float distance =
        std::hypot(
            in.north,
            in.east);

    _returnStartDistance = distance;
    _bestReturnDistance = distance;
    _noProgressElapsed = 0.0f;

    if (distance <= p.landDistance)
    {
      _phase =
          altitude <= p.minLandingAltitude
              ? GpsRescuePhase::LAND
              : GpsRescuePhase::DESCEND;
    }
    else if (distance < p.minDistance)
    {
      // Already inside the configured rescue envelope: do not start a full
      // return leg; use the final approach/descent path.
      _phase =
          altitude <=
                  p.minLandingAltitude +
                  p.altitudeTolerance
              ? GpsRescuePhase::DESCEND
              : GpsRescuePhase::APPROACH;
    }
    else
    {
      _phase =
          _targetAltitude >
              altitude + p.altitudeTolerance
              ? GpsRescuePhase::CLIMB
              : GpsRescuePhase::ALIGN;
    }
  }

  float navigationHeading(
      const GpsRescueInput& in) const
  {
    return Navigation::wrapRadians(
        in.yawHeading +
        (_headingOffsetValid
             ? _headingOffset
             : 0.0f));
  }

  void updateHeadingEstimate(
      const GpsRescueInput& in,
      const GpsRescueParams& p)
  {
    if (!in.courseValid || in.groundSpeed < 0.5f)
      return;

    const float desiredOffset =
        Navigation::wrapRadians(
            in.courseOverGround -
            in.yawHeading);

    if (!_headingOffsetValid)
    {
      _headingOffset = desiredOffset;
      _headingOffsetValid = true;
      return;
    }

    const float offsetError =
        Navigation::wrapRadians(
            desiredOffset -
            _headingOffset);

    _headingOffset =
        Navigation::wrapRadians(
            _headingOffset +
            std::clamp(
                offsetError * p.headingBlend,
                -0.15f,
                0.15f));
  }

  static float verticalReturnRate(
      float altitude,
      const GpsRescueParams& p)
  {
    if (
        altitude >
        p.minLandingAltitude +
            p.altitudeTolerance * 2.0f)
      return 0.0f;

    return std::clamp(
        (p.minLandingAltitude +
         p.altitudeTolerance * 2.0f -
         altitude),
        0.0f,
        p.climbRate);
  }

  static void yawControl(
      float bearing,
      float heading,
      const GpsRescueParams& p,
      GpsRescueOutput& out)
  {
    const float error =
        Navigation::wrapRadians(
            bearing -
            heading);

    out.yawRate =
        -std::clamp(
            error * p.yawGain,
            -p.maxYawRate,
            p.maxYawRate);
  }

  static void horizontalControl(
      const GpsRescueInput& in,
      const GpsRescueParams& p,
      float targetNorth,
      float targetEast,
      float maxSpeed,
      GpsRescueOutput& out)
  {
    const float errorNorth =
        targetNorth - in.north;
    const float errorEast =
        targetEast - in.east;

    float targetVNorth =
        p.positionGain * errorNorth;
    float targetVEast =
        p.positionGain * errorEast;

    const float speed =
        std::hypot(
            targetVNorth,
            targetVEast);

    if (speed > maxSpeed)
    {
      const float scale = maxSpeed / speed;
      targetVNorth *= scale;
      targetVEast *= scale;
    }

    float aNorth =
        p.velocityGain *
        (targetVNorth - in.velocityNorth);
    float aEast =
        p.velocityGain *
        (targetVEast - in.velocityEast);

    const float accel =
        std::hypot(
            aNorth,
            aEast);

    if (accel > p.maxAcceleration)
    {
      const float scale =
          p.maxAcceleration / accel;
      aNorth *= scale;
      aEast *= scale;
    }

    const float heading =
        Navigation::wrapRadians(
            in.yawHeading);

    const float c = std::cos(heading);
    const float s = std::sin(heading);
    const float forward =
        aNorth * c + aEast * s;
    const float right =
        -aNorth * s + aEast * c;

    out.pitchAngle =
        std::clamp(
            std::atan(
                forward /
                Navigation::ACCEL_G),
            -p.maxAngle,
            p.maxAngle);
    out.rollAngle =
        std::clamp(
            std::atan(
                right /
                Navigation::ACCEL_G),
            -p.maxAngle,
            p.maxAngle);

    out.targetVelocityNorth = targetVNorth;
    out.targetVelocityEast = targetVEast;
  }

  GpsRescuePhase _phase{GpsRescuePhase::IDLE};
  float _targetAltitude{0.0f};
  float _phaseElapsed{0.0f};
  float _returnStartDistance{0.0f};
  float _bestReturnDistance{0.0f};
  float _noProgressElapsed{0.0f};
  float _headingOffset{0.0f};
  bool _headingOffsetValid{false};
};

} // namespace Espfc::Control
