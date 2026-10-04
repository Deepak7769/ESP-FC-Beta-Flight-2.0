#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace Espfc::Control {

struct NavigationInput
{
  bool homeValid{false};
  bool gpsValid{false};
  bool attitudeValid{false};
  bool altitudeValid{false};
  uint32_t gpsAgeUs{0};
  uint8_t sats{0};
  float horizontalAccuracy{0.0f};
  int32_t latitudeE7{0};
  int32_t longitudeE7{0};
  int32_t homeLatitudeE7{0};
  int32_t homeLongitudeE7{0};
  float velocityNorth{0.0f};
  float velocityEast{0.0f};
  float velocityDown{0.0f};
  float altitude{0.0f};
  float homeAltitude{0.0f};
};

struct NavigationState
{
  bool homeValid{false};
  bool positionValid{false};
  bool velocityValid{false};
  bool altitudeValid{false};
  bool healthy{false};
  uint8_t sats{0};
  float horizontalAccuracy{0.0f};
  uint32_t gpsAgeUs{0};
  float north{0.0f};
  float east{0.0f};
  float down{0.0f};
  float velocityNorth{0.0f};
  float velocityEast{0.0f};
  float velocityDown{0.0f};
  float distanceToHome{0.0f};
  float bearingToHome{0.0f};
  float altitudeAboveHome{0.0f};
};

class Navigation
{
public:
  static constexpr float PI = 3.14159265358979323846f;
  static constexpr float DEG_TO_RAD = PI / 180.0f;
  static constexpr float ACCEL_G = 9.80665f;
  static constexpr double METERS_PER_DEG = 111319.49079327357;

  static float wrapRadians(float a)
  {
    while (a > PI) a -= 2.0f * PI;
    while (a < -PI) a += 2.0f * PI;
    return a;
  }

  static NavigationState update(const NavigationInput& in)
  {
    NavigationState out{};
    out.homeValid = in.homeValid;
    out.sats = in.sats;
    out.horizontalAccuracy = in.horizontalAccuracy;
    out.gpsAgeUs = in.gpsAgeUs;

    out.altitudeValid =
        in.altitudeValid &&
        std::isfinite(in.altitude) &&
        std::isfinite(in.homeAltitude);

    if (out.altitudeValid)
    {
      out.down = in.homeAltitude - in.altitude;
      out.altitudeAboveHome = in.altitude - in.homeAltitude;
    }

    out.velocityNorth = in.velocityNorth;
    out.velocityEast = in.velocityEast;
    out.velocityDown = in.velocityDown;

    out.velocityValid =
        in.gpsValid &&
        std::isfinite(in.velocityNorth) &&
        std::isfinite(in.velocityEast) &&
        std::isfinite(in.velocityDown);

    if (in.homeValid && in.gpsValid)
    {
      const double lat =
          static_cast<double>(in.latitudeE7) * 1e-7;
      const double lon =
          static_cast<double>(in.longitudeE7) * 1e-7;
      const double homeLat =
          static_cast<double>(in.homeLatitudeE7) * 1e-7;
      const double homeLon =
          static_cast<double>(in.homeLongitudeE7) * 1e-7;

      double dLon = lon - homeLon;
      while (dLon > 180.0) dLon -= 360.0;
      while (dLon < -180.0) dLon += 360.0;

      const double cosLat =
          std::max(
              0.1,
              std::fabs(
                  std::cos(
                      homeLat *
                      static_cast<double>(DEG_TO_RAD))));

      out.north =
          static_cast<float>(
              (lat - homeLat) *
              METERS_PER_DEG);
      out.east =
          static_cast<float>(
              dLon *
              METERS_PER_DEG *
              cosLat);

      out.positionValid =
          std::isfinite(out.north) &&
          std::isfinite(out.east);

      if (out.positionValid)
      {
        out.distanceToHome =
            std::hypot(
                out.north,
                out.east);

        out.bearingToHome =
            std::atan2(
                -out.east,
                -out.north);

        if (out.bearingToHome < 0.0f)
        {
          out.bearingToHome +=
              2.0f * PI;
        }
      }
    }

    out.healthy =
        out.homeValid &&
        out.positionValid &&
        out.velocityValid &&
        out.altitudeValid &&
        in.attitudeValid;

    return out;
  }
};

} // namespace Espfc::Control
