#pragma once
#include "ModelConfig.h"

namespace Espfc::Control {
inline uint8_t rescueMinimumSatellites(const ModelConfig& c)
{ return c.extension.stockRescueEnabled ? c.extension.stockRescue.minSats : c.gps.minSats; }
inline bool stockRescueSettings(const ModelConfig& c, StockRescueSettings& out)
{
  if (c.extension.stockRescueEnabled) { out = c.extension.stockRescue; return true; }
  const auto& r = c.gpsRescue;
  if (r.altitudeMode == 3) return false; // AT_LEAST is an ESP-FC-only mode.
  out.angle = r.maxAngleDeg;
  out.returnAltitudeM = r.altitudeMode == 2 ? r.maxAltitudeM : r.fixedAltitudeM;
  out.descentDistanceM = r.approachDistanceM;
  out.groundSpeedCms = r.speedDmS * 10u;
  out.throttleMin = 1000; out.throttleMax = 2000;
  out.throttleHover = 1000u + c.altHold.hoverThrottle * 10u;
  out.sanityChecks = 1; out.minSats = c.gps.minSats;
  out.ascendRateCms = r.climbRateDmS * 10u;
  out.descendRateCms = r.descentRateDmS * 10u;
  out.allowArmingWithoutFix = 1;
  out.altitudeMode = r.altitudeMode == 0 ? 2 : 1;
  out.minStartDistanceM = r.minDistanceM; out.initialClimbM = r.altitudeMarginM;
  return true;
}
} // namespace Espfc::Control
