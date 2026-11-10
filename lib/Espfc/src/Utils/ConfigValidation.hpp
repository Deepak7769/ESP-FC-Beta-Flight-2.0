#pragma once
#include "ModelConfig.h"
#include "Utils/SerialPolicy.hpp"
#include "Control/Capabilities.hpp"
#include <cstring>

namespace Espfc::Utils {
inline bool stockRescueValid(const StockRescueSettings& r)
{
  return r.angle >= 1 && r.angle <= 60 && r.returnAltitudeM >= 1 && r.returnAltitudeM <= 1000 &&
      r.descentDistanceM >= 1 && r.descentDistanceM <= 1000 &&
      r.groundSpeedCms >= 1 && r.groundSpeedCms <= 3000 &&
      r.throttleMin >= 1000 && r.throttleMax <= 2000 &&
      r.throttleMin < r.throttleMax && r.throttleHover >= r.throttleMin &&
      r.throttleHover <= r.throttleMax && r.sanityChecks <= 2 &&
      r.minSats >= 4 && r.minSats <= 32 && r.ascendRateCms >= 1 &&
      r.ascendRateCms <= 2000 && r.descendRateCms >= 1 &&
      r.descendRateCms <= 1000 && r.allowArmingWithoutFix <= 1 &&
      r.altitudeMode <= 2 && r.minStartDistanceM <= 1000 &&
      r.initialClimbM <= 1000;
}
inline bool configurationValid(const ModelConfig& c)
{
  const auto mixerType = static_cast<MixerType>(c.mixer.type);
  const bool customMixer = mixerType == FC_MIXER_CUSTOM || mixerType == FC_MIXER_CUSTOM_TRI ||
      mixerType == FC_MIXER_CUSTOM_AIRPLANE;
  if ((!customMixer && !Output::Mixers::getMixer(mixerType, MixerConfig{}).count) ||
      !serialConfigurationValid(c) ||
      !Control::outputTimingValid(c) ||
      (c.featureMask & ~Control::supportedFeatures()) ||
      c.failsafe.procedure >= FAILSAFE_PROCEDURE_COUNT ||
      c.fusion.mode < 0 || c.fusion.mode >= FUSION_MAX ||
      c.gyro.dev < 0 || c.gyro.dev >= GYRO_MAX ||
      c.accel.dev < 0 || c.accel.dev >= GYRO_MAX ||
      c.mag.dev < 0 || c.mag.dev >= MAG_MAX || c.baro.dev < 0 || c.baro.dev >= BARO_MAX ||
      c.gyro.align < 0 || c.gyro.align > ALIGN_CUSTOM || c.mag.align < 0 || c.mag.align > ALIGN_CUSTOM ||
      c.input.airModeActivateThreshold < 0 || c.input.airModeActivateThreshold > 100 ||
      c.input.minRc < 750 || c.input.maxRc > 2250 || c.input.minRc >= c.input.midRc ||
      c.input.midRc >= c.input.maxRc || c.input.minCheck < c.input.minRc ||
      c.input.maxCheck > c.input.maxRc || c.input.minCheck >= c.input.maxCheck ||
      c.input.rateType < 0 || c.input.rateType > 5 ||
      c.output.protocol < ESC_PROTOCOL_PWM || c.output.protocol >= ESC_PROTOCOL_COUNT ||
      c.output.protocol == ESC_PROTOCOL_PROSHOT ||
      c.output.minCommand < 900 || c.output.maxThrottle > 2000 ||
      c.output.minCommand >= c.output.maxThrottle ||
      c.output.motorIdle < 0 || c.output.motorIdle > 2000 ||
      int32_t(c.output.motorIdle) >
          (int32_t(c.output.maxThrottle) - int32_t(c.output.minCommand)) * 10 ||
      c.output.rate < 50 || c.output.rate > 8000 || c.output.servoRate < 0 || c.output.servoRate > 1000 ||
      c.output.motorPoles < 2 || c.output.motorPoles > 60 || (c.output.motorPoles & 1) ||
      c.output.motorLimit < 1 || c.output.motorLimit > 100 ||
      c.output.throttleLimitType < 0 || c.output.throttleLimitType >= THROTTLE_LIMIT_TYPE_MAX ||
      c.output.throttleLimitPercent < 1 || c.output.throttleLimitPercent > 100 ||
      c.level.angleLimit < 1 || c.level.angleLimit > 85 ||
      c.arming.smallAngle > 180 ||
      c.controller.tpaScale > 100 || c.controller.tpaBreakpoint < 1000 || c.controller.tpaBreakpoint > 2000 ||
      c.iterm.relax < 0 || c.iterm.relax > 4 || c.antiGravityGain > 250 ||
      c.vtx.band < 1 || c.vtx.band > 5 || c.vtx.channel < 1 || c.vtx.channel > 8 ||
      c.vtx.power > 8 || c.vtx.lowPowerDisarm > 1 ||
      c.gpsRescue.altitudeMode > 3 || !c.gpsRescue.climbRateDmS || !c.gpsRescue.descentRateDmS ||
      !c.gpsRescue.speedDmS || !c.gpsRescue.maxAltitudeM || !c.gpsRescue.maxAccelerationCms2 ||
      c.gpsRescue.maxAngleDeg < 1 || c.gpsRescue.maxAngleDeg > 60 ||
      c.customMixerCount < 0 || c.customMixerCount > MIXER_RULE_MAX ||
      c.rangefinder.type >= RANGEFINDER_MAX ||
      (c.blackbox.dev != BLACKBOX_DEV_NONE && c.blackbox.dev != BLACKBOX_DEV_SERIAL && c.blackbox.dev != BLACKBOX_DEV_FLASH) ||
      c.blackbox.pDenom < 0 || c.blackbox.pDenom > 4 ||
      c.vbat.source < 0 || c.vbat.source > 1 || c.vbat.resDiv == 0 || c.vbat.resMult == 0 ||
      (c.ibat.source != CURRENT_METER_NONE && c.ibat.source != CURRENT_METER_ADC && c.ibat.source != CURRENT_METER_MSP) ||
      (c.ibat.source == CURRENT_METER_ADC && c.ibat.scale <= 0) ||
      c.extension.cellMinimum < 100 || c.extension.cellMaximum > 500 ||
      c.extension.cellMinimum > c.vbat.cellWarning ||
      c.vbat.cellWarning > c.extension.cellMaximum ||
      c.extension.stockRescueEnabled > 1 ||
      (c.extension.stockRescueEnabled && !Control::modeSupported(MODE_GPS_RESCUE)) || !stockRescueValid(c.extension.stockRescue)) return false;
  if (!std::memchr(c.modelName, 0, sizeof(c.modelName)) ||
      !std::memchr(c.wireless.ssid, 0, sizeof(c.wireless.ssid)) ||
      !std::memchr(c.wireless.pass, 0, sizeof(c.wireless.pass))) return false;
  for (const auto& ch : c.input.channel)
    if (ch.map < 0 || ch.map >= INPUT_CHANNELS || ch.min < 750 || ch.max > 2250 ||
        ch.min > ch.neutral || ch.neutral > ch.max || ch.fsMode < 0 || ch.fsMode > 2) return false;
  for (const auto& ch : c.output.channel)
    if (ch.min < 750 || ch.max > 2250 || ch.min > ch.neutral || ch.neutral > ch.max) return false;
  for (const auto& condition : c.conditions)
    if (condition.id >= MODE_COUNT || condition.linkId >= MODE_COUNT || condition.ch < AXIS_AUX_1 ||
        condition.ch >= AXIS_COUNT || condition.min < 900 || condition.max > 2100 ||
        condition.min > condition.max || condition.logicMode > 1) return false;
  for (const auto& sc : c.scaler)
    if (sc.channel < 0 || sc.channel >= AXIS_COUNT || sc.minScale > sc.maxScale) return false;
  const FilterConfig filters[] = {c.gyro.filter, c.gyro.filter2, c.gyro.filter3,
      c.gyro.dynLpfFilter, c.gyro.notch1Filter, c.gyro.notch2Filter,
      c.accel.filter, c.baro.filter, c.mag.filter, c.yaw.filter,
      c.dterm.filter, c.dterm.filter2, c.dterm.dynLpfFilter, c.dterm.notchFilter,
      c.input.filter, c.input.filterThrottle, c.input.filterDerivative};
  for (const auto& f : filters)
    if (f.type < 0 || f.type > FILTER_NONE || f.freq < 0 || f.cutoff < 0) return false;
  for (size_t i = 0; i < static_cast<size_t>(c.customMixerCount); ++i)
    if (c.customMixes[i].src < 0 || c.customMixes[i].src >= MIXER_SOURCE_MAX ||
        c.customMixes[i].dst < 0 || c.customMixes[i].dst >= OUTPUT_CHANNELS ||
        c.customMixes[i].rate < -1000 || c.customMixes[i].rate > 1000) return false;
  return true;
}
} // namespace Espfc::Utils
