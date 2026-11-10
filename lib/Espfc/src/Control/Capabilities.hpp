#pragma once
#include "ModelConfig.h"
#include "Control/AssistedModeV2.h"

namespace Espfc::Control {
constexpr uint32_t supportedFeatures()
{
  return FEATURE_RX_PPM | FEATURE_RX_SERIAL | FEATURE_MOTOR_STOP |
      FEATURE_GPS | FEATURE_TELEMETRY
#if defined(ESPFC_SERIAL_SOFT_0)
      | FEATURE_SOFTSERIAL
#endif
#if defined(ESPFC_ESPNOW)
      | FEATURE_RX_SPI
#endif
#if defined(ESPFC_ANTI_GRAVITY_ACTIVE)
      | FEATURE_ANTI_GRAVITY
#endif
      ;
}
constexpr bool softSerialSupported()
{
#if defined(ESPFC_SERIAL_SOFT_0)
  return true;
#else
  return false;
#endif
}
constexpr bool vtxSupported()
{
#if defined(ESP32) || defined(UNIT_TEST)
  return true;
#else
  return false;
#endif
}
constexpr bool modeSupported(uint8_t mode)
{
  if (mode >= MODE_COUNT) return false;
  switch (mode)
  {
    case MODE_ALTHOLD:
    case MODE_POSHOLD:
    case MODE_GPS_RESCUE:
#if defined(ESPFC_ALTHOLD_V2_ACTIVE) && defined(ESPFC_LAND_V2_ACTIVE)
      return true;
#else
      return false;
#endif
    case MODE_ANTI_GRAVITY:
#if defined(ESPFC_ANTI_GRAVITY_ACTIVE)
      return true;
#else
      return false;
#endif
    default: return true;
  }
}
constexpr bool outputProtocolSupported(int protocol)
{
  return protocol >= ESC_PROTOCOL_PWM && protocol <= ESC_PROTOCOL_DSHOT600;
}
inline bool outputTimingValid(const ModelConfig& c)
{
  // A PWM period must contain the complete longest high pulse. Oneshot
  // protocols scale the logical 1000..2000us command by 8 or 24.
  uint32_t periodScale = 0;
  if (c.output.protocol == ESC_PROTOCOL_PWM) periodScale = 1;
  if (c.output.protocol == ESC_PROTOCOL_ONESHOT125) periodScale = 8;
  if (c.output.protocol == ESC_PROTOCOL_ONESHOT42) periodScale = 24;
  if (periodScale && (c.output.rate <= 0 || c.output.maxThrottle <= 0 ||
      uint64_t(c.output.rate) * c.output.maxThrottle >= 1000000u * periodScale)) return false;
  for (const auto& channel : c.output.channel)
    if (channel.servo && (c.output.servoRate <= 0 || channel.max <= 0 ||
        uint64_t(c.output.servoRate) * channel.max >= 1000000u)) return false;
  return true;
}
} // namespace Espfc::Control
