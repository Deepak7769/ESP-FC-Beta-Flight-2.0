#pragma once
#include "ModelConfig.h"
#include "Control/Capabilities.hpp"

namespace Espfc::Utils {
inline bool serialPortValid(const SerialPortConfig& port)
{
  const uint32_t mask = static_cast<uint32_t>(port.functionMask);
  constexpr uint32_t supported = SERIAL_FUNCTION_MSP | SERIAL_FUNCTION_GPS |
      SERIAL_FUNCTION_RX_SERIAL | SERIAL_FUNCTION_BLACKBOX |
      SERIAL_FUNCTION_TELEMETRY_IBUS | SERIAL_FUNCTION_VTX_SMARTAUDIO;
  if (mask & ~supported) return false;
  if (port.id >= SERIAL_ID_SOFTSERIAL_1 && mask && mask != SERIAL_FUNCTION_MSP) return false;
  if (!Control::vtxSupported() && (mask & SERIAL_FUNCTION_VTX_SMARTAUDIO)) return false;
  // Each implemented UART consumer owns its framing and receive stream.
  // CRSF telemetry travels through its RX provider, not a second consumer.
  if (mask && (mask & (mask - 1u))) return false;
  if (mask == SERIAL_FUNCTION_MSP || mask == SERIAL_FUNCTION_GPS)
    return port.baud >= 1200 && port.baud <= 2000000;
  if (mask == SERIAL_FUNCTION_BLACKBOX)
    return port.blackboxBaud >= 1200 && port.blackboxBaud <= 2000000;
  return true;
}
inline bool serialConfigurationValid(const ModelConfig& config)
{
  unsigned gps = 0, rx = 0, vtx = 0, blackbox = 0;
  for (size_t i = 0; i < SERIAL_UART_COUNT; ++i)
  {
    const auto& p = config.serial[i];
    if (!serialPortValid(p)) return false;
    for (size_t j = 0; j < i; ++j)
      if (p.id == config.serial[j].id) return false;
    gps += !!(p.functionMask & SERIAL_FUNCTION_GPS);
    rx += !!(p.functionMask & SERIAL_FUNCTION_RX_SERIAL);
    vtx += !!(p.functionMask & SERIAL_FUNCTION_VTX_SMARTAUDIO);
    blackbox += !!(p.functionMask & SERIAL_FUNCTION_BLACKBOX);
  }
  return gps <= 1 && rx <= 1 && vtx <= 1 && blackbox <= 1;
}
} // namespace Espfc::Utils
