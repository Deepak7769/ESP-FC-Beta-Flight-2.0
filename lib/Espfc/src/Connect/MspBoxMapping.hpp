#pragma once
#include "ModelConfig.h"
#include "Control/Capabilities.hpp"
#include <array>
#include <cstdint>

namespace Espfc::Connect {

// Wire IDs from Betaflight msp_box.c. Internal MODE_* IDs remain persistent
// and are never exposed directly to MSP. Array order matches MSP_BOXNAMES,
// MSP_BOXIDS, and MSP_STATUS flag positions.
struct MspBoxMapping
{
  static constexpr std::array<uint8_t, MODE_COUNT> permanentIds{{
    0,  // ARM
    28, // AIR MODE
    1,  // ANGLE
    200, // ESP-FC ALTHOLD extension; not retired Betaflight BARO ID 3
    13, // BEEPER
    27, // FAILSAFE
    26, // BLACKBOX
    31, // BLACKBOX ERASE
    4,  // ANTI GRAVITY
    201, // ESP-FC POS HOLD extension; not retired GPS HOLD ID 11
    46  // GPS RESCUE
  }};

  static constexpr int decode(uint8_t permanentId)
  {
    for (size_t i = 0; i < permanentIds.size(); ++i)
      if (permanentIds[i] == permanentId && Control::modeSupported(i))
        return static_cast<int>(i);
    return -1;
  }
  static constexpr uint8_t encode(uint8_t internalMode)
  {
    return internalMode < permanentIds.size() && Control::modeSupported(internalMode)
        ? permanentIds[internalMode] : 0xff;
  }
  static constexpr uint32_t statusMask(uint32_t internalMask)
  {
    uint32_t result = 0u;
    unsigned packed = 0;
    for (size_t i = 0; i < permanentIds.size(); ++i)
    {
      if (!Control::modeSupported(i)) continue;
      if (internalMask & (uint32_t{1} << i)) result |= uint32_t{1} << packed;
      ++packed;
    }
    return result;
  }
};
static_assert(MODE_COUNT == 11, "Review permanent IDs if MODE_* is extended");
static_assert(MspBoxMapping::permanentIds[MODE_GPS_RESCUE] == 46, "Bad Rescue ID");
static_assert(MspBoxMapping::permanentIds[MODE_POSHOLD] == 201, "Bad ESP-FC PosHold ID");
} // namespace Espfc::Connect
