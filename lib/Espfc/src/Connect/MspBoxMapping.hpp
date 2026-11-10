#pragma once
#include "ModelConfig.h"
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
    3,  // ALTHOLD
    13, // BEEPER
    27, // FAILSAFE
    26, // BLACKBOX
    31, // BLACKBOX ERASE
    4,  // ANTI GRAVITY
    11, // POS HOLD
    46  // GPS RESCUE
  }};

  static constexpr int decode(uint8_t permanentId)
  {
    for (size_t i = 0; i < permanentIds.size(); ++i)
      if (permanentIds[i] == permanentId)
        return static_cast<int>(i);
    return -1;
  }
  static constexpr uint8_t encode(uint8_t internalMode)
  {
    return internalMode < permanentIds.size()
        ? permanentIds[internalMode] : 0xff;
  }
  static constexpr uint32_t statusMask(uint32_t internalMask)
  {
    uint32_t result = 0u;
    for (size_t i = 0; i < permanentIds.size(); ++i)
      if ((internalMask & (uint32_t{1} << i)) != 0u)
        result |= uint32_t{1} << i;
    return result;
  }
};
static_assert(MODE_COUNT == 11, "Review permanent IDs if MODE_* is extended");
static_assert(MspBoxMapping::decode(46) == MODE_GPS_RESCUE, "Bad Rescue MSP box ID");
static_assert(MspBoxMapping::encode(MODE_POSHOLD) == 11, "Bad Position Hold MSP box ID");
} // namespace Espfc::Connect
