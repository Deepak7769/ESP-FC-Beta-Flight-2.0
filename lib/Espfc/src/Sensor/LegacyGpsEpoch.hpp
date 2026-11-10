#pragma once

#include <GpsProtocol.hpp>
#include <cstdint>

namespace Espfc::Sensor {

// Legacy UBX consists of separate SOL/POSLLH/VELNED frames. One
// published sample must have matching iTOW and bounded receipt age.
class LegacyGpsEpoch
{
public:
  static constexpr uint32_t WEEK_MS = 604800000u;
  static constexpr uint32_t MAX_PENDING_AGE_US = 500000u;

  void reset()
  {
    _active = false;
    _hasPublished = false;
    _hasRetired = false;
    _haveSol = _havePos = _haveVel = false;
    _tow = _publishedTow = _retiredTow = _firstReceivedUs = 0;
  }

  bool put(const Gps::UbxNavSol& sol, uint32_t nowUs)
  {
    if (!acceptTow(sol.iTow, nowUs)) return false;
    _sol = sol;
    _haveSol = true;
    return ready();
  }

  bool put(const Gps::UbxNavPosllh& pos, uint32_t nowUs)
  {
    if (!acceptTow(pos.iTow, nowUs)) return false;
    _pos = pos;
    _havePos = true;
    return ready();
  }

  bool put(const Gps::UbxNavVelned& vel, uint32_t nowUs)
  {
    if (!acceptTow(vel.iTow, nowUs)) return false;
    _vel = vel;
    _haveVel = true;
    return ready();
  }

  // Return the receipt time of the FIRST component, not the moment all
  // components arrived. Otherwise delayed assembly renews GPS freshness.
  bool take(Gps::UbxNavSol& sol, Gps::UbxNavPosllh& pos,
            Gps::UbxNavVelned& vel, uint32_t& firstReceivedUs,
            uint32_t nowUs)
  {
    expire(nowUs);
    if (!ready()) return false;
    if (_hasPublished && !forward(_tow, _publishedTow))
    {
      clearPending();
      return false;
    }
    sol = _sol;
    pos = _pos;
    vel = _vel;
    firstReceivedUs = _firstReceivedUs;
    _hasPublished = true;
    _publishedTow = _tow;
    clearPending();
    return true;
  }

  bool ready() const
  {
    return _active && _haveSol && _havePos && _haveVel;
  }

  static bool forward(uint32_t nextTow, uint32_t previousTow)
  {
    if (nextTow >= WEEK_MS || previousTow >= WEEK_MS)
      return false;
    const uint32_t elapsed = nextTow >= previousTow
        ? nextTow - previousTow
        : WEEK_MS - previousTow + nextTow;
    // Half-week guard rejects out-of-order data across a week rollover.
    return elapsed > 0u && elapsed < WEEK_MS / 2u;
  }

private:
  void clearPending()
  {
    _active = false;
    _haveSol = _havePos = _haveVel = false;
  }

  void expire(uint32_t nowUs)
  {
    if (!_active ||
        static_cast<uint32_t>(nowUs - _firstReceivedUs) <=
            MAX_PENDING_AGE_US)
      return;

    // Retire this epoch so a late frame with the SAME iTOW cannot
    // create an apparently fresh pending sample after expiry.
    _retiredTow = _tow;
    _hasRetired = true;
    clearPending();
  }

  bool acceptTow(uint32_t tow, uint32_t nowUs)
  {
    if (tow >= WEEK_MS) return false;
    expire(nowUs);
    if (_hasPublished && !forward(tow, _publishedTow)) return false;
    if (_hasRetired && !forward(tow, _retiredTow)) return false;

    if (!_active)
    {
      _tow = tow;
      _firstReceivedUs = nowUs;
      _active = true;
      _haveSol = _havePos = _haveVel = false;
      return true;
    }
    if (tow == _tow) return true;
    if (!forward(tow, _tow)) return false;

    // The newer epoch supersedes any incomplete prior epoch.
    _tow = tow;
    _firstReceivedUs = nowUs;
    _haveSol = _havePos = _haveVel = false;
    return true;
  }

  bool _active{false};
  bool _hasPublished{false};
  bool _hasRetired{false};
  bool _haveSol{false};
  bool _havePos{false};
  bool _haveVel{false};
  uint32_t _tow{0};
  uint32_t _publishedTow{0};
  uint32_t _retiredTow{0};
  uint32_t _firstReceivedUs{0};
  Gps::UbxNavSol _sol{};
  Gps::UbxNavPosllh _pos{};
  Gps::UbxNavVelned _vel{};
};

} // namespace Espfc::Sensor
