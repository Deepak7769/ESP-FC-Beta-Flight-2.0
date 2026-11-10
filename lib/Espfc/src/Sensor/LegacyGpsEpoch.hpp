#pragma once

#include <GpsProtocol.hpp>
#include <cstdint>

namespace Espfc::Sensor {

// UBX NAV-SOL, POSLLH and VELNED are separate frames from one navigation
// epoch. Do not expose a new position, fix, or velocity to navigation until
// all three iTOW values match. Drop missing/stale epochs rather than merging
// frames from different moments.
class LegacyGpsEpoch
{
public:
  static constexpr uint32_t WEEK_MS = 604800000u;

  void reset()
  {
    _active = false;
    _hasPublished = false;
    _haveSol = _havePos = _haveVel = false;
    _tow = _publishedTow = 0;
  }

  bool put(const Gps::UbxNavSol& sol)
  {
    if (!acceptTow(sol.iTow)) return false;
    _sol = sol;
    _haveSol = true;
    return ready();
  }

  bool put(const Gps::UbxNavPosllh& pos)
  {
    if (!acceptTow(pos.iTow)) return false;
    _pos = pos;
    _havePos = true;
    return ready();
  }

  bool put(const Gps::UbxNavVelned& vel)
  {
    if (!acceptTow(vel.iTow)) return false;
    _vel = vel;
    _haveVel = true;
    return ready();
  }

  bool take(Gps::UbxNavSol& sol, Gps::UbxNavPosllh& pos,
            Gps::UbxNavVelned& vel)
  {
    if (!ready()) return false;
    if (_hasPublished && !forward(_tow, _publishedTow))
    {
      clearPending();
      return false;
    }
    sol = _sol;
    pos = _pos;
    vel = _vel;
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
    // A frame more than half a week away is considered old/out-of-order,
    // not a forward progress indication. Week rollover remains supported.
    return elapsed > 0u && elapsed < WEEK_MS / 2u;
  }

private:
  bool acceptTow(uint32_t tow)
  {
    if (tow >= WEEK_MS) return false;
    if (!_active)
    {
      if (_hasPublished && !forward(tow, _publishedTow)) return false;
      _tow = tow;
      _active = true;
      _haveSol = _havePos = _haveVel = false;
      return true;
    }
    if (tow == _tow) return true;
    if (!forward(tow, _tow) ||
        (_hasPublished && !forward(tow, _publishedTow)))
      return false;
    // A newer navigation epoch makes the incomplete previous one obsolete.
    _tow = tow;
    _haveSol = _havePos = _haveVel = false;
    return true;
  }

  void clearPending()
  {
    _active = false;
    _haveSol = _havePos = _haveVel = false;
  }

  bool _active{false};
  bool _hasPublished{false};
  bool _haveSol{false};
  bool _havePos{false};
  bool _haveVel{false};
  uint32_t _tow{0};
  uint32_t _publishedTow{0};
  Gps::UbxNavSol _sol{};
  Gps::UbxNavPosllh _pos{};
  Gps::UbxNavVelned _vel{};
};

} // namespace Espfc::Sensor
