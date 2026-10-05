#include "ResourceManager.h"
#if !defined(ESP32S2)
#include "Hal/Time.hpp"

#include <algorithm>

namespace Espfc {

namespace {
constexpr uint32_t POLICY_RATE_HZ = 100u;
constexpr uint32_t CPU_LAND_CONFIRM_US = 100000u;
constexpr uint32_t PEAK_WINDOW_US = 1000000u;
constexpr float CPU_WATCH = 65.0f;
constexpr float CPU_LOAD_MANAGEMENT = 70.0f;
constexpr float CPU_AGGRESSIVE = 75.0f;
constexpr float CPU_CRITICAL = 78.0f;
constexpr float CPU_EMERGENCY = 80.0f;
constexpr float CPU_LAND = 95.0f;
constexpr float HYST_WATCH = 62.0f;
constexpr float HYST_LOAD_MANAGEMENT = 67.0f;
constexpr float HYST_AGGRESSIVE = 72.0f;
constexpr float HYST_CRITICAL = 75.0f;
constexpr float HYST_EMERGENCY = 77.0f;
}

ResourceManager::ResourceManager(Model& model): _model(model) {}

int ResourceManager::begin()
{
  _timer.setRate(POLICY_RATE_HZ);
  _state = ResourceState::NORMAL;
  _blackboxAllowed = true;
  _mspAllowed = true;
  _telemetryAllowed = true;
  _optionalAllowed = true;
  _cpuLandActive = false;
  _cpuLoad = 0.0f;
  _cpuPeak = 0.0f;
  _peakWindowStartUs = micros();
  _highLoadSinceUs = 0;
  _flightDeadlineMisses = 0;
  _lastDeadlineMisses = 0;
  _deadlineStressUntilUs = 0;
  return 1;
}

void ResourceManager::noteFlightDeadlineMisses(uint32_t count)
{
  _flightDeadlineMisses += count;
}

ResourceState ResourceManager::classify(float cpu) const
{
  switch (_state)
  {
    case ResourceState::NORMAL:
      if (cpu >= CPU_WATCH) return ResourceState::WATCH;
      break;
    case ResourceState::WATCH:
      if (cpu >= CPU_LOAD_MANAGEMENT) return ResourceState::LOAD_MANAGEMENT;
      if (cpu < HYST_WATCH) return ResourceState::NORMAL;
      break;
    case ResourceState::LOAD_MANAGEMENT:
      if (cpu >= CPU_AGGRESSIVE) return ResourceState::AGGRESSIVE;
      if (cpu < HYST_LOAD_MANAGEMENT) return ResourceState::WATCH;
      break;
    case ResourceState::AGGRESSIVE:
      if (cpu >= CPU_CRITICAL) return ResourceState::CRITICAL_RESERVE;
      if (cpu < HYST_AGGRESSIVE) return ResourceState::LOAD_MANAGEMENT;
      break;
    case ResourceState::CRITICAL_RESERVE:
      if (cpu >= CPU_EMERGENCY) return ResourceState::EMERGENCY_RESOURCE;
      if (cpu < HYST_CRITICAL) return ResourceState::AGGRESSIVE;
      break;
    case ResourceState::EMERGENCY_RESOURCE:
      if (cpu < HYST_EMERGENCY) return ResourceState::CRITICAL_RESERVE;
      break;
    case ResourceState::CPU_LAND:
      return ResourceState::CPU_LAND;
  }
  return _state;
}

void ResourceManager::enterCpuLand(uint32_t now)
{
#if defined(ESPFC_LAND_V2_ACTIVE)
  if (_cpuLandActive || !_model.isModeActive(MODE_ARMED))
  {
    return;
  }

  auto& failsafe = _model.state.failsafe;
  failsafe.phase = FC_FAILSAFE_LANDING;
  failsafe.landingRequested = true;
  failsafe.landingRequestedUs = now;
  failsafe.landingEntryHeight = _model.state.altitude.height;
  failsafe.landingEntryVario = _model.state.altitude.vario;
  failsafe.landingEntryThrust = _model.state.output.ch[AXIS_THRUST];
  failsafe.landingEstimatorHealthy = false;
  failsafe.landingEligible = false;
  failsafe.landingActive = false;
  failsafe.landingLevelRequested = false;
  failsafe.landingDescentRequested = false;
  failsafe.landingFault = false;
  failsafe.landingOutputBlocked = true;
  failsafe.landingLastUpdateUs = 0;
  failsafe.landingTouchdownCandidate = false;
  failsafe.landingTouchdownStartedUs = 0;
  failsafe.gpsRescueLandingRequested = false;
  failsafe.gpsRescueLandNoDrop = true;

  _model.state.mode.mask &= ~(uint32_t{1} << MODE_GPS_RESCUE);
  _model.state.mode.mask &= ~(uint32_t{1} << MODE_POSHOLD);
  _model.state.mode.mask &= ~(uint32_t{1} << MODE_ALTHOLD);

  _cpuLandActive = true;
  _state = ResourceState::CPU_LAND;
#else
  (void)now;
#endif
}

void ResourceManager::applyPolicy(float cpu, uint32_t now)
{
  _cpuLoad = std::clamp(cpu, 0.0f, 100.0f);

  const uint32_t deadlineMisses = _flightDeadlineMisses;
  const uint32_t newDeadlineMisses = deadlineMisses - _lastDeadlineMisses;
  _lastDeadlineMisses = deadlineMisses;
  if (newDeadlineMisses > 0) _deadlineStressUntilUs = now + 500000u;
  const bool deadlineStress = static_cast<int32_t>(now - _deadlineStressUntilUs) < 0;

  if (_peakWindowStartUs == 0 ||
      static_cast<uint32_t>(now - _peakWindowStartUs) >= PEAK_WINDOW_US)
  {
    _cpuPeak = _cpuLoad;
    _peakWindowStartUs = now;
  }
  else
  {
    _cpuPeak = std::max(_cpuPeak, _cpuLoad);
  }

  if (_cpuLoad > CPU_LAND)
  {
    if (_highLoadSinceUs == 0) _highLoadSinceUs = now;
  }
  else
  {
    _highLoadSinceUs = 0;
  }

  if (_cpuLandActive)
  {
    _blackboxAllowed = false;
    _mspAllowed = false;
    _telemetryAllowed = false;
    _optionalAllowed = false;
    return;
  }

  if (_highLoadSinceUs != 0 &&
      static_cast<uint32_t>(now - _highLoadSinceUs) >= CPU_LAND_CONFIRM_US)
  {
    enterCpuLand(now);
    _blackboxAllowed = false;
    _mspAllowed = false;
    _telemetryAllowed = false;
    _optionalAllowed = false;
    return;
  }

  const float policyCpu = std::max(_cpuLoad, _cpuPeak);
  _state = classify(policyCpu);

  _blackboxAllowed = policyCpu < CPU_AGGRESSIVE && !deadlineStress;
  _mspAllowed = policyCpu < CPU_EMERGENCY;
  _telemetryAllowed = policyCpu < CPU_AGGRESSIVE && !deadlineStress;
  _optionalAllowed = policyCpu < CPU_LOAD_MANAGEMENT && !deadlineStress;

  if (_state == ResourceState::EMERGENCY_RESOURCE)
  {
    _blackboxAllowed = false;
    _mspAllowed = false;
    _telemetryAllowed = false;
    _optionalAllowed = false;
  }
}

void ResourceManager::update()
{
  if (!_timer.check()) return;
  const uint32_t now = micros();
  applyPolicy(_model.state.stats.getCpuLoad(), now);
}

bool ResourceManager::blackboxAllowed() const { return _blackboxAllowed; }
bool ResourceManager::mspAllowed() const { return _mspAllowed; }
bool ResourceManager::telemetryAllowed() const { return _telemetryAllowed; }
bool ResourceManager::optionalAllowed() const { return _optionalAllowed; }
bool ResourceManager::navigationAllowed() const { return !_cpuLandActive; }
ResourceState ResourceManager::state() const { return _state; }
float ResourceManager::cpuLoad() const { return _cpuLoad; }
float ResourceManager::cpuPeak() const { return _cpuPeak; }
uint32_t ResourceManager::flightDeadlineMisses() const { return _flightDeadlineMisses; }
bool ResourceManager::cpuLandActive() const { return _cpuLandActive; }

} // namespace Espfc
#endif
