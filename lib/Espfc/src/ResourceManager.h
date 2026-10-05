#pragma once

#include <cstdint>
#include "Model.h"

namespace Espfc {

#if defined(ESP32S2)

enum class ResourceState : uint8_t
{
  NORMAL = 0,
  WATCH,
  LOAD_MANAGEMENT,
  AGGRESSIVE,
  CRITICAL_RESERVE,
  EMERGENCY_RESOURCE,
  CPU_LAND,
};

class ResourceManager
{
public:
  explicit ResourceManager(Model&) {}
  int begin() { return 1; }
  void update() {}
  void noteFlightDeadlineMisses(uint32_t) {}
  bool blackboxAllowed() const { return true; }
  bool mspAllowed() const { return true; }
  bool telemetryAllowed() const { return true; }
  bool optionalAllowed() const { return true; }
  bool navigationAllowed() const { return true; }
  ResourceState state() const { return ResourceState::NORMAL; }
  float cpuLoad() const { return 0.0f; }
  float cpuPeak() const { return 0.0f; }
  uint32_t flightDeadlineMisses() const { return 0; }
  bool cpuLandActive() const { return false; }
};

#else

enum class ResourceState : uint8_t
{
  NORMAL = 0,
  WATCH,
  LOAD_MANAGEMENT,
  AGGRESSIVE,
  CRITICAL_RESERVE,
  EMERGENCY_RESOURCE,
  CPU_LAND,
};

class ResourceManager
{
public:
  explicit ResourceManager(Model& model);
  int begin();
  void update();
  void noteFlightDeadlineMisses(uint32_t count);
  bool blackboxAllowed() const;
  bool mspAllowed() const;
  bool telemetryAllowed() const;
  bool optionalAllowed() const;
  bool navigationAllowed() const;
  ResourceState state() const;
  float cpuLoad() const;
  float cpuPeak() const;
  uint32_t flightDeadlineMisses() const;
  bool cpuLandActive() const;

private:
  void applyPolicy(float cpu, uint32_t now);
  void enterCpuLand(uint32_t now);
  ResourceState classify(float cpu) const;

  Model& _model;
  Utils::Timer _timer;
  ResourceState _state{ResourceState::NORMAL};
  bool _blackboxAllowed{true};
  bool _mspAllowed{true};
  bool _telemetryAllowed{true};
  bool _optionalAllowed{true};
  bool _cpuLandActive{false};
  float _cpuLoad{0.0f};
  float _cpuPeak{0.0f};
  uint32_t _peakWindowStartUs{0};
  uint32_t _highLoadSinceUs{0};
  uint32_t _flightDeadlineMisses{0};
};

#endif

} // namespace Espfc
