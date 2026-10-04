#pragma once

#include <cstdint>

#include "ModelConfig.h"

namespace Espfc::Device {

struct RangefinderMeasurement
{
  bool valid{false};
  float distance{0.0f};
  uint8_t quality{0};
};

class RangefinderDevice
{
public:
  using DeviceType = RangefinderType;

  virtual ~RangefinderDevice() = default;
  virtual int begin(const RangefinderConfig& config) = 0;
  virtual int update() = 0;
  virtual bool present() const = 0;
  virtual RangefinderMeasurement measurement() const = 0;
  virtual DeviceType getType() const = 0;

  static const char** getNames();
  static const char* getName(DeviceType type);
};

} // namespace Espfc::Device
