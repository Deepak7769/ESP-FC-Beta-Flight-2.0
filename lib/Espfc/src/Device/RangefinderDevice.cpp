#include "Device/RangefinderDevice.hpp"

namespace Espfc::Device {

const char** RangefinderDevice::getNames()
{
  static const char* names[] = {
      "NONE",
      "HC-SR04",
      nullptr};
  return names;
}

const char* RangefinderDevice::getName(DeviceType type)
{
  if (type >= RANGEFINDER_MAX)
    return "?";
  return getNames()[type];
}

} // namespace Espfc::Device
