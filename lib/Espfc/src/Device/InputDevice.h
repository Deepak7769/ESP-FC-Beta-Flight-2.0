#ifndef _ESPFC_DEVICE_INPUT_DEVICE_H_
#define _ESPFC_DEVICE_INPUT_DEVICE_H_

#include <cstddef>
#include <cstdint>

namespace Espfc {

enum InputStatus {
  INPUT_IDLE,
  INPUT_RECEIVED,
  // A complete receiver frame was present, but the protocol reports that one
  // or more preceding RF frames were dropped. Keep the repeated frame briefly,
  // but do not refresh receiver-health timing.
  INPUT_DROPPED,
  INPUT_LOST,
  INPUT_FAILSAFE
};

namespace Device {

class InputDevice
{
  public:
    virtual InputStatus update() = 0;
    virtual uint16_t get(uint8_t channel) const = 0;
    virtual void get(uint16_t * data, size_t len) const = 0;
    virtual size_t getChannelCount() const = 0;
    virtual bool needAverage() const = 0;
};

}

}

#endif
