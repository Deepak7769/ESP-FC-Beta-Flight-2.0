#include "Device/InputPPM.h"

#include <Arduino.h>
#include <algorithm>

#include "Utils/MemoryHelper.h"
 
namespace Espfc {

namespace Device {

void InputPPM::begin(int pin, int mode)
{
  if(_pin != -1)
  {
    detachInterrupt(_pin);
    _pin = -1;
  }

  _channel = 0;
  _channelCount = 0;
  _lastFrameChannels = 0;
  _stableFrames = 0;
  _frameValid = true;
  _new_data = false;
  _last_tick = micros();

  for(size_t i = 0; i < CHANNELS; i++)
  {
    _channels[i] = (i == 2) ? 1000 : 1500; // throttle
  }

  if(pin != -1)
  {
    _pin = pin;
    pinMode(_pin, INPUT);
#if defined(UNIT_TEST)
    // no mock available
#elif defined(ARCH_RP2040)
    attachInterruptParam(_pin, InputPPM::handle_isr, (PinStatus)mode, this);
#else
    attachInterruptArg(_pin, InputPPM::handle_isr, this, mode);
#endif
  }
}

InputStatus FAST_CODE_ATTR InputPPM::update()
{
  if(_new_data)
  {
    _new_data = false;
    return INPUT_RECEIVED;
  }
  return INPUT_IDLE;
}

uint16_t FAST_CODE_ATTR InputPPM::get(uint8_t i) const
{
  if (i >= CHANNELS)
  {
    return 0;
  }

  return _channels[i];
}

void FAST_CODE_ATTR InputPPM::get(
    uint16_t* data,
    size_t len) const
{
  if (!data)
  {
    return;
  }

  len = std::min(
      len,
      CHANNELS);

  for (size_t i = 0; i < len; i++)
  {
    data[i] = _channels[i];
  }
}

size_t InputPPM::getChannelCount() const
{
  return _channelCount;
}

bool InputPPM::needAverage() const { return true; }

void IRAM_ATTR InputPPM::handle()
{
  const uint32_t now =
      micros();

  const uint32_t width =
      static_cast<uint32_t>(
          now -
          _last_tick);

  _last_tick =
      now;

  if(width > 3000) // sync
  {
    const bool completeFrame =
        _frameValid &&
        _channel >=
            MIN_FRAME_CHANNELS &&
        _channel <=
            CHANNELS;

    if (completeFrame)
    {
      if (_channel ==
          _lastFrameChannels)
      {
        if (_stableFrames <
            STABLE_FRAMES_REQUIRED)
        {
          ++_stableFrames;
        }
      }
      else
      {
        _lastFrameChannels =
            _channel;

        _stableFrames =
            1;
      }

      // Match mature PPM decoders conceptually: only expose a frame after the
      // receiver has demonstrated a stable channel count. Do not publish after
      // only the first four pulses; AUX/mode channels belong to the same frame.
      if (_stableFrames >=
          STABLE_FRAMES_REQUIRED)
      {
        _channelCount =
            _channel;

        _new_data =
            true;
      }
    }
    else
    {
      _stableFrames =
          0;

      _lastFrameChannels =
          0;
    }

    _channel =
        0;

    _frameValid =
        true;

    return;
  }

  constexpr uint32_t MIN_PULSE_US =
      750;

  constexpr uint32_t MAX_PULSE_US =
      2250;

  if (width < MIN_PULSE_US ||
      width > MAX_PULSE_US ||
      _channel >= CHANNELS)
  {
    _frameValid =
        false;

    return;
  }

  _channels[
      _channel++] =
      static_cast<uint16_t>(
          width);
}

void IRAM_ATTR InputPPM::handle_isr(void* args)
{
  if(args) reinterpret_cast<InputPPM*>(args)->handle();
}

}

}
