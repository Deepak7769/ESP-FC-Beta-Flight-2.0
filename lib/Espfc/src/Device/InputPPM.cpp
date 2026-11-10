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
  _lastFrameChannels = 0;
  _stableFrames = 0;
  _frameValid = true;
  _writeIndex = 0;
  _readIndex = 1;
#if defined(ESP8266) || defined(ESP32) || defined(ESP32S2) || defined(ESP32S3) || defined(ESP32C3)
  _mailbox = 2u;
#else
  _mailbox.store(2u, std::memory_order_relaxed);
#endif
  _last_tick = micros();

  for(size_t i = 0; i < CHANNELS; i++)
  {
    for (auto& frame : _frames)
    {
      frame.channels[i] = (i == 2) ? 1000 : 1500; // throttle
      frame.count = 0;
    }
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
#if defined(ESP8266)
  // No lock-free exchange on LX106. The ISR cannot run while this
  // single-core consumer swaps the mailbox; no spinlock or heap access.
  noInterrupts();
  const uint32_t published = _mailbox;
  if (published & FRAME_DIRTY) _mailbox = _readIndex;
  interrupts();
  if (!(published & FRAME_DIRTY)) return INPUT_IDLE;
  _readIndex = published & FRAME_INDEX_MASK;
  return INPUT_RECEIVED;
#elif defined(ESP32) || defined(ESP32S2) || defined(ESP32S3) || defined(ESP32C3)
  // FreeRTOS spinlock works when the receiver ISR and decoder task execute
  // on different ESP32 cores; no libatomic fallback in ISR context.
  portENTER_CRITICAL(&_mailboxMux);
  const uint32_t published = _mailbox;
  if (published & FRAME_DIRTY) _mailbox = _readIndex;
  portEXIT_CRITICAL(&_mailboxMux);
  if (!(published & FRAME_DIRTY)) return INPUT_IDLE;
  _readIndex = published & FRAME_INDEX_MASK;
  return INPUT_RECEIVED;
#else
  if (_mailbox.load(std::memory_order_acquire) & FRAME_DIRTY)
  {
    const uint32_t published =
        _mailbox.exchange(_readIndex, std::memory_order_acq_rel);
    _readIndex = published & FRAME_INDEX_MASK;
    return INPUT_RECEIVED;
  }
  return INPUT_IDLE;
#endif
}

uint16_t FAST_CODE_ATTR InputPPM::get(uint8_t i) const
{
  if (i >= CHANNELS)
  {
    return 0;
  }

  return _frames[_readIndex].channels[i];
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
    data[i] = _frames[_readIndex].channels[i];
  }
}

size_t InputPPM::getChannelCount() const
{
  return _frames[_readIndex].count;
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
        auto& frame = _frames[_writeIndex];
        frame.count = _channel;
        // Never retain AUX values outside the completed frame's count.
        for (size_t i = _channel; i < CHANNELS; ++i)
          frame.channels[i] = i == 2 ? 1000 : 1500;
#if defined(ESP8266)
        // ISR already excludes the task on this single-core target.
        const uint32_t available = _mailbox;
        _mailbox = uint32_t(_writeIndex) | FRAME_DIRTY;
#elif defined(ESP32) || defined(ESP32S2) || defined(ESP32S3) || defined(ESP32C3)
        portENTER_CRITICAL_ISR(&_mailboxMux);
        const uint32_t available = _mailbox;
        _mailbox = uint32_t(_writeIndex) | FRAME_DIRTY;
        portEXIT_CRITICAL_ISR(&_mailboxMux);
#else
        const uint32_t available = _mailbox.exchange(
            uint32_t(_writeIndex) | FRAME_DIRTY,
            std::memory_order_acq_rel);
#endif
        _writeIndex = available & FRAME_INDEX_MASK;
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

  _frames[_writeIndex].channels[
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
