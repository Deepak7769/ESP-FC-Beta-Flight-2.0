#include "InputIBUS.hpp"
#include "Hal/Time.hpp"
#include "Utils/MemoryHelper.h"
#include <algorithm>

namespace Espfc::Device
{

InputIBUS::InputIBUS() : _serial(nullptr), _state(IBUS_LENGTH), _idx(0), _new_data(false) {}

int InputIBUS::begin(Stream::ReadWritable* serial)
{
  _serial = serial;
  _state = IBUS_LENGTH;
  _idx = 0;
  _new_data = false;
  _lastByteUs = 0;
  _timingValid = false;

  std::fill_n((uint8_t*)&_data, IBUS_FRAME_SIZE, 0);
  std::fill_n(_channels, CHANNELS, 0);

  return 1;
}

InputStatus FAST_CODE_ATTR InputIBUS::update()
{
  if (!_serial) return INPUT_IDLE;

  uint8_t buff[64] = {0};
  size_t len = std::min((size_t)_serial->available(), (size_t)sizeof(buff));

  if (len)
  {
    _serial->readMany(buff, len);
    uint8_t* ptr = buff;
    while(len--)
    {
      parse(_data, *ptr++);
    }
  }

  if (_new_data)
  {
    _new_data = false;
    return INPUT_RECEIVED;
  }
  return INPUT_IDLE;
}

void FAST_CODE_ATTR InputIBUS::parse(
    IBusData& frameData,
    int d)
{
  uint8_t* data =
      reinterpret_cast<uint8_t*>(&frameData);

  const uint8_t c =
      static_cast<uint8_t>(d & 0xff);

  auto resetFrame = [&]()
  {
    _state = IBUS_LENGTH;
    _idx = 0;
  };

  auto pushByte = [&](uint8_t value) -> bool
  {
    if (_idx >= IBUS_FRAME_SIZE)
    {
      resetFrame();
      return false;
    }

    data[_idx++] = value;
    return true;
  };

  constexpr uint32_t IBUS_FRAME_GAP_US =
      500;

  const uint32_t now =
      micros();

  if (_timingValid &&
      static_cast<uint32_t>(
          now -
          _lastByteUs) >
          IBUS_FRAME_GAP_US)
  {
    // FlySky iBUS receivers delimit frames with a gap. Reset on that boundary
    // so a truncated/corrupt frame cannot keep the parser shifted into the
    // following valid frame.
    resetFrame();
  }

  _lastByteUs =
      now;

  _timingValid =
      true;

  switch (_state)
  {
    case IBUS_LENGTH:
      _idx = 0;

      if (c == IBUS_FRAME_SIZE)
      {
        if (pushByte(c))
        {
          _state = IBUS_CMD;
        }
      }
      break;

    case IBUS_CMD:
      if (c == IBUS_COMMAND)
      {
        if (pushByte(c))
        {
          _state = IBUS_DATA;
        }
      }
      else
      {
        resetFrame();
      }
      break;

    case IBUS_DATA:
      if (!pushByte(c))
      {
        break;
      }

      if (_idx >= IBUS_FRAME_SIZE - 2)
      {
        _state = IBUS_CRC_LO;
      }
      break;

    case IBUS_CRC_LO:
      if (pushByte(c))
      {
        _state = IBUS_CRC_HI;
      }
      break;

    case IBUS_CRC_HI:
    {
      if (!pushByte(c))
      {
        break;
      }

      uint16_t csum = 0xffff;

      for (size_t i = 0;
           i < IBUS_FRAME_SIZE - 2;
           i++)
      {
        csum -= data[i];
      }

      if (frameData.checksum == csum)
      {
        apply(frameData);
      }

      resetFrame();
      break;
    }
  }
}

void FAST_CODE_ATTR InputIBUS::apply(IBusData& data)
{
  for(size_t i = 0; i < CHANNELS; i++)
  {
    _channels[i] = data.ch[i];
  }
  _new_data = true;
}

uint16_t FAST_CODE_ATTR InputIBUS::get(uint8_t i) const
{
  return
      i < CHANNELS
          ? _channels[i]
          : 0;
}

void FAST_CODE_ATTR InputIBUS::get(
    uint16_t* data,
    size_t len) const
{
  if (!data)
  {
    return;
  }

  len =
      std::min(
          len,
          CHANNELS);

  for (size_t i = 0;
       i < len;
       ++i)
  {
    data[i] =
        _channels[i];
  }
}

size_t InputIBUS::getChannelCount() const { return CHANNELS; }

bool InputIBUS::needAverage() const { return false; }

}
