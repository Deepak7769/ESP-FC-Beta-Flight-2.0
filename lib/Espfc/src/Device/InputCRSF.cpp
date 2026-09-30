#include <algorithm>
#include "InputCRSF.h"
#include "Hal/Time.hpp"
#include "Utils/MemoryHelper.h"

namespace Espfc::Device {

using namespace Espfc::Rc;

InputCRSF::InputCRSF(): _serial(nullptr), _telemetry(nullptr), _state(CRSF_ADDR), _idx(0), _new_data(false) {}

int InputCRSF::begin(Stream::ReadWritable* serial, TelemetryManager* telemetry)
{
  _serial = serial;
  _telemetry = telemetry;
  _telemetry_next = micros() + TELEMETRY_INTERVAL;
  _frameStartUs = 0;
  _timingValid = false;
  reset();
  std::fill_n((uint8_t*)&_frame, sizeof(_frame), 0);
  std::fill_n(_channels, CHANNELS, 0);
  return 1;
}

InputStatus FAST_CODE_ATTR InputCRSF::update()
{
  if(!_serial) return INPUT_IDLE;

  size_t len = _serial->available();
  if(len)
  {
    uint8_t buff[64] = {0};
    len = std::min(len, sizeof(buff));
    _serial->readMany(buff, len);
    size_t i = 0;
    while(i < len)
    {
      parse(_frame, buff[i++]);
    }
  }

  const uint32_t now = micros();

  // Signed subtraction keeps the deadline check correct across the 32-bit
  // micros() wrap-around.
  if (_telemetry &&
      static_cast<int32_t>(
          now - _telemetry_next) >= 0)
  {
    _telemetry_next =
        now + TELEMETRY_INTERVAL;

    _telemetry->process(
        *_serial,
        TELEMETRY_PROTOCOL_CRSF);
  }

  if(_new_data)
  {
    _new_data = false;
    return INPUT_RECEIVED;
  }

  return INPUT_IDLE;
}

uint16_t FAST_CODE_ATTR InputCRSF::get(uint8_t i) const
{
  return
      i < CHANNELS
          ? _channels[i]
          : 0;
}

void FAST_CODE_ATTR InputCRSF::get(
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

size_t InputCRSF::getChannelCount() const { return CHANNELS; }

bool InputCRSF::needAverage() const { return false; }

void FAST_CODE_ATTR InputCRSF::parse(CrsfMessage& msg, int d)
{
  uint8_t *data = reinterpret_cast<uint8_t*>(&msg);
  uint8_t c = (uint8_t)(d & 0xff);

  constexpr uint32_t CRSF_FRAME_TIMEOUT_US =
      2500;

  const uint32_t now =
      micros();

  if (_state != CRSF_ADDR &&
      _timingValid &&
      static_cast<uint32_t>(
          now -
          _frameStartUs) >
          CRSF_FRAME_TIMEOUT_US)
  {
    // A complete 64-byte CRSF frame fits comfortably inside this window at
    // the protocol baud rate. Discard truncated frames before accepting bytes
    // from the next packet, matching the frame-timeout strategy used by
    // mature CRSF implementations.
    reset();
  }

  switch(_state)
  {
    case CRSF_ADDR:
      if(c == CRSF_SYNC_BYTE)
      {
        _frameStartUs =
            now;

        _timingValid =
            true;

        data[_idx++] = c;
        _state = CRSF_SIZE;
      }
      break;
    case CRSF_SIZE:
      if(c >= 2 && c <= CRSF_FRAME_SIZE_MAX - 2) // allowed size is in range 2-62
      {
        data[_idx++] = c;
        _state = CRSF_TYPE;
      } else {
        reset();
      }
      break;
    case CRSF_TYPE:
    {
      const bool validRc =
          c == CRSF_FRAMETYPE_RC_CHANNELS_PACKED &&
          msg.size == sizeof(CrsfData) + 2;

      const bool validLinkStats =
          c == CRSF_FRAMETYPE_LINK_STATISTICS &&
          msg.size == sizeof(CrsfLinkStats) + 2;

      const bool validMsp =
          (c == CRSF_FRAMETYPE_MSP_REQ ||
           c == CRSF_FRAMETYPE_MSP_WRITE) &&
          msg.size >= 5;

      if (!(validRc ||
            validLinkStats ||
            validMsp))
      {
        reset();
        break;
      }

      if (_idx >= sizeof(CrsfMessage))
      {
        reset();
        break;
      }

      data[_idx++] = c;
      _state =
          msg.size > 2
              ? CRSF_DATA
              : CRSF_CRC;
      break;
    }

    case CRSF_DATA:
      if (_idx >= sizeof(CrsfMessage))
      {
        reset();
        break;
      }

      data[_idx++] = c;

      if (_idx > msg.size) // operator > accounts for address/length bytes
      {
        _state = CRSF_CRC;
      }
      break;

    case CRSF_CRC:
    {
      if (_idx >= sizeof(CrsfMessage))
      {
        reset();
        break;
      }

      data[_idx++] = c;

      const uint8_t crc =
          msg.crc();

      reset();

      if (c == crc)
      {
        apply(msg);
      }
      break;
    }
    }
}

void FAST_CODE_ATTR InputCRSF::reset()
{
  _state = CRSF_ADDR;
  _idx = 0;
  _timingValid = false;
}

void FAST_CODE_ATTR InputCRSF::apply(const CrsfMessage& msg)
{
  switch (msg.type)
  {
    case CRSF_FRAMETYPE_RC_CHANNELS_PACKED:
      applyChannels(msg);
      break;

    case CRSF_FRAMETYPE_LINK_STATISTICS:
      applyLinkStats(msg);
      break;

    case CRSF_FRAMETYPE_MSP_REQ:
    case CRSF_FRAMETYPE_MSP_WRITE:
      applyMspReq(msg);
      break;

    default:
      break;
  }
}

void FAST_CODE_ATTR InputCRSF::applyLinkStats(const CrsfMessage& msg)
{
  const auto * stats = reinterpret_cast<const CrsfLinkStats*>(msg.payload);
  (void)stats;
  // TODO:
}

void FAST_CODE_ATTR InputCRSF::applyChannels(const CrsfMessage& msg)
{
  const auto * data = reinterpret_cast<const CrsfData*>(msg.payload);
  Crsf::decodeRcDataShift8(_channels, data);
  //Crsf::decodeRcData(_channels, frame);
  _new_data = true;
}

void FAST_CODE_ATTR InputCRSF::applyMspReq(const CrsfMessage& frame)
{
  if(!_telemetry) return;

  uint8_t origin = 0;

  Crsf::decodeMsp(frame, _msg, origin);

  if(_msg.isCmd() && _msg.isReady())
  {
    _telemetry->processMsp(*_serial, TELEMETRY_PROTOCOL_CRSF, _msg, origin);
  }

  _telemetry_next = micros() + TELEMETRY_INTERVAL;
}

}

