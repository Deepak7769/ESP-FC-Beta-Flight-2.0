#pragma once

#include "Hal/Serial.hpp"
#include "Target/Target.h"
#include "Stream/ReadWritable.hpp"
#include <algorithm>
#include <cstring>

namespace Espfc {

namespace Blackbox {

class BlackboxSerialBuffer: public Stream::ReadWritable
{
  public:
    static constexpr size_t SIZE = targetSerialTxBufferSize(); // 128;

    BlackboxSerialBuffer(): _dev(nullptr), _idx(0), _data(nullptr) {}

    ~BlackboxSerialBuffer()
    {
      if(!_data) return;

      delete[] _data;
      _data = nullptr;
    }
void wrap(
    Stream::ReadWritable* s)
{
  if (_data)
  {
    delete[] _data;
    _data = nullptr;
  }

  _dev = s;
  _idx = 0;

  if (SIZE > 0)
  {
    _data =
        new uint8_t[SIZE];
  }
}
 

    void begin(const Hal::SerialDeviceConfig& conf) override
    {
      //_dev->begin(conf);
    }

    void updateBaudRate(int baud) override
    {
      //_dev->updateBaudRate(baud);
    }

    size_t write(uint8_t c) override
    {
      if (!_data || SIZE == 0) return 0;
      if (_idx >= SIZE)
      {
        flush();
        if (_idx >= SIZE) return 0; // explicit upstream backpressure
      }
      _data[_idx++] = c;
      if (_idx >= SIZE) flush();
      return 1;
    }

    void flush() override
    {
      if (!_dev || !_data || _idx == 0) return;
      // The transport may enqueue fewer bytes than requested. Keep the
      // unwritten suffix, in order, instead of silently dropping it.
      const int available = _dev->availableForWrite();
      if (available <= 0) return;
      const size_t requested =
          std::min(_idx, static_cast<size_t>(available));
      const size_t sent =
          std::min(_dev->write(_data, requested), requested);
      if (sent == 0) return;
      _idx -= sent;
      if (_idx > 0) std::memmove(_data, _data + sent, _idx);
    }

    int availableForWrite() override
    {
      //return _dev->availableForWrite();
      return SIZE - _idx;
    }

    bool isTxFifoEmpty() override
    {
      //return _dev->isTxFifoEmpty();
      return _idx == 0 && (!_dev || _dev->isTxFifoEmpty());
    }

   int available() override
{
  return _dev
      ? _dev->available()
      : 0;
}
    int read() override { return _dev ? _dev->read() : -1; }
    size_t readMany(uint8_t * c, size_t l) override
    {
      return _dev ? _dev->readMany(c, l) : 0;
    }
    int peek() override { return _dev ? _dev->peek() : -1; }

    size_t write(const uint8_t * c, size_t l) override
    {
      if (!c) return 0;
      size_t accepted = 0;
      while (accepted < l && write(c[accepted]) == 1)
      {
        ++accepted;
      }
      return accepted;
    }

    Stream::ReadWritable * _dev;
    size_t _idx;
    uint8_t* _data;
};

}

}
