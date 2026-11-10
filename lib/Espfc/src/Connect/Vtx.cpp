#include "Vtx.hpp"
#include "Utils/Crc.hpp"
#include "Hal/Time.hpp"
#include <algorithm>

namespace Espfc::Connect {
// TBS SmartAudio rev09: TX CRC includes preamble; RX CRC begins at command.
// RX length includes its CRC. Idle is input; the ESP32 HAL releases the
// one-wire output only after the UART shift register finishes transmitting.
int Vtx::begin(Stream::ReadWritable* serial)
{
  _serial = serial;
  _version = _powerCount = _rxOffset = _preamble = _attempts = 0;
  _appliedChannel = 255; _appliedPower = _normalPower = 0;
  _phase = Phase::IDLE;
  _model.state.vtx = VtxState{};
  type = VTXDEV_UNSUPPORTED;
  return serial != nullptr;
}
void Vtx::queue(uint8_t command, uint8_t value, bool hasValue, uint32_t now)
{
  _command = command;
  _tx[0] = 0; _tx[1] = 0xaa; _tx[2] = 0x55;
  _tx[3] = (command << 1) | 1;
  _tx[4] = hasValue ? 1 : 0;
  if (hasValue) _tx[5] = value;
  _txSize = hasValue ? 7 : 6;
  _tx[_txSize - 1] = Utils::crc8_dvb_s2(0, _tx + 1, _txSize - 2);
  _txOffset = 0; _attempts = 0; _phaseStarted = now;
  _phase = Phase::TRANSMIT;
}
void Vtx::fail(uint32_t now)
{
  _model.state.vtx.ready = false;
  if (++_attempts < 3)
  {
    ++_model.state.vtx.retries;
    _txOffset = 0; _phaseStarted = now; _phase = Phase::TRANSMIT;
    return;
  }
  _phase = Phase::BACKOFF; _phaseStarted = now;
  _version = 0; _model.state.vtx.active = false;
}
void Vtx::receive(uint8_t byte, uint32_t now)
{
  if (static_cast<uint32_t>(now - _lastByte) > 50000u) _preamble = _rxOffset = 0;
  _lastByte = now;
  if (_preamble == 0) { if (byte == 0xaa) _preamble = 1; return; }
  if (_preamble == 1) { _preamble = byte == 0x55 ? 2 : (byte == 0xaa ? 1 : 0); return; }
  if (_rxOffset >= sizeof(_rx)) { _preamble = _rxOffset = 0; return; }
  _rx[_rxOffset++] = byte;
  if (_rxOffset == 2)
  {
    _rxLength = byte;
    if (_rxLength < 1 || _rxLength > 32) _preamble = _rxOffset = 0;
  }
  if (_rxOffset >= 2 && _rxOffset == unsigned(_rxLength) + 2u)
  {
    if (Utils::crc8_dvb_s2(0, _rx, _rxOffset - 1) == _rx[_rxOffset - 1]) response(now);
    else ++_model.state.vtx.crcErrors;
    _preamble = _rxOffset = 0;
  }
}
void Vtx::response(uint32_t now)
{
  if (_phase != Phase::WAIT_ACK) return;
  const uint8_t command = _rx[0], length = _rxLength - 1;
  const uint8_t* p = _rx + 2;
  if (_command == 1 && (command == 1 || command == 9 || command == 17))
  {
    if (length < 5 || p[0] >= 40) return;
    const uint8_t version = command == 1 ? 1 : command == 9 ? 2 : 3;
    if (version == 3)
    {
      if (length < 8 || p[6] < 1 || p[6] > 8 || length < 8u + p[6]) return;
      _powerCount = p[6];
      for (uint8_t i = 0; i < _powerCount; ++i)
      {
        if (p[8+i] == 0 || p[8+i] > 127) return;
        _powerValues[i] = p[8+i];
      }
    }
    else
    {
      _powerCount = 4;
      static constexpr uint8_t dac[] = {7,16,25,40};
      for (uint8_t i = 0; i < 4; ++i) _powerValues[i] = version == 1 ? dac[i] : i;
    }
    const uint8_t rawPower = version == 3 ? p[5] : p[1];
    _appliedPower = 0;
    for (uint8_t i = 0; i < _powerCount; ++i)
      if (rawPower == _powerValues[i]) _appliedPower = i + 1;
    if (!_version) _normalPower = _appliedPower;
    _version = version; _appliedChannel = p[0];
    _model.state.vtx.pitMode = (p[2] & 2u) != 0;
    _model.state.vtx.frequencyMode = (p[2] & 1u) != 0;
    _model.state.vtx.frequency = (uint16_t(p[3]) << 8) | p[4];
  }
  else if (_command == 2 && command == 2 && length >= 1 && p[0] == (_sentPowerValue & 127u))
    _appliedPower = _sentPowerIndex;
  else if (_command == 3 && command == 3 && length >= 1 && p[0] == _sentChannel)
    _appliedChannel = _sentChannel;
  else return;
  _lastResponse = now; _phase = Phase::IDLE;
  type = VTXDEV_SMARTAUDIO;
  auto& state = _model.state.vtx;
  state.active = true; state.version = _version; state.powerLevels = _powerCount;
  state.appliedBand = _appliedChannel / 8 + 1;
  state.appliedChannel = _appliedChannel % 8 + 1;
  state.appliedPower = _appliedPower;
}
int Vtx::update()
{
  if (!_serial) return 0;
  const uint32_t now = micros();
  // Release a completed one-wire transfer before accepting a reply.
  if (_phase == Phase::DRAIN && _serial->isTxFifoEmpty())
  { _phase = Phase::WAIT_ACK; _phaseStarted = now; }
  for (unsigned budget = 32; budget && _serial->available() > 0; --budget)
  { const int byte = _serial->read(); if (byte < 0) break; receive(byte, now); }
  auto& state = _model.state.vtx;
  if (state.active && static_cast<uint32_t>(now - _lastResponse) > 2000000u)
    state.active = state.ready = false;
  switch (_phase)
  {
    case Phase::TRANSMIT: {
      const int available = _serial->availableForWrite();
      if (available > 0)
      {
        const size_t count = std::min<size_t>(_txSize - _txOffset, std::min(available, 8));
        _txOffset += std::min<size_t>(_serial->write(_tx + _txOffset, count), count);
      }
      if (_txOffset == _txSize) { _phase = Phase::DRAIN; _phaseStarted = now; }
      else if (static_cast<uint32_t>(now - _phaseStarted) >= 300000u) fail(now);
      break;
    }
    case Phase::DRAIN:
    case Phase::WAIT_ACK:
      if (static_cast<uint32_t>(now - _phaseStarted) >= 300000u) fail(now);
      break;
    case Phase::BACKOFF:
      if (static_cast<uint32_t>(now - _phaseStarted) >= 1000000u && _serial->isTxFifoEmpty())
      {
        static constexpr int bauds[] = {4800,4900,4700,5000,4600};
        _baudIndex = (_baudIndex + 1) % 5; _serial->updateBaudRate(bauds[_baudIndex]);
        _phase = Phase::IDLE;
      }
      break;
    case Phase::IDLE: {
      if (!_version) { queue(1,0,false,now); break; }
      const auto& desired = _model.config.vtx;
      const bool armed = _model.isModeActive(MODE_ARMED);
      const uint8_t channel = (desired.band - 1u) * 8u + desired.channel - 1u;
      uint8_t power = desired.power;
      if (desired.lowPowerDisarm) power = armed ? (power ? power : _normalPower) : 1;
      state.configurationRejected = state.pitMode || state.frequencyMode || desired.band < 1 || desired.band > 5 ||
          desired.channel < 1 || desired.channel > 8 || power > _powerCount ||
          (desired.lowPowerDisarm && armed && power == 0);
      state.ready = state.active && !state.configurationRejected && channel == _appliedChannel &&
          (power == 0 || power == _appliedPower);
      if (state.configurationRejected) break;
      if (channel != _appliedChannel)
      { _sentChannel = channel; state.ready = false; queue(3, channel, true, now); }
      else if (power && power != _appliedPower)
      {
        _sentPowerIndex = power; _sentPowerValue = _powerValues[power-1];
        if (_version == 3) _sentPowerValue |= 0x80;
        state.ready = false; queue(2, _sentPowerValue, true, now);
      }
      else if (static_cast<uint32_t>(now - _lastResponse) >= 1000000u) queue(1,0,false,now);
      break;
    }
  }
  return 1;
}
} // namespace Espfc::Connect
