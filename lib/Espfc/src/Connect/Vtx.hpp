#pragma once
#include "Model.h"
#include "Stream/ReadWritable.hpp"

namespace Espfc::Connect {
enum VtxDeviceType { VTXDEV_UNSUPPORTED = 0, VTXDEV_RTC6705 = 1,
  VTXDEV_SMARTAUDIO = 3, VTXDEV_TRAMP = 4, VTXDEV_MSP = 5, VTXDEV_UNKNOWN = 255 };
class Vtx
{
public:
  explicit Vtx(Model& model): _model(model) {}
  int begin(Stream::ReadWritable* serial);
  int update();
  VtxDeviceType type = VTXDEV_UNSUPPORTED;
private:
  enum class Phase { IDLE, TRANSMIT, DRAIN, WAIT_ACK, BACKOFF };
  void queue(uint8_t command, uint8_t value, bool hasValue, uint32_t now);
  void receive(uint8_t byte, uint32_t now);
  void response(uint32_t now);
  void fail(uint32_t now);
  Stream::ReadWritable* _serial = nullptr;
  Model& _model;
  Phase _phase = Phase::IDLE;
  uint8_t _command = 0, _attempts = 0, _baudIndex = 0;
  uint8_t _tx[8]{}, _txSize = 0, _txOffset = 0;
  uint8_t _rx[36]{}, _rxOffset = 0, _rxLength = 0, _preamble = 0;
  uint8_t _powerValues[8]{}, _version = 0, _powerCount = 0;
  uint8_t _appliedChannel = 255, _appliedPower = 0, _normalPower = 0;
  uint8_t _sentChannel = 0, _sentPowerIndex = 0, _sentPowerValue = 0;
  uint32_t _phaseStarted = 0, _lastResponse = 0, _lastByte = 0;
};
} // namespace Espfc::Connect
