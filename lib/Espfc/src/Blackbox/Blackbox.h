#pragma once

#include "BlackboxSerialBuffer.h"
#include "Model.h"
#include "Stream/ReadWritable.hpp"

namespace Espfc::Blackbox {

class Blackbox
{
public:
  Blackbox(Model& model);
  int begin();
  int update();
  void serviceLifecycle(uint32_t nowUs);

private:
  void updateData();
  void updateArmed(uint32_t nowUs);
  void updateMode();

Model& _model;
Stream::ReadWritable* _serial = nullptr;
BlackboxSerialBuffer _buffer;

bool _initialized = false;
bool _servicePrimed = false;
uint32_t _lastServiceUs = 0;
uint32_t _lastGpsTs = 0;

uint32_t _armingBeepDeadline = 0;
uint32_t _stopDeadline = 0;

bool _armingBeepPending = false;
bool _stopPending = false;
};

} // namespace Espfc::Blackbox
