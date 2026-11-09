#pragma once

#include "Connect/Msp.hpp"

namespace Espfc::Connect {

class MspParser
{
public:
  MspParser();
  void parse(uint8_t c, MspMessage& msg);
};

} // namespace Espfc::Connect
