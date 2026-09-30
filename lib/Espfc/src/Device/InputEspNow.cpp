#if defined(ESP32) || defined(ESP8266)

#include "InputEspNow.h"
#include "Utils/MemoryHelper.h"
#include <algorithm>

namespace Espfc {

namespace Device {

int InputEspNow::begin(void)
{
  for(size_t i = 0; i < CHANNELS; i++)
  {
    _channels[i] = i == 2 ? 1000 : 1500;
  }
  return _rx.begin();
}

InputStatus FAST_CODE_ATTR InputEspNow::update()
{
  _rx.update();
  if(_rx.available())
  {
    for(size_t i = 0; i < CHANNELS; i++)
    {
      _channels[i] = _rx.getChannel(i);
    }
    return INPUT_RECEIVED;
  }
  return INPUT_IDLE;
}

uint16_t FAST_CODE_ATTR InputEspNow::get(uint8_t i) const
{
  return
      i < CHANNELS
          ? _channels[i]
          : 0;
}

void FAST_CODE_ATTR InputEspNow::get(
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

size_t InputEspNow::getChannelCount() const { return CHANNELS; }

bool InputEspNow::needAverage() const { return false; }

}

}

#endif
