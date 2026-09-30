#pragma once

#include <cstdint>
#include <cstddef>
#include "InputDevice.h"

namespace Espfc {

enum PPMMode {
  PPM_MODE_NORMAL   = 0x01, // RISING edge
  PPM_MODE_INVERTED = 0x02  // FALLING edge
};

namespace Device {

class InputPPM: public InputDevice
{
  public:
    void begin(int pin, int mode = PPM_MODE_NORMAL);
    InputStatus update() override;
    uint16_t get(uint8_t i) const override;
    void get(uint16_t * data, size_t len) const override;
    size_t getChannelCount() const override;
    bool needAverage() const override;

#ifndef UNIT_TEST
  private:
#endif
    static constexpr size_t CHANNELS = 16;
    static constexpr uint32_t BROKEN_LINK_US = 100000UL; // 100ms
    static constexpr uint8_t MIN_FRAME_CHANNELS = 4;
    static constexpr uint8_t STABLE_FRAMES_REQUIRED = 3;

    void handle();
    static void handle_isr(void* args);

volatile uint16_t _channels[CHANNELS] = {};
volatile uint32_t _last_tick = 0;
volatile uint8_t _channel = 0;
volatile uint8_t _channelCount = 0;
volatile uint8_t _lastFrameChannels = 0;
volatile uint8_t _stableFrames = 0;
volatile bool _frameValid = true;
volatile bool _new_data = false;

int _pin = -1;
};

}

}
