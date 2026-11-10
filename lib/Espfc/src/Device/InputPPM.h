#pragma once

#include <cstdint>
#include <cstddef>
#include <atomic>
#include "InputDevice.h"
#if defined(ESP32) || defined(ESP32S2) || defined(ESP32S3) || defined(ESP32C3)
#include <Arduino.h> // FreeRTOS portMUX_TYPE: cross-core task/ISR exclusion
#endif

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

// Single ISR producer / single task consumer. The producer, consumer and
// mailbox each own a different frame; only the mailbox index is shared.
struct Frame
{
  uint16_t channels[CHANNELS] = {};
  uint8_t count = 0;
};
// Espressif's embedded libstdc++ does not guarantee an ISR-safe lock-free
// 32-bit atomic exchange. Use a FreeRTOS portMUX on ESP32 (including SMP),
// and a short task-side interrupt mask on single-core ESP8266.
#if !defined(ESP8266) && !defined(ESP32) && !defined(ESP32S2) && !defined(ESP32S3) && !defined(ESP32C3)
static_assert(std::atomic<uint32_t>::is_always_lock_free,
              "PPM ISR requires lock-free 32-bit exchange");
#endif
static constexpr uint32_t FRAME_DIRTY = 4u;
static constexpr uint32_t FRAME_INDEX_MASK = 3u;
Frame _frames[3];
#if defined(ESP8266) || defined(ESP32) || defined(ESP32S2) || defined(ESP32S3) || defined(ESP32C3)
volatile uint32_t _mailbox{2u};
#else
std::atomic<uint32_t> _mailbox{2u};
#endif
#if defined(ESP32) || defined(ESP32S2) || defined(ESP32S3) || defined(ESP32C3)
portMUX_TYPE _mailboxMux = portMUX_INITIALIZER_UNLOCKED;
#endif
uint8_t _writeIndex = 0; // ISR-owned
uint8_t _readIndex = 1;  // consumer-owned
volatile uint32_t _last_tick = 0;
volatile uint8_t _channel = 0;
volatile uint8_t _lastFrameChannels = 0;
volatile uint8_t _stableFrames = 0;
volatile bool _frameValid = true;

int _pin = -1;
};

}

}
