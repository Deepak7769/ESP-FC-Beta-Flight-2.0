#pragma once
#include "ModelConfig.h"
#include "Utils/ConfigValidation.hpp"
#include <cstddef>
#include <cstring>
#include <memory>
#include <type_traits>

namespace Espfc::Utils {
// Schema 6 is a little-endian, target-ABI-specific payload. It intentionally
// rejects cross-target/layout records; bump SCHEMA for any field-semantics
// change, including changes that keep sizeof() constant.
struct ConfigRecord
{
  static constexpr size_t HEADER = 16;
  static constexpr uint8_t SCHEMA = 6;
#if defined(ESP32C3)
  static constexpr uint8_t TARGET = 3;
#elif defined(ESP32S3)
  static constexpr uint8_t TARGET = 2;
#elif defined(ESP32)
  static constexpr uint8_t TARGET = 1;
#elif defined(ESP8266)
  static constexpr uint8_t TARGET = 4;
#elif defined(ARCH_RP2040)
#if defined(ARCH_RP2350)
  static constexpr uint8_t TARGET = 6;
#else
  static constexpr uint8_t TARGET = 5;
#endif
#else
  static constexpr uint8_t TARGET = 0;
#endif
  static uint32_t recordWord(const uint8_t* p, unsigned n)
  { uint32_t v = 0; for (unsigned i = 0; i < n; ++i) v |= uint32_t(p[i]) << (8*i); return v; }
  static void recordWord(uint8_t* p, uint32_t v, unsigned n)
  { for (unsigned i = 0; i < n; ++i) p[i] = static_cast<uint8_t>(v >> (8*i)); }
  static uint32_t crc(uint32_t state, const uint8_t* p, size_t n)
  {
    while (n--) { state ^= *p++; for (unsigned b = 0; b < 8; ++b)
      state = (state >> 1) ^ ((state & 1) ? 0xedb88320u : 0u); }
    return state;
  }
  static uint32_t layout()
  {
    const uint32_t parts[] = { sizeof(ModelConfig), alignof(ModelConfig), sizeof(bool),
      offsetof(ModelConfig, gyro), offsetof(ModelConfig, input), offsetof(ModelConfig, conditions),
      offsetof(ModelConfig, pid), offsetof(ModelConfig, altHold), offsetof(ModelConfig, rangefinder),
      offsetof(ModelConfig, pin), offsetof(ModelConfig, serial), offsetof(ModelConfig, customMixes),
      offsetof(ModelConfig, output), offsetof(ModelConfig, wireless), offsetof(ModelConfig, modelName),
      offsetof(ModelConfig, gpsRescue), offsetof(ModelConfig, extension),
      sizeof(GyroConfig), sizeof(InputConfig), sizeof(SerialPortConfig), sizeof(StockRescueSettings) };
    uint32_t h = 2166136261u;
    for (uint32_t part : parts) for (unsigned i = 0; i < 4; ++i)
      h = (h ^ uint8_t(part >> (i*8))) * 16777619u;
    return h;
  }
  static bool booleansValid(const uint8_t* payload)
  {
    const size_t offsets[] = {
      offsetof(ModelConfig, input) + offsetof(InputConfig, filterEnable),
      offsetof(ModelConfig, iterm) + offsetof(ItermConfig, lowThrottleZeroIterm),
      offsetof(ModelConfig, mixer) + offsetof(MixerConfiguration, yawReverse),
      offsetof(ModelConfig, telemetry) };
    for (size_t off : offsets) if (payload[off] > 1) return false;
    for (size_t i = 0; i < OUTPUT_CHANNELS; ++i)
    {
      const size_t base = offsetof(ModelConfig, output) + offsetof(OutputConfig, channel) + i*sizeof(OutputChannelConfig);
      if (payload[base + offsetof(OutputChannelConfig, reverse)] > 1 ||
          payload[base + offsetof(OutputChannelConfig, servo)] > 1) return false;
    }
    return true;
  }
  static bool encode(const ModelConfig& config, uint8_t* out, size_t capacity)
  {
    static_assert(std::is_trivially_copyable<ModelConfig>::value, "Config requires explicit encoding");
#if __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__
#error "Schema 6 requires a new encoding on a big-endian target"
#endif
    if (capacity < HEADER + sizeof(config) || !configurationValid(config)) return false;
    std::memset(out, 0, capacity);
    std::memcpy(out, "EFC6", 4); out[4] = SCHEMA; out[5] = TARGET;
    recordWord(out+6, sizeof(config), 2); recordWord(out+8, layout(), 4);
    std::memcpy(out+HEADER, &config, sizeof(config));
    recordWord(out+12, ~crc(crc(0xffffffffu, out, 12), out+HEADER, sizeof(config)), 4);
    return true;
  }
  static bool decode(const uint8_t* in, size_t size, ModelConfig& config)
  {
    if (size < HEADER + sizeof(config) || std::memcmp(in, "EFC6", 4) ||
        in[4] != SCHEMA || in[5] != TARGET || recordWord(in+6,2) != sizeof(config) ||
        recordWord(in+8,4) != layout() ||
        recordWord(in+12,4) != ~crc(crc(0xffffffffu,in,12),in+HEADER,sizeof(config)) ||
        !booleansValid(in+HEADER)) return false;
    std::unique_ptr<ModelConfig> candidate(new ModelConfig());
    std::memcpy(candidate.get(), in+HEADER, sizeof(config));
    if (!configurationValid(*candidate)) return false;
    config = *candidate;
    return true;
  }
};
} // namespace Espfc::Utils
