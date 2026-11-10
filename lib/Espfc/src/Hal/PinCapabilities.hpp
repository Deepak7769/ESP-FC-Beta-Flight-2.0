#pragma once
#include <cstdint>

namespace Espfc::Hal {

// Conservative board-family policy for the *optional direct-GPIO sonar*.
// Excludes flash/PSRAM pins and pins commonly dedicated to USB or boot
// configuration. This deliberately does not claim every valid GPIO is
// exposed on every board. The Model's pin conflict check applies too.
enum class PinBoard : uint8_t
{
  Esp32, Esp32S3, Esp32C3, Esp8266, RpPico, Native
};

constexpr bool supportsSonarGpio(PinBoard board, int pin, bool output)
{
  if (pin < 0 || pin >= 64) return false;
  switch (board)
  {
    case PinBoard::Esp32:
      // GPIO6-11: SPI flash, 16-17: potential PSRAM on WROVER;
      // 0,2,12,15: bootstrapping; 34-39 are input-only.
      if (pin > 39 || (pin >= 6 && pin <= 11) ||
          pin == 16 || pin == 17 || pin == 0 || pin == 2 ||
          pin == 12 || pin == 15 ||
          pin == 20 || pin == 24 || (pin >= 28 && pin <= 31))
        return false;
      return !output || pin <= 33;

    case PinBoard::Esp32S3:
      // S3 memory interface 26-37 is module dependent, so reserve it.
      // GPIO19/20 are USB D-/D+ on boards using native USB.
      if (pin > 48 || (pin >= 22 && pin <= 37) ||
          pin == 19 || pin == 20 || pin == 0 ||
          pin == 3 || pin == 45 || pin == 46)
        return false;
      return true;

    case PinBoard::Esp32C3:
      // C3 flash 12-17 and USB-JTAG 18/19 are not sonar GPIO.
      if (pin > 21 || (pin >= 12 && pin <= 19) ||
          pin == 2 || pin == 8 || pin == 9)
        return false;
      return true;

    case PinBoard::Esp8266:
      // GPIO6-11 are flash; 0/2/15 affect bootstrapping.
      if (pin > 16 || (pin >= 6 && pin <= 11) ||
          pin == 0 || pin == 2 || pin == 15)
        return false;
      return true;

    case PinBoard::RpPico:
      // Pico / Pico2: safe, board-exposed GPIO only; 23-25 are used
      // internally on Pico family and 29 is used for voltage sensing.
      return pin <= 22 || (pin >= 26 && pin <= 28);

    case PinBoard::Native:
      // Unit tests must not infer any specific physical board policy.
      return pin < 64;
  }
  return false;
}

} // namespace Espfc::Hal
