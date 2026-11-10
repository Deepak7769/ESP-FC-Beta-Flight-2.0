#pragma once

// Board-family output policy. Keep memory and native-USB wiring out of
// actuator allocation; the driver additionally checks silicon capability.
inline bool escOutputPinAllowed(int pin)
{
  if (pin < 0) return false;
#if defined(ESP32C3)
  return pin <= 21 && !(pin >= 12 && pin <= 19);
#elif defined(ESP32S3)
  return pin <= 48 && !(pin >= 22 && pin <= 37) &&
         pin != 19 && pin != 20 && pin != 46;
#elif defined(ESP32S2)
  return pin <= 45 && !(pin >= 22 && pin <= 32) && pin != 19 && pin != 20;
#elif defined(ESP32)
  if (pin > 33 || (pin >= 6 && pin <= 11) || pin == 20 ||
      pin == 24 || (pin >= 28 && pin <= 31)) return false;
#if defined(BOARD_HAS_PSRAM) || CONFIG_SPIRAM
  if (pin == 16 || pin == 17) return false;
#endif
  return true;
#elif defined(ESP8266)
  return pin <= 16 && !(pin >= 6 && pin <= 11);
#elif defined(ARCH_RP2040)
  return pin <= 22 || (pin >= 26 && pin <= 28);
#else
  return pin < 64;
#endif
}

