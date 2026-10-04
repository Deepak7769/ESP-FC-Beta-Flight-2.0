#ifndef UNIT_TEST

#include "Utils/Storage.h"
#include "ModelConfig.h"
#include <Arduino.h>
#include <EEPROM.h>
#include <algorithm>
#include <cstddef>

#if defined(NO_GLOBAL_INSTANCES) || defined(NO_GLOBAL_EEPROM)
static EEPROMClass EEPROM;
#endif

namespace Espfc::Utils {

int Storage::begin()
{
  EEPROM.begin(EEPROM_SIZE);
  static_assert(sizeof(ModelConfig) <= EEPROM_SIZE, "ModelConfig Size too big");
  return 1;
}

StorageResult Storage::load(ModelConfig& config) const
{
  // return STORAGE_ERR_BAD_MAGIC;

  int addr = 0;
  uint8_t magic = EEPROM.read(addr++);
  if (EEPROM_MAGIC != magic)
  {
    return STORAGE_ERR_BAD_MAGIC;
  }

  uint8_t version = EEPROM.read(addr++);

  uint16_t size = 0;
  size = EEPROM.read(addr++);
  size |= EEPROM.read(addr++) << 8;

  if (version == EEPROM_VERSION)
  {
    if (size != sizeof(ModelConfig))
    {
      return STORAGE_ERR_BAD_SIZE;
    }

    EEPROM.get(addr, config);
    return STORAGE_LOAD_SUCCESS;
  }

  if (version == EEPROM_VERSION_LEGACY)
  {
    // v2 stored the original three-byte AltHoldConfig in place. The new
    // fields must not shift the bytes that follow it, so migrate the prefix,
    // then skip the inserted v3 AltHold bytes before copying the tail.
    ModelConfig migrated{};

    const size_t altHoldOffset =
        offsetof(ModelConfig, altHold);
    constexpr size_t LEGACY_ALTHOLD_SIZE = 3;
    const size_t storedSize =
        static_cast<size_t>(size);

    const size_t prefixSize =
        std::min(
            storedSize,
            altHoldOffset);

    for (size_t i = 0; i < prefixSize; ++i)
    {
      reinterpret_cast<uint8_t*>(&migrated)[i] =
          EEPROM.read(
              addr +
              static_cast<int>(i));
    }

    if (storedSize > altHoldOffset)
    {
      const size_t legacyAltBytes =
          std::min(
              LEGACY_ALTHOLD_SIZE,
              storedSize -
                  altHoldOffset);

      for (size_t i = 0; i < legacyAltBytes; ++i)
      {
        reinterpret_cast<uint8_t*>(&migrated.altHold)[i] =
            EEPROM.read(
                addr +
                static_cast<int>(
                    altHoldOffset + i));
      }
    }

    const size_t oldTailStart =
        altHoldOffset +
        LEGACY_ALTHOLD_SIZE;

    const size_t newTailStart =
        altHoldOffset +
        sizeof(AltHoldConfig);

    if (storedSize > oldTailStart &&
        newTailStart < sizeof(ModelConfig))
    {
      const size_t oldTailSize =
          storedSize -
          oldTailStart;

      const size_t newTailSize =
          sizeof(ModelConfig) -
          newTailStart;

      const size_t tailSize =
          std::min(
              oldTailSize,
              newTailSize);

      for (size_t i = 0; i < tailSize; ++i)
      {
        reinterpret_cast<uint8_t*>(&migrated)[
            newTailStart + i] =
            EEPROM.read(
                addr +
                static_cast<int>(
                    oldTailStart + i));
      }
    }

    config = migrated;
    return STORAGE_LOAD_SUCCESS;
  }

  return STORAGE_ERR_BAD_VERSION;
}

StorageResult Storage::save(const ModelConfig& config)
{
  int addr = 0;
  uint16_t size = sizeof(ModelConfig);
  EEPROM.write(addr++, EEPROM_MAGIC);
  EEPROM.write(addr++, EEPROM_VERSION);
  EEPROM.write(addr++, size & 0xFF);
  EEPROM.write(addr++, (size >> 8) & 0xFF);
  EEPROM.put(addr, config);
  bool ok = EEPROM.commit();
  if (!ok) return STORAGE_SAVE_ERROR;
  return STORAGE_SAVE_SUCCESS;
}

} // namespace Espfc::Utils

#endif
