#ifndef UNIT_TEST
#include "Utils/Storage.h"
#include "Utils/ConfigRecord.hpp"
#include <Arduino.h>
#include <EEPROM.h>
#include <memory>
#include <type_traits>
#if defined(NO_GLOBAL_INSTANCES) || defined(NO_GLOBAL_EEPROM)
static EEPROMClass EEPROM;
#endif
namespace Espfc::Utils {
namespace {
template<class T> bool beginEeprom(T& store, size_t size)
{
  if constexpr (std::is_void<decltype(store.begin(size))>::value)
  { store.begin(size); return true; }
  else return store.begin(size);
}
}
int Storage::begin()
{
  static_assert(sizeof(ModelConfig) + ConfigRecord::HEADER <= EEPROM_SIZE, "Config record exceeds EEPROM");
  return beginEeprom(EEPROM, EEPROM_SIZE) ? 1 : 0;
}
StorageResult Storage::load(ModelConfig& config) const
{
  std::unique_ptr<uint8_t[]> record(new uint8_t[EEPROM_SIZE]);
  for (size_t i = 0; i < EEPROM_SIZE; ++i) record[i] = EEPROM.read(i);
  // Legacy versions have no integrity field. Never guess their layout or
  // silently activate a partially migrated configuration. Export/import via
  // CLI is the default migration route; no automatic rewrite occurs on load.
  if (record[0] == 0xa5) return STORAGE_ERR_BAD_VERSION;
  if (std::memcmp(record.get(), "EFC6", 4)) return STORAGE_ERR_BAD_MAGIC;
  if (!ConfigRecord::decode(record.get(), EEPROM_SIZE, config)) return STORAGE_ERR_INTEGRITY;
  return STORAGE_LOAD_SUCCESS;
}
StorageResult Storage::save(const ModelConfig& config)
{
  std::unique_ptr<uint8_t[]> record(new uint8_t[EEPROM_SIZE]);
  if (!ConfigRecord::encode(config, record.get(), EEPROM_SIZE)) return STORAGE_SAVE_ERROR;
  for (size_t i = 0; i < EEPROM_SIZE; ++i) EEPROM.write(i, record[i]);
  if (!EEPROM.commit()) return STORAGE_SAVE_ERROR;
  // Reload through the backend, not just the write cache. On a backend with
  // non-atomic sector writes, interrupted/corrupt saves fail closed on boot.
  EEPROM.end();
  if (!beginEeprom(EEPROM, EEPROM_SIZE)) return STORAGE_SAVE_ERROR;
  for (size_t i = 0; i < EEPROM_SIZE; ++i)
    if (EEPROM.read(i) != record[i]) return STORAGE_SAVE_ERROR;
  return STORAGE_SAVE_SUCCESS;
}
} // namespace Espfc::Utils
#endif
