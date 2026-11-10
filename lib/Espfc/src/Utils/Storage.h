#pragma once

namespace Espfc {

enum StorageResult
{

  STORAGE_NONE,
  STORAGE_LOAD_SUCCESS,
  STORAGE_SAVE_SUCCESS,
  STORAGE_SAVE_ERROR,
  STORAGE_ERR_BAD_MAGIC,
  STORAGE_ERR_BAD_VERSION,
  STORAGE_ERR_BAD_SIZE,
  STORAGE_ERR_INTEGRITY,
  STORAGE_ERR_IO,
};

}

#ifndef UNIT_TEST

#include "ModelConfig.h"

namespace Espfc::Utils {

class Storage
{
public:
  int begin();
  StorageResult load(ModelConfig& config) const;
  StorageResult save(const ModelConfig& config);

private:
  static constexpr size_t EEPROM_SIZE = 2048;
};

} // namespace Espfc::Utils

#endif
