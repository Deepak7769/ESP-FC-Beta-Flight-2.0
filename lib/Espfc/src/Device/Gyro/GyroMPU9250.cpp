#include "GyroMPU9250.hpp"

#define MPU6050_RA_WHO_AM_I 0x75
#define MPU9250_ACCEL_CONF2 0x1D
#define MPU9250_WHOAMI_DEFAULT_VALUE 0x71
#define MPU9250_WHOAMI_ALT_VALUE 0x73

namespace Espfc::Device::Gyro {

GyroDeviceType GyroMPU9250::getType() const
{
  return GYRO_MPU9250;
}

void GyroMPU9250::setDLPFMode(uint8_t mode)
{
  GyroMPU6050::setDLPFMode(mode);
  if (!_configOk) return;
  const uint8_t expected = mode & 0x07;
  if (!_bus->writeByte(_addr, MPU9250_ACCEL_CONF2, expected))
  {
    _configOk = false;
    return;
  }
  uint8_t actual = 0;
  if (_bus->readByte(_addr, MPU9250_ACCEL_CONF2, &actual) != 1 ||
      (actual & 0x07) != expected)
  {
    _configOk = false;
  }
}

bool GyroMPU9250::testConnection()
{
  uint8_t whoami = 0;
  if (_bus->readByte(_addr, MPU6050_RA_WHO_AM_I, &whoami) != 1) return false;
  setChipId(whoami);
  return whoami == MPU9250_WHOAMI_DEFAULT_VALUE || whoami == MPU9250_WHOAMI_ALT_VALUE;
}

} // namespace Espfc::Device::Gyro
