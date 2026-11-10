#include "MagAK8963.hpp"
#include <Arduino.h>

#define AK8963_ADDRESS_FIRST 0x0C  // this device only has one address
#define AK8963_ADDRESS_SECOND 0x0D // 0x0E and 0x0F also possible
#define AK8963_ST1 0x02
#define AK8963_HXL 0x03
#define AK8963_ST2 0x09
#define AK8963_CNTL1 0x0A
#define AK8963_PWR_DOWN 0x00
#define AK8963_CNT_MEAS1 0x12
#define AK8963_CNT_MEAS2 0x16
#define AK8963_FUSE_ROM 0x0F
#define AK8963_CNTL2 0x0B
#define AK8963_RESET 0x01
#define AK8963_ASA 0x10
#define AK8963_WHO_AM_I 0x00
#define AK8963_INIT_DELAY 20

namespace Espfc::Device::Mag {

int MagAK8963::begin(BusDevice* bus)
{
  return begin(bus, AK8963_ADDRESS_FIRST) ? 1 : begin(bus, AK8963_ADDRESS_SECOND);
}

int MagAK8963::begin(BusDevice* bus, uint8_t addr)
{
  setBus(bus, addr);

  if (!testConnection()) return 0;

  // set AK8963 to Power Down
  _bus->writeByte(_addr, AK8963_CNTL1, AK8963_PWR_DOWN);
  delay(AK8963_INIT_DELAY); // long wait between AK8963 mode changes

  // set AK8963 to FUSE ROM access
  _bus->writeByte(_addr, AK8963_CNTL1, AK8963_FUSE_ROM);
  delay(AK8963_INIT_DELAY);

  /* get the magnetometer calibration */
  // read the AK8963 ASA registers and compute magnetometer scale factors
  _bus->read(_addr, AK8963_ASA, 3, _buffer);

  // align CW90_FLIP (swap X Y)
  _scale.y = ((((float)_buffer[0]) - 128.0f) / 256.0f + 1.0f) * 4912.0f / 32760.0f; // micro Tesla
  _scale.x = ((((float)_buffer[1]) - 128.0f) / 256.0f + 1.0f) * 4912.0f / 32760.0f; // micro Tesla
  _scale.z = ((((float)_buffer[2]) - 128.0f) / 256.0f + 1.0f) * 4912.0f / 32760.0f; // micro Tesla

  // scale *= 10.f; // convert to milli Gauss
  _scale *= 0.01f; // convert to Gauss

  // set AK8963 to Power Down
  _bus->writeByte(_addr, AK8963_CNTL1, AK8963_PWR_DOWN);
  delay(AK8963_INIT_DELAY);

  // set AK8963 to 16 bit resolution, 100 Hz update rate
  _bus->writeByte(_addr, AK8963_CNTL1, AK8963_CNT_MEAS2);
  delay(AK8963_INIT_DELAY);

  // Preconfigure BUS_SLV's fast external sensor window: ST1, XYZ, ST2.
  if (_bus->read(_addr, AK8963_ST1, 8, _buffer) != 8) return 0;

  return 1;
}

int MagAK8963::readMag(VectorInt16& v)
{
  if (_bus->readFast(_addr, AK8963_ST1, 8, _buffer) != 8) return 0;
  // No fresh conversion or ST2 magnetic overflow: revoke heading authority.
  if (!(_buffer[0] & 0x01u) || (_buffer[7] & 0x08u)) return 0;
  const auto axis = [&](size_t i) {
    return static_cast<int16_t>(
        (static_cast<uint16_t>(_buffer[i + 1]) << 8) | _buffer[i]);
  };
  // align CW90_FLIP (swap X/Y, invert Z).
  v.y = axis(1);
  v.x = axis(3);
  const int16_t z = axis(5);
  if (z == INT16_MIN) return 0;
  v.z = static_cast<int16_t>(-static_cast<int32_t>(z));
  return 1;
}

const VectorFloat MagAK8963::convert(const VectorInt16& v) const
{
  return static_cast<VectorFloat>(v) * _scale;
}

int MagAK8963::getRate() const
{
  return 100;
}

MagDeviceType MagAK8963::getType() const
{
  return MAG_AK8963;
}

bool MagAK8963::testConnection()
{
  if (_bus->read(_addr, AK8963_WHO_AM_I, 1, _buffer) != 1) return false;
  setChipId(_buffer[0]);
  return _buffer[0] == 0x48;
}

} // namespace Espfc::Device::Mag
