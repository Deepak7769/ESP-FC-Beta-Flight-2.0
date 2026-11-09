#include "BaroBMP085.hpp"
#include "Hal/Time.hpp"
#include <cmath>

#define BMP085_DEFAULT_ADDRESS 0x77
#define BMP085_WHOAMI_ID 0x55

#define BMP085_CALIB_REG 0xAA
#define BMP085_WHOAMI_REG 0xD0
#define BMP085_RESET_REG 0xE0
#define BMP085_CONTROL_REG 0xF4
#define BMP085_MEASUREMENT_REG 0xF6

#define BMP085_MEASURE_T 0x2E  // temperature measurent
#define BMP085_MEASURE_P0 0x34 // pressure measurement no oversampling
#define BMP085_MEASURE_P1 0x74 // pressure measurement oversampling x2
#define BMP085_MEASURE_P2 0xB4 // pressure measurement oversampling x4
#define BMP085_MEASURE_P3 0xF4 // pressure measurement oversampling x8

namespace Espfc::Device::Baro {

int BaroBMP085::begin(BusDevice* bus)
{
  return begin(bus, BMP085_DEFAULT_ADDRESS);
}

int BaroBMP085::begin(BusDevice* bus, uint8_t addr)
{
  setBus(bus, addr);

  if (!testConnection()) return 0;

  _temperatureValid = false;
  _lastTemperatureUs = 0u;
  _t_fine = 0;
  _cal = CalibrationData{};

  if (!_bus->writeByte(_addr, BMP085_RESET_REG, 0xB6)) return 0;
  delay(10);

  // Bosch BMP085 calibration words are big-endian on the wire. A direct
  // read into a packed native int16_t struct reverses them on little-endian
  // processors, corrupting both temperature and pressure compensation.
  uint8_t raw[22]{};
  if (_bus->read(_addr, BMP085_CALIB_REG, sizeof(raw), raw) !=
      static_cast<int8_t>(sizeof(raw)))
  {
    return 0;
  }
  const auto u16 = [&](size_t i) -> uint16_t
  {
    return (static_cast<uint16_t>(raw[i]) << 8) | raw[i + 1];
  };
  const auto s16 = [&](size_t i) -> int16_t
  {
    const int32_t value = u16(i);
    return static_cast<int16_t>(value >= 0x8000 ? value - 0x10000 : value);
  };
  _cal.ac1 = s16(0);
  _cal.ac2 = s16(2);
  _cal.ac3 = s16(4);
  _cal.ac4 = u16(6);
  _cal.ac5 = u16(8);
  _cal.ac6 = u16(10);
  _cal.b1 = s16(12);
  _cal.b2 = s16(14);
  _cal.mb = s16(16);
  _cal.mc = s16(18);
  _cal.md = s16(20);
  // These divisor/scale coefficients cannot be zero in a valid device.
  return _cal.ac4 != 0u && _cal.ac5 != 0u && _cal.ac6 != 0u ? 1 : 0;
}

BaroDeviceType BaroBMP085::getType() const
{
  return BARO_BMP085;
}

float BaroBMP085::readTemperature()
{
  uint8_t buffer[2]{};
  if (_bus->read(_addr, BMP085_MEASUREMENT_REG, 2, buffer) != 2)
  {
    _temperatureValid = false;
    return NAN;
  }
  const int32_t ut = (static_cast<int32_t>(buffer[0]) << 8) | buffer[1];
  if (ut == 0)
  {
    _temperatureValid = false;
    return NAN;
  }

  // 64-bit intermediates avoid overflow on unusual/invalid raw samples.
  const int64_t x1 =
      ((static_cast<int64_t>(ut) - _cal.ac6) * _cal.ac5) >> 15;
  const int64_t denominator = x1 + _cal.md;
  if (denominator == 0)
  {
    _temperatureValid = false;
    return NAN;
  }
  const int64_t x2 = (static_cast<int64_t>(_cal.mc) * 2048) / denominator;
  const int64_t fine = x1 + x2;
  if (fine < -100000 || fine > 100000)
  {
    _temperatureValid = false;
    return NAN;
  }
  _t_fine = static_cast<int32_t>(fine);
  _lastTemperatureUs = micros();
  _temperatureValid = true;
  return (static_cast<float>((_t_fine + 8) >> 4)) * 0.1f;
}

float BaroBMP085::readPressure()
{
  if (!_temperatureValid ||
      static_cast<uint32_t>(micros() - _lastTemperatureUs) > 1000000u)
  {
    return NAN;
  }
  uint8_t buffer[3]{};
  if (_bus->read(_addr, BMP085_MEASUREMENT_REG, 3, buffer) != 3)
  {
    return NAN;
  }
  const uint8_t oss = (_mode & 0xc0) >> 6;
  const int64_t uncompensated =
      (static_cast<uint32_t>(buffer[0]) << 16) |
      (static_cast<uint32_t>(buffer[1]) << 8) | buffer[2];
  const int64_t up = uncompensated >> (8 - oss);
  if (up <= 0) return NAN;

  const int64_t b6 = static_cast<int64_t>(_t_fine) - 4000;
  int64_t x1 = (static_cast<int64_t>(_cal.b2) * ((b6 * b6) >> 12)) >> 11;
  int64_t x2 = (static_cast<int64_t>(_cal.ac2) * b6) >> 11;
  int64_t x3 = x1 + x2;
  const int64_t b3 = (((static_cast<int64_t>(_cal.ac1) * 4 + x3)
      * (int64_t{1} << oss)) + 2) >> 2;

  x1 = (static_cast<int64_t>(_cal.ac3) * b6) >> 13;
  x2 = (static_cast<int64_t>(_cal.b1) * ((b6 * b6) >> 12)) >> 16;
  x3 = ((x1 + x2) + 2) >> 2;
  const int64_t b4 = (static_cast<int64_t>(_cal.ac4) * (x3 + 32768)) >> 15;
  if (b4 <= 0) return NAN;
  const int64_t b7 = (up - b3) * (50000 >> oss);
  if (b7 <= 0) return NAN;
  int64_t pressure = b7 < 0x80000000LL
      ? (b7 * 2) / b4 : ((b7 / b4) * 2);
  x1 = (pressure >> 8) * (pressure >> 8);
  x1 = (x1 * 3038) >> 16;
  x2 = (-7357 * pressure) >> 16;
  pressure += (x1 + x2 + 3791) >> 4;
  if (pressure <= 0 || pressure > 200000) return NAN;
  return static_cast<float>(pressure);
}

void BaroBMP085::setMode(BaroDeviceMode mode)
{
  _mode = mode == BARO_MODE_TEMP ? BMP085_MEASURE_T : BMP085_MEASURE_P3;
  _bus->writeByte(_addr, BMP085_CONTROL_REG, _mode);
}

int BaroBMP085::getDelay(BaroDeviceMode mode) const
{
  switch (mode)
  {
    case BARO_MODE_TEMP:
      return 4550; // temp
    default:
      // return 4550;  // press_0
      // return 7550;  // press_1
      // return 13550; // press_2
      return 25550; // press_3
  }
}

bool BaroBMP085::testConnection()
{
  uint8_t whoami = 0;
  if (_bus->readByte(_addr, BMP085_WHOAMI_REG, &whoami) != 1) return false;
  setChipId(whoami);
  if (whoami != BMP085_WHOAMI_ID) return false;
  return true;
}

} // namespace Espfc::Device::Baro
