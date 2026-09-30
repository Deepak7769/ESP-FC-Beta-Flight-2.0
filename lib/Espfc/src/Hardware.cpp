#include "Hardware.h"
#include <algorithm>
#include "Device/Baro/BaroBMP085.hpp"
#include "Device/Baro/BaroBMP280.hpp"
#include "Device/Baro/BaroSPL06.hpp"
#include "Device/BaroDevice.hpp"
#include "Device/Gyro/GyroBMI160.hpp"
#include "Device/Gyro/GyroICM20602.hpp"
#include "Device/Gyro/GyroICM42688.hpp"
#include "Device/Gyro/GyroLSM6DSO.hpp"
#include "Device/Gyro/GyroMPU6050.hpp"
#include "Device/Gyro/GyroMPU6500.hpp"
#include "Device/Gyro/GyroMPU9250.hpp"
#include "Device/GyroDevice.hpp"
#include "Device/Mag/MagAK8963.hpp"
#include "Device/Mag/MagHMC5883L.hpp"
#include "Device/Mag/MagQMC5883L.hpp"
#include "Device/Mag/MagQMC5883P.hpp"
#include "Hal/Gpio.hpp"
#include "Hal/Time.hpp"
#if defined(ESPFC_WIFI_ALT)
#include <ESP8266WiFi.h>
#elif defined(ESPFC_WIFI)
#include <WiFi.h>
#endif


namespace {
#if defined(ESPFC_SPI_0)
#if defined(ESP32C3) || defined(ESP32S3) || defined(ESP32S2)
static SPIClass SPI1(HSPI);
#elif defined(ESP32)
static SPIClass SPI1(VSPI);
#endif
static Espfc::Device::BusSPI spiBus(ESPFC_SPI_0_DEV);
#endif
#if defined(ESPFC_I2C_0)
static Espfc::Device::BusI2C i2cBus(WireInstance);
#endif
static Espfc::Device::BusSlave gyroSlaveBus;
static Espfc::Device::Gyro::GyroMPU6050 mpu6050;
static Espfc::Device::Gyro::GyroMPU6500 mpu6500;
static Espfc::Device::Gyro::GyroMPU9250 mpu9250;
static Espfc::Device::Gyro::GyroLSM6DSO lsm6dso;
static Espfc::Device::Gyro::GyroICM20602 icm20602;
static Espfc::Device::Gyro::GyroICM42688 icm42688;
static Espfc::Device::Gyro::GyroBMI160 bmi160;
static Espfc::Device::Mag::MagHMC5883L hmc5883l;
static Espfc::Device::Mag::MagQMC5883L qmc5883l;
static Espfc::Device::Mag::MagQMC5883P qmc5883p;
static Espfc::Device::Mag::MagAK8963 ak8963;
static Espfc::Device::Baro::BaroBMP085 bmp085;
static Espfc::Device::Baro::BaroBMP280 bmp280;
static Espfc::Device::Baro::BaroSPL06 spl06;
static bool busAllowed(
    int8_t configured,
    Espfc::BusType actual)
{
  return
      configured ==
          static_cast<int8_t>(
              Espfc::BUS_AUTO) ||
      configured ==
          static_cast<int8_t>(
              actual);
}

template<typename DeviceType>
static bool deviceAllowed(
    int8_t configured,
    DeviceType automaticValue,
    DeviceType actual)
{
  return
      configured ==
          static_cast<int8_t>(
              automaticValue) ||
      configured ==
          static_cast<int8_t>(
              actual);
}
} // namespace

namespace Espfc {

Hardware::Hardware(Model& model): _model(model) {}

int Hardware::begin()
{
  initBus();
  detectGyro();
  detectMag();
  detectBaro();
  return 1;
}

void Hardware::onI2CError()
{
  _model.state.i2cErrorCount++;
  _model.state.i2cErrorDelta++;
}

void Hardware::initBus()
{
#if defined(ESPFC_SPI_0)
  int spiResult = spiBus.begin(_model.config.pin[PIN_SPI_0_SCK], _model.config.pin[PIN_SPI_0_MOSI],
                               _model.config.pin[PIN_SPI_0_MISO]);
  _model.logger.info()
      .log("SPI")
      .log(_model.config.pin[PIN_SPI_0_SCK])
      .log(_model.config.pin[PIN_SPI_0_MOSI])
      .log(_model.config.pin[PIN_SPI_0_MISO])
      .logln(spiResult);
#endif
#if defined(ESPFC_I2C_0)
  _model.config.i2cSpeed =
      std::clamp<int16_t>(
          _model.config.i2cSpeed,
          10,
          400);

  int i2cResult =
      i2cBus.begin(
          _model.config.pin[PIN_I2C_0_SDA],
          _model.config.pin[PIN_I2C_0_SCL],
          static_cast<uint32_t>(_model.config.i2cSpeed) * 1000ul);
  i2cBus.onError = [this]() { onI2CError(); };
  _model.logger.info()
      .log("I2C")
      .log(_model.config.pin[PIN_I2C_0_SDA])
      .log(_model.config.pin[PIN_I2C_0_SCL])
      .log(_model.config.i2cSpeed)
      .logln(i2cResult);
#endif
}

void Hardware::detectGyro()
{
  _model.state.gyro.dev = nullptr;
  _model.state.gyro.present = false;
  _model.state.accel.present = false;

  const int8_t configuredDev =
      _model.config.gyro.dev;

const int8_t configuredBus =
    _model.config.gyro.bus;

// Some native builds compile without SPI/I2C detection
// blocks, so configuredBus is intentionally unused there.
(void)configuredBus;

  if (configuredDev == GYRO_NONE)
  {
    return;
  }

  Device::GyroDevice* detectedGyro =
      nullptr;

#if defined(ESPFC_SPI_0)
  if (busAllowed(configuredBus, BUS_SPI) &&
      _model.config.pin[PIN_SPI_CS0] != -1)
  {
    const int cs =
        _model.config.pin[PIN_SPI_CS0];

    Hal::Gpio::digitalWrite(
        cs,
        Hal::Gpio::High);

    Hal::Gpio::pinMode(
        cs,
        Hal::Gpio::Output);

    if (!detectedGyro &&
        deviceAllowed(
            configuredDev,
            GYRO_AUTO,
            mpu9250.getType()) &&
        detectDevice(
            mpu9250,
            spiBus,
            cs))
    {
      detectedGyro = &mpu9250;
    }

    if (!detectedGyro &&
        deviceAllowed(
            configuredDev,
            GYRO_AUTO,
            mpu6500.getType()) &&
        detectDevice(
            mpu6500,
            spiBus,
            cs))
    {
      detectedGyro = &mpu6500;
    }

    if (!detectedGyro &&
        deviceAllowed(
            configuredDev,
            GYRO_AUTO,
            icm20602.getType()) &&
        detectDevice(
            icm20602,
            spiBus,
            cs))
    {
      detectedGyro = &icm20602;
    }

    if (!detectedGyro &&
        deviceAllowed(
            configuredDev,
            GYRO_AUTO,
            icm42688.getType()) &&
        detectDevice(
            icm42688,
            spiBus,
            cs))
    {
      detectedGyro = &icm42688;
    }

    if (!detectedGyro &&
        deviceAllowed(
            configuredDev,
            GYRO_AUTO,
            bmi160.getType()) &&
        detectDevice(
            bmi160,
            spiBus,
            cs))
    {
      detectedGyro = &bmi160;
    }

    if (!detectedGyro &&
        deviceAllowed(
            configuredDev,
            GYRO_AUTO,
            lsm6dso.getType()) &&
        detectDevice(
            lsm6dso,
            spiBus,
            cs))
    {
      detectedGyro = &lsm6dso;
    }

    if (detectedGyro)
    {
      gyroSlaveBus.begin(
          &spiBus,
          detectedGyro->getAddress());
    }
  }
#endif

#if defined(ESPFC_I2C_0)
  if (!detectedGyro &&
      busAllowed(configuredBus, BUS_I2C) &&
      _model.config.pin[PIN_I2C_0_SDA] != -1 &&
      _model.config.pin[PIN_I2C_0_SCL] != -1)
  {
    if (deviceAllowed(
            configuredDev,
            GYRO_AUTO,
            mpu9250.getType()) &&
        detectDevice(
            mpu9250,
            i2cBus))
    {
      detectedGyro = &mpu9250;
    }

    if (!detectedGyro &&
        deviceAllowed(
            configuredDev,
            GYRO_AUTO,
            mpu6500.getType()) &&
        detectDevice(
            mpu6500,
            i2cBus))
    {
      detectedGyro = &mpu6500;
    }

    if (!detectedGyro &&
        deviceAllowed(
            configuredDev,
            GYRO_AUTO,
            icm20602.getType()) &&
        detectDevice(
            icm20602,
            i2cBus))
    {
      detectedGyro = &icm20602;
    }

    if (!detectedGyro &&
        deviceAllowed(
            configuredDev,
            GYRO_AUTO,
            bmi160.getType()) &&
        detectDevice(
            bmi160,
            i2cBus))
    {
      detectedGyro = &bmi160;
    }

    if (!detectedGyro &&
        deviceAllowed(
            configuredDev,
            GYRO_AUTO,
            mpu6050.getType()) &&
        detectDevice(
            mpu6050,
            i2cBus))
    {
      detectedGyro = &mpu6050;
    }

    if (!detectedGyro &&
        deviceAllowed(
            configuredDev,
            GYRO_AUTO,
            lsm6dso.getType()) &&
        detectDevice(
            lsm6dso,
            i2cBus))
    {
      detectedGyro = &lsm6dso;
    }

    if (detectedGyro)
    {
      gyroSlaveBus.begin(
          &i2cBus,
          detectedGyro->getAddress());
    }
  }
#endif

  if (!detectedGyro)
  {
    return;
  }

detectedGyro->setDLPFMode(
    _model.config.gyro.dlpf);

if (!detectedGyro->configurationValid())
{
  return;
}

_model.state.gyro.dev =
    detectedGyro;

  _model.state.gyro.present =
      true;

  _model.state.gyro.clock =
      detectedGyro->getRate();

  const Device::BusDevice* detectedBus =
      detectedGyro->getBus();

  const bool accelDeviceMatches =
      _model.config.accel.dev == GYRO_AUTO ||
      _model.config.accel.dev ==
          static_cast<int8_t>(
              detectedGyro->getType());

  const bool accelBusMatches =
      detectedBus &&
      busAllowed(
          _model.config.accel.bus,
          detectedBus->getType());

  _model.state.accel.present =
      _model.config.accel.dev != GYRO_NONE &&
      accelDeviceMatches &&
      accelBusMatches;
}

void Hardware::detectMag()
{
  _model.state.mag.dev = nullptr;
  _model.state.mag.present = false;
  _model.state.mag.rate = 0;

  const int8_t configuredDev =
      _model.config.mag.dev;

  const int8_t configuredBus =
      _model.config.mag.bus;

  if (configuredDev == MAG_NONE)
  {
    return;
  }

  Device::MagDevice* detectedMag =
      nullptr;

#if defined(ESPFC_I2C_0)
  if (busAllowed(configuredBus, BUS_I2C) &&
      _model.config.pin[PIN_I2C_0_SDA] != -1 &&
      _model.config.pin[PIN_I2C_0_SCL] != -1)
  {
    if (!detectedMag &&
        deviceAllowed(
            configuredDev,
            MAG_DEFAULT,
            ak8963.getType()) &&
        detectDevice(ak8963, i2cBus))
    {
      detectedMag = &ak8963;
    }

    if (!detectedMag &&
        deviceAllowed(
            configuredDev,
            MAG_DEFAULT,
            hmc5883l.getType()) &&
        detectDevice(hmc5883l, i2cBus))
    {
      detectedMag = &hmc5883l;
    }

    if (!detectedMag &&
        deviceAllowed(
            configuredDev,
            MAG_DEFAULT,
            qmc5883l.getType()) &&
        detectDevice(qmc5883l, i2cBus))
    {
      detectedMag = &qmc5883l;
    }

    if (!detectedMag &&
        deviceAllowed(
            configuredDev,
            MAG_DEFAULT,
            qmc5883p.getType()) &&
        detectDevice(qmc5883p, i2cBus))
    {
      detectedMag = &qmc5883p;
    }
  }
#endif

  if (!detectedMag &&
      busAllowed(configuredBus, BUS_SLV) &&
      gyroSlaveBus.getBus())
  {
    if (deviceAllowed(
            configuredDev,
            MAG_DEFAULT,
            ak8963.getType()) &&
        detectDevice(ak8963, gyroSlaveBus))
    {
      detectedMag = &ak8963;
    }

    if (!detectedMag &&
        deviceAllowed(
            configuredDev,
            MAG_DEFAULT,
            hmc5883l.getType()) &&
        detectDevice(hmc5883l, gyroSlaveBus))
    {
      detectedMag = &hmc5883l;
    }

    if (!detectedMag &&
        deviceAllowed(
            configuredDev,
            MAG_DEFAULT,
            qmc5883l.getType()) &&
        detectDevice(qmc5883l, gyroSlaveBus))
    {
      detectedMag = &qmc5883l;
    }

    if (!detectedMag &&
        deviceAllowed(
            configuredDev,
            MAG_DEFAULT,
            qmc5883p.getType()) &&
        detectDevice(qmc5883p, gyroSlaveBus))
    {
      detectedMag = &qmc5883p;
    }
  }

  _model.state.mag.dev =
      detectedMag;

  _model.state.mag.present =
      detectedMag != nullptr;

  _model.state.mag.rate =
      detectedMag
          ? detectedMag->getRate()
          : 0;
}

void Hardware::detectBaro()
{
  _model.state.baro.dev = nullptr;
  _model.state.baro.present = false;

  const int8_t configuredDev =
      _model.config.baro.dev;

  const int8_t configuredBus =
      _model.config.baro.bus;

  if (configuredDev == BARO_NONE)
  {
    return;
  }

  Device::BaroDevice* detectedBaro =
      nullptr;

#if defined(ESPFC_SPI_0)
  if (busAllowed(configuredBus, BUS_SPI) &&
      _model.config.pin[PIN_SPI_CS1] != -1)
  {
    const int cs =
        _model.config.pin[PIN_SPI_CS1];

    Hal::Gpio::digitalWrite(
        cs,
        Hal::Gpio::High);

    Hal::Gpio::pinMode(
        cs,
        Hal::Gpio::Output);

    if (!detectedBaro &&
        deviceAllowed(
            configuredDev,
            BARO_DEFAULT,
            bmp280.getType()) &&
        detectDevice(
            bmp280,
            spiBus,
            cs))
    {
      detectedBaro = &bmp280;
    }

    if (!detectedBaro &&
        deviceAllowed(
            configuredDev,
            BARO_DEFAULT,
            bmp085.getType()) &&
        detectDevice(
            bmp085,
            spiBus,
            cs))
    {
      detectedBaro = &bmp085;
    }

    if (!detectedBaro &&
        deviceAllowed(
            configuredDev,
            BARO_DEFAULT,
            spl06.getType()) &&
        detectDevice(
            spl06,
            spiBus,
            cs))
    {
      detectedBaro = &spl06;
    }
  }
#endif

#if defined(ESPFC_I2C_0)
  if (!detectedBaro &&
      busAllowed(configuredBus, BUS_I2C) &&
      _model.config.pin[PIN_I2C_0_SDA] != -1 &&
      _model.config.pin[PIN_I2C_0_SCL] != -1)
  {
    if (deviceAllowed(
            configuredDev,
            BARO_DEFAULT,
            bmp280.getType()) &&
        detectDevice(bmp280, i2cBus))
    {
      detectedBaro = &bmp280;
    }

    if (!detectedBaro &&
        deviceAllowed(
            configuredDev,
            BARO_DEFAULT,
            bmp085.getType()) &&
        detectDevice(bmp085, i2cBus))
    {
      detectedBaro = &bmp085;
    }

    if (!detectedBaro &&
        deviceAllowed(
            configuredDev,
            BARO_DEFAULT,
            spl06.getType()) &&
        detectDevice(spl06, i2cBus))
    {
      detectedBaro = &spl06;
    }
  }
#endif

  if (!detectedBaro &&
      busAllowed(configuredBus, BUS_SLV) &&
      gyroSlaveBus.getBus())
  {
    if (deviceAllowed(
            configuredDev,
            BARO_DEFAULT,
            bmp280.getType()) &&
        detectDevice(bmp280, gyroSlaveBus))
    {
      detectedBaro = &bmp280;
    }

    if (!detectedBaro &&
        deviceAllowed(
            configuredDev,
            BARO_DEFAULT,
            bmp085.getType()) &&
        detectDevice(bmp085, gyroSlaveBus))
    {
      detectedBaro = &bmp085;
    }

    if (!detectedBaro &&
        deviceAllowed(
            configuredDev,
            BARO_DEFAULT,
            spl06.getType()) &&
        detectDevice(spl06, gyroSlaveBus))
    {
      detectedBaro = &spl06;
    }
  }

  _model.state.baro.dev =
      detectedBaro;

  _model.state.baro.present =
      detectedBaro != nullptr;
}

void Hardware::restart(const Model& model)
{
  if (model.state.mixer.escMotor) model.state.mixer.escMotor->end();
  if (model.state.mixer.escServo) model.state.mixer.escServo->end();
#ifdef ESPFC_SERIAL_SOFT_0_WIFI
  WiFi.disconnect();
  WiFi.softAPdisconnect();
#endif
  delay(100);
  targetReset();
}

} // namespace Espfc
