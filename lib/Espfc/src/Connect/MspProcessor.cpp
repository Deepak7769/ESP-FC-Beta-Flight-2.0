#include "Connect/MspProcessor.hpp"
#include "Utils/ConfigValidation.hpp"
#include "Control/RescueSettings.hpp"
#include <memory>
#include "Connect/MspBoxMapping.hpp"
#include "Control/AssistedModeV2.h"
#include "Hardware.h"
#include "Hal/Time.hpp"
#include "Model.h"
#include "ModelConfig.h"
#include "Stream/Printer.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <platform.h>
#if defined(ESPFC_MULTI_CORE) && defined(ESPFC_FREE_RTOS)
#include <driver/timer.h>
#endif

#define VTXCOMMON_MSP_BANDCHAN_CHKVAL ((uint16_t)((7 << 3) + 7))

extern "C" {
#include "blackbox/blackbox.h"
#include "blackbox/blackbox_io.h"
#include "io/serial_4way.h"
#include "msp/msp_protocol.h"
#include "msp/msp_protocol_v2_betaflight.h"
#include "msp/msp_protocol_v2_common.h"
int blackboxCalculatePDenom(int rateNum, int rateDenom);
uint8_t blackboxCalculateSampleRate(uint16_t pRatio);
uint8_t blackboxGetRateDenom(void);
uint16_t blackboxGetPRatio(void);
bool blackboxMayEditConfig(void);
}

namespace {

// Configuration writes, maintenance and actuator tests are never permitted
// from MSP during an armed flight. Custom dual-use GET/SET commands keep
// their zero-length telemetry reads available.
static bool mspCommandRequiresDisarmed(uint16_t cmd, int payloadSize)
{
  switch (cmd)
  {
    case MSP_SET_NAME:
    case MSP2_SET_TEXT:
    case MSP_SET_MODE_RANGE:
    case MSP_SET_FEATURE_CONFIG:
    case MSP_SET_BATTERY_CONFIG:
    case MSP_SET_VOLTAGE_METER_CONFIG:
    case MSP_SET_CURRENT_METER_CONFIG:
    case MSP_DATAFLASH_ERASE:
    case MSP_SET_ACC_TRIM:
    case MSP_SET_MIXER_CONFIG:
    case MSP_SET_SENSOR_CONFIG:
    case MSP_SET_SENSOR_ALIGNMENT:
    case MSP_SET_CF_SERIAL_CONFIG:
    case MSP2_COMMON_SET_SERIAL_CONFIG:
    case MSP_SET_BLACKBOX_CONFIG:
    case MSP_SET_BEEPER_CONFIG:
    case MSP_SET_BOARD_ALIGNMENT_CONFIG:
    case MSP_SET_RX_MAP:
    case MSP_SET_RSSI_CONFIG:
    case MSP_SET_MOTOR_CONFIG:
    case MSP_SET_ARMING_CONFIG:
    case MSP_SET_RC_DEADBAND:
    case MSP_SET_RX_CONFIG:
    case MSP_SET_FAILSAFE_CONFIG:
    case MSP_SET_RXFAIL_CONFIG:
    case MSP_SET_RC_TUNING:
    case MSP_SET_ADVANCED_CONFIG:
    case MSP_SET_COMPASS_CONFIG:
    case MSP_SET_FILTER_CONFIG:
    case MSP_SET_PID_CONTROLLER:
    case MSP_SET_PID:
    case MSP_SET_PID_ADVANCED:
    case MSP_SET_SIMPLIFIED_TUNING:
    case MSP_SET_MOTOR:
    case MSP_SET_SERVO_CONFIGURATION:
    case MSP_ACC_CALIBRATION:
    case MSP_MAG_CALIBRATION:
    case MSP_SET_VTX_CONFIG:
    case MSP_SET_ARMING_DISABLED:
    case MSP_SET_PASSTHROUGH:
    case MSP_SET_GPS_RESCUE:
    case MSP_SET_GPS_CONFIG:
    case MSP_EEPROM_WRITE:
    case MSP_RESET_CONF:
    case MSP_REBOOT:
    case MSP_SET_RTC:
      return true;

    case Espfc::Connect::MSP2_ESPFC_ALTHOLD_CONFIG:
    case Espfc::Connect::MSP2_ESPFC_GPS_RESCUE_CONFIG:
      return payloadSize != 0;

    default:
      return false;
  }
}

static bool mspConfigurationWrite(uint16_t cmd, int size)
{
  if (!mspCommandRequiresDisarmed(cmd, size)) return false;
  switch (cmd)
  {
    case MSP_DATAFLASH_ERASE: case MSP_SET_MOTOR: case MSP_SET_ARMING_DISABLED:
    case MSP_SET_PASSTHROUGH: case MSP_ACC_CALIBRATION: case MSP_MAG_CALIBRATION:
    case MSP_EEPROM_WRITE: case MSP_RESET_CONF: case MSP_REBOOT: case MSP_SET_RTC:
      return false;
    default: return true;
  }
}

// The low-level MSP read helpers deliberately return zero for truncated
// data, so validate fixed or versioned payload lengths BEFORE any setter
// mutates configuration. Never interpret a short packet as a real zero value.
static bool mspMutatorPayloadSizeValid(uint16_t cmd, int size)
{
  switch (cmd)
  {
    case MSP_SET_FEATURE_CONFIG:
    case MSP_SET_ACC_TRIM:
    case MSP_SET_BEEPER_CONFIG:
    case MSP_SET_VOLTAGE_METER_CONFIG:
    case MSP_SET_GPS_CONFIG:
      if (cmd == MSP_SET_GPS_CONFIG) return size == 4 || size == 6;
      if (cmd == MSP_SET_BEEPER_CONFIG) return size == 4 || size == 9;
      return size == 4;

    case MSP_SET_CURRENT_METER_CONFIG:
      return size == 5;

    case MSP_SET_MODE_RANGE:
      return size == 5 || size == 7;

    case MSP_SET_BATTERY_CONFIG:
      return size == 7 || size == 13;

    case MSP_SET_BOARD_ALIGNMENT_CONFIG:
      return size == 6;

    case MSP_SET_RX_MAP:
      return size == 8;

    case MSP_SET_RSSI_CONFIG:
    case MSP_SET_PID_CONTROLLER:
      return size == 1;

    case MSP_SET_SENSOR_CONFIG:
      return size >= 3 && size <= 5;

    // Legal payload versions stop at whole optional field groups. A partial
    // group must never silently leave some parameters changed.
    case MSP_SET_ARMING_CONFIG:
      return size == 4;
    case MSP_SET_RC_DEADBAND:
      return size == 5;
    case MSP_SET_FAILSAFE_CONFIG:
      return size == 8;
    case MSP_SET_MOTOR_CONFIG:
      return size == 6 || size == 8 || size == 9;
    case MSP_SET_RX_CONFIG:
      return size == 12 || size == 16 || size == 22 ||
             size == 23 || size == 29 || size == 30 ||
             size == 31 || size == 32 || size == 38;
    case MSP_SET_RC_TUNING:
      return size == 10 || size == 11 || size == 12 ||
             size == 13 || size == 14 || size == 16 ||
             size == 22 || size == 23;
    case MSP_SET_ADVANCED_CONFIG:
      return size == 6 || size == 8 || size == 9 ||
             size == 10 || size == 18 || size == 19 ||
             size == 20; // last byte is read-only DEBUG_COUNT
    case MSP_SET_FILTER_CONFIG:
      return size == 5 || size == 13 || size == 17 ||
             size == 18 || size == 28 || size == 37 ||
             size == 45 || size == 47 || size == 49;

    case MSP_SET_GPS_RESCUE: return size == 26;
    case MSP_SET_BLACKBOX_CONFIG: return size == 3 || size == 5 || size == 6 || size == 10;
    case MSP_SET_VTX_CONFIG: return size == 2 || size == 4 || size == 5 || size == 7 || size == 11;
    case MSP_SET_RTC: return size == 6;
    case MSP_ACC_CALIBRATION: case MSP_MAG_CALIBRATION: case MSP_EEPROM_WRITE:
    case MSP_RESET_CONF: case MSP_DATAFLASH_ERASE: return size == 0;
    case MSP_REBOOT: return size == 0 || size == 1;
    case MSP_SET_PASSTHROUGH: return size == 0 || size == 2;
    case MSP_SET_ARMING_DISABLED: return size == 1 || size == 2;
    default:
      return true; // Other setters have their own variable-length contracts.
  }
}

// Validate dangerous values before the setter touches any persistent field.
// Access only indices already covered by the version-aware length contract.
static uint16_t mspPayloadU16(const Espfc::Connect::MspMessage& m,
                              uint16_t offset)
{
  const uint16_t start = static_cast<uint16_t>(m.read + offset);
  return static_cast<uint16_t>(m.buffer[start]) |
         (static_cast<uint16_t>(m.buffer[start + 1]) << 8u);
}

static bool mspMutatorValuesValid(const Espfc::Connect::MspMessage& m)
{
  if (m.cmd == MSP_SET_RC_TUNING)
  {
    if (m.remain() >= 22)
      for (uint16_t offset : {16u, 18u, 20u})
      {
        const uint16_t limit = mspPayloadU16(m, offset);
        if (limit < 1u || limit > 1998u) return false;
      }
    if (m.remain() == 23 && m.buffer[m.read + 22] > 4u) return false;
  }
  if (m.cmd == MSP_SET_PID_ADVANCED && m.remain() >= 23 && mspPayloadU16(m, 21) > 250) return false;
  if (m.cmd == MSP_SET_RX_CONFIG && m.remain() >= 16)
  {
    // Airmode threshold is supplied in microseconds; never let a value
    // below 1000 underflow when converted to internal 0..100 percent.
    const uint16_t airmodeUs = mspPayloadU16(m, 14);
    if (airmodeUs < 1000u || airmodeUs > 2000u ||
        ((airmodeUs - 1000u) % 10u) != 0u)
      return false;
  }
  if (m.cmd == MSP_SET_MOTOR_CONFIG && m.remain() >= 6)
  {
    const uint16_t maxThrottle = mspPayloadU16(m, 2);
    const uint16_t minCommand = mspPayloadU16(m, 4);
    if (maxThrottle < 1000u || maxThrottle > 2000u ||
        minCommand < 900u || minCommand > maxThrottle)
      return false;
  }
  return true;
}

// MSP uses Betaflight batteryState_e values. This is a display-only
// warning: a missing pack or configured low-cell threshold should not be
// reported as an unrestricted healthy battery. Critical voltage and
// per-cell chemistry settings are not yet modeled by ESP-FC.
static uint8_t mspBatteryState(const Espfc::Model& model)
{
  const auto& battery = model.state.battery;
  if (battery.cells == 0) return battery.voltageUnfiltered >= 2.5f ? 4u : 3u; // BATTERY_NOT_PRESENT
  if (!std::isfinite(battery.voltage) || battery.voltage <= 0.0f)
    return 4u; // BATTERY_INIT / measurement unavailable

  const int16_t warningCellCentivolts = model.config.vbat.cellWarning;
  if (warningCellCentivolts > 0 &&
      battery.voltage * 100.0f <
        static_cast<float>(battery.cells) * warningCellCentivolts)
    return 1u; // BATTERY_WARNING

  return 0u; // BATTERY_OK
}

enum SerialSpeedIndex
{
  SERIAL_SPEED_INDEX_AUTO = 0,
  SERIAL_SPEED_INDEX_9600,
  SERIAL_SPEED_INDEX_19200,
  SERIAL_SPEED_INDEX_38400,
  SERIAL_SPEED_INDEX_57600,
  SERIAL_SPEED_INDEX_115200,
  SERIAL_SPEED_INDEX_230400,
  SERIAL_SPEED_INDEX_250000,
  SERIAL_SPEED_INDEX_400000,
  SERIAL_SPEED_INDEX_460800,
  SERIAL_SPEED_INDEX_500000,
  SERIAL_SPEED_INDEX_921600,
  SERIAL_SPEED_INDEX_1000000,
  SERIAL_SPEED_INDEX_1500000,
  SERIAL_SPEED_INDEX_2000000,
  SERIAL_SPEED_INDEX_2470000,
};

static SerialSpeedIndex toBaudIndex(int32_t speed)
{
  // clang-format off
  using namespace Espfc;
  if(speed >= SERIAL_SPEED_2470000) return SERIAL_SPEED_INDEX_2470000;
  if(speed >= SERIAL_SPEED_2000000) return SERIAL_SPEED_INDEX_2000000;
  if(speed >= SERIAL_SPEED_1500000) return SERIAL_SPEED_INDEX_1500000;
  if(speed >= SERIAL_SPEED_1000000) return SERIAL_SPEED_INDEX_1000000;
  if(speed >= SERIAL_SPEED_921600)  return SERIAL_SPEED_INDEX_921600;
  if(speed >= SERIAL_SPEED_500000)  return SERIAL_SPEED_INDEX_500000;
  if(speed >= SERIAL_SPEED_460800)  return SERIAL_SPEED_INDEX_460800;
  if(speed >= SERIAL_SPEED_400000)  return SERIAL_SPEED_INDEX_400000;
  if(speed >= SERIAL_SPEED_250000)  return SERIAL_SPEED_INDEX_250000;
  if(speed >= SERIAL_SPEED_230400)  return SERIAL_SPEED_INDEX_230400;
  if(speed >= SERIAL_SPEED_115200)  return SERIAL_SPEED_INDEX_115200;
  if(speed >= SERIAL_SPEED_57600)   return SERIAL_SPEED_INDEX_57600;
  if(speed >= SERIAL_SPEED_38400)   return SERIAL_SPEED_INDEX_38400;
  if(speed >= SERIAL_SPEED_19200)   return SERIAL_SPEED_INDEX_19200;
  if(speed >= SERIAL_SPEED_9600)    return SERIAL_SPEED_INDEX_9600;
  return SERIAL_SPEED_INDEX_AUTO;
  // clang-format on
}

static Espfc::SerialSpeed fromBaudIndex(SerialSpeedIndex index)
{
  // clang-format off
  using namespace Espfc;
  switch(index)
  {
    case SERIAL_SPEED_INDEX_9600:    return SERIAL_SPEED_9600;
    case SERIAL_SPEED_INDEX_19200:   return SERIAL_SPEED_19200;
    case SERIAL_SPEED_INDEX_38400:   return SERIAL_SPEED_38400;
    case SERIAL_SPEED_INDEX_57600:   return SERIAL_SPEED_57600;
    case SERIAL_SPEED_INDEX_115200:  return SERIAL_SPEED_115200;
    case SERIAL_SPEED_INDEX_230400:  return SERIAL_SPEED_230400;
    case SERIAL_SPEED_INDEX_250000:  return SERIAL_SPEED_250000;
    case SERIAL_SPEED_INDEX_400000:  return SERIAL_SPEED_400000;
    case SERIAL_SPEED_INDEX_460800:  return SERIAL_SPEED_460800;
    case SERIAL_SPEED_INDEX_500000:  return SERIAL_SPEED_500000;
    case SERIAL_SPEED_INDEX_921600:  return SERIAL_SPEED_921600;
    case SERIAL_SPEED_INDEX_1000000: return SERIAL_SPEED_1000000;
    case SERIAL_SPEED_INDEX_1500000: return SERIAL_SPEED_1500000;
    case SERIAL_SPEED_INDEX_2000000: return SERIAL_SPEED_2000000;
    case SERIAL_SPEED_INDEX_2470000: return SERIAL_SPEED_2470000;
    case SERIAL_SPEED_INDEX_AUTO:
    default: return SERIAL_SPEED_NONE;
  }
  // clang-format on
}

static uint8_t fromGyroDlpf(uint8_t t)
{
  switch (t)
  {
    case Espfc::GYRO_DLPF_256:
      return 0;
    case Espfc::GYRO_DLPF_EX:
      return 1;
    default:
      return 2;
  }
}

static int8_t toVbatSource(uint8_t t)
{
  switch (t)
  {
    case 0:
      return 0; // none
    case 1:
      return 1; // internal adc
    default:
      return 0;
  }
}

static int8_t toIbatSource(uint8_t t)
{
  switch (t)
  {
    case CURRENT_METER_NONE:
      return CURRENT_METER_NONE;

    case CURRENT_METER_ADC:
      return CURRENT_METER_ADC;

    case CURRENT_METER_MSP:
      return CURRENT_METER_MSP;

    // VIRTUAL and ESC are Betaflight IDs, but ESP-FC has no implementation
    // for those sources yet. Do not silently retain an unsupported source.
    default:
      return CURRENT_METER_NONE;
  }
}

static uint8_t toVbatVoltageLegacy(float voltage)
{
  return std::clamp<long>(lrintf(voltage * 10.0f), 0L, 255L);
}

static uint16_t toVbatVoltage(float voltage)
{
  return std::clamp<long>(lrintf(voltage * 100.0f), 0L, 32000L);
}

static uint16_t toIbatCurrent(float current)
{
  return std::clamp<long>(lrintf(current * 100.0f), -32000L, 32000L);
}

constexpr uint8_t MSP_PASSTHROUGH_ESC_4WAY = 0xff;

// ESP-FC project version in the three-byte MSP version envelope.
constexpr uint8_t MSP_FC_VERSION_YEAR = 2026 - 2000;
constexpr uint8_t MSP_FC_VERSION_MONTH = 6;
constexpr uint8_t MSP_FC_VERSION_PATCH = FC_VERSION_PATCH_LEVEL;

// MCU type id sentinel telling the configurator the name follows as a string
constexpr uint8_t MCU_TYPE_ID_PROVIDED_BY_NAME = 255;

// Reported for sensor slots the target does not support at all
constexpr uint8_t SENSOR_NOT_AVAILABLE = 0xff;

static uint8_t toAccHw(uint8_t dev)
{
  if (dev == 0) return 1;
  if (dev == 1) return 0;
  return dev;
}

static uint8_t fromAccHw(uint8_t dev)
{
  if (dev == 0) return 1;
  if (dev == 1) return 0;
  return dev;
}

} // namespace

namespace Espfc::Connect {

MspProcessor::MspProcessor(Model& model): _model(model) {}

bool MspProcessor::parse(char c, MspMessage& msg)
{
  _parser.parse(c, msg);

  if (msg.isReady())
  {
    debugMessage(msg);
  }

  return !msg.isIdle();
}

void MspProcessor::processReply(MspMessage& m)
{
  // Betaflight CURRENT_METER_MSP consumes the standard MSP_ANALOG reply.
  // This lets a companion processor publish INA219-derived current using the
  // same payload and units as a Betaflight MSP current meter.
  if (m.cmd != MSP_ANALOG ||
      _model.config.ibat.source !=
          CURRENT_METER_MSP ||
      m.remain() < 7)
  {
    return;
  }

  // Legacy voltage is part of MSP_ANALOG but ESP-FC keeps voltage-source
  // selection independent from current-source selection.
  m.readU8();

  const uint16_t mAhDrawn =
      m.readU16();

  // RSSI is not owned by the current-meter bridge.
  m.readU16();

  const int16_t currentCentiAmps =
      static_cast<int16_t>(
          m.readU16());

  auto& battery =
      _model.state.battery;

  battery.mspCurrentCentiAmps =
      currentCentiAmps;

  battery.mspMahDrawn =
      mAhDrawn;

  battery.mspCurrentLastUpdateUs =
      micros();

  battery.mspCurrentValid =
      true;
}


void MspProcessor::processCommand(MspMessage& m, MspResponse& r, Stream::ReadWritable& s)
{
  r.cmd = m.cmd;
  r.version = m.version;
  r.result = 1;
  r.len = 0;
  if (m.received > MSP_BUF_SIZE || m.read > m.received) { r.result = -1; return; }
  if (!mspMutatorPayloadSizeValid(m.cmd, m.remain()) ||
      !mspMutatorValuesValid(m))
  {
    r.result = -1;
    return;
  }
  if (_model.isModeActive(MODE_ARMED) &&
      mspCommandRequiresDisarmed(m.cmd, m.remain()))
  {
    r.result = -1;
    return;
  }
  const bool configWrite = mspConfigurationWrite(m.cmd, m.remain());
  std::unique_ptr<ModelConfig> staged;
  if (configWrite) staged.reset(new ModelConfig(_model.config));
  ModelConfig& config = staged ? *staged : _model.config;
  bool needsReload = false, needsReboot = false, needsSanitize = false;
  bool events[5]{};
  switch (m.cmd)
  {
    case MSP_API_VERSION:
      r.writeU8(MSP_PROTOCOL_VERSION);
      r.writeU8(API_VERSION_MAJOR);
      r.writeU8(API_VERSION_MINOR);
      break;

    case MSP_FC_VARIANT:
      r.writeData(flightControllerIdentifier, FLIGHT_CONTROLLER_IDENTIFIER_LENGTH);
      break;

    case MSP_FC_VERSION:
      r.writeU8(MSP_FC_VERSION_YEAR);
      r.writeU8(MSP_FC_VERSION_MONTH);
      r.writeU8(MSP_FC_VERSION_PATCH);
      break;

    case MSP2_MCU_INFO:
      r.writeU8(MCU_TYPE_ID_PROVIDED_BY_NAME);
      r.writePString(targetName);
      break;

    case MSP_BOARD_INFO:
      r.writeData(boardIdentifier, BOARD_IDENTIFIER_LENGTH);
      r.writeU16(0); // No other build targets currently have hardware revision detection.
      r.writeU8(0);  // 0 == FC
      {
        uint8_t targetCapabilities = 0;
#if defined(ESPFC_SERIAL_USB)
        constexpr uint8_t TARGET_HAS_VCP = 0;
        targetCapabilities |= 1 << TARGET_HAS_VCP;
#endif
        r.writeU8(targetCapabilities); // target capabilities
      }
      r.writeU8(std::strlen(targetName)); // target name
      r.writeData(targetName, std::strlen(targetName));
      r.writeU8(0); // board name
      r.writeU8(0); // manufacturer name
      for (size_t i = 0; i < 32; i++)
      {
        r.writeU8(0); // signature
      }
      r.writeU8(MCU_TYPE_ID_PROVIDED_BY_NAME); // mcu id
      // 1.42
      r.writeU8(2); // configuration state: configured
      // 1.43
      r.writeU16(_model.state.gyro.present ? _model.state.gyro.timer.rate : 0); // sample rate
      {
        uint32_t problems = 0;
        if (_model.state.accel.bias.x == 0 && _model.state.accel.bias.y == 0 && _model.state.accel.bias.z == 0)
        {
          problems |= 1 << 0; // acc calibration required
        }
        if (config.output.protocol == ESC_PROTOCOL_DISABLED)
        {
          problems |= 1 << 1; // no motor protocol
        }
        r.writeU32(problems); // configuration problems
      }
      // 1.44
      r.writeU8(0); // spi dev count
      r.writeU8(0); // i2c dev count
      break;

case MSP_BUILD_INFO:
  r.writeData(buildDate, BUILD_DATE_LENGTH);
  r.writeData(buildTime, BUILD_TIME_LENGTH);
  r.writeData(shortGitRevision, GIT_SHORT_REVISION_LENGTH);

  // MSP API 1.46+: report every protocol/feature that this firmware actually
  // exposes to Configurator. An incomplete build-option list is dangerous:
  // Configurator uses it to disable unsupported receiver and motor choices.
  r.writeU16(4097);  // USE_SERIALRX_CRSF
  r.writeU16(4100);  // USE_SERIALRX_IBUS
  r.writeU16(4102);  // USE_RX_PPM
  r.writeU16(4103);  // USE_SERIALRX_SBUS
  r.writeU16(16412); // USE_GPS
  r.writeU16(16415); // USE_MAG
  #if defined(ESPFC_ALTHOLD_V2_ACTIVE)
  r.writeU16(16422); // USE_ALTITUDE_HOLD only with authority
  #endif
  if (Control::softSerialSupported()) r.writeU16(16423); // USE_SOFTSERIAL
  if (Control::vtxSupported()) r.writeU16(16421); // USE_VTX
#if defined(ESPFC_ALTHOLD_V2_ACTIVE) && defined(ESPFC_LAND_V2_ACTIVE)
  r.writeU16(16425); // USE_POSITION_HOLD
#endif
  r.writeU16(8230);  // USE_BRUSHED
  r.writeU16(8231);  // USE_DSHOT
  r.writeU16(8232);  // USE_MULTISHOT
  r.writeU16(8233);  // USE_ONESHOT
  r.writeU16(8235);  // USE_PWM_OUTPUT
  r.writeU16(0);     // end of build-option list
  break;

    case MSP_UID:
      r.writeU32(getBoardId0());
      r.writeU32(getBoardId1());
      r.writeU32(getBoardId2());
      break;

    case MSP_STATUS_EX:
    case MSP_STATUS:
      // r.writeU16(_model.state.loopTimer.delta);
      r.writeU16(_model.state.stats.loopTime());
      r.writeU16(_model.state.i2cErrorCount); // i2c error count
      //         acc,     baro,    mag,     gps,     sonar,   gyro
      r.writeU16(_model.accelActive() | _model.baroActive() << 1 | _model.magActive() << 2 | _model.gpsActive() << 3 |
                 0 << 4 | _model.gyroActive() << 5);
      r.writeU32(MspBoxMapping::statusMask(_model.state.mode.mask)); // MSP_BOXIDS-indexed bits
      r.writeU8(0);                       // pid profile
      r.writeU16(lrintf(_model.state.stats.getCpuLoad()));
      if (m.cmd == MSP_STATUS_EX)
      {
        r.writeU8(1); // max profile count
        r.writeU8(0); // current rate profile index
      }
      else // MSP_STATUS
      {
        r.writeU16(0); // unused - gyro cycle time
      }
      // flight mode flags (above 32 bits)
      r.writeU8(0); // count + rest of flags
      // Write arming disable flags
      r.writeU8(26); // Betaflight 4.5 / API 1.46 arming flag count
      {
        const uint32_t internal = _model.state.mode.armingDisabledFlags;
        uint32_t wire = internal & ((uint32_t{1} << 25) - 1u);
        // Newer private flags have no API 1.46 names. Expose their inhibit
        // through REBOOT_REQUIRED and remap ARM_SWITCH from internal bit 29.
        if (internal & (ARMING_DISABLED_CRASHFLIP | ARMING_DISABLED_ALTHOLD |
                        ARMING_DISABLED_POSHOLD | ARMING_DISABLED_AUTOPILOT))
          wire |= ARMING_DISABLED_REBOOT_REQUIRED;
        if (internal & ARMING_DISABLED_ARM_SWITCH) wire |= uint32_t{1} << 25;
        r.writeU32(wire);
      }
      r.writeU8(_model.getRebootRequired());             // reboot required
      // 1.46
      r.writeU16(0); // getCoreTemperatureCelsius() // cpu temperature
      break;

    case MSP_NAME:
      r.writeString(config.modelName);
      break;

    case MSP_SET_NAME:
      std::fill_n(&config.modelName[0], MODEL_NAME_LEN + 1, 0);
      for (size_t i = 0; i < std::min((size_t)m.received, MODEL_NAME_LEN); i++)
      {
        config.modelName[i] = m.readU8();
      }
      break;

    case MSP2_GET_TEXT: {
      const uint8_t textType = m.remain() ? m.readU8() : 0;
      r.writeU8(textType);
      switch (textType)
      {
        case MSP2TEXT_CRAFT_NAME:
          r.writePString(config.modelName);
          break;
        default:
          r.writePString(""); // unsupported text types reported as empty
          break;
      }
      break;
    }

    case MSP2_SET_TEXT: {
      const uint8_t textType = m.readU8();
      const uint8_t textLength = m.readU8();
      if (m.decodeError || m.remain() != textLength || textType != MSP2TEXT_CRAFT_NAME || textLength > MODEL_NAME_LEN)
      { r.result = -1; break; }
      if (textType == MSP2TEXT_CRAFT_NAME)
      {
        std::fill_n(&config.modelName[0], MODEL_NAME_LEN + 1, 0);
        for (size_t i = 0; i < textLength; i++)
        {
          const uint8_t c = m.readU8();
          if (i < MODEL_NAME_LEN) config.modelName[i] = c;
        }
      }
      else
      {
        m.advance(textLength); // ignore unsupported text types
      }
      break;
    }

    case MSP_BOXNAMES:
      for (uint8_t mode = 0; mode < MODE_COUNT; ++mode)
      {
        if (!Control::modeSupported(mode)) continue;
        static const char* names[] = {"ARM", "AIR MODE", "ANGLE", "ALTHOLD", "BEEPER",
            "FAILSAFE", "BLACKBOX", "BLACKBOX ERASE", "ANTI GRAVITY", "POS HOLD", "GPS RESCUE"};
        r.writeString(names[mode]);
        r.writeU8(';');
      }
      break;

    case MSP_BOXIDS:
      for (uint8_t mode = 0; mode < MODE_COUNT; ++mode)
        if (Control::modeSupported(mode)) r.writeU8(MspBoxMapping::encode(mode));
      break;

    case MSP_MODE_RANGES:
      for (size_t i = 0; i < ACTUATOR_CONDITIONS; i++)
      {
        r.writeU8(MspBoxMapping::encode(config.conditions[i].id));
        r.writeU8(config.conditions[i].ch - AXIS_AUX_1);
        r.writeU8((config.conditions[i].min - 900) / 25);
        r.writeU8((config.conditions[i].max - 900) / 25);
      }
      break;

    case MSP_MODE_RANGES_EXTRA:
      r.writeU8(3); // bytes per entry: permanentId, logicMode, linkedPermanentId
      for (size_t i = 0; i < ACTUATOR_CONDITIONS; i++)
      {
        r.writeU8(MspBoxMapping::encode(config.conditions[i].id));
        r.writeU8(config.conditions[i].logicMode);
        r.writeU8(config.conditions[i].linkId == 0 ? 0 : MspBoxMapping::encode(config.conditions[i].linkId));
      }
      break;

    case MSP_SET_MODE_RANGE: {
      const size_t i = m.readU8();
      if (i >= ACTUATOR_CONDITIONS)
      {
        r.result = -1;
        break;
      }
      // Retain optional link and logic fields for legacy 5-byte updates.
      ActuatorCondition candidate = config.conditions[i];
      const int internalMode = MspBoxMapping::decode(m.readU8());
      const int channel = static_cast<int>(m.readU8()) + AXIS_AUX_1;
      candidate.min = static_cast<int16_t>(m.readU8() * 25 + 900);
      candidate.max = static_cast<int16_t>(m.readU8() * 25 + 900);
      int internalLink = 0;
      if (m.remain() == 2)
      {
        candidate.logicMode = m.readU8();
        const uint8_t linkedPermanentId = m.readU8();
        internalLink = linkedPermanentId == 0
            ? 0 : MspBoxMapping::decode(linkedPermanentId);
      }
      if (internalMode < 0 || internalLink < 0 ||
          channel < AXIS_AUX_1 || channel >= AXIS_COUNT ||
          candidate.min > candidate.max ||
          candidate.max > 2100 ||
          candidate.logicMode > 1)
      {
        r.result = -1;
        break;
      }
      candidate.id = static_cast<uint8_t>(internalMode);
      if (m.received == 7)
        candidate.linkId = static_cast<uint8_t>(internalLink);
      candidate.ch = static_cast<uint8_t>(channel);
      config.conditions[i] = candidate;
    }
    break;

    case MSP_ANALOG:
      r.writeU8(toVbatVoltageLegacy(_model.state.battery.voltage)); // voltage in 0.1V
      r.writeU16(static_cast<uint16_t>(
          std::min<uint32_t>(_model.state.battery.reportedConsumedMah(), 65535u))); // mAh drawn
      r.writeU16(_model.getRssi());                                 // rssi
      r.writeU16(toIbatCurrent(_model.state.battery.current));      // amperage in 0.01A
      r.writeU16(toVbatVoltage(_model.state.battery.voltage));      // voltage in 0.01V
      break;

    case MSP_FEATURE_CONFIG:
      r.writeU32(config.featureMask);
      break;

    case MSP_SET_FEATURE_CONFIG:
      config.featureMask = m.readU32();
      needsReload = true;
      needsReboot = true;
      break;

    case MSP_BATTERY_CONFIG:
      r.writeU8((config.extension.cellMinimum + 5) / 10); // vbatmincellvoltage
      r.writeU8((config.extension.cellMaximum + 5) / 10); // vbatmaxcellvoltage
      r.writeU8((config.vbat.cellWarning + 5) / 10); // vbatwarningcellvoltage
      r.writeU16(config.extension.capacityMah); // batteryCapacity
      r.writeU8(config.vbat.source);                 // voltageMeterSource
      r.writeU8(config.ibat.source);                 // currentMeterSource
      r.writeU16(config.extension.cellMinimum); // vbatmincellvoltage
      r.writeU16(config.extension.cellMaximum); // vbatmaxcellvoltage
      r.writeU16(config.vbat.cellWarning);           // vbatwarningcellvoltage
      break;

    case MSP_SET_BATTERY_CONFIG:
      // Supported meter-source IDs: none/ADC voltage, none/ADC/MSP current.
      // Do not silently replace an unsupported setting with NONE.
      if (m.buffer[m.read + 5u] > 1u ||
          static_cast<int>(toIbatSource(m.buffer[m.read + 6u])) !=
              m.buffer[m.read + 6u])
      {
        r.result = -1;
        break;
      }
      config.extension.cellMinimum = m.readU8() * 10;
      config.extension.cellMaximum = m.readU8() * 10;
      config.vbat.cellWarning = m.readU8() * 10;     // vbatwarningcellvoltage
      config.extension.capacityMah = m.readU16();
      config.vbat.source = toVbatSource(m.readU8()); // voltageMeterSource
      config.ibat.source = toIbatSource(m.readU8()); // currentMeterSource
      if (m.remain() >= 6)
      {
        config.extension.cellMinimum = m.readU16();
        config.extension.cellMaximum = m.readU16();
        config.vbat.cellWarning = m.readU16();
      }
      break;

    case MSP_BATTERY_STATE:
      // battery characteristics
      r.writeU8(_model.state.battery.cells); // cell count, 0 indicates battery not detected.
      r.writeU16(config.extension.capacityMah); // capacity in mAh

      // battery state
      r.writeU8(toVbatVoltageLegacy(_model.state.battery.voltage)); // in 0.1V steps
      r.writeU16(static_cast<uint16_t>(
          std::min<uint32_t>(_model.state.battery.reportedConsumedMah(), 65535u))); // mAh drawn
      r.writeU16(toIbatCurrent(_model.state.battery.current));      // send current in 0.01 A steps, range is -320A to 320A

      // Battery status uses the configured warning voltage and the
      // detected cell count rather than unconditionally reporting "OK".
      r.writeU8(mspBatteryState(_model));
      r.writeU16(toVbatVoltage(_model.state.battery.voltage)); // in 0.01 steps
      break;

    case MSP_VOLTAGE_METERS:
      for (int i = 0; i < 1; i++)
      {
        r.writeU8(i + 10);                                            // meter id (10-19 vbat adc)
        r.writeU8(toVbatVoltageLegacy(_model.state.battery.voltage)); // meter value
      }
      break;

    case MSP_CURRENT_METERS:
    {
      const bool mspSource =
          config.ibat.source ==
          CURRENT_METER_MSP;

      // Betaflight reserves 10..19 for battery ADC current meters and 90..99
      // for MSP current meters. Report the ID corresponding to the selected
      // source so Configurator identifies an external companion meter.
      r.writeU8(
          mspSource
              ? 90
              : 10);

      r.writeU16(static_cast<uint16_t>(
          std::min<uint32_t>(_model.state.battery.reportedConsumedMah(), 65535u)));

      const long milliAmps =
          lrintf(
              _model.state.battery.current *
              1000.0f);

      r.writeU16(
          static_cast<uint16_t>(
              std::clamp<long>(
                  milliAmps,
                  0L,
                  65535L)));
      break;
    }

    case MSP_VOLTAGE_METER_CONFIG:
      r.writeU8(1); // num voltage sensors
      for (int i = 0; i < 1; i++)
      {
        r.writeU8(5);                          // frame size (5)
        r.writeU8(i + 10);                     // id (10-19 vbat adc)
        r.writeU8(0);                          // type resistor divider
        r.writeU8(config.vbat.scale);   // scale
        r.writeU8(config.vbat.resDiv);  // resdivval
        r.writeU8(config.vbat.resMult); // resdivmultiplier
      }
      break;

    case MSP_SET_VOLTAGE_METER_CONFIG: {
      const uint8_t id = m.readU8();
      const uint8_t scale = m.readU8();
      const uint8_t divisor = m.readU8();
      const uint8_t multiplier = m.readU8();
      if (id != 10u || divisor == 0u || multiplier == 0u)
      {
        r.result = -1;
        break;
      }
      config.vbat.scale = scale;
      config.vbat.resDiv = divisor;
      config.vbat.resMult = multiplier;
    }
    break;

    case MSP_CURRENT_METER_CONFIG:
      r.writeU8(1); // num voltage sensors
      for (int i = 0; i < 1; i++)
      {
        r.writeU8(6);                          // frame size (6)
        r.writeU8(i + 10);                     // id (10-19 ibat adc)
        r.writeU8(1);                          // type adc
        r.writeU16(config.ibat.scale);  // scale
        r.writeU16(config.ibat.offset); // offset
      }
      break;

    case MSP_SET_CURRENT_METER_CONFIG: {
      const uint8_t id = m.readU8();
      const uint16_t scale = m.readU16();
      const uint16_t offset = m.readU16();
      if (id != 10u || scale == 0u || scale > INT16_MAX)
      {
        r.result = -1;
        break;
      }
      config.ibat.scale = static_cast<int16_t>(scale);
      config.ibat.offset = static_cast<int16_t>(offset);
    }
    break;

    case MSP_DATAFLASH_SUMMARY:
#ifdef USE_FLASHFS
    {
      uint8_t flags = flashfsIsSupported() ? 2 : 0;
      flags |= flashfsIsReady() ? 1 : 0;
      r.writeU8(flags);
      r.writeU32(flashfsGetSectors());
      r.writeU32(flashfsGetSize());
      r.writeU32(flashfsGetOffset());
    }
#else
      r.writeU8(0);
      r.writeU32(0);
      r.writeU32(0);
      r.writeU32(0);
#endif
    break;

    case MSP_DATAFLASH_ERASE:
#ifdef USE_FLASHFS
      if (blackboxConfig()->device != BLACKBOX_DEVICE_FLASH) r.result = -1;
      else
      {
        blackboxEraseAll();
        if (!flashfsIsReady()) r.result = -1;
      }
#else
      r.result = -1;
#endif
      break;

    case MSP_DATAFLASH_READ:
#ifdef USE_FLASHFS
    {
      const unsigned int dataSize = m.remain();
      const uint32_t readAddress = m.readU32();
      uint16_t readLength;
      bool allowCompression = false;
      bool useLegacyFormat;

      if (dataSize >= sizeof(uint32_t) + sizeof(uint16_t))
      {
        readLength = m.readU16();
        if (m.remain())
        {
          allowCompression = m.readU8();
        }
        useLegacyFormat = false;
      }
      else
      {
        readLength = 128;
        useLegacyFormat = true;
      }
      serializeFlashData(r, readAddress, readLength, useLegacyFormat, allowCompression);
    }
#endif
    break;

    case MSP_ACC_TRIM:
      r.writeU16(config.accel.trim[0]); // pitch
      r.writeU16(config.accel.trim[1]); // roll
      break;

    case MSP_SET_ACC_TRIM:
      config.accel.trim[0] = std::clamp<int16_t>(m.readU16(), -300, 300); // pitch
      config.accel.trim[1] = std::clamp<int16_t>(m.readU16(), -300, 300); // roll
      events[static_cast<size_t>(ModelChangeEvent::MODEL_CHANGE_ACCEL)] = true;
      break;

    case MSP_MIXER_CONFIG:
      r.writeU8(config.mixer.type);       // mixerMode, QUAD_X
      r.writeU8(config.mixer.yawReverse); // yaw_motors_reversed
      break;

    case MSP_SET_MIXER_CONFIG:
      config.mixer.type = m.readU8();       // mixerMode, QUAD_X
      config.mixer.yawReverse = m.readU8(); // yaw_motors_reversed
      break;

    case MSP_SENSOR_CONFIG:
      r.writeU8(toAccHw(config.accel.dev)); // 3 acc mpu6050
      r.writeU8(config.baro.dev);           // 2 baro bmp085
      r.writeU8(config.mag.dev);            // 3 mag hmc5883l
      // 1.46
      r.writeU8(0); // rangefinder 0=none
      r.writeU8(0); // opticalflow 0=none
      break;

    case MSP_SET_SENSOR_CONFIG:
      config.accel.dev = fromAccHw(m.readU8()); // 3 acc mpu6050
      config.baro.dev = m.readU8();             // 2 baro bmp085
      config.mag.dev = m.readU8();              // 3 mag hmc5883l
      // 1.46
      if (m.remain() >= 1)
      {
        m.readU8(); // rangefinder skip
      }
      if (m.remain() >= 1)
      {
        m.readU8(); // opticalflow skip
      }
      needsReload = true;
      needsReboot = true;
      break;

    case MSP2_SENSOR_CONFIG_ACTIVE: {
      const auto& state = _model.state;
      r.writeU8(_model.gyroActive() && state.gyro.dev ? toAccHw(state.gyro.dev->getType())
                                                      : toAccHw(GYRO_NONE)); // gyro
      r.writeU8(_model.accelActive() && state.gyro.dev ? toAccHw(state.gyro.dev->getType())
                                                       : toAccHw(GYRO_NONE));                   // acc
      r.writeU8(_model.baroActive() && state.baro.dev ? state.baro.dev->getType() : BARO_NONE); // baro
      r.writeU8(_model.magActive() && state.mag.dev ? state.mag.dev->getType() : MAG_NONE);     // mag
      r.writeU8(SENSOR_NOT_AVAILABLE);                                                          // rangefinder
      r.writeU8(SENSOR_NOT_AVAILABLE);                                                          // opticalflow
      break;
    }

    case MSP2_GYRO_SENSOR_ACTIVE:
      r.writeU8(1);                                                                  // gyro count, single gyro only
      r.writeU8(_model.gyroActive() ? _model.state.gyro.dev->getType() : GYRO_NONE); // gyro 1
      break;

    case MSP_SENSOR_ALIGNMENT:
      r.writeU8(config.gyro.align); // gyro align
      r.writeU8(config.gyro.align); // acc align, Starting with 4.0 gyro and acc alignment are the same
      r.writeU8(config.mag.align);  // mag align
      // 1.41+
      r.writeU8(_model.state.gyro.present ? 1 : 0); // gyro detection mask GYRO_1_MASK
      r.writeU8(_model.state.gyro.present ? 1 : 0); // gyro_enable_mask, was gyro_to_use
      r.writeU16(0);                                // mag align roll
      r.writeU16(0);                                // mag align pitch
      r.writeU16(0);                                // mag align yaw
      break;

    case MSP_SET_SENSOR_ALIGNMENT: {
      uint8_t gyroAlign = m.readU8();       // gyro align
      m.readU8();                           // discard deprecated acc align
      config.mag.align = m.readU8(); // mag align
      // API >= 1.41 - support the gyro_to_use and alignment for gyros 1 & 2
      if (m.remain() >= 1)
      {
        uint8_t gyroEnableMask = m.readU8(); // gyro_enable_mask
        config.gyro.dev = gyroEnableMask & 1 ? GYRO_AUTO : GYRO_NONE;
      }
      if (m.remain() >= 6)
      {
        m.readU16(); // gyro 1 roll
        m.readU16(); // gyro 1 pitch
        m.readU16(); // gyro 1 yaw
      }
      config.gyro.align = gyroAlign;
      break;
    }

    case MSP_CF_SERIAL_CONFIG:
      for (int i = 0; i < SERIAL_UART_COUNT; i++)
      {
        if (config.serial[i].id >= SERIAL_ID_SOFTSERIAL_1 && !_model.isFeatureActive(FEATURE_SOFTSERIAL))
        {
          continue;
        }
        r.writeU8(config.serial[i].id);                        // identifier
        r.writeU16(config.serial[i].functionMask);             // functionMask
        r.writeU8(toBaudIndex(config.serial[i].baud));         // msp_baudrateIndex
        r.writeU8(0);                                                 // gps_baudrateIndex
        r.writeU8(0);                                                 // telemetry_baudrateIndex
        r.writeU8(toBaudIndex(config.serial[i].blackboxBaud)); // blackbox_baudrateIndex
      }
      break;

    case MSP2_COMMON_SERIAL_CONFIG: {
      uint8_t count = 0;
      for (int i = 0; i < SERIAL_UART_COUNT; i++)
      {
        if (config.serial[i].id >= SERIAL_ID_SOFTSERIAL_1 && !_model.isFeatureActive(FEATURE_SOFTSERIAL))
        {
          continue;
        }
        count++;
      }
      r.writeU8(count);
      for (int i = 0; i < SERIAL_UART_COUNT; i++)
      {
        if (config.serial[i].id >= SERIAL_ID_SOFTSERIAL_1 && !_model.isFeatureActive(FEATURE_SOFTSERIAL))
        {
          continue;
        }
        r.writeU8(config.serial[i].id);                        // identifier
        r.writeU32(config.serial[i].functionMask);             // functionMask
        r.writeU8(toBaudIndex(config.serial[i].baud));         // msp_baudrateIndex
        r.writeU8(0);                                                 // gps_baudrateIndex
        r.writeU8(0);                                                 // telemetry_baudrateIndex
        r.writeU8(toBaudIndex(config.serial[i].blackboxBaud)); // blackbox_baudrateIndex
      }
      break;
    }

    case MSP_SET_CF_SERIAL_CONFIG: {
      const int packetSize = 1 + 2 + 4;
      if (m.remain() % packetSize != 0)
      {
        r.result = -1;
        return;
      }
      while (m.remain() >= packetSize)
      {
        int id = m.readU8();
        int k = _model.getSerialIndex((SerialPortId)id);
        if (k == -1)
        {
          r.result = -1;
          break;
        }
        config.serial[k].id = id;
        config.serial[k].functionMask = m.readU16();
        config.serial[k].baud = fromBaudIndex((SerialSpeedIndex)m.readU8());
        m.readU8();
        m.readU8();
        config.serial[k].blackboxBaud = fromBaudIndex((SerialSpeedIndex)m.readU8());
      }
      needsReload = true;
      needsReboot = true;
      break;
    }

    case MSP2_COMMON_SET_SERIAL_CONFIG: {
      const unsigned count = m.readU8();
      const int packetSize = 1 + 4 + 4;
      if (m.decodeError || m.remain() != static_cast<int>(count) * packetSize)
      { r.result = -1; break; }
      if (m.remain() % packetSize != 0)
      {
        r.result = -1;
        return;
      }
      while (m.remain() >= packetSize)
      {
        int id = m.readU8();
        int k = _model.getSerialIndex((SerialPortId)id);
        if (k == -1)
        {
          r.result = -1;
          break;
        }
        config.serial[k].id = id;
        config.serial[k].functionMask = m.readU32();
        config.serial[k].baud = fromBaudIndex((SerialSpeedIndex)m.readU8());
        m.readU8();
        m.readU8();
        config.serial[k].blackboxBaud = fromBaudIndex((SerialSpeedIndex)m.readU8());
      }
      needsReload = true;
      needsReboot = true;
      break;
    }

    case MSP_BLACKBOX_CONFIG:
      r.writeU8(1);                          // Blackbox supported
      r.writeU8(config.blackbox.dev); // device serial or none
      r.writeU8(1);                          // blackboxGetRateNum()); // unused
      r.writeU8(1);                          // blackboxGetRateDenom());
      r.writeU16(0);                         // blackboxGetPRatio()); // p_denom
      // 1.44
      r.writeU8(config.blackbox.pDenom); // sample_rate
      // 1.45
      r.writeU32(~config.blackbox.fieldsMask); // fields mask
      break;

    case MSP_SET_BLACKBOX_CONFIG:
      if (!blackboxMayEditConfig()) { r.result = -1; break; }
      if (m.remain() < 6)
      {
        // Legacy rate conversion divides by both denominator and pRatio.
        // Logging may be disabled, leaving targetPidLooptime uninitialized.
        const uint16_t rateNum = m.buffer[m.read + 1];
        const uint16_t rateDenom = m.buffer[m.read + 2];
        if (!targetPidLooptime || (m.remain() == 3 && (!rateNum || !rateDenom)))
        { r.result = -1; break; }
        const uint32_t ratio = m.remain() == 5 ? mspPayloadU16(m, 3) :
            (32000u / targetPidLooptime) * rateNum / rateDenom;
        if (!ratio || uint64_t(targetPidLooptime) * ratio > 32000u)
        { r.result = -1; break; }
      }
      // Don't allow config to be updated while Blackbox is logging
      if (blackboxMayEditConfig())
      {
        config.blackbox.dev = m.readU8();
        const int rateNum = m.readU8();   // was rate_num
        const int rateDenom = m.readU8(); // was rate_denom
        uint16_t pRatio = 0;
        if (m.remain() >= 2)
        {
          // p_denom specified, so use it directly
          pRatio = m.readU16();
        }
        else
        {
          // p_denom not specified in MSP, so calculate it from old rateNum and rateDenom
          pRatio = blackboxCalculatePDenom(rateNum, rateDenom);
        }

        // 1.44
        if (m.remain() >= 1)
        {
          // sample_rate specified, so use it directly
          config.blackbox.pDenom = m.readU8();
        }
        else
        {
          // sample_rate not specified in MSP, so calculate it from old p_ratio
          config.blackbox.pDenom = blackboxCalculateSampleRate(pRatio);
        }

        // 1.45
        if (m.remain() >= 4)
        {
          config.blackbox.fieldsMask = ~m.readU32();
        }
      }
      break;

    case MSP_ATTITUDE:
      r.writeU16(lrintf(Utils::toDeg(_model.state.attitude.euler.x) * 10.f)); // roll  [decidegrees]
      r.writeU16(lrintf(Utils::toDeg(_model.state.attitude.euler.y) * 10.f)); // pitch [decidegrees]
      r.writeU16(lrintf(Utils::toDeg(-_model.state.attitude.euler.z)));       // yaw   [degrees]
      break;

    case MSP_ATTITUDE_QUATERNION: {
      const float scale = 32767.0f; // int16 unit-quaternion scaling
      const auto& q = _model.state.attitude.quaternion;
      r.writeU16((int16_t)lrintf(q.w * scale));
      r.writeU16((int16_t)lrintf(q.x * scale));
      r.writeU16((int16_t)lrintf(q.y * scale));
      r.writeU16((int16_t)lrintf(q.z * scale));
      break;
    }

    case MSP_ALTITUDE:
      r.writeU32(lrintf(_model.state.altitude.height * 100.f)); // alt [cm]
      r.writeU16(lrintf(_model.state.altitude.vario * 100.f));  // vario [cm/s]
      break;

    case MSP_BEEPER_CONFIG:
      r.writeU32(~config.buzzer.beeperMask); // beeper mask
      r.writeU8(0);                                 // dshot beacon tone
      r.writeU32(0);                                // dshot beacon off flags
      break;

    case MSP_SET_BEEPER_CONFIG:
      config.buzzer.beeperMask = ~m.readU32();
      if (m.remain() == 5 && (m.readU8() != 0 || m.readU32() != 0)) r.result = -1;
      break;

    case MSP_BOARD_ALIGNMENT_CONFIG:
      r.writeU16(config.boardAlignment[0]); // roll
      r.writeU16(config.boardAlignment[1]); // pitch
      r.writeU16(config.boardAlignment[2]); // yaw
      break;

    case MSP_SET_BOARD_ALIGNMENT_CONFIG:
      config.boardAlignment[0] = m.readU16();
      config.boardAlignment[1] = m.readU16();
      config.boardAlignment[2] = m.readU16();
      events[static_cast<size_t>(ModelChangeEvent::MODEL_CHANGE_ACCEL)] = true;
      break;

    case MSP_RX_MAP:
      for (size_t i = 0; i < 8; i++)
      {
        r.writeU8(config.input.channel[i].map);
      }
      break;

    case MSP_SET_RX_MAP:
      for (size_t i = 0; i < 8; i++)
      {
        config.input.channel[i].map = m.readU8();
      }
      break;

    case MSP_RSSI_CONFIG:
      r.writeU8(config.input.rssiChannel);
      break;

    case MSP_SET_RSSI_CONFIG:
      config.input.rssiChannel = m.readU8();
      break;

    case MSP_MOTOR_CONFIG:
      r.writeU16(0);                                // minthrottle (dropped in 1.46)
      r.writeU16(config.output.maxThrottle); // maxthrottle
      r.writeU16(config.output.minCommand);  // mincommand
      r.writeU8(_model.state.currentMixer.count);   // motor count
      // 1.42+
      r.writeU8(config.output.motorPoles);     // motor pole count
      r.writeU8(config.output.dshotTelemetry); // dshot telemtery
      r.writeU8(0);                                   // esc sensor
      break;

    case MSP_SET_MOTOR_CONFIG:
      if (m.remain() < 6) { r.result = -1; break; }
      m.readU16();                                    // minthrottle (dropped in 1.46)
      config.output.maxThrottle = m.readU16(); // maxthrottle
      config.output.minCommand = m.readU16();  // mincommand
      if (m.remain() >= 2)
      {
#ifdef ESPFC_DSHOT_TELEMETRY
        config.output.motorPoles = m.readU8();
        config.output.dshotTelemetry = m.readU8();
#else
        m.readU8();
        m.readU8();
#endif
      }
      if (m.remain() == 1 && m.readU8() != 0) { r.result = -1; break; }
      needsReload = true;
      needsReboot = true;
      break;

    case MSP_MOTOR_3D_CONFIG:
      r.writeU16(1406); // deadband3d_low;
      r.writeU16(1514); // deadband3d_high;
      r.writeU16(1460); // neutral3d;
      break;

    case MSP_ARMING_CONFIG:
      r.writeU8(5);                               // auto_disarm delay
      r.writeU8(0);                               // disarm kill switch
      r.writeU8(config.arming.smallAngle); // small angle
      r.writeU8(0);                               // gyro_cal_on_first_arm
      break;

    case MSP_SET_ARMING_CONFIG:
      m.readU8();                                                           // auto_disarm delay
      m.readU8();                                                           // disarm kill switch
      config.arming.smallAngle = m.readU8(); // checked before commit
      m.readU8();                                                           // gyro_cal_on_first_arm
      break;

    case MSP_RC_DEADBAND:
      r.writeU8(config.input.deadband);
      r.writeU8(0);  // yaw deadband
      r.writeU8(0);  // pos hold deadband
      r.writeU16(0); // deadband 3d throttle
      break;

    case MSP_SET_RC_DEADBAND:
      config.input.deadband = m.readU8();
      m.readU8();  // yaw deadband
      m.readU8();  // pos hod deadband
      m.readU16(); // deadband 3d throttle
      break;

    case MSP_RX_CONFIG:
      r.writeU8(config.input.serialRxProvider);                      // serialrx_provider
      r.writeU16(config.input.maxCheck);                             // maxcheck
      r.writeU16(config.input.midRc);                                // midrc
      r.writeU16(config.input.minCheck);                             // mincheck
      r.writeU8(0);                                                         // spectrum bind
      r.writeU16(config.input.minRc);                                // min_us
      r.writeU16(config.input.maxRc);                                // max_us
      r.writeU8(0);                                                         // rc interpolation
      r.writeU8(0);                                                         // rc interpolation interval
      r.writeU16(config.input.airModeActivateThreshold * 10 + 1000); // airmode activate threshold
      r.writeU8(0);                                                         // rx spi prot
      r.writeU32(0);                                                        // rx spi id
      r.writeU8(0);                                                         // rx spi chan count
      r.writeU8(0);                                                         // fpv camera angle
      r.writeU8(2);                                                         // rc iterpolation channels: RPYT
      r.writeU8(0);                                                         // deprecated: rc_smoothing_type
      r.writeU8(config.input.filter.freq);                           // rc_smoothing_setpoint_cutoff
      r.writeU8(config.input.filterThrottle.freq);                   // rc_smoothing_throtle_cutoff
      r.writeU8(config.input.filterAutoThrottleFactor);              // rc_smoothing_auto_factor_throttle
      r.writeU8(0);                                                         // rc_smoothing_derivative_type
      r.writeU8(0);                                                         // usb type
      // 1.42+
      r.writeU8(config.input.filterAutoFactor); // rc_smoothing_auto_factor
      // 1.44
      r.writeU8(config.input.filterEnable); // rc_smoothing
      // 1.45
      {
        uint8_t uid[6] = {};
        r.writeData((const char*)uid, sizeof(uid)); // elrs uid
      }
      break;

    case MSP_SET_RX_CONFIG:
      if (m.remain() < 12) { r.result = -1; break; }
      config.input.serialRxProvider = m.readU8(); // serialrx_provider
      config.input.maxCheck = m.readU16();        // maxcheck
      config.input.midRc = m.readU16();           // midrc
      config.input.minCheck = m.readU16();        // mincheck
      m.readU8();                                        // spectrum bind
      config.input.minRc = m.readU16();           // min_us
      config.input.maxRc = m.readU16();           // max_us
      if (m.remain() >= 4)
      {
        m.readU8();                                                               // rc interpolation
        m.readU8();                                                               // rc interpolation interval
        config.input.airModeActivateThreshold = (m.readU16() - 1000) / 10; // airmode activate threshold
      }
      if (m.remain() >= 6)
      {
        m.readU8();  // rx spi prot
        m.readU32(); // rx spi id
        m.readU8();  // rx spi chan count
      }
      if (m.remain() >= 1)
      {
        m.readU8(); // fpv camera angle
      }
      // 1.40+
      if (m.remain() >= 6)
      {
        m.readU8();                                                // was rc iterpolation channels
        m.readU8();                                                // was rc_smoothing_type
        config.input.filter.freq = m.readU8();              // rc_smoothing_setpoint_cutoff
        config.input.filterThrottle.freq = m.readU8();      // rc_smoothing_throttle_cutoff
        config.input.filterAutoThrottleFactor = m.readU8(); // rc_smoothing_auto_factor_throttle
        m.readU8();                                                // was rc_smoothing_derivative_type
      }
      if (m.remain() >= 1)
      {
        m.readU8(); // usb type
      }
      // 1.42+
      if (m.remain() >= 1)
      {
        config.input.filterAutoFactor = m.readU8(); // rc_smoothing_auto_factor
      }
      // 1.44
      if (m.remain() >= 1)
      {
        config.input.filterEnable = m.readU8(); // rc_smoothing
      }
      // 1.45
      if (m.remain() >= 6)
      {
        m.advance(6); // elrs uid
      }

      events[static_cast<size_t>(MODEL_CHANGE_INPUT)] = true;
      break;

  case MSP_FAILSAFE_CONFIG:
  r.writeU8(
      config.failsafe.delay);

  r.writeU8(
      config.failsafe.landingTime);

  r.writeU16(
      config.failsafe.landingThrottle);

  r.writeU8(
      config.failsafe.killSwitch);

  r.writeU16(
      config.failsafe.throttleLowDelay);

  r.writeU8(
      config.failsafe.procedure);

  break;

case MSP_SET_FAILSAFE_CONFIG:
{
    if (m.remain() < 8)
    {
        r.result = -1;
        break;
    }

    config.failsafe.delay =
        m.readU8();

    config.failsafe.landingTime =
        m.readU8();

    config.failsafe.landingThrottle =
        m.readU16();

    config.failsafe.killSwitch =
        m.readU8();

    config.failsafe.throttleLowDelay =
        m.readU16();

    const uint8_t procedure =
        m.readU8();

    config.failsafe.procedure = procedure; // invalid procedures must be rejected

    break;
}

    case MSP_RXFAIL_CONFIG:
      for (size_t i = 0; i < _model.state.input.channelCount; i++)
      {
        r.writeU8(config.input.channel[i].fsMode);
        r.writeU16(config.input.channel[i].fsValue);
      }
      break;

    case MSP_SET_RXFAIL_CONFIG: {
      size_t i = m.readU8();
      if (i < INPUT_CHANNELS)
      {
        config.input.channel[i].fsMode = m.readU8();   // mode
        config.input.channel[i].fsValue = m.readU16(); // pulse
      }
      else
      {
        r.result = -1;
      }
    }
    break;

    case MSP_RC:
      for (size_t i = 0; i < _model.state.input.channelCount; i++)
      {
        r.writeU16(lrintf(_model.state.input.us[i]));
      }
      break;

    case MSP_RC_TUNING:
      r.writeU8(config.input.rate[AXIS_ROLL]);
      r.writeU8(config.input.expo[AXIS_ROLL]);
      for (size_t i = 0; i < AXIS_COUNT_RPY; i++)
      {
        r.writeU8(config.input.superRate[i]);
      }
      r.writeU8(0);                                    // was tpa scale
      r.writeU8(50);                                   // thrMid8
      r.writeU8(0);                                    // thr expo
      r.writeU16(0);                                   // was tpa breakpoint
      r.writeU8(config.input.expo[AXIS_YAW]);   // yaw expo
      r.writeU8(config.input.rate[AXIS_YAW]);   // yaw rate
      r.writeU8(config.input.rate[AXIS_PITCH]); // pitch rate
      r.writeU8(config.input.expo[AXIS_PITCH]); // pitch expo
      // 1.41+
      r.writeU8(config.output.throttleLimitType);    // throttle_limit_type (off)
      r.writeU8(config.output.throttleLimitPercent); // throtle_limit_percent (100%)
      // 1.42+
      r.writeU16(config.input.rateLimit[0]); // rate limit roll
      r.writeU16(config.input.rateLimit[1]); // rate limit pitch
      r.writeU16(config.input.rateLimit[2]); // rate limit yaw
      // 1.43+
      r.writeU8(config.input.rateType); // rates type
      break;

    case MSP_SET_RC_TUNING:
      if (m.remain() >= 10)
      {
        const uint8_t rate = m.readU8();
        if (config.input.rate[AXIS_PITCH] == config.input.rate[AXIS_ROLL])
        {
          config.input.rate[AXIS_PITCH] = rate;
        }
        config.input.rate[AXIS_ROLL] = rate;

        const uint8_t expo = m.readU8();
        if (config.input.expo[AXIS_PITCH] == config.input.expo[AXIS_ROLL])
        {
          config.input.expo[AXIS_PITCH] = expo;
        }
        config.input.expo[AXIS_ROLL] = expo;

        for (size_t i = 0; i < AXIS_COUNT_RPY; i++)
        {
          config.input.superRate[i] = m.readU8();
        }
        m.readU8();  // was tpa scale
        m.readU8();  // thrMid8
        m.readU8();  // thr expo
        m.readU16(); // was tpa breakpoint
        if (m.remain() >= 1)
        {
          config.input.expo[AXIS_YAW] = m.readU8(); // yaw expo
        }
        if (m.remain() >= 1)
        {
          config.input.rate[AXIS_YAW] = m.readU8(); // yaw rate
        }
        if (m.remain() >= 1)
        {
          config.input.rate[AXIS_PITCH] = m.readU8(); // pitch rate
        }
        if (m.remain() >= 1)
        {
          config.input.expo[AXIS_PITCH] = m.readU8(); // pitch expo
        }
        // 1.41
        if (m.remain() >= 2)
        {
          config.output.throttleLimitType = m.readU8();    // throttle_limit_type
          config.output.throttleLimitPercent = m.readU8(); // throttle_limit_percent
        }
        // 1.42
        if (m.remain() >= 6)
        {
          config.input.rateLimit[0] = m.readU16(); // roll
          config.input.rateLimit[1] = m.readU16(); // pitch
          config.input.rateLimit[2] = m.readU16(); // yaw
        }
        // 1.43
        if (m.remain() >= 1)
        {
          config.input.rateType = m.readU8();
        }

        events[static_cast<size_t>(MODEL_CHANGE_RATES)] = true;
      }
      else
      {
        r.result = -1;
        // error
      }
      break;

    case MSP_ADVANCED_CONFIG:
      r.writeU8(1); // gyroSync unused
      r.writeU8(config.loopSync);
      r.writeU8(config.output.async);
      r.writeU8(config.output.protocol);
      r.writeU16(config.output.rate);
      r.writeU16(config.output.motorIdle);
      r.writeU8(0);    // 32k gyro
      r.writeU8(0);    // PWM inversion
      r.writeU8(0);    // gyro_to_use: {1:0, 2:1. 2:both}
      r.writeU8(0);    // gyro high fsr (flase)
      r.writeU8(48);   // gyro cal threshold
      r.writeU16(125); // gyro cal duration (1.25s)
      r.writeU16(0);   // gyro offset yaw
      r.writeU8(0);    // check overflow
      r.writeU8(config.debug.mode);
      r.writeU8(DEBUG_COUNT);
      break;

    case MSP_SET_ADVANCED_CONFIG:
      m.readU8(); // ignore gyroSync, removed in 1.43
      config.loopSync = m.readU8();
      config.output.async = m.readU8();
      config.output.protocol = m.readU8();
      config.output.rate = m.readU16();
      if (m.remain() >= 2)
      {
        const uint16_t motorIdle = m.readU16(); // 0.1us command units
        if (motorIdle > 2000u)
        {
          r.result = -1;
          break;
        }
        config.output.motorIdle = static_cast<int16_t>(motorIdle);
      }
      if (m.remain())
      {
        m.readU8(); // 32k gyro
      }
      if (m.remain())
      {
        m.readU8(); // PWM inversion
      }
      if (m.remain() >= 8)
      {
        m.readU8();  // gyro_to_use
        m.readU8();  // gyro high fsr
        m.readU8();  // gyro cal threshold
        m.readU16(); // gyro cal duration
        m.readU16(); // gyro offset yaw
        m.readU8();  // check overflow
      }
      if (m.remain())
      {
        config.debug.mode = m.readU8();
      }
      needsReload = true;
      needsReboot = true;
      break;

    case MSP_COMPASS_CONFIG:
      r.writeU16(config.mag.declination); // mag_declination * 10
      break;

    case MSP_SET_COMPASS_CONFIG: {
      // Forbid live navigation-frame rotation while armed and reject
      // truncated MSP setters instead of interpreting missing bytes as 0.
      if (_model.isModeActive(MODE_ARMED) || m.remain() != 2)
      {
        r.result = -1;
        break;
      }
      const uint16_t encoded = m.readU16();
      const int32_t signedTenths =
          encoded <= 0x7fff
              ? static_cast<int32_t>(encoded)
              : static_cast<int32_t>(encoded) - 65536;
      if (signedTenths < -1800 || signedTenths > 1800)
      {
        r.result = -1;
        break;
      }
      config.mag.declination =
          static_cast<int16_t>(signedTenths);
      break;
    }

    case MSP_FILTER_CONFIG:
      r.writeU8(config.gyro.filter.freq);          // gyro lpf
      r.writeU16(config.dterm.filter.freq);        // dterm lpf
      r.writeU16(config.yaw.filter.freq);          // yaw lpf
      r.writeU16(config.gyro.notch1Filter.freq);   // gyro notch 1 hz
      r.writeU16(config.gyro.notch1Filter.cutoff); // gyro notch 1 cutoff
      r.writeU16(config.dterm.notchFilter.freq);   // dterm notch hz
      r.writeU16(config.dterm.notchFilter.cutoff); // dterm notch cutoff
      r.writeU16(config.gyro.notch2Filter.freq);   // gyro notch 2 hz
      r.writeU16(config.gyro.notch2Filter.cutoff); // gyro notch 2 cutoff
      r.writeU8(config.dterm.filter.type);         // dterm type
      r.writeU8(fromGyroDlpf(config.gyro.dlpf));
      r.writeU8(0);                                 // dlfp 32khz type
      r.writeU16(config.gyro.filter.freq);   // lowpass1 freq
      r.writeU16(config.gyro.filter2.freq);  // lowpass2 freq
      r.writeU8(config.gyro.filter.type);    // lowpass1 type
      r.writeU8(config.gyro.filter2.type);   // lowpass2 type
      r.writeU16(config.dterm.filter2.freq); // dterm lopwass2 freq
      // 1.41+
      r.writeU8(config.dterm.filter2.type);         // dterm lopwass2 type
      r.writeU16(config.gyro.dynLpfFilter.cutoff);  // dyn lpf gyro min
      r.writeU16(config.gyro.dynLpfFilter.freq);    // dyn lpf gyro max
      r.writeU16(config.dterm.dynLpfFilter.cutoff); // dyn lpf dterm min
      r.writeU16(config.dterm.dynLpfFilter.freq);   // dyn lpf dterm max
      // gyro analyse
      r.writeU8(0);                                          // deprecated dyn notch range
      r.writeU8(0);                                          // deprecated dyn_notch_width_percent
      r.writeU16(config.gyro.dynamicFilter.q);        // dyn_notch_q
      r.writeU16(config.gyro.dynamicFilter.min_freq); // dyn_notch_min_hz
      // rpm filter
      r.writeU8(config.gyro.rpmFilter.harmonics); // gyro_rpm_notch_harmonics
      r.writeU8(config.gyro.rpmFilter.minFreq);   // gyro_rpm_notch_min
      // 1.43+
      r.writeU16(config.gyro.dynamicFilter.max_freq); // dyn_notch_max_hz
      // 1.44
      r.writeU8(0); // dterm lpf1 dyn expo
      r.writeU8(config.gyro.dynamicFilter.count);
      break;

    case MSP_SET_FILTER_CONFIG:
      if (m.remain() < 5) { r.result = -1; break; }
      config.gyro.filter.freq = m.readU8();
      config.dterm.filter.freq = m.readU16();
      config.yaw.filter.freq = m.readU16();
      if (m.remain() >= 8)
      {
        config.gyro.notch1Filter.freq = m.readU16();
        config.gyro.notch1Filter.cutoff = m.readU16();
        config.dterm.notchFilter.freq = m.readU16();
        config.dterm.notchFilter.cutoff = m.readU16();
      }
      if (m.remain() >= 4)
      {
        config.gyro.notch2Filter.freq = m.readU16();
        config.gyro.notch2Filter.cutoff = m.readU16();
      }
      if (m.remain() >= 1)
      {
        config.dterm.filter.type = (FilterType)m.readU8();
      }
      if (m.remain() >= 10)
      {
        m.readU8(); // dlfp type
        m.readU8(); // 32k dlfp type
        config.gyro.filter.freq = m.readU16();
        config.gyro.filter2.freq = m.readU16();
        config.gyro.filter.type = m.readU8();
        config.gyro.filter2.type = m.readU8();
        config.dterm.filter2.freq = m.readU16();
      }
      // 1.41+
      if (m.remain() >= 9)
      {
        config.dterm.filter2.type = m.readU8();
        config.gyro.dynLpfFilter.cutoff = m.readU16();  // dyn gyro lpf min
        config.gyro.dynLpfFilter.freq = m.readU16();    // dyn gyro lpf max
        config.dterm.dynLpfFilter.cutoff = m.readU16(); // dyn dterm lpf min
        config.dterm.dynLpfFilter.freq = m.readU16();   // dyn dterm lpf min
      }
      if (m.remain() >= 8)
      {
        m.readU8();                                              // deprecated dyn_notch_range
        config.gyro.dynamicFilter.count = m.readU8();     // dyn_notch_width_percent
        config.gyro.dynamicFilter.q = m.readU16();        // dyn_notch_q
        config.gyro.dynamicFilter.min_freq = m.readU16(); // dyn_notch_min_hz
        config.gyro.rpmFilter.harmonics = m.readU8();     // gyro_rpm_notch_harmonics
        config.gyro.rpmFilter.minFreq = m.readU8();       // gyro_rpm_notch_min
      }
      // 1.43+
      if (m.remain() >= 2)
      {
        config.gyro.dynamicFilter.max_freq = m.readU16(); // dyn_notch_max_hz
      }
      // 1.44
      if (m.remain() >= 2)
      {
        m.readU8(); // dterm lpf1 dyn expo
        config.gyro.dynamicFilter.count = m.readU8();
      }

      events[static_cast<size_t>(MODEL_CHANGE_FILTER)] = true;
      break;

    case MSP_PID_CONTROLLER:
      r.writeU8(1); // betaflight controller id
      break;

    case MSP_SET_PID_CONTROLLER:
      break;

    case MSP_PIDNAMES:
      r.writeString("ROLL;PITCH;YAW;LEVEL;MAG;ALT;VEL;Pos;PosR;NavR;");
      break;

    case MSP_PID:
      for (size_t i = 0; i < PID_ITEM_COUNT; i++)
      {
        r.writeU8(config.pid[i].P);
        r.writeU8(config.pid[i].I);
        r.writeU8(config.pid[i].D);
      }
      break;

     case MSP_SET_PID:
    {
constexpr int PID_BYTES_PER_ITEM =
    3;

constexpr int REQUIRED_PID_BYTES =
    PID_ITEM_COUNT *
    PID_BYTES_PER_ITEM;

           if (m.remain() <
          REQUIRED_PID_BYTES)
      {
        r.result = -1;
        break;
      }

      for (int i = 0;
           i < PID_ITEM_COUNT;
           i++)
      {
        config.pid[i].P =
            m.readU8();

        config.pid[i].I =
            m.readU8();

        config.pid[i].D =
            m.readU8();
      }

      events[static_cast<size_t>(MODEL_CHANGE_PID)] = true;

      break;
    }

    case MSP_PID_ADVANCED:
      r.writeU16(0);
      r.writeU16(0);
      r.writeU16(0);                             // was pidProfile.yaw_p_limit
      r.writeU8(0);                              // reserved
      r.writeU8(0);                              // vbatPidCompensation;
      r.writeU8(0);                              // feedForwardTransition;
      r.writeU8(0);                              // was low byte of dtermSetpointWeight
      r.writeU8(0);                              // reserved
      r.writeU8(0);                              // reserved
      r.writeU8(0);                              // reserved
      r.writeU16(0);                             // rateAccelLimit;
      r.writeU16(0);                             // yawRateAccelLimit;
      r.writeU8(config.level.angleLimit); // levelAngleLimit;
      r.writeU8(0);                              // was pidProfile.levelSensitivity
      r.writeU16(0);                             // itermThrottleThreshold;
      r.writeU16(config.antiGravityGain); // anti_gravity_gain
      r.writeU16(0);
      r.writeU8(0);                                  // iterm rotation
      r.writeU8(0);                                  // smart feed forward
      r.writeU8(config.iterm.relax);          // iterm relax
      r.writeU8(1);                                  // iterm relax type (setpoint only)
      r.writeU8(0);                                  // abs control gain
      r.writeU8(0);                                  // throttle boost
      r.writeU8(0);                                  // acro trainer max angle
      r.writeU16(config.pid[FC_PID_ROLL].F);  // pid roll f
      r.writeU16(config.pid[FC_PID_PITCH].F); // pid pitch f
      r.writeU16(config.pid[FC_PID_YAW].F);   // pid yaw f
      r.writeU8(0);                                  // antigravity mode
      // 1.41+
      r.writeU8(0); // d max roll
      r.writeU8(0); // d max pitch
      r.writeU8(0); // d max yaw
      r.writeU8(0); // d max gain
      r.writeU8(0); // d max advance
      r.writeU8(0); // use_integrated_yaw
      r.writeU8(0); // integrated_yaw_relax
      // 1.42+
      r.writeU8(config.iterm.relaxCutoff); // iterm_relax_cutoff
      // 1.43+
      r.writeU8(config.output.motorLimit); // motor_output_limit
      r.writeU8(0);                               // auto_profile_cell_count
      r.writeU8(0);                               // idle_min_rpm
      // 1.44
      r.writeU8(0);                                       // ff avg
      r.writeU8(0);                                       // ff smooth
      r.writeU8(0);                                       // ff boost
      r.writeU8(0);                                       // ff max rate limit
      r.writeU8(0);                                       // ff jitter factor
      r.writeU8(0);                                       // vbat sag compensation
      r.writeU8(0);                                       // thrust linearization
      r.writeU8(config.controller.tpaMode);        // tpa mode
      r.writeU8(config.controller.tpaScale);       // tpa rate
      r.writeU16(config.controller.tpaBreakpoint); // tpa breakpoint
      break;

    case MSP_SET_PID_ADVANCED:
      m.readU16();
      m.readU16();
      m.readU16(); // was pidProfile.yaw_p_limit
      m.readU8();  // reserved
      m.readU8();
      m.readU8();
      m.readU8();
      m.readU8(); // reserved
      m.readU8(); // reserved
      m.readU8(); // reserved
      m.readU16();
      m.readU16();
      if (m.remain() >= 2)
      {
        config.level.angleLimit = m.readU8();
        m.readU8(); // was pidProfile.levelSensitivity
      }
      if (m.remain() >= 4)
      {
        m.readU16(); // itermThrottleThreshold;
        config.antiGravityGain =
            std::clamp<uint16_t>(
                m.readU16(),
                0,
                250); // anti_gravity_gain
      }
      if (m.remain() >= 2)
      {
        m.readU16();
      }
      if (m.remain() >= 14)
      {
        m.readU8();                                      // iterm rotation
        m.readU8();                                      // smart feed forward
        config.iterm.relax = m.readU8();          // iterm relax
        m.readU8();                                      // iterm relax type
        m.readU8();                                      // abs control gain
        m.readU8();                                      // throttle boost
        m.readU8();                                      // acro trainer max angle
        config.pid[FC_PID_ROLL].F = m.readU16();  // pid roll f
        config.pid[FC_PID_PITCH].F = m.readU16(); // pid pitch f
        config.pid[FC_PID_YAW].F = m.readU16();   // pid yaw f
        m.readU8();                                      // antigravity mode
      }
      // 1.41+
      if (m.remain() >= 7)
      {
        m.readU8(); // d max roll
        m.readU8(); // d max pitch
        m.readU8(); // d max yaw
        m.readU8(); // d max gain
        m.readU8(); // d max advance
        m.readU8(); // use_integrated_yaw
        m.readU8(); // integrated_yaw_relax
      }
      // 1.42+
      if (m.remain() >= 1)
      {
        config.iterm.relaxCutoff = m.readU8(); // iterm_relax_cutoff
      }
      // 1.43+
      if (m.remain() >= 3)
      {
        config.output.motorLimit = m.readU8(); // motor_output_limit
        m.readU8();                                   // auto_profile_cell_count
        m.readU8();                                   // idle_min_rpm
      }
      // 1.44
      if (m.remain() >= 5)
      {
        m.readU8(); // ff avg
        m.readU8(); // ff smooth
        m.readU8(); // ff boost
        m.readU8(); // ff max rate limit
        m.readU8(); // ff jitter factor
      }
      if (m.remain() >= 1)
      {
        m.readU8(); // vbat sag compensation
      }
      if (m.remain() >= 1)
      {
        m.readU8(); // thrust linearization
      }
      if (m.remain() >= 4)
      {
        config.controller.tpaMode = m.readU8();                                          // tpa mode
        config.controller.tpaScale = m.readU8(); // checked before commit
        config.controller.tpaBreakpoint = m.readU16();
      }
      events[static_cast<size_t>(MODEL_CHANGE_PID)] = true;
      break;

    case MSP_SIMPLIFIED_TUNING: {
      const auto& s = config.simplifiedTuning;
      r.writeU8(s.pidsMode);
      r.writeU8(s.masterMultiplier);
      r.writeU8(s.rollPitchRatio);
      r.writeU8(s.iGain);
      r.writeU8(s.dGain);
      r.writeU8(s.piGain);
      r.writeU8(s.dMaxGain);
      r.writeU8(s.ffGain);
      r.writeU8(s.pitchPiGain);
      r.writeU32(0);
      r.writeU32(0);
      r.writeU8(s.dtermFilter);
      r.writeU8(s.dtermFilterMultiplier);
      r.writeU16(config.dterm.filter.freq);
      r.writeU16(config.dterm.filter2.freq);
      r.writeU16(config.dterm.dynLpfFilter.cutoff);
      r.writeU16(config.dterm.dynLpfFilter.freq);
      r.writeU32(0);
      r.writeU32(0);
      r.writeU8(s.gyroFilter);
      r.writeU8(s.gyroFilterMultiplier);
      r.writeU16(config.gyro.filter.freq);
      r.writeU16(config.gyro.filter2.freq);
      r.writeU16(config.gyro.dynLpfFilter.cutoff);
      r.writeU16(config.gyro.dynLpfFilter.freq);
      r.writeU32(0);
      r.writeU32(0);
      break;
    }

    case MSP_SET_SIMPLIFIED_TUNING: {
      auto& s = config.simplifiedTuning;
      s.pidsMode = m.readU8();
      s.masterMultiplier = m.readU8();
      s.rollPitchRatio = m.readU8();
      s.iGain = m.readU8();
      s.dGain = m.readU8();
      s.piGain = m.readU8();
      s.dMaxGain = m.readU8();
      s.ffGain = m.readU8();
      s.pitchPiGain = m.readU8();
      m.readU32();
      m.readU32();
      s.dtermFilter = m.readU8();
      s.dtermFilterMultiplier = m.readU8();
      config.dterm.filter.freq = m.readU16();
      config.dterm.filter2.freq = m.readU16();
      config.dterm.dynLpfFilter.cutoff = m.readU16();
      config.dterm.dynLpfFilter.freq = m.readU16();
      m.readU32();
      m.readU32();
      s.gyroFilter = m.readU8();
      s.gyroFilterMultiplier = m.readU8();
      config.gyro.filter.freq = m.readU16();
      config.gyro.filter2.freq = m.readU16();
      config.gyro.dynLpfFilter.cutoff = m.readU16();
      config.gyro.dynLpfFilter.freq = m.readU16();
      m.readU32();
      m.readU32();
      _model.calculateSimplifiedPids(s, config.pid);
      if (s.dtermFilter)
      {
        _model.calculateSimplifiedDtermFilters(
            s.dtermFilterMultiplier, config.dterm.filter.freq, config.dterm.filter2.freq,
            config.dterm.dynLpfFilter.cutoff, config.dterm.dynLpfFilter.freq);
      }
      if (s.gyroFilter)
      {
        _model.calculateSimplifiedGyroFilters(s.gyroFilterMultiplier, config.gyro.filter.freq,
                                              config.gyro.filter2.freq, config.gyro.dynLpfFilter.cutoff,
                                              config.gyro.dynLpfFilter.freq);
      }
      events[static_cast<size_t>(MODEL_CHANGE_PID)] = true;
      if (s.gyroFilter || s.dtermFilter)
      {
        events[static_cast<size_t>(MODEL_CHANGE_FILTER)] = true;
      }
      break;
    }

    case MSP_CALCULATE_SIMPLIFIED_PID: {
      SimplifiedTuningConfig s;
      s.pidsMode = m.readU8();
      s.masterMultiplier = m.readU8();
      s.rollPitchRatio = m.readU8();
      s.iGain = m.readU8();
      s.dGain = m.readU8();
      s.piGain = m.readU8();
      s.dMaxGain = m.readU8();
      s.ffGain = m.readU8();
      s.pitchPiGain = m.readU8();
      m.readU32();
      m.readU32();
      PidConfig tmp[3] = {config.pid[FC_PID_ROLL], config.pid[FC_PID_PITCH],
                          config.pid[FC_PID_YAW]};
      _model.calculateSimplifiedPids(s, tmp);
      for (int i = 0; i < 3; i++)
      {
        r.writeU8(tmp[i].P);
        r.writeU8(tmp[i].I);
        r.writeU8(tmp[i].D);
        r.writeU8(0); // d_max not supported
        r.writeU16(tmp[i].F);
      }
      break;
    }

    case MSP_CALCULATE_SIMPLIFIED_GYRO: {
      uint8_t filter = m.readU8();
      uint8_t mult = m.readU8();
      int16_t lpf1 = m.readU16();
      int16_t lpf2 = m.readU16();
      int16_t dynMin = m.readU16();
      int16_t dynMax = m.readU16();
      m.readU32();
      m.readU32();
      if (filter) _model.calculateSimplifiedGyroFilters(mult, lpf1, lpf2, dynMin, dynMax);
      r.writeU8(filter);
      r.writeU8(mult);
      r.writeU16(lpf1);
      r.writeU16(lpf2);
      r.writeU16(dynMin);
      r.writeU16(dynMax);
      r.writeU32(0);
      r.writeU32(0);
      break;
    }

    case MSP_CALCULATE_SIMPLIFIED_DTERM: {
      uint8_t filter = m.readU8();
      uint8_t mult = m.readU8();
      int16_t lpf1 = m.readU16();
      int16_t lpf2 = m.readU16();
      int16_t dynMin = m.readU16();
      int16_t dynMax = m.readU16();
      m.readU32();
      m.readU32();
      if (filter) _model.calculateSimplifiedDtermFilters(mult, lpf1, lpf2, dynMin, dynMax);
      r.writeU8(filter);
      r.writeU8(mult);
      r.writeU16(lpf1);
      r.writeU16(lpf2);
      r.writeU16(dynMin);
      r.writeU16(dynMax);
      r.writeU32(0);
      r.writeU32(0);
      break;
    }

    case MSP_VALIDATE_SIMPLIFIED_TUNING: {
      auto [pidOk, gyroOk, dtermOk] = _model.validateSimplifiedTuning();
      r.writeU8(pidOk);
      r.writeU8(gyroOk);
      r.writeU8(dtermOk);
      break;
    }

    case MSP_RAW_IMU: {
      auto accel = _model.state.accel.adc.fetch();
      for (int i = 0; i < AXIS_COUNT_RPY; i++)
      {
        r.writeU16(lrintf(accel[i] * ACCEL_G_INV * 2048.f));
      }
      for (int i = 0; i < AXIS_COUNT_RPY; i++)
      {
        r.writeU16(lrintf(Utils::toDeg(_model.state.gyro.adc[i])));
      }
      for (int i = 0; i < AXIS_COUNT_RPY; i++)
      {
        r.writeU16(lrintf(_model.state.mag.adc[i] * 1090));
      }
      break;
    }

    case MSP_MOTOR:
      for (size_t i = 0; i < 8; i++)
      {
        if (i >= OUTPUT_CHANNELS || config.pin[i + PIN_OUTPUT_0] == -1)
        {
          r.writeU16(0);
          continue;
        }
        r.writeU16(_model.state.output.us[i]);
      }
      break;

    case MSP_MOTOR_TELEMETRY:
      r.writeU8(OUTPUT_CHANNELS);
      for (size_t i = 0; i < OUTPUT_CHANNELS; i++)
      {
        int rpm = 0;
        uint16_t invalidPct = 0;
        uint8_t escTemperature = 0;  // degrees celcius
        uint16_t escVoltage = 0;     // 0.01V per unit
        uint16_t escCurrent = 0;     // 0.01A per unit
        uint16_t escConsumption = 0; // mAh

        if (config.pin[i + PIN_OUTPUT_0] != -1)
        {
          rpm = lrintf(_model.state.output.telemetry.rpm[i]);
          invalidPct = _model.state.output.telemetry.errors[i];
          escTemperature = _model.state.output.telemetry.temperature[i];
          escVoltage = _model.state.output.telemetry.voltage[i];
          escCurrent = _model.state.output.telemetry.current[i];
        }

        r.writeU32(rpm);
        r.writeU16(invalidPct);
        r.writeU8(escTemperature);
        r.writeU16(escVoltage);
        r.writeU16(escCurrent);
        r.writeU16(escConsumption);
      }
      break;

    case MSP_SET_MOTOR: {
      // Motors tab is a DISARMED-only, bounded actuator session. Never
      // overwrite the normal stop commands with tester values and never
      // let a truncated packet leave half a command applied.
      if (_model.isModeActive(MODE_ARMED) ||
          _model.state.failsafe.phase != FC_FAILSAFE_IDLE ||
          _model.state.pinConflict ||
          m.remain() != 2 * OUTPUT_CHANNELS)
      {
        r.result = -1;
        break;
      }

      int16_t requested[OUTPUT_CHANNELS]{};
      bool anyRunning = false;
      bool invalid = false;
      const size_t motorCount =
          _model.state.currentMixer.count <= OUTPUT_CHANNELS
              ? _model.state.currentMixer.count : 0u;
      for (size_t i = 0; i < OUTPUT_CHANNELS; ++i)
      {
        const uint16_t value = m.readU16();
        const bool validMotor = i < motorCount &&
            !config.output.channel[i].servo;
        if (value < 1000u || value > 2000u ||
            (!validMotor && value != 1000u))
        {
          invalid = true;
        }
        requested[i] = static_cast<int16_t>(value);
        anyRunning |= validMotor && value > 1000u;
      }
      if (invalid)
      {
        r.result = -1;
        break;
      }

      auto& output = _model.state.output;
      output.motorTestActive = false; // publish complete snapshot atomically
      for (size_t i = 0; i < OUTPUT_CHANNELS; ++i)
      {
        output.motorTestUs[i] = requested[i];
      }
      output.motorTestLastUpdateUs = micros();
      output.motorTestActive = anyRunning;
      break;
    }

    case MSP_SERVO:
      for (size_t i = 0; i < OUTPUT_CHANNELS; i++)
      {
        if (i >= OUTPUT_CHANNELS || config.pin[i + PIN_OUTPUT_0] == -1)
        {
          r.writeU16(1500);
          continue;
        }
        r.writeU16(_model.state.output.us[i]);
      }
      break;

    case MSP_SERVO_CONFIGURATIONS:
      for (size_t i = 0; i < 8; i++)
      {
        if (i < OUTPUT_CHANNELS)
        {
          r.writeU16(config.output.channel[i].min);
          r.writeU16(config.output.channel[i].max);
          r.writeU16(config.output.channel[i].neutral);
        }
        else
        {
          r.writeU16(1000);
          r.writeU16(2000);
          r.writeU16(1500);
        }
        r.writeU8(100);
        r.writeU8(-1);
        r.writeU32(0);
      }
      break;

    case MSP_SET_SERVO_CONFIGURATION: {
      uint8_t i = m.readU8();
      if (i < OUTPUT_CHANNELS)
      {
        config.output.channel[i].min = m.readU16();
        config.output.channel[i].max = m.readU16();
        config.output.channel[i].neutral = m.readU16();
        m.readU8();
        m.readU8();
        m.readU32();
      }
      else
      {
        r.result = -1;
      }
    }
    break;

    case MSP_ACC_CALIBRATION:
      if (!_model.isModeActive(MODE_ARMED)) _model.calibrateGyro();
      break;

    case MSP_MAG_CALIBRATION:
      if (!_model.isModeActive(MODE_ARMED)) _model.calibrateMag();
      break;

    case MSP_VTX_CONFIG:
      if (!_model.state.vtx.active)
      {
        r.writeU8(0);  // vtx type
        r.writeU8(0);  // band
        r.writeU8(0);  // channel
        r.writeU8(0);  // power
        r.writeU8(0);  // status
        r.writeU16(0); // freq
        r.writeU8(0);  // ready
        r.writeU8(0);  // low power disarm
      }
      else
      {
        r.writeU8(3 /* SMARTAUDIO */);               // vtx type unknown
        r.writeU8(_model.state.vtx.appliedBand); // band
        r.writeU8(_model.state.vtx.appliedChannel); // channel
        r.writeU8(_model.state.vtx.appliedPower); // power
        r.writeU8(_model.state.vtx.pitMode); // observed pit status
        r.writeU16(_model.state.vtx.frequency); // observed frequency
        r.writeU8(_model.state.vtx.ready); // acknowledged configuration
        r.writeU8(config.vtx.lowPowerDisarm); // low power disarm
      }
      // 1.42
      r.writeU16(0); // pit mode freq
      r.writeU8(0);  // vtx table available (no)
      r.writeU8(0);  // vtx table bands
      r.writeU8(0);  // vtx table channels
      r.writeU8(0);  // vtx power levels
      break;

    case MSP_SET_VTX_CONFIG: {
      const uint16_t encoded = m.readU16();
      if (encoded >= 40) { r.result = -1; break; }
      config.vtx.band = encoded / 8u + 1u;
      config.vtx.channel = encoded % 8u + 1u;
      if (m.remain() >= 2)
      {
        config.vtx.power = m.readU8();
        if (m.readU8() != 0) { r.result = -1; break; } // Pit mode not implemented.
      }
      if (m.remain() >= 1) config.vtx.lowPowerDisarm = m.readU8();
      if (m.remain() >= 2 && m.readU16() != 0) { r.result = -1; break; }
      if (m.remain() >= 4)
      {
        config.vtx.band = m.readU8(); config.vtx.channel = m.readU8();
        const uint16_t frequency = m.readU16();
        if (config.vtx.band == 0 || frequency > 6000) { r.result = -1; break; }
      }
      if (_model.state.vtx.powerLevels && config.vtx.power > _model.state.vtx.powerLevels)
        r.result = -1;
      break;
    }

    case MSP_SET_ARMING_DISABLED: {
      const uint8_t cmd = m.readU8();
      uint8_t disableRunawayTakeoff = 0;
      if (m.remain())
      {
        disableRunawayTakeoff = m.readU8();
      }
      (void)disableRunawayTakeoff;
#if defined(ESPFC_DEV_PRESET_UNSAFE_ARMING)
      (void)cmd;
#warning "Danger macro used ESPFC_DEV_PRESET_UNSAFE_ARMING"
#else
      _model.setArmingDisabled(ARMING_DISABLED_MSP, cmd);
#endif
      if (_model.isModeActive(MODE_ARMED)) _model.disarm(DISARM_REASON_ARMING_DISABLED);
    }
    break;

    case MSP_SET_PASSTHROUGH: {
      // An armed ESC passthrough could pause the flight loop.
      if (_model.isModeActive(MODE_ARMED)) { r.result = -1; break; }
      uint8_t ptMode = MSP_PASSTHROUGH_ESC_4WAY;
      uint8_t ptArg = 0;
      if (m.remain() >= 2)
      {
        ptMode = m.readU8();
        ptArg = m.readU8();
      }
      switch (ptMode)
      {
        case MSP_PASSTHROUGH_ESC_4WAY:
          r.writeU8(esc4wayInit());
          serialDeviceInit(&s, 0);
          _postCommand = [this]() { processEsc4way(); };
          break;
        default:
          r.writeU8(0);
          break;
      }
      (void)ptArg;
    }
    break;

    case MSP_DEBUG:
      for (int i = 0; i < 8; i++)
      {
        r.writeU16(_model.state.debug[i]);
      }
      break;

    case MSP_SET_GPS_CONFIG:
      m.readU8(); // provider
      m.readU8(); // sbas mode
      m.readU8(); // auto config
      m.readU8(); // auto baud
      if (m.remain() >= 2)
      {
        // Added in API version 1.43
        config.gps.setHomeOnce = m.readU8(); // gps_set_home_point_once
        m.readU8();                                 // gps_ublox_use_galileo
      }
      break;

    case MSP_GPS_CONFIG:
      r.writeU8(1); // provider
      r.writeU8(0); // sbasMode, 0: auto
      r.writeU8(1); // autoConfig, 0: off, 1: on
      r.writeU8(1); // autoBaud, 0: off, 1: on
      // Added in API version 1.43
      r.writeU8(config.gps.setHomeOnce); // gps_set_home_point_once
      r.writeU8(1);                             // gps_ublox_use_galileo
      break;

    case MSP_RAW_GPS:
      r.writeU8(_model.state.gps.fixType > 2);       // STATE(GPS_FIX));
      r.writeU8(_model.state.gps.numSats);           // numSat
      r.writeU32(_model.state.gps.location.raw.lat); // lat
      r.writeU32(_model.state.gps.location.raw.lon); // lon
      r.writeU16(std::clamp<int>(_model.state.gps.location.raw.height / 1000, 0,
                                 std::numeric_limits<uint16_t>::max())); // height [m]
      r.writeU16(_model.state.gps.velocity.raw.groundSpeed / 10);        // cm/s
      r.writeU16(_model.state.gps.velocity.raw.heading / 10000);         // deg * 10
      // Added in API version 1.44
      r.writeU16(_model.state.gps.accuracy.pDop); // pDOP
      break;

    case MSP2_ESPFC_ALTHOLD_CONFIG: {
      const int payloadSize = m.remain();
      if (payloadSize != 0 && payloadSize != 10 && payloadSize != 22)
      {
        r.result = -1;
        break;
      }
      auto& a = config.altHold;
      auto& rf = config.rangefinder;

      if (m.remain() == 0)
      {
        // AltHold V2 config, fixed 22-byte payload.
        r.writeU8(a.itermCenter);
        r.writeU8(a.itermRange);
        r.writeU8(a.baroTau);
        r.writeU8(a.hoverThrottle);
        r.writeU8(a.hoverLearnRate);
        r.writeU8(a.maxClimbRate);
        r.writeU8(a.maxDescentRate);
        r.writeU8(a.verticalAccelLimit);
        r.writeU8(a.verticalJerkLimit);
        r.writeU8(a.baroInnovationGate);
        r.writeU8(a.baroRateInnovationGate);
        r.writeU8(a.groundEffectHeight);
        r.writeU8(a.propWashAccelThreshold);

        r.writeU8(rf.type);
        r.writeU8(static_cast<uint8_t>(rf.triggerPin));
        r.writeU8(static_cast<uint8_t>(rf.echoPin));
        r.writeU16(rf.minDistanceCm);
        r.writeU16(rf.maxDistanceCm);
        r.writeU16(rf.updateIntervalMs);
      }
      else if (m.remain() == 10)
      {
        // Backward compatible with the original ten-byte V2 payload.
        a.hoverThrottle = m.readU8();
        a.hoverLearnRate = m.readU8();
        a.maxClimbRate = m.readU8();
        a.maxDescentRate = m.readU8();
        a.verticalAccelLimit = m.readU8();
        a.verticalJerkLimit = m.readU8();
        a.baroInnovationGate = m.readU8();
        a.baroRateInnovationGate = m.readU8();
        a.groundEffectHeight = m.readU8();
        a.propWashAccelThreshold = m.readU8();

        needsSanitize = true;
        needsReboot = true;
      }
      else if (m.remain() == 22)
      {
        a.itermCenter = m.readU8();
        a.itermRange = m.readU8();
        a.baroTau = m.readU8();
        a.hoverThrottle = m.readU8();
        a.hoverLearnRate = m.readU8();
        a.maxClimbRate = m.readU8();
        a.maxDescentRate = m.readU8();
        a.verticalAccelLimit = m.readU8();
        a.verticalJerkLimit = m.readU8();
        a.baroInnovationGate = m.readU8();
        a.baroRateInnovationGate = m.readU8();
        a.groundEffectHeight = m.readU8();
        a.propWashAccelThreshold = m.readU8();

        rf.type =
            static_cast<uint8_t>(m.readU8());
        rf.triggerPin =
            static_cast<int8_t>(m.readU8());
        rf.echoPin =
            static_cast<int8_t>(m.readU8());
        rf.minDistanceCm = m.readU16();
        rf.maxDistanceCm = m.readU16();
        rf.updateIntervalMs = m.readU16();

        needsSanitize = true;
        needsReboot = true;
      }
      break;
    }

    case MSP2_ESPFC_ALTHOLD: {
      const auto& altitude = _model.state.altitude;
      const auto& assisted = _model.state.assistedMode;
      const auto& mixer = _model.state.mixer;

      r.writeU8(static_cast<uint8_t>(altitude.estimatorState));
      r.writeU16(altitude.faultFlags);
      r.writeU8(altitude.baroAccepted ? 1 : 0);
      r.writeU8(altitude.baroRateAccepted ? 1 : 0);

      r.writeU32(static_cast<uint32_t>(lrintf(altitude.height * 1000.0f)));
      r.writeU32(static_cast<uint32_t>(lrintf(altitude.vario * 1000.0f)));
      r.writeU32(static_cast<uint32_t>(lrintf(altitude.acceleration * 1000.0f)));
      r.writeU32(static_cast<uint32_t>(lrintf(altitude.accelerationBias * 1000.0f)));
      r.writeU32(static_cast<uint32_t>(lrintf(altitude.baroBias * 1000.0f)));
      r.writeU32(static_cast<uint32_t>(lrintf(altitude.baroInnovation * 1000.0f)));
      r.writeU32(static_cast<uint32_t>(lrintf(altitude.baroRateInnovation * 1000.0f)));
      r.writeU32(static_cast<uint32_t>(lrintf(altitude.rangefinderInnovation * 1000.0f)));

      r.writeU32(static_cast<uint32_t>(lrintf(assisted.hoverThrust * 1000.0f)));
      r.writeU32(static_cast<uint32_t>(lrintf(assisted.tiltCompensatedHover * 1000.0f)));
      r.writeU32(static_cast<uint32_t>(lrintf(assisted.altitudeTarget * 1000.0f)));
      r.writeU32(static_cast<uint32_t>(lrintf(assisted.verticalRateTarget * 1000.0f)));
      r.writeU32(static_cast<uint32_t>(lrintf(_model.state.output.ch[AXIS_THRUST] * 1000.0f)));

      uint8_t flags = 0;
      flags |= mixer.verticalSaturationHigh ? 1u : 0u;
      flags |= mixer.verticalSaturationLow ? 2u : 0u;
      flags |= altitude.rangefinderUsed ? 4u : 0u;
      flags |= altitude.estimatorState == ALTITUDE_ESTIMATOR_DEGRADED ? 8u : 0u;
      flags |= altitude.estimatorState == ALTITUDE_ESTIMATOR_FAULT ? 16u : 0u;
      r.writeU8(flags);

      r.writeU32(altitude.baroAcceptedSamples);
      r.writeU32(altitude.baroRejectedSamples);
      r.writeU32(altitude.baroRateRejectedSamples);
      r.writeU16(altitude.baroConsecutiveRejects);

      // Normalized live rangefinder diagnostics. A backend may be absent or
      // invalid without affecting the rest of the AltHold telemetry payload.
      const auto& range = _model.state.rangefinder;
      r.writeU8(
          range.present ? 1 : 0);
      r.writeU8(
          range.sampleValid ? 1 : 0);
      r.writeU16(
          static_cast<uint16_t>(
              std::clamp(
                  lrintf(range.distance * 1000.0f),
                  0L,
                  65535L)));
      r.writeU8(range.quality);
      r.writeU32(
          range.lastUpdateUs);
      r.writeU8(
          static_cast<uint8_t>(
              config.rangefinder.type));
      break;
    }

    case MSP_GPS_RESCUE: {
      StockRescueSettings v;
      if (!Control::modeSupported(MODE_GPS_RESCUE) || !Control::stockRescueSettings(config, v))
      { r.result = -1; break; }
      r.writeU16(v.angle);
      r.writeU16(v.returnAltitudeM);
      r.writeU16(v.descentDistanceM);
      r.writeU16(v.groundSpeedCms);
      r.writeU16(v.throttleMin);
      r.writeU16(v.throttleMax);
      r.writeU16(v.throttleHover);
      r.writeU8(v.sanityChecks);
      r.writeU8(v.minSats);
      r.writeU16(v.ascendRateCms);
      r.writeU16(v.descendRateCms);
      r.writeU8(v.allowArmingWithoutFix);
      r.writeU8(v.altitudeMode);
      r.writeU16(v.minStartDistanceM);
      r.writeU16(v.initialClimbM);
      break;
    }
    case MSP_SET_GPS_RESCUE: {
      if (!Control::modeSupported(MODE_GPS_RESCUE)) { r.result = -1; break; }
      auto& v = config.extension.stockRescue;
      v.angle = m.readU16();
      v.returnAltitudeM = m.readU16();
      v.descentDistanceM = m.readU16();
      v.groundSpeedCms = m.readU16();
      v.throttleMin = m.readU16();
      v.throttleMax = m.readU16();
      v.throttleHover = m.readU16();
      v.sanityChecks = m.readU8();
      v.minSats = m.readU8();
      v.ascendRateCms = m.readU16();
      v.descendRateCms = m.readU16();
      v.allowArmingWithoutFix = m.readU8();
      v.altitudeMode = m.readU8();
      v.minStartDistanceM = m.readU16();
      v.initialClimbM = m.readU16();
      config.extension.stockRescueEnabled = 1;
      break;
    }

    case MSP2_ESPFC_GPS_RESCUE_CONFIG: {
      if (m.remain() == 0 && config.extension.stockRescueEnabled)
      { r.result = -1; break; }
      const int payloadSize = m.remain();
      if (payloadSize != 0 && payloadSize != 18)
      {
        r.result = -1;
        break;
      }
      const auto& c = config.gpsRescue;
      if (m.remain() == 0)
      {
        r.writeU8(c.altitudeMode);
        r.writeU8(c.altitudeMarginM);
        r.writeU8(c.fixedAltitudeM);
        r.writeU8(c.maxAltitudeM);
        r.writeU8(c.climbRateDmS);
        r.writeU8(c.descentRateDmS);
        r.writeU8(c.speedDmS);
        r.writeU8(c.approachSpeedDmS);
        r.writeU8(c.maxAngleDeg);
        r.writeU8(c.approachDistanceM);
        r.writeU8(c.landDistanceM);
        r.writeU8(c.minDistanceM);
        r.writeU8(c.alignToleranceDeg);
        r.writeU8(c.minLandingAltitudeDm);
        r.writeU8(c.gpsStaleDs);
        r.writeU8(c.maxHorizontalAccuracyM);
        r.writeU16(c.maxAccelerationCms2);
      }
      else if (m.remain() == 18)
      {
        config.extension.stockRescueEnabled = 0;
        auto& cfg = config.gpsRescue;
        cfg.altitudeMode = m.readU8();
        cfg.altitudeMarginM = m.readU8();
        cfg.fixedAltitudeM = m.readU8();
        cfg.maxAltitudeM = m.readU8();
        cfg.climbRateDmS = m.readU8();
        cfg.descentRateDmS = m.readU8();
        cfg.speedDmS = m.readU8();
        cfg.approachSpeedDmS = m.readU8();
        cfg.maxAngleDeg = m.readU8();
        cfg.approachDistanceM = m.readU8();
        cfg.landDistanceM = m.readU8();
        cfg.minDistanceM = m.readU8();
        cfg.alignToleranceDeg = m.readU8();
        cfg.minLandingAltitudeDm = m.readU8();
        cfg.gpsStaleDs = m.readU8();
        cfg.maxHorizontalAccuracyM = m.readU8();
        cfg.maxAccelerationCms2 = m.readU16();
        needsSanitize = true;
        needsReboot = true;
      }
      break;
    }

    case MSP2_ESPFC_GPS_RESCUE: {
      const auto& state = _model.state.gpsRescue;
      const auto& nav = _model.state.navigation;

      r.writeU8(state.phase);
      r.writeU16(state.faultFlags);

      uint8_t flags = 0;
      flags |= state.active ? 1u : 0u;
      flags |= state.controlling ? 2u : 0u;
      flags |= state.requestLand ? 4u : 0u;
      flags |= state.gpsHealthy ? 8u : 0u;
      flags |= state.altitudeHealthy ? 16u : 0u;
      flags |= state.attitudeHealthy ? 32u : 0u;
      flags |= nav.positionValid ? 64u : 0u;
      flags |= nav.velocityValid ? 128u : 0u;
      r.writeU8(flags);

      r.writeU32(static_cast<uint32_t>(
          static_cast<int32_t>(
              std::clamp(
                  lrintf(state.north * 100.0f),
                  -2147483647L,
                  2147483647L))));
      r.writeU32(static_cast<uint32_t>(
          static_cast<int32_t>(
              std::clamp(
                  lrintf(state.east * 100.0f),
                  -2147483647L,
                  2147483647L))));
      r.writeU32(static_cast<uint32_t>(
          static_cast<int32_t>(
              std::clamp(
                  lrintf(state.down * 100.0f),
                  -2147483647L,
                  2147483647L))));
      r.writeU32(static_cast<uint32_t>(
          std::clamp<int64_t>(
              static_cast<int64_t>(
                  lrintf(state.distanceToHome * 100.0f)),
              int64_t{0},
              static_cast<int64_t>(
                  std::numeric_limits<uint32_t>::max()))));
      r.writeU32(static_cast<uint32_t>(
          lrintf(state.bearingToHome * 100000.0f)));
      r.writeU32(static_cast<uint32_t>(
          static_cast<int32_t>(
              std::clamp(
                  lrintf(state.altitudeAboveHome * 100.0f),
                  -2147483647L,
                  2147483647L))));
      r.writeU32(static_cast<uint32_t>(
          std::clamp<int64_t>(
              static_cast<int64_t>(
                  lrintf(state.targetAltitude * 100.0f)),
              int64_t{0},
              static_cast<int64_t>(
                  std::numeric_limits<uint32_t>::max()))));
      r.writeU16(static_cast<uint16_t>(
          std::clamp(
              lrintf(state.rollAngle * 1000.0f),
              -32768L,
              32767L)));
      r.writeU16(static_cast<uint16_t>(
          std::clamp(
              lrintf(state.pitchAngle * 1000.0f),
              -32768L,
              32767L)));
      r.writeU16(static_cast<uint16_t>(
          std::clamp(
              lrintf(state.yawRate * 1000.0f),
              -32768L,
              32767L)));
      r.writeU16(static_cast<uint16_t>(
          std::clamp(
              lrintf(state.verticalRate * 1000.0f),
              -32768L,
              32767L)));
      r.writeU8(state.sats);
      r.writeU16(static_cast<uint16_t>(
          std::clamp(
              lrintf(state.horizontalAccuracy * 1000.0f),
              0L,
              65535L)));
      break;
    }

    case MSP2_ESPFC_GPS_FILTER: {
      // ESP-FC custom MSPv2 GPS filter telemetry used by the matching Configurator.
      // Payload: flags, raw/filtered lat/lon, raw N/E/ground speed,
      // filtered N/E/ground speed, raw-to-filtered distance,
      // accepted/rejected filter sample counters.
      const auto& gps = _model.state.gps;
      const auto& d = gps.diagnostics;
      r.writeU8(d.filterAccepted ? 0x01 : 0x00);
      r.writeU32(static_cast<uint32_t>(gps.location.raw.lat));
      r.writeU32(static_cast<uint32_t>(gps.location.raw.lon));
      r.writeU32(static_cast<uint32_t>(gps.location.filtered.lat));
      r.writeU32(static_cast<uint32_t>(gps.location.filtered.lon));
      r.writeU32(static_cast<uint32_t>(d.rawNorthSpeed));
      r.writeU32(static_cast<uint32_t>(d.rawEastSpeed));
      r.writeU32(d.rawGroundSpeed);
      r.writeU32(static_cast<uint32_t>(d.filteredNorthSpeed));
      r.writeU32(static_cast<uint32_t>(d.filteredEastSpeed));
      r.writeU32(d.filteredGroundSpeed);
      r.writeU32(d.rawFilteredDistance);
      r.writeU32(d.acceptedSamples);
      r.writeU32(d.rejectedSamples);
      break;
    }

    case MSP_COMP_GPS:
      r.writeU16(std::clamp<uint16_t>(lrintf(_model.state.gps.distanceToHome), 0,
                                      std::numeric_limits<uint16_t>::max()));      // meters
      r.writeU16((int16_t)lrintf(Utils::toDeg(_model.state.gps.directionToHome))); // deg
      r.writeU8(_model.state.gps.homeSet ? 1 : 0);                                 // GPS update
      break;

    case MSP_GPSSVINFO:
      r.writeU8(_model.state.gps.numCh); // GPS_numCh
      for (size_t i = 0; i < _model.state.gps.numCh; i++)
      {
        r.writeU8(_model.state.gps.svinfo[i].gnssId);                                     // GPS_svinfo_chn[i]
        r.writeU8(_model.state.gps.svinfo[i].id);                                         // GPS_svinfo_svid[i]
        r.writeU8(static_cast<uint8_t>(_model.state.gps.svinfo[i].quality.value & 0xff)); // GPS_svinfo_quality[i]
        r.writeU8(_model.state.gps.svinfo[i].cno);                                        // GPS_svinfo_cno[i]
      }
      break;

    case MSP_EEPROM_WRITE:
      if (!_model.isModeActive(MODE_ARMED))
      {
        if (!_model.save()) r.result = -1;
      }
      else
      {
        r.result = -1;
      }
      break;

    case MSP_RESET_CONF:
      if (!_model.isModeActive(MODE_ARMED))
      {
        _model.reset();
        if (!_model.save()) { r.result = -1; break; }
        _postCommand = [this]() { processRestart(); };
        r.writeU8(1); // success
      }
      else
      {
        r.writeU8(0); // fail
      }
      break;

    case MSP_REBOOT: {
      // Never turn an armed maintenance packet into flight termination.
      if (_model.isModeActive(MODE_ARMED)) { r.result = -1; break; }
      uint8_t rebootType = 0; // reboot to firmware only
      if (m.remain())
      {
        // TODO: reboot to bootloader
        rebootType = m.readU8();
        if (rebootType != 0)
        {
          r.result = -1; // fail
          break;
        }
      }
      r.writeU8(rebootType);
      _postCommand = [this]() { processRestart(); };
      break;
    }

    case MSP_SET_RTC:
      m.readU32(); // secs: ignore
      m.readU16(); // msecs: ignore
      break;       // RTC not supported, accept and ignore

    default:
      r.result = -1;
      break;
  }
  if (m.decodeError || (configWrite && m.remain() != 0))
  {
    r.result = -1;
    r.len = 0;
    return;
  }
  if (staged && r.result >= 0)
  {
    if (!Utils::configurationValid(*staged))
    {
      r.result = -1;
      r.len = 0;
      return;
    }
    _model.config = *staged;
    if (needsSanitize) _model.sanitize();
    if (needsReload) _model.reload();
    for (size_t i = 0; i < 5; ++i)
      if (events[i]) _model.notifyConfigChange(static_cast<ModelChangeEvent>(i));
    if (needsReboot) _model.setRebootRequired();
  }

}

void MspProcessor::processEsc4way()
{
#if defined(ESPFC_MULTI_CORE) && defined(ESPFC_FREE_RTOS)
  timer_pause(TIMER_GROUP_0, TIMER_0);
#endif
  esc4wayProcess(getSerialPort());
  processRestart();
}

void MspProcessor::processRestart()
{
  Hardware::restart(_model);
}

void MspProcessor::serializeFlashData(MspResponse& r, uint32_t address, const uint16_t size, bool useLegacyFormat,
                                      bool allowCompression)
{
#ifdef USE_FLASHFS
  (void)allowCompression; // not supported

  const uint32_t allowedToRead = r.remain() - 16;
  const uint32_t flashfsSize = flashfsGetSize();

  if (address > flashfsSize)
  {
    r.result = -1;
    return;
  }

  uint16_t readLen = std::min(std::min((uint32_t)size, allowedToRead), flashfsSize - address);

  r.writeU32(address);

  uint16_t* readLenPtr = (uint16_t*)&r.data[r.len];
  if (!useLegacyFormat)
  {
    // new format supports variable read lengths
    r.writeU16(readLen);
    r.writeU8(0); // NO_COMPRESSION
  }

  const size_t bytesRead = flashfsReadAbs(address, &r.data[r.len], readLen);
  r.advance(bytesRead);

  if (!useLegacyFormat)
  {
    // update the 'read length' with the actual amount read from flash.
    *readLenPtr = bytesRead;
  }
  else
  {
    // pad the buffer with zeros
    // for (int i = bytesRead; i < allowedToRead; i++) r.writeU8(0);
  }
#endif
}

void MspProcessor::sendResponse(MspResponse& r, Stream::ReadWritable& s)
{
  debugResponse(r);
  uint8_t buff[256];
  size_t len = r.serialize(buff, sizeof(buff));
  s.write(buff, len);
}

void MspProcessor::requestCurrentMeter(Stream::ReadWritable& s)
{
  // Betaflight CURRENT_METER_MSP polls MSP_ANALOG at 10 Hz. MSPv1 request
  // frame with an empty payload: "$M<", size=0, cmd=MSP_ANALOG, XOR checksum.
  const uint8_t cmd =
      static_cast<uint8_t>(
          MSP_ANALOG);

  const uint8_t frame[] = {
      static_cast<uint8_t>('$'),
      static_cast<uint8_t>('M'),
      static_cast<uint8_t>('<'),
      0,
      cmd,
      cmd};

  s.write(
      frame,
      sizeof(frame));
}

void MspProcessor::postCommand()
{
  if (!_postCommand) return;
  auto cb = _postCommand;
  _postCommand = {};
  cb();
}

bool MspProcessor::debugSkip(uint8_t cmd)
{
  // return true;
  // return false;
  if (cmd == MSP_STATUS) return true;
  if (cmd == MSP_STATUS_EX) return true;
  if (cmd == MSP_BOXNAMES) return true;
  if (cmd == MSP_ANALOG) return true;
  if (cmd == MSP_ATTITUDE) return true;
  if (cmd == MSP_ALTITUDE) return true;
  if (cmd == MSP_RC) return true;
  if (cmd == MSP_RAW_IMU) return true;
  if (cmd == MSP_MOTOR) return true;
  if (cmd == MSP_SERVO) return true;
  if (cmd == MSP_BATTERY_STATE) return true;
  if (cmd == MSP_VOLTAGE_METERS) return true;
  if (cmd == MSP_CURRENT_METERS) return true;
  return false;
}

void MspProcessor::debugMessage(const MspMessage& m)
{
  if (debugSkip(m.cmd)) return;
  auto* dev = _model.getSerialStream(SERIAL_FUNCTION_TELEMETRY_HOTT);
  if (!dev) return;

  Stream::Printer s{*dev};

  s.print(m.dir == MSP_TYPE_REPLY ? '>' : '<');
  s.print(m.cmd);
  s.print('.');
  s.print(m.expected);
  s.print(' ');
  for (size_t i = 0; i < m.expected; i++)
  {
    s.print(m.buffer[i], HEX);
    s.print(' ');
  }
  s.println();
}

void MspProcessor::debugResponse(const MspResponse& r)
{
  if (debugSkip(r.cmd)) return;
  auto* dev = _model.getSerialStream(SERIAL_FUNCTION_TELEMETRY_HOTT);
  if (!dev) return;

  Stream::Printer s{*dev};

  s.print(r.result == 1 ? '>' : (r.result == -1 ? '!' : '@'));
  s.print(r.cmd);
  s.print('.');
  s.print(r.len);
  s.print(' ');
  for (size_t i = 0; i < r.len; i++)
  {
    s.print(r.data[i], HEX);
    s.print(' ');
  }
  s.println();
}

} // namespace Espfc::Connect
