#ifndef _ESPFC_MODEL_H_
#define _ESPFC_MODEL_H_

#include <cstddef>
#include <cstdint>
#include <functional>
#include <tuple>
#include <EscDriver.h>
#include "ModelConfig.h"
#include "ModelState.h"
#include "Stream/ReadWritable.hpp"
#include "Utils/Storage.h"
#include "Utils/Logger.hpp"
#include "Utils/Math.hpp"

namespace Espfc {

enum ModelChangeEvent
{
  MODEL_CHANGE_FILTER,
  MODEL_CHANGE_PID,
  MODEL_CHANGE_RATES,
  MODEL_CHANGE_ACCEL,
  MODEL_CHANGE_INPUT,
};

class Model
{
  public:
    Model()
    {
      initialize();
    }

    void initialize()
    {
      config = ModelConfig();
      #ifdef UNIT_TEST
      state = ModelState(); // FIXME: causes board wdt reset
      #endif
      //config.brobot();
    }

    bool isModeActive(FlightMode mode) const
    {
      return state.mode.mask & (1 << mode);
    }

    bool hasChanged(FlightMode mode) const
    {
      return (state.mode.mask & (1 << mode)) != (state.mode.maskPrev & (1 << mode));
    }

    void clearMode(FlightMode mode)
    {
      state.mode.maskPrev |= state.mode.mask & (1 << mode);
      state.mode.mask &= ~(1 << mode);
    }

    void updateModes(uint32_t mask)
    {
      state.mode.maskPrev = state.mode.mask;
      state.mode.mask = mask;
    }

    bool isSwitchActive(FlightMode mode) const
    {
      return state.mode.maskSwitch & (1 << mode);
    }

    void updateSwitchActive(uint32_t mask)
    {
      state.mode.maskSwitch = mask;
    }

    void disarm(DisarmReason r)
    {
      state.mode.disarmReason = r;
      clearMode(MODE_ARMED);
      clearMode(MODE_AIRMODE);
      state.appQueue.send(Event(EVENT_DISARM));
    }

    bool isFeatureActive(Feature feature) const
    {
      return config.featureMask & feature;
    }

    bool isAirModeActive() const
    {
      return isModeActive(MODE_AIRMODE);// || isFeatureActive(FEATURE_AIRMODE);
    }

    bool isThrottleLow() const
    {
      return state.input.us[AXIS_THRUST] < config.input.minCheck;
    }

    bool blackboxEnabled() const
    {
      // serial or flash
      return (config.blackbox.dev == BLACKBOX_DEV_SERIAL || config.blackbox.dev == BLACKBOX_DEV_FLASH);
    }

    bool gyroActive() const /* IRAM_ATTR */
    {
      return state.gyro.present && config.gyro.dev != GYRO_NONE;
    }

    bool gpsActive() const /* IRAM_ATTR */
    {
      return state.gps.present;
    }

    bool accelActive() const
    {
      return state.accel.present && config.accel.dev != GYRO_NONE;
    }

    bool magActive() const
    {
      return state.mag.present && config.mag.dev != MAG_NONE;
    }

    bool baroActive() const
    {
      return state.baro.present && config.baro.dev != BARO_NONE;
    }

    bool calibrationActive() const
    {
      return state.accel.calibrationState != CALIBRATION_IDLE || state.gyro.calibrationState != CALIBRATION_IDLE || state.mag.calibrationState != CALIBRATION_IDLE;
    }

    void calibrateGyro()
    {
      state.gyro.calibrationState = CALIBRATION_START;
      if(accelActive())
      {
        state.accel.calibrationState = CALIBRATION_START;
      }
    }

    void calibrateMag()
    {
      state.mag.calibrationState = CALIBRATION_START;
    }

    void finishCalibration()
    {
      if(state.gyro.calibrationState == CALIBRATION_SAVE)
      {
        //save();
        state.buzzer.push(BUZZER_GYRO_CALIBRATED);
        logger.info().log("GYRO BIAS").log(Utils::toDeg(state.gyro.bias.x)).log(Utils::toDeg(state.gyro.bias.y)).logln(Utils::toDeg(state.gyro.bias.z));
      }
      if(state.accel.calibrationState == CALIBRATION_SAVE)
      {
        save();
        logger.info().log("ACCEL BIAS").log(state.accel.bias.x).log(state.accel.bias.y).logln(state.accel.bias.z);
      }
      if(state.mag.calibrationState == CALIBRATION_SAVE)
      {
        save();
        logger.info().log("MAG BIAS").log(state.mag.calibrationOffset.x).log(state.mag.calibrationOffset.y).logln(state.mag.calibrationOffset.z);
        logger.info().log("MAG SCALE").log(state.mag.calibrationScale.x).log(state.mag.calibrationScale.y).logln(state.mag.calibrationScale.z);
      }
    }

    bool armingDisabled() const /* IRAM_ATTR */
    {
#if defined(ESPFC_DEV_PRESET_UNSAFE_ARMING)
      return false;
#warning "Danger macro used ESPFC_DEV_PRESET_UNSAFE_ARMING"
#else
     return
    state.mode.armingDisabledFlags != 0 ||
    state.pinConflict;
#endif
    }

    void setArmingDisabled(ArmingDisabledFlags flag, bool value)
    {
      if(value) state.mode.armingDisabledFlags |= flag;
      else state.mode.armingDisabledFlags &= ~flag;
    }

    bool getArmingDisabled(ArmingDisabledFlags flag)
    {
      return state.mode.armingDisabledFlags & flag;
    }

void setOutputSaturated(bool val)
{
  state.output.saturated = val;

for(size_t i = 0; i < AXIS_COUNT_RPY; i++)
{
  state.innerPid[i].outputSaturated = val;
}

  // Prevent vertical I-term accumulation while the mixer
  // has no additional actuator authority.
  state.innerPid[AXIS_THRUST].outputSaturated = val;
}
    bool areMotorsRunning() const
    {
      size_t count = state.currentMixer.count;
      for(size_t i = 0; i < count; i++)
      {
        if(config.output.channel[i].servo) continue;
        if(state.output.disarmed[i] != config.output.minCommand) return true;
        //if(state.output.us[i] != config.output.minCommand) return true;
      }
      return false;
    }

    void inline setDebug(DebugMode mode, size_t index, int16_t value)
    {
      if(index >= 8) return;
      if(config.debug.mode != mode) return;
      state.debug[index] = value;
    }

    void setGpsHome(bool force = false)
    {
      if(force || (state.gps.fix && state.gps.numSats >= config.gps.minSats))
      {
        if(!state.gps.homeSet || !config.gps.setHomeOnce)
        {
          state.gps.location.home = state.gps.location.raw;
          state.gps.homeSet = true;
        }
      }
    }

    Stream::ReadWritable * getSerialStream(SerialPort i)
    {
      return state.serial[i].stream;
    }

    Stream::ReadWritable * getSerialStream(SerialFunction sf)
    {
      for(size_t i = 0; i < SERIAL_UART_COUNT; i++)
      {
        if(config.serial[i].functionMask & sf) return state.serial[i].stream;
      }
      return nullptr;
    }

    int getSerialIndex(SerialPortId id)
    {
      switch(id)
      {
#ifdef ESPFC_SERIAL_0
        case SERIAL_ID_UART_1: return SERIAL_UART_0;
#endif
#ifdef ESPFC_SERIAL_1
        case SERIAL_ID_UART_2: return SERIAL_UART_1;
#endif
#ifdef ESPFC_SERIAL_2
        case SERIAL_ID_UART_3: return SERIAL_UART_2;
#endif
#ifdef ESPFC_SERIAL_USB
        case SERIAL_ID_USB_VCP: return SERIAL_USB;
#endif
#ifdef ESPFC_SERIAL_SOFT_0
        case SERIAL_ID_SOFTSERIAL_1: return SERIAL_SOFT_0;
#endif
        default: break;
      }
      return -1;
    }

    uint16_t getRssi() const
    {
      size_t channel = config.input.rssiChannel;
      if(channel < 4 || channel > state.input.channelCount) return 0;
      float value = state.input.ch[channel - 1];
      return std::clamp<uint16_t>(lrintf(Utils::map(value, -1.0f, 1.0f, 0.0f, 1023.0f)), 0, 1023);
    }

    int load()
    {
      logger.begin();
      #ifndef UNIT_TEST
      _storage.begin();
      logger.info().log("F_CPU").logln(F_CPU);
      _storageResult = _storage.load(config);
      logStorageResult();
      #endif
      postLoad();
      return 1;
    }

    void save()
    {
      preSave();
      #ifndef UNIT_TEST
      _storageResult = _storage.save(config);
      logStorageResult();
      #endif
    }

    void reload()
    {
      begin();
    }

    void setRebootRequired()
    {
      state.rebootRequired = true;
      setArmingDisabled(ARMING_DISABLED_REBOOT_REQUIRED, true);
    }

    bool getRebootRequired() const
    {
      return state.rebootRequired;
    }

    void calculateSimplifiedPids(const SimplifiedTuningConfig& s, PidConfig out[3]) const
    {
      // ESP-FC compile-time PID defaults for roll/pitch/yaw (no D-Max on this target)
      static const PidConfig def[3] = {
        { 45, 80, 30, 110 },
        { 47, 84, 34, 115 },
        { 45, 80,  0, 110 },
      };
      if (s.pidsMode == SIMPLIFIED_TUNING_OFF) return;
      const float master = s.masterMultiplier * 0.01f;
      const float pi = s.piGain * 0.01f;
      const float d = s.dGain * 0.01f;
      const float ff = s.ffGain * 0.01f;
      const float ig = s.iGain * 0.01f;
      for (int axis = FC_PID_ROLL; axis <= std::clamp<int>(s.pidsMode, FC_PID_ROLL, FC_PID_YAW); axis++)
      {
        const float pitchD = (axis == FC_PID_PITCH) ? s.rollPitchRatio * 0.01f : 1.0f;
        const float pitchPi = (axis == FC_PID_PITCH) ? s.pitchPiGain * 0.01f : 1.0f;
        out[axis].P = std::clamp<long>(lrintf(def[axis].P * master * pi * pitchPi), 0L, SIMPLIFIED_PID_GAIN_MAX);
        out[axis].I = std::clamp<long>(lrintf(def[axis].I * master * pi * ig * pitchPi), 0L, SIMPLIFIED_PID_GAIN_MAX);
        out[axis].D = std::clamp<long>(lrintf(def[axis].D * master * d * pitchD), 0L, SIMPLIFIED_PID_GAIN_MAX);
        out[axis].F = std::clamp<long>(lrintf(def[axis].F * master * pitchPi * ff), 0L, SIMPLIFIED_F_GAIN_MAX);
      }
    }

    void calculateSimplifiedDtermFilters(uint8_t mult, int16_t& lpf1, int16_t& lpf2, int16_t& dynMin, int16_t& dynMax) const
    {
      if (dynMin)
      {
        dynMin = std::clamp<int>(SIMPLIFIED_DTERM_LPF1_DYN_MIN_HZ * mult / 100, 0, SIMPLIFIED_DYN_LPF_MAX_HZ);
        dynMax = std::clamp<int>(SIMPLIFIED_DTERM_LPF1_DYN_MAX_HZ * mult / 100, 0, SIMPLIFIED_DYN_LPF_MAX_HZ);
      }
      if (lpf1) lpf1 = std::clamp<int>(SIMPLIFIED_DTERM_LPF1_DYN_MIN_HZ * mult / 100, 0, SIMPLIFIED_DYN_LPF_MAX_HZ);
      if (lpf2) lpf2 = std::clamp<int>(SIMPLIFIED_DTERM_LPF2_HZ * mult / 100, 0, SIMPLIFIED_LPF_MAX_HZ);
    }

    void calculateSimplifiedGyroFilters(uint8_t mult, int16_t& lpf1, int16_t& lpf2, int16_t& dynMin, int16_t& dynMax) const
    {
      if (dynMin)
      {
        dynMin = std::clamp<int>(SIMPLIFIED_GYRO_LPF1_DYN_MIN_HZ * mult / 100, 0, SIMPLIFIED_DYN_LPF_MAX_HZ);
        dynMax = std::clamp<int>(SIMPLIFIED_GYRO_LPF1_DYN_MAX_HZ * mult / 100, 0, SIMPLIFIED_DYN_LPF_MAX_HZ);
      }
      if (lpf1) lpf1 = std::clamp<int>(SIMPLIFIED_GYRO_LPF1_DYN_MIN_HZ * mult / 100, 0, SIMPLIFIED_DYN_LPF_MAX_HZ);
      if (lpf2) lpf2 = std::clamp<int>(SIMPLIFIED_GYRO_LPF2_HZ * mult / 100, 0, SIMPLIFIED_LPF_MAX_HZ);
    }

    std::tuple<bool, bool, bool> validateSimplifiedTuning() const
    {
      const auto& s = config.simplifiedTuning;
      
      const auto& pids = config.pid;
      PidConfig tmp[3] = {pids[0], pids[1], pids[2]};

      calculateSimplifiedPids(s, tmp);
      bool pidOk = tmp[0].P == pids[0].P && tmp[0].I == pids[0].I &&
                   tmp[0].D == pids[0].D && tmp[0].F == pids[0].F &&
                   tmp[1].P == pids[1].P && tmp[1].I == pids[1].I &&
                   tmp[1].D == pids[1].D && tmp[1].F == pids[1].F &&
                   tmp[2].P == pids[2].P && tmp[2].I == pids[2].I &&
                   tmp[2].D == pids[2].D && tmp[2].F == pids[2].F;

      const auto& gyro = config.gyro;
      int16_t glpf1 = gyro.filter.freq;
      int16_t glpf2 = gyro.filter2.freq;
      int16_t gmin = gyro.dynLpfFilter.cutoff;
      int16_t gmax = gyro.dynLpfFilter.freq;
      if (s.gyroFilter) calculateSimplifiedGyroFilters(s.gyroFilterMultiplier, glpf1, glpf2, gmin, gmax);
      bool gyroOk = glpf1 == gyro.filter.freq && glpf2 == gyro.filter2.freq &&
                    gmin == gyro.dynLpfFilter.cutoff && gmax == gyro.dynLpfFilter.freq;

      const auto& dterm = config.dterm;
      int16_t dlpf1 = dterm.filter.freq;
      int16_t dlpf2 = dterm.filter2.freq;
      int16_t dmin = dterm.dynLpfFilter.cutoff;
      int16_t dmax = dterm.dynLpfFilter.freq;
      if (s.dtermFilter) calculateSimplifiedDtermFilters(s.dtermFilterMultiplier, dlpf1, dlpf2, dmin, dmax);
      bool dtermOk = dlpf1 == dterm.filter.freq && dlpf2 == dterm.filter2.freq &&
                     dmin == dterm.dynLpfFilter.cutoff && dmax == dterm.dynLpfFilter.freq;

      return std::make_tuple(pidOk, gyroOk, dtermOk);
    }

    void reset()
{
  initialize();
  postLoad();
  reload();
}
    bool validatePinResources()
{
  bool used[64] = {};
  bool conflict = false;

  auto reservePin =
      [&](int pin)
  {
    if (pin < 0)
    {
      return;
    }

    if (pin >= 64)
    {
      conflict = true;

      logger.err()
          .log("PIN RANGE ")
          .logln(pin);

      return;
    }

    if (used[pin])
    {
      conflict = true;

      logger.err()
          .log("PIN CONFLICT ")
          .logln(pin);

      return;
    }

    used[pin] = true;
  };

  // -----------------------------
  // MOTOR / SERVO OUTPUTS
  // -----------------------------
  for (size_t i = 0;
       i < ESPFC_OUTPUT_COUNT;
       i++)
  {
    reservePin(
        config.pin[
            PIN_OUTPUT_0 + i]);
  }

#ifdef ESPFC_INPUT
  if (isFeatureActive(
          FEATURE_RX_PPM))
  {
    reservePin(
        config.pin[
            PIN_INPUT_RX]);
  }
#endif

  reservePin(
      config.pin[
          PIN_BUTTON]);

  reservePin(
      config.pin[
          PIN_BUZZER]);

  reservePin(
      config.pin[
          PIN_LED_BLINK]);

  // -----------------------------
  // UART
  // -----------------------------

#ifdef ESPFC_SERIAL_0
  if (config.serial[
          SERIAL_UART_0]
          .functionMask)
  {
    reservePin(
        config.pin[
            PIN_SERIAL_0_TX]);

    reservePin(
        config.pin[
            PIN_SERIAL_0_RX]);
  }
#endif

#ifdef ESPFC_SERIAL_1
  if (config.serial[
          SERIAL_UART_1]
          .functionMask)
  {
    reservePin(
        config.pin[
            PIN_SERIAL_1_TX]);

    reservePin(
        config.pin[
            PIN_SERIAL_1_RX]);
  }
#endif

#ifdef ESPFC_SERIAL_2
  if (config.serial[
          SERIAL_UART_2]
          .functionMask)
  {
    reservePin(
        config.pin[
            PIN_SERIAL_2_TX]);

    reservePin(
        config.pin[
            PIN_SERIAL_2_RX]);
  }
#endif

  // -----------------------------
  // I2C
  // -----------------------------

#ifdef ESPFC_I2C_0

  reservePin(
      config.pin[
          PIN_I2C_0_SCL]);

  reservePin(
      config.pin[
          PIN_I2C_0_SDA]);

#endif

  // -----------------------------
  // SPI
  // -----------------------------

#ifdef ESPFC_SPI_0

  reservePin(
      config.pin[
          PIN_SPI_0_SCK]);

  reservePin(
      config.pin[
          PIN_SPI_0_MOSI]);

  reservePin(
      config.pin[
          PIN_SPI_0_MISO]);

  reservePin(
      config.pin[
          PIN_SPI_CS0]);

  reservePin(
      config.pin[
          PIN_SPI_CS1]);

  reservePin(
      config.pin[
          PIN_SPI_CS2]);

#endif

  // -----------------------------
  // ADC
  // -----------------------------

#ifdef ESPFC_ADC_0

  reservePin(
      config.pin[
          PIN_INPUT_ADC_0]);

#endif

#ifdef ESPFC_ADC_1

  reservePin(
      config.pin[
          PIN_INPUT_ADC_1]);

#endif

  state.pinConflict =
      conflict;

  return !conflict;
}

    void sanitize()
    {
            // -------------------------------------------------
      // FAILSAFE CONFIG SANITIZATION
      // -------------------------------------------------

      if (config.failsafe.procedure >=
          FAILSAFE_PROCEDURE_COUNT)
      {
        config.failsafe.procedure =
            FAILSAFE_PROCEDURE_DROP;
      }
      // for spi gyro allow full speed mode
      if (state.gyro.dev && state.gyro.dev->getBus()->isSPI())
      {
        state.gyro.rate = Utils::alignToClock(state.gyro.clock, ESPFC_GYRO_SPI_RATE_MAX);
      }
      else
      {
        state.gyro.rate = Utils::alignToClock(state.gyro.clock, ESPFC_GYRO_I2C_RATE_MAX);
        // first usage
        if(_storageResult == STORAGE_ERR_BAD_MAGIC || _storageResult == STORAGE_ERR_BAD_SIZE || _storageResult == STORAGE_ERR_BAD_VERSION)
        {
          config.loopSync = 1;
        }
      }

      int loopSyncMax = 1;
      //if(config.mag.dev != MAG_NONE || config.baro.dev != BARO_NONE) loopSyncMax /= 2;

      config.loopSync = std::max((int)config.loopSync, loopSyncMax);
      config.loopSync =
    std::max<int8_t>(
        config.loopSync,
        1);

config.mixerSync =
    std::max<int8_t>(
        config.mixerSync,
        1);

config.customMixerCount =
    std::clamp<int>(
        config.customMixerCount,
        0,
        OUTPUT_CHANNELS);

// eRPM -> mechanical RPM conversion divides by pole-pairs.
config.output.motorPoles =
    std::max<int8_t>(
        config.output.motorPoles,
        2);

config.gyro.rpmFilter.harmonics =
    std::min<uint8_t>(
        config.gyro.rpmFilter.harmonics,
        RPM_FILTER_HARMONICS_MAX);

config.gyro.dynamicFilter.count =
    std::min<uint8_t>(
        config.gyro.dynamicFilter.count,
        DYN_NOTCH_COUNT_MAX);

config.gyro.dynamicFilter.q =
    std::max<int16_t>(
        config.gyro.dynamicFilter.q,
        1);

config.gyro.dynamicFilter.min_freq =
    std::max<int16_t>(
        config.gyro.dynamicFilter.min_freq,
        1);

config.gyro.dynamicFilter.max_freq =
    std::max(
        config.gyro.dynamicFilter.max_freq,
        config.gyro.dynamicFilter.min_freq);

config.controller.tpaScale =
    std::clamp<int8_t>(
        config.controller.tpaScale,
        0,
        100);

config.controller.tpaMode =
    std::clamp<int8_t>(
        config.controller.tpaMode,
        0,
        1);

config.controller.tpaBreakpoint =
    std::clamp<int16_t>(
        config.controller.tpaBreakpoint,
        1000,
        1999);

// Anti-Gravity uses bytes that were legacy tail padding. Treat an absent tag
// as an old EEPROM image and initialize those bytes deterministically instead
// of accepting arbitrary historical padding as a gain value.
if (config.antiGravityConfigTag !=
    ModelConfig::ANTI_GRAVITY_CONFIG_TAG)
{
  config.antiGravityGain =
      ModelConfig::ANTI_GRAVITY_GAIN_DEFAULT;

  // MODE_ANTI_GRAVITY was previously outside MODE_COUNT. If a legacy EEPROM
  // happened to contain that numeric value in an ignored/stale row, do not
  // reinterpret it as a newly-live switch or linked mode after this upgrade.
  for (size_t i = 0;
       i < ACTUATOR_CONDITIONS;
       ++i)
  {
    auto& condition =
        config.conditions[i];

    if (condition.id ==
            MODE_ANTI_GRAVITY ||
        condition.linkId ==
            MODE_ANTI_GRAVITY)
    {
      condition =
          ActuatorCondition{};
    }
  }

  config.antiGravityConfigTag =
      ModelConfig::ANTI_GRAVITY_CONFIG_TAG;
}

config.antiGravityGain =
    std::clamp<uint8_t>(
        config.antiGravityGain,
        0,
        250);

// Receiver/control settings are persisted and can be edited over MSP/CLI.
// Keep them inside ranges that preserve the arming and smoothing invariants.
if (config.input.ppmMode != PPM_MODE_NORMAL &&
    config.input.ppmMode != PPM_MODE_INVERTED)
{
  config.input.ppmMode =
      PPM_MODE_NORMAL;
}

switch (config.input.serialRxProvider)
{
  case SERIALRX_SBUS:
  case SERIALRX_IBUS:
  case SERIALRX_CRSF:
    break;

  default:
    config.input.serialRxProvider =
        SERIALRX_SBUS;
    break;
}

config.input.minCheck =
    std::clamp<int16_t>(
        config.input.minCheck,
        1000,
        1500);

config.input.maxCheck =
    std::clamp<int16_t>(
        config.input.maxCheck,
        1500,
        2000);

if (config.input.minCheck >=
    config.input.maxCheck)
{
  config.input.minCheck =
      1050;

  config.input.maxCheck =
      1900;
}

config.input.minRc =
    std::clamp<int16_t>(
        config.input.minRc,
        750,
        1000);

config.input.midRc =
    std::clamp<int16_t>(
        config.input.midRc,
        1200,
        1800);

config.input.maxRc =
    std::clamp<int16_t>(
        config.input.maxRc,
        2000,
        2250);

config.input.deadband =
    std::clamp<int8_t>(
        config.input.deadband,
        0,
        100);

config.input.airModeActivateThreshold =
    std::clamp<int8_t>(
        config.input.airModeActivateThreshold,
        0,
        100);

config.input.filterAutoFactor =
    std::clamp<int8_t>(
        config.input.filterAutoFactor,
        0,
        100);

config.input.filterAutoThrottleFactor =
    std::clamp<int8_t>(
        config.input.filterAutoThrottleFactor,
        0,
        100);

// RateType currently has five concrete algorithms: 0..4.
config.input.rateType =
    std::clamp<int8_t>(
        config.input.rateType,
        0,
        4);

config.level.angleLimit =
    std::clamp<int8_t>(
        config.level.angleLimit,
        0,
        90);

config.level.rateLimit =
    std::clamp<int16_t>(
        config.level.rateLimit,
        1,
        1998);

// Keep the persisted AltHold thrust-center contract identical to the range
// used by Controller::reloadPid(). This also keeps the LAND touchdown thrust
// reference from being derived from an impossible hover-center value.
config.altHold.itermCenter =
    std::clamp<uint8_t>(
        config.altHold.itermCenter,
        10,
        60);

config.altHold.itermRange =
    std::clamp<uint8_t>(
        config.altHold.itermRange,
        10,
        60);

config.arming.smallAngle =
    std::min<uint8_t>(
        config.arming.smallAngle,
        180);

if (config.debug.mode < DEBUG_NONE ||
    config.debug.mode >= DEBUG_COUNT)
{
  config.debug.mode =
      DEBUG_NONE;
}

config.iterm.limit =
    std::clamp<int8_t>(
        config.iterm.limit,
        0,
        100);

if (config.vbat.resDiv == 0)
{
  config.vbat.resDiv = 1;
}

switch (config.ibat.source)
{
  case CURRENT_METER_NONE:
  case CURRENT_METER_ADC:
  case CURRENT_METER_MSP:
    break;

  default:
    config.ibat.source =
        CURRENT_METER_NONE;
    break;
}

for (size_t i = 0; i < 3; i++)
{
  config.input.rateLimit[i] =
      std::clamp<int16_t>(
          config.input.rateLimit[i],
          1,
          1998);
}

// Keep persisted actuator pulse settings inside the same broad validity window
// used by mature RC/motor stacks. Invalid stored/MSP values must not expand
// into out-of-range PWM commands or invert the available throttle range.
config.output.minCommand =
    std::clamp<int16_t>(
        config.output.minCommand,
        750,
        2250);

config.output.maxThrottle =
    std::clamp<int16_t>(
        config.output.maxThrottle,
        750,
        2250);

if (config.output.minCommand >=
    config.output.maxThrottle)
{
  config.output.minCommand =
      1000;

  config.output.maxThrottle =
      2000;
}

config.output.motorIdle =
    std::clamp<int16_t>(
        config.output.motorIdle,
        0,
        2000);

const int32_t maxMotorIdle =
    std::max<int32_t>(
        0,
        static_cast<int32_t>(
            config.output.maxThrottle -
            config.output.minCommand) *
            10);

config.output.motorIdle =
    static_cast<int16_t>(
        std::min<int32_t>(
            config.output.motorIdle,
            maxMotorIdle));

if (config.output.servoRate != 0)
{
  config.output.servoRate =
      std::clamp<int16_t>(
          config.output.servoRate,
          50,
          498);
}

for (size_t i = 0;
     i < OUTPUT_CHANNELS;
     i++)
{
  auto& ch = config.output.channel[i];

  ch.min =
      std::clamp<int16_t>(
          ch.min,
          750,
          2250);

  ch.max =
      std::clamp<int16_t>(
          ch.max,
          750,
          2250);

  if (ch.min > ch.max)
  {
    std::swap(ch.min, ch.max);
  }

  ch.neutral =
      std::clamp(
          ch.neutral,
          ch.min,
          ch.max);
}
      state.loopRate = state.gyro.rate / config.loopSync;
   
      config.output.protocol =
    ESC_PROTOCOL_SANITIZE(config.output.protocol);



switch(config.output.protocol)
{
        case ESC_PROTOCOL_BRUSHED:
          config.output.async = true;
          break;
        case ESC_PROTOCOL_DSHOT150:
        case ESC_PROTOCOL_DSHOT300:
        case ESC_PROTOCOL_DSHOT600:
        case ESC_PROTOCOL_PROSHOT:
          config.output.async = false;
          break;
      }

      if(config.output.async)
      {
        // for async limit pwm rate
        switch(config.output.protocol)
        {
          case ESC_PROTOCOL_PWM:
            config.output.rate = std::clamp<int16_t>(config.output.rate, 50, 480);
            break;
          case ESC_PROTOCOL_ONESHOT125:
            config.output.rate = std::clamp<int16_t>(config.output.rate, 50, 2000);
            break;
          case ESC_PROTOCOL_ONESHOT42:
            config.output.rate = std::clamp<int16_t>(config.output.rate, 50, 4000);
            break;
          case ESC_PROTOCOL_BRUSHED:
          case ESC_PROTOCOL_MULTISHOT:
            config.output.rate = std::clamp<int16_t>(config.output.rate, 50, 8000);
            break;
          default:
            config.output.rate = std::clamp<int16_t>(config.output.rate, 50, 2000);
            break;
        }
      }
      else
      {
        // for synced and standard PWM limit loop rate and pwm pulse width
if(config.output.protocol == ESC_PROTOCOL_PWM)
{
  // Conventional PWM should stay comfortably below 500Hz.
  // 450Hz ceiling allows integer gyro divisors such as:
  // 4000 / 10 = 400Hz
  // 4000 / 9  = 444Hz
  constexpr int32_t PWM_SYNC_MAX_HZ = 450;

  if(state.loopRate > PWM_SYNC_MAX_HZ)
  {
    const int8_t minLoopSync =
        (int8_t)((state.gyro.rate + PWM_SYNC_MAX_HZ - 1) / PWM_SYNC_MAX_HZ);

    config.loopSync = std::max(config.loopSync, minLoopSync);
    state.loopRate = state.gyro.rate / config.loopSync;
  }

  // In synchronous PWM mode, actual motor update rate follows mixer/PID rate.
  config.output.rate = state.loopRate;
}
        // for onshot125 limit loop rate to 2kHz
        if(config.output.protocol == ESC_PROTOCOL_ONESHOT125 && state.loopRate > 2000)
        {
          config.loopSync = std::max(config.loopSync, (int8_t)((state.gyro.rate + 1999) / 2000)); // align loop rate to lower than 2000Hz
          state.loopRate = state.gyro.rate / config.loopSync;
        }
      }
       
      // sanitize throttle and motor limits
      if(config.output.throttleLimitType < 0 || config.output.throttleLimitType >= THROTTLE_LIMIT_TYPE_MAX) {
        config.output.throttleLimitType = THROTTLE_LIMIT_TYPE_NONE;
      }

      if(config.output.throttleLimitPercent < 1 || config.output.throttleLimitPercent > 100) {
        config.output.throttleLimitPercent = 100;
      }

      if(config.output.motorLimit < 1 || config.output.motorLimit > 100) {
        config.output.motorLimit = 100;
      }

      // configure serial ports
      constexpr uint32_t serialFunctionAllowedMask = SERIAL_FUNCTION_MSP | SERIAL_FUNCTION_RX_SERIAL | SERIAL_FUNCTION_BLACKBOX | 
        SERIAL_FUNCTION_GPS | SERIAL_FUNCTION_TELEMETRY_FRSKY | SERIAL_FUNCTION_TELEMETRY_HOTT | SERIAL_FUNCTION_TELEMETRY_IBUS | SERIAL_FUNCTION_VTX_SMARTAUDIO;
      uint32_t featureAllowMask =  FEATURE_RX_PPM | FEATURE_RX_SERIAL | FEATURE_MOTOR_STOP | FEATURE_SOFTSERIAL | FEATURE_GPS |
        FEATURE_TELEMETRY | FEATURE_RX_SPI | FEATURE_ANTI_GRAVITY;// | FEATURE_AIRMODE;

      config.featureMask &= featureAllowMask;

      for(int i = 0; i < SERIAL_UART_COUNT; i++)
      {
        config.serial[i].functionMask &= serialFunctionAllowedMask;
      }
      validatePinResources();

      if (config.fusion.mode < FUSION_NONE ||
    config.fusion.mode >= FUSION_MAX)
{
  config.fusion.mode =
      FUSION_MAHONY;
}

if (config.debug.axis >= AXIS_COUNT_RPY)
{
  config.debug.axis =
      AXIS_ROLL;
}

      // only few beeper modes allowed
      config.buzzer.beeperMask &=
        1 << (BUZZER_GYRO_CALIBRATED - 1) |
        1 << (BUZZER_SYSTEM_INIT - 1) |
        1 << (BUZZER_RX_LOST - 1) |
        1 << (BUZZER_RX_SET - 1) |
        1 << (BUZZER_DISARMING - 1) |
        1 << (BUZZER_ARMING - 1) |
        1 << (BUZZER_BAT_LOW - 1);

        if(config.gyro.dynamicFilter.count > DYN_NOTCH_COUNT_MAX)
        {
          config.gyro.dynamicFilter.count = DYN_NOTCH_COUNT_MAX;
        }
    }

    void begin()
    {
      sanitize();

            // init timers
      // sample rate = clock / ( divider + 1)

      bool timersOk =
          true;

      timersOk &=
          state.gyro.timer.setRate(
              state.gyro.rate) != 0;

      const int accelRate =
          Utils::alignToClock(
              state.gyro.timer.rate,
              500);

      if (accelRate <= 0)
      {
        timersOk = false;
      }
      else
      {
        timersOk &=
            state.accel.timer.setRate(
                state.gyro.timer.rate,
                std::max<uint32_t>(
                    1u,
                    state.gyro.timer.rate /
                        static_cast<uint32_t>(
                            accelRate))) != 0;
      }

      timersOk &=
          state.loopTimer.setRate(
              state.gyro.timer.rate,
              config.loopSync) != 0;

      timersOk &=
          state.mixer.timer.setRate(
              state.loopTimer.rate,
              config.mixerSync) != 0;

      const int inputRate =
          Utils::alignToClock(
              state.gyro.timer.rate,
              1000);

      if (inputRate <= 0)
      {
        timersOk = false;
      }
      else
      {
        timersOk &=
            state.input.timer.setRate(
                state.gyro.timer.rate,
                std::max<uint32_t>(
                    1u,
                    state.gyro.timer.rate /
                        static_cast<uint32_t>(
                            inputRate))) != 0;
      }

      timersOk &=
          state.actuatorTimer.setRate(50) != 0;

      timersOk &=
          state.gyro.dynamicFilterTimer
              .setRate(50) != 0;

      timersOk &=
          state.telemetryTimer
              .setInterval(
                  static_cast<uint32_t>(
                      config.telemetryInterval) *
                  1000u) != 0;

      timersOk &=
          state.stats.timer.setRate(3) != 0;

      if (magActive())
      {
        timersOk &=
            state.mag.timer.setRate(
                state.mag.rate) != 0;
      }

      if (!timersOk)
      {
        setRebootRequired();
      }
          
      // ensure disarmed pulses
      for(size_t i = 0; i < OUTPUT_CHANNELS; i++)
      {
        state.output.disarmed[i] = config.output.channel[i].servo ? config.output.channel[i].neutral : config.output.minCommand; // ROBOT
      }

      state.buzzer.beeperMask = config.buzzer.beeperMask;

      state.customMixer = MixerConfig(config.customMixerCount, config.customMixes);

      // override temporary
      //state.telemetryTimer.setRate(100);
    }

    void postLoad()
    {
      // load current sensor calibration
      for(size_t i = 0; i < AXIS_COUNT_RPY; i++)
      {
        state.gyro.bias.set(i, config.gyro.bias[i] / 1000.0f);
        state.accel.bias.set(i, config.accel.bias[i] / 1000.0f);
        state.mag.calibrationOffset.set(i, config.mag.offset[i] / 10.0f);
        state.mag.calibrationScale.set(i, config.mag.scale[i] / 1000.0f);
      }
    }

    void preSave()
    {
      // store current sensor calibration
      for(size_t i = 0; i < AXIS_COUNT_RPY; i++)
      {
        config.gyro.bias[i] = lrintf(state.gyro.bias[i] * 1000.0f);
        config.accel.bias[i] = lrintf(state.accel.bias[i] * 1000.0f);
        config.mag.offset[i] = lrintf(state.mag.calibrationOffset[i] * 10.0f);
        config.mag.scale[i] = lrintf(state.mag.calibrationScale[i] * 1000.0f);
      }
    }

    ModelState state{};
    ModelConfig config;
    Utils::Logger logger;

    void logStorageResult()
    {
#ifndef UNIT_TEST
      switch(_storageResult)
      {
        case STORAGE_LOAD_SUCCESS:    logger.info().logln("EEPROM load ok"); break;
        case STORAGE_SAVE_SUCCESS:    logger.info().logln("EEPROM save ok"); break;
        case STORAGE_SAVE_ERROR:      logger.err().logln("EEPROM save failed"); break;
        case STORAGE_ERR_BAD_MAGIC:   logger.err().logln("EEPROM wrong magic"); break;
        case STORAGE_ERR_BAD_VERSION: logger.err().logln("EEPROM wrong version"); break;
        case STORAGE_ERR_BAD_SIZE:    logger.err().logln("EEPROM wrong size"); break;
        case STORAGE_NONE:
        default:
          logger.err().logln("EEPROM unknown result"); break;
      }
#endif
    }

    void notifyConfigChange(ModelChangeEvent event)
    {
      if (_onConfigChange) _onConfigChange(event);
    }

    void setConfigChangeListener(std::function<void(ModelChangeEvent)> listener)
    {
      _onConfigChange = listener;
    }

  private:
    #ifndef UNIT_TEST
    Utils::Storage _storage;
    #endif
    StorageResult _storageResult = STORAGE_NONE;

    std::function<void(ModelChangeEvent)> _onConfigChange{};
};

}

#endif
