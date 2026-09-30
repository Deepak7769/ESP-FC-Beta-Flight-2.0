#pragma once

#include "Device/InputCRSF.h"
#include "Device/InputDevice.h"
#include "Device/InputIBUS.hpp"
#include "Device/InputPPM.h"
#include "Device/InputSBUS.h"
#include "Model.h"
#include "TelemetryManager.h"
#if defined(ESPFC_ESPNOW)
#include "Device/InputEspNow.h"
#endif

namespace Espfc {

enum FailsafeChannelMode
{
  FAILSAFE_MODE_AUTO,
  FAILSAFE_MODE_HOLD,
  FAILSAFE_MODE_SET,
  FAILSAFE_MODE_INVALID
};

enum InputPwmRange
{
  PWM_RANGE_MIN = 1000,
  PWM_RANGE_MID = 1500,
  PWM_RANGE_MAX = 2000
};

class Input
{
public:
  Input(Model& model, TelemetryManager& telemetry);

  int begin();
  int reload(ModelChangeEvent event);
  int update();

  int16_t getFailsafeValue(uint8_t c);
  void setInput(Axis i, float v, bool newFrame, bool noFilter = false);

  InputStatus readInputs();
  void processInputs();

  bool failsafe(InputStatus status);
  void failsafeIdle();
  void failsafeStage1();
  void failsafeStage2();
  void filterInputs(InputStatus status);

  void updateFrameRate();
  Device::InputDevice* getInputDevice();

private:
  Model& _model;
  TelemetryManager& _telemetry;
  Device::InputDevice* _device;
  Utils::Filter _filter[INPUT_CHANNELS];
  Device::InputPPM _ppm;
  Device::InputIBUS _ibus;
  Device::InputSBUS _sbus;
  Device::InputCRSF _crsf;
#if defined(ESPFC_ESPNOW)
  Device::InputEspNow _espnow;
#endif

 static constexpr uint32_t TENTH_TO_US =
    100000UL;

// Require 500 ms of continuously healthy receiver data
// before declaring the receiver recovered/ready.
static constexpr uint32_t RX_RECOVERY_US =
    500000UL;

// A gap of 200 ms breaks receiver qualification.
// This matches the beginning of the existing Stage-1
// receiver-loss window.
static constexpr uint32_t RX_RECOVERY_GAP_US =
    2UL * TENTH_TO_US;

static constexpr uint32_t FRAME_TIME_DEFAULT_US =
    23000;
};

} // namespace Espfc
