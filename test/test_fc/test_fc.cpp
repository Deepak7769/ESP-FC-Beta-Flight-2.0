#include "Control/Actuator.h"
#include "Control/AssistedModeV2.h"
#include "Control/Controller.h"
#include "Control/Altitude.hpp"
#include "Connect/MspProcessor.hpp"
#include "Input.h"
#include "Device/InputPPM.h"
#include "TelemetryManager.h"
#include <Complementary.hpp>
#include "Control/Fusion.h"
#include "Sensor/BaroSensor.hpp"
#include "Sensor/VoltageSensor.hpp"
#include "Model.h"
#include "Output/Mixer.h"
#include "Utils/Timer.h"
#include "msp/msp_protocol.h"

#include <ArduinoFake.h>
#include <Gps.hpp>
#include <unity.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

using namespace fakeit;
using namespace Espfc;
using Espfc::Control::Actuator;
using Espfc::Control::Controller;
using Espfc::Control::Rates;
using Espfc::Utils::Timer;

static constexpr size_t ALTHOLD_PILOT_CHANNEL =
    static_cast<size_t>(
        ESPFC_ALTHOLD_V2_CENTERED_STICK_CHANNEL);

static_assert(
    ALTHOLD_PILOT_CHANNEL < AXIS_COUNT,
    "test AltHold pilot channel exceeds input channel count");
static void setHealthyAssistedEstimatorState(
    Model& model,
    uint32_t nowUs)
{
  // --------------------------------------------------
  // Fresh attitude estimator
  // --------------------------------------------------

  model.state.gyro.present =
      true;

  model.state.gyro.sampleValid =
      true;

  model.state.gyro.lastUpdateUs =
      nowUs;

  model.state.accel.present =
      true;

  model.state.attitude.healthy =
      true;

  model.state.attitude.lastUpdateUs =
      nowUs;

  model.state.attitude.quaternion =
      Quaternion(
          1.0f,
          0.0f,
          0.0f,
          0.0f);

  model.state.attitude.euler =
      VectorFloat(
          0.0f,
          0.0f,
          0.0f);

  // --------------------------------------------------
  // Fresh barometer
  // --------------------------------------------------

  model.config.baro.dev =
      BARO_BMP280;

  model.state.baro.present =
      true;

  model.state.baro.sampleValid =
      true;

  model.state.baro.lastUpdateUs =
      nowUs;

  model.state.altitude.lastUpdateUs =
      nowUs;

  // Assisted-mode tests represent a receiver frame that actually contains
  // the configured AltHold pilot channel unless a test overrides this.
  model.state.input.channelCount =
      std::max<size_t>(
          ALTHOLD_PILOT_CHANNEL + 1,
          static_cast<size_t>(
              AXIS_AUX_1 + 1));

  model.state.input.channelsValid =
      true;

  model.state.input.raw[
      ALTHOLD_PILOT_CHANNEL] =
      PWM_RANGE_MID;
}
/*void setUp(void)
{
  ArduinoFakeReset();
}*/

// void tearDown(void) {
// // clean stuff up here
// }

void test_timer_rate_100hz()
{
  Timer timer;
  timer.setRate(100);
  TEST_ASSERT_EQUAL_UINT32(100, timer.rate);
  TEST_ASSERT_EQUAL_UINT32(10000, timer.interval);
  TEST_ASSERT_EQUAL_UINT32(10000, timer.delta);
  TEST_ASSERT_FLOAT_WITHIN(0.000001f, 1.f / 100.f, timer.intervalf);
}

void test_timer_rate_100hz_div2()
{
  Timer timer;
  timer.setRate(100, 2);
  TEST_ASSERT_EQUAL_UINT32(50, timer.rate);
  TEST_ASSERT_EQUAL_UINT32(20000, timer.interval);
  TEST_ASSERT_EQUAL_UINT32(20000, timer.delta);
  TEST_ASSERT_FLOAT_WITHIN(0.000001f, 1.f / 50.f, timer.intervalf);
}

void test_timer_interval_10ms()
{
  Timer timer;
  timer.setInterval(10000);
  TEST_ASSERT_EQUAL_UINT32(100, timer.rate);
  TEST_ASSERT_EQUAL_UINT32(10000, timer.interval);
  TEST_ASSERT_EQUAL_UINT32(10000, timer.delta);
  TEST_ASSERT_FLOAT_WITHIN(0.000001f, 1.f / 100.f, timer.intervalf);
}

void test_timer_check()
{
  Timer timer;
  timer.setInterval(1000);

  TEST_ASSERT_EQUAL_UINT32(1000, timer.rate);
  TEST_ASSERT_EQUAL_UINT32(1000, timer.interval);

  TEST_ASSERT_TRUE(timer.check(1000));
  TEST_ASSERT_EQUAL_UINT32(1, timer.iteration);
  TEST_ASSERT_EQUAL_UINT32(1000, timer.delta);

  TEST_ASSERT_FALSE(timer.check(1500));
  TEST_ASSERT_EQUAL_UINT32(1, timer.iteration);
  TEST_ASSERT_EQUAL_UINT32(1000, timer.delta);

  TEST_ASSERT_TRUE(timer.check(2000));
  TEST_ASSERT_EQUAL_UINT32(2, timer.iteration);
  TEST_ASSERT_EQUAL_UINT32(1000, timer.delta);

  TEST_ASSERT_TRUE(timer.check(3000));
  TEST_ASSERT_EQUAL_UINT32(3, timer.iteration);
  TEST_ASSERT_EQUAL_UINT32(1000, timer.delta);

  TEST_ASSERT_FALSE(timer.check(3999));
  TEST_ASSERT_EQUAL_UINT32(3, timer.iteration);
  TEST_ASSERT_EQUAL_UINT32(1000, timer.delta);

  TEST_ASSERT_TRUE(timer.check(4050));
  TEST_ASSERT_EQUAL_UINT32(4, timer.iteration);
  TEST_ASSERT_EQUAL_UINT32(1050, timer.delta);
}

void test_timer_check_micros()
{
  When(Method(ArduinoFake(), micros)).Return(1000, 1500, 2000, 3000, 3999, 4050);

  Timer timer;
  timer.setInterval(1000);

  TEST_ASSERT_EQUAL_UINT32(1000, timer.rate);
  TEST_ASSERT_EQUAL_UINT32(1000, timer.interval);

  TEST_ASSERT_TRUE(timer.check());
  TEST_ASSERT_FALSE(timer.check());
  TEST_ASSERT_TRUE(timer.check());
  TEST_ASSERT_TRUE(timer.check());
  TEST_ASSERT_FALSE(timer.check());
  TEST_ASSERT_TRUE(timer.check());

  Verify(Method(ArduinoFake(), micros)).Exactly(6_Times);
}

void test_model_gyro_init_1k_256dlpf()
{
  Model model;
  model.state.gyro.clock = 8000;
  model.config.gyro.dlpf = GYRO_DLPF_256;
  model.config.loopSync = 1;
  model.config.mixerSync = 1;
  model.begin();

  TEST_ASSERT_EQUAL_INT32(8000, model.state.gyro.clock);
  TEST_ASSERT_EQUAL_INT32(2000, model.state.gyro.rate);
  TEST_ASSERT_EQUAL_INT32(2000, model.state.gyro.timer.rate);
  TEST_ASSERT_EQUAL_INT32(2000, model.state.loopRate);
  TEST_ASSERT_EQUAL_INT32(2000, model.state.loopTimer.rate);
  TEST_ASSERT_EQUAL_INT32(2000, model.state.mixer.timer.rate);
}

void test_model_gyro_init_1k_188dlpf()
{
  Model model;
  model.state.gyro.clock = 1000;
  model.config.gyro.dlpf = GYRO_DLPF_188;
  model.config.loopSync = 2;
  model.config.mixerSync = 2;
  model.begin();

  TEST_ASSERT_EQUAL_INT32(1000, model.state.gyro.clock);
  TEST_ASSERT_EQUAL_INT32(1000, model.state.gyro.rate);
  TEST_ASSERT_EQUAL_INT32(1000, model.state.gyro.timer.rate);
  TEST_ASSERT_EQUAL_INT32(500, model.state.loopRate);
  TEST_ASSERT_EQUAL_INT32(500, model.state.loopTimer.rate);
  TEST_ASSERT_EQUAL_INT32(250, model.state.mixer.timer.rate);
}

void test_model_inner_pid_init()
{
  Model model;
  model.state.gyro.clock = 1000;
  model.config.gyro.dlpf = GYRO_DLPF_256;
  model.config.loopSync = 1;
  model.config.mixerSync = 1;
  model.config.mixer.type = FC_MIXER_QUADX;
  model.config.pid[FC_PID_ROLL] = {.P = 100u, .I = 100u, .D = 100u, .F = 100};
  model.config.pid[FC_PID_PITCH] = {.P = 100u, .I = 100u, .D = 100u, .F = 100};
  model.config.pid[FC_PID_YAW] = {.P = 100u, .I = 100u, .D = 100u, .F = 100};
  model.begin();

  Control::Controller controller(model);
  controller.begin();

  TEST_ASSERT_FLOAT_WITHIN(0.1f, 1000.0f, model.state.innerPid[FC_PID_ROLL].rate);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.1835f, model.state.innerPid[FC_PID_ROLL].Kp);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1.4002f, model.state.innerPid[FC_PID_ROLL].Ki);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0030f, model.state.innerPid[FC_PID_ROLL].Kd);
  TEST_ASSERT_FLOAT_WITHIN(0.000001f, 0.000788f, model.state.innerPid[FC_PID_ROLL].Kf);

  TEST_ASSERT_FLOAT_WITHIN(0.1f, 1000.0f, model.state.innerPid[FC_PID_PITCH].rate);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.1835f, model.state.innerPid[FC_PID_PITCH].Kp);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1.4002f, model.state.innerPid[FC_PID_PITCH].Ki);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0030f, model.state.innerPid[FC_PID_PITCH].Kd);
  TEST_ASSERT_FLOAT_WITHIN(0.000001f, 0.000788f, model.state.innerPid[FC_PID_PITCH].Kf);

  TEST_ASSERT_FLOAT_WITHIN(0.1f, 1000.0f, model.state.innerPid[FC_PID_YAW].rate);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.1835f, model.state.innerPid[FC_PID_YAW].Kp);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1.4002f, model.state.innerPid[FC_PID_YAW].Ki);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0030f, model.state.innerPid[FC_PID_YAW].Kd);
  TEST_ASSERT_FLOAT_WITHIN(0.000001f, 0.000788f, model.state.innerPid[FC_PID_YAW].Kf);
}



void test_controller_rates()
{
  Model model;
  model.state.gyro.clock = 8000;
  model.config.gyro.dlpf = GYRO_DLPF_256;
  model.config.loopSync = 8;
  model.config.mixerSync = 1;
  model.config.mixer.type = FC_MIXER_QUADX;

  model.config.input.rateType = RATES_TYPE_BETAFLIGHT;
  model.config.input.rate[AXIS_ROLL] = 70;
  model.config.input.expo[AXIS_ROLL] = 0;
  model.config.input.superRate[AXIS_ROLL] = 80;
  model.config.input.rateLimit[AXIS_ROLL] = 1998;

  model.config.input.rate[AXIS_PITCH] = 70;
  model.config.input.expo[AXIS_PITCH] = 0;
  model.config.input.superRate[AXIS_PITCH] = 80;
  model.config.input.rateLimit[AXIS_PITCH] = 1998;

  model.config.input.rate[AXIS_YAW] = 120;
  model.config.input.expo[AXIS_YAW] = 0;
  model.config.input.superRate[AXIS_YAW] = 50;
  model.config.input.rateLimit[AXIS_YAW] = 1998;

  model.begin();

  Controller controller(model);
  controller.begin();

  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, controller.calculateSetpointRate(AXIS_ROLL, 0.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.76f, controller.calculateSetpointRate(AXIS_ROLL, 0.25f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 2.04f, controller.calculateSetpointRate(AXIS_ROLL, 0.5f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 4.58f, controller.calculateSetpointRate(AXIS_ROLL, 0.75f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 6.49f, controller.calculateSetpointRate(AXIS_ROLL, 0.85f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 11.92f, controller.calculateSetpointRate(AXIS_ROLL, 1.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 11.92f, controller.calculateSetpointRate(AXIS_ROLL, 1.1f));

  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, controller.calculateSetpointRate(AXIS_PITCH, 0.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, -2.04f, controller.calculateSetpointRate(AXIS_PITCH, -0.5f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, -11.92f, controller.calculateSetpointRate(AXIS_PITCH, -1.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 2.04f, controller.calculateSetpointRate(AXIS_PITCH, 0.5f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 11.92f, controller.calculateSetpointRate(AXIS_PITCH, 1.0f));

  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, controller.calculateSetpointRate(AXIS_YAW, 0.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, -1.48f, controller.calculateSetpointRate(AXIS_YAW, 0.3f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, -3.59f, controller.calculateSetpointRate(AXIS_YAW, 0.6f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, -8.29f, controller.calculateSetpointRate(AXIS_YAW, 1.0f));
}

void test_controller_rates_limit()
{
  Model model;
  model.state.gyro.clock = 8000;
  model.config.gyro.dlpf = GYRO_DLPF_256;
  model.config.loopSync = 8;
  model.config.mixerSync = 1;
  model.config.mixer.type = FC_MIXER_QUADX;

  model.config.input.rateType = RATES_TYPE_BETAFLIGHT;
  model.config.input.rate[AXIS_ROLL] = 70;
  model.config.input.expo[AXIS_ROLL] = 0;
  model.config.input.superRate[AXIS_ROLL] = 80;
  model.config.input.rateLimit[AXIS_ROLL] = 500;

  model.config.input.rate[AXIS_PITCH] = 70;
  model.config.input.expo[AXIS_PITCH] = 0;
  model.config.input.superRate[AXIS_PITCH] = 80;
  model.config.input.rateLimit[AXIS_PITCH] = 500;

  model.config.input.rate[AXIS_YAW] = 120;
  model.config.input.expo[AXIS_YAW] = 0;
  model.config.input.superRate[AXIS_YAW] = 50;
  model.config.input.rateLimit[AXIS_YAW] = 400;

  model.begin();

  Controller controller(model);
  controller.begin();

  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, controller.calculateSetpointRate(AXIS_ROLL, 0.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 2.03f, controller.calculateSetpointRate(AXIS_ROLL, 0.5f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 8.73f, controller.calculateSetpointRate(AXIS_ROLL, 1.0f));

  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, controller.calculateSetpointRate(AXIS_PITCH, 0.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 2.03f, controller.calculateSetpointRate(AXIS_PITCH, 0.5f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, -2.03f, controller.calculateSetpointRate(AXIS_PITCH, -0.5f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 8.73f, controller.calculateSetpointRate(AXIS_PITCH, 1.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, -8.73f, controller.calculateSetpointRate(AXIS_PITCH, -1.0f));

  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, controller.calculateSetpointRate(AXIS_YAW, 0.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, -2.79f, controller.calculateSetpointRate(AXIS_YAW, 0.5f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, -6.98f, controller.calculateSetpointRate(AXIS_YAW, 1.0f));
}

void test_controller_angle_mode_does_not_latch_fterm_scale()
{
  When(Method(ArduinoFake(), micros)).AlwaysReturn(0);

  Model model;
  model.state.gyro.clock = 1000;
  model.config.gyro.dlpf = GYRO_DLPF_256;
  model.config.loopSync = 1;
  model.config.mixerSync = 1;
  model.config.mixer.type = FC_MIXER_QUADX;
  model.config.pid[FC_PID_ROLL] = {.P = 0u, .I = 0u, .D = 0u, .F = 100};
  model.config.input.filterDerivative = {FILTER_NONE, 0};
  model.begin();

  Controller controller(model);
  controller.begin();

  model.state.input.ch[AXIS_ROLL] = 1.f;
  model.updateModes(1 << MODE_ANGLE);
  controller.update();

  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, model.state.innerPid[AXIS_ROLL].fScale);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, model.state.innerPid[AXIS_ROLL].fTerm);

  model.clearMode(MODE_ANGLE);
  model.state.input.ch[AXIS_ROLL] = 0.f;
  controller.update();

  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, model.state.innerPid[AXIS_ROLL].fScale);
  TEST_ASSERT_TRUE(model.state.innerPid[AXIS_ROLL].fTerm < -0.001f || model.state.innerPid[AXIS_ROLL].fTerm > 0.001f);
}
// =========================================================
// V2 ASSISTED MODE CONTROLLER TESTS
// =========================================================

void test_controller_angle_v2_activates_and_slews()
{
  When(Method(ArduinoFake(), micros)).AlwaysReturn(0);

  Model model;

  model.state.gyro.clock = 1000;
  model.config.gyro.dlpf = GYRO_DLPF_256;
  model.config.loopSync = 1;
  model.config.mixerSync = 1;
  model.config.mixer.type = FC_MIXER_QUADX;

  model.config.level.angleLimit = 45;
  model.config.level.rateLimit = 300;

  model.config.pid[FC_PID_LEVEL] =
      {.P = 45u, .I = 0u, .D = 0u, .F = 0};

  model.begin();
      // Angle V2 requires a valid attitude estimate.
  model.state.attitude.healthy =
      true;



  Controller controller(model);
  controller.begin();

  // Drone starts level.
model.state.attitude.euler.set(
    AXIS_ROLL,
    0.0f);

model.state.attitude.euler.set(
    AXIS_PITCH,
    0.0f);

  // Pilot requests positive roll and pitch.
  model.state.input.ch[AXIS_ROLL] = 1.0f;
  model.state.input.ch[AXIS_PITCH] = 0.5f;

  model.updateModes(
      uint32_t{1} << MODE_ANGLE);

  controller.update();

 const auto& angleV2 =
    model.state.angleV2;

  TEST_ASSERT_TRUE(
      angleV2.active);

  // Target must start moving toward the requested angle.
  TEST_ASSERT_TRUE(
      angleV2.angleTarget[AXIS_ROLL] > 0.0f);

  TEST_ASSERT_TRUE(
      angleV2.angleTarget[AXIS_PITCH] > 0.0f);

  // But slew limiting must prevent an instantaneous jump
  // to the full 45 degree command.
TEST_ASSERT_TRUE(
    angleV2.angleTarget[
        AXIS_ROLL] <
    0.7854f);

TEST_ASSERT_TRUE(
    angleV2.angleTarget[
        AXIS_PITCH] <
    0.7854f);

  // Positive angle error should create positive rate targets.
  TEST_ASSERT_TRUE(
      angleV2.rateTarget[AXIS_ROLL] > 0.0f);

  TEST_ASSERT_TRUE(
     angleV2.rateTarget[AXIS_PITCH] > 0.0f);
}


void test_controller_angle_v2_bumpless_entry()
{
  When(Method(ArduinoFake(), micros)).AlwaysReturn(0);

  Model model;

  model.state.gyro.clock = 1000;
  model.config.gyro.dlpf = GYRO_DLPF_256;
  model.config.loopSync = 1;
  model.config.mixerSync = 1;
  model.config.mixer.type = FC_MIXER_QUADX;

  model.config.level.angleLimit = 45;
  model.config.level.rateLimit = 300;

  model.config.pid[FC_PID_LEVEL] =
      {.P = 45u, .I = 0u, .D = 0u, .F = 0};

  model.begin();
  model.state.attitude.healthy = true;

  Controller controller(model);
  controller.begin();


  // Imagine Angle mode is enabled while the aircraft
  // already has some roll attitude.
model.state.attitude.euler.set(
    AXIS_ROLL,
    0.30f);

model.state.attitude.euler.set(
    AXIS_PITCH,
    0.0f);
  // Stick centered.
  model.state.input.ch[AXIS_ROLL] =
      0.0f;

  model.state.input.ch[AXIS_PITCH] =
      0.0f;

  model.updateModes(
      uint32_t{1} << MODE_ANGLE);

  controller.update();

 const auto& angleV2 = model.state.angleV2;
    
  TEST_ASSERT_TRUE(
      angleV2.active);

  // Bumpless entry means the target should begin close
  // to the current attitude instead of immediately
  // jumping to zero.
  TEST_ASSERT_TRUE(
      angleV2.angleTarget[AXIS_ROLL] > 0.20f);

  TEST_ASSERT_TRUE(
      angleV2.angleTarget[AXIS_ROLL] <= 0.30f);
}



void test_controller_angle_v2_active_path_is_bumpless_and_negative_feedback()
{
  ArduinoFakeReset();

  constexpr uint32_t NOW_US =
      1000000;

  When(
      Method(
          ArduinoFake(),
          micros))
      .AlwaysReturn(
          NOW_US);

  Model model;

  model.state.gyro.clock =
      1000;

  model.config.gyro.dlpf =
      GYRO_DLPF_256;

  model.config.loopSync =
      1;

  model.config.mixerSync =
      1;

  model.config.mixer.type =
      FC_MIXER_QUADX;

  model.config.level.angleLimit =
      30;

  model.config.level.rateLimit =
      200;

  model.config.pid[
      FC_PID_LEVEL] =
      {
          .P = 45u,
          .I = 0u,
          .D = 0u,
          .F = 0
      };

  model.begin();

  Controller controller(
      model);

  controller.begin();

  // Valid estimator state.
  model.state.attitude.healthy =
      true;

  model.state.attitude.lastUpdateUs =
      NOW_US;

  // Begin already tilted:
  // positive Roll and negative Pitch.
model.state.attitude.euler.set(
    AXIS_ROLL,
    Utils::toRad(
        10.0f));

model.state.attitude.euler.set(
    AXIS_PITCH,
    Utils::toRad(
        -8.0f));

  // Centered sticks request level attitude.
  model.state.input.ch[
      AXIS_ROLL] =
      0.0f;

  model.state.input.ch[
      AXIS_PITCH] =
      0.0f;

  model.updateModes(
      uint32_t{1} <<
      MODE_ANGLE);

  controller.update();

const auto& v2 = model.state.angleV2;

  TEST_ASSERT_TRUE(
     v2.active);

  // Positive measured Roll must request a negative
  // Roll rate.
  TEST_ASSERT_TRUE(
     v2.rateTarget[AXIS_ROLL] <
      0.0f);

  // Negative measured Pitch must request a positive
  // Pitch rate.
  TEST_ASSERT_TRUE(
     v2.rateTarget[AXIS_PITCH] >
      0.0f);

  // In this dedicated validation build, V2 must be
  // the authoritative Roll/Pitch rate target.
  TEST_ASSERT_FLOAT_WITHIN(
      0.0001f,
     v2.rateTarget[AXIS_ROLL],
      model.state.setpoint.rate[
          AXIS_ROLL]);

  TEST_ASSERT_FLOAT_WITHIN(
      0.0001f,
      v2.rateTarget[AXIS_PITCH],
      model.state.setpoint.rate[
          AXIS_PITCH]);

  const float rateLimit =
      Utils::toRad(
          static_cast<float>(
              model.config.level.rateLimit));

  TEST_ASSERT_TRUE(
      std::fabs(
          model.state.setpoint.rate[
              AXIS_ROLL]) <=
      rateLimit +
          0.0001f);

  TEST_ASSERT_TRUE(
      std::fabs(
          model.state.setpoint.rate[
              AXIS_PITCH]) <=
      rateLimit +
          0.0001f);

  // Bumpless entry:
  //
  // V2 captured the current attitude before beginning
  // to slew toward level, therefore the first command
  // must be much smaller than the old instantaneous
  // 10-degree LEVEL-P correction.
  const float oldLegacyEquivalent =
      4.5f *
      Utils::toRad(
          10.0f);

  TEST_ASSERT_TRUE(
      std::fabs(
          model.state.setpoint.rate[
              AXIS_ROLL]) <
      std::fabs(
          oldLegacyEquivalent));
}


void test_controller_althold_v2_captures_current_altitude()
{
  When(Method(ArduinoFake(), micros)).AlwaysReturn(0);

  Model model;

  model.state.gyro.clock = 1000;
  model.config.gyro.dlpf = GYRO_DLPF_256;
  model.config.loopSync = 1;
  model.config.mixerSync = 1;
  model.config.mixer.type = FC_MIXER_QUADX;

  model.begin();

  Controller controller(model);
  controller.begin();
    setHealthyAssistedEstimatorState(
      model,
      0);

  // Simulated estimator state.
  model.state.altitude.height =
      2.50f;

  model.state.altitude.vario =
      0.0f;

  model.state.altitude.healthy =
      true;

  // Centered throttle = zero climb request.
  model.state.input.ch[ALTHOLD_PILOT_CHANNEL] =
      0.0f;

  model.updateModes(
      uint32_t{1} << MODE_ALTHOLD);

  controller.update();

  const auto& assisted =
      model.state.assistedMode;

  TEST_ASSERT_TRUE(
      assisted.altitudeActive);

  TEST_ASSERT_TRUE(
      assisted.altitudeTargetValid);

  // AltHold should capture current altitude on entry.
  TEST_ASSERT_FLOAT_WITHIN(
      0.001f,
      2.50f,
      assisted.altitudeTarget);

  // Centered stick means no pilot climb/descent request.
  TEST_ASSERT_FLOAT_WITHIN(
      0.001f,
      0.0f,
      assisted.verticalRatePilot);
}


void test_controller_althold_v2_center_stick_holds_target()
{
  When(Method(ArduinoFake(), micros)).AlwaysReturn(0);

  Model model;

  model.state.gyro.clock = 1000;
  model.config.gyro.dlpf = GYRO_DLPF_256;
  model.config.loopSync = 1;
  model.config.mixerSync = 1;
  model.config.mixer.type = FC_MIXER_QUADX;

  model.begin();

  Controller controller(model);
  controller.begin();
    setHealthyAssistedEstimatorState(
      model,
      0);

  model.state.altitude.height =
      3.0f;

  model.state.altitude.vario =
      0.0f;

  model.state.altitude.healthy =
      true;

  model.state.input.ch[ALTHOLD_PILOT_CHANNEL] =
      0.0f;

  model.updateModes(
      uint32_t{1} << MODE_ALTHOLD);

  controller.update();

  const float initialTarget =
      model.state.assistedMode.altitudeTarget;

  // Simulate many controller iterations with
  // centered throttle.
  for (int i = 0; i < 100; ++i)
  {
    controller.update();
  }

  TEST_ASSERT_FLOAT_WITHIN(
      0.001f,
      initialTarget,
      model.state.assistedMode.altitudeTarget);
}


void test_controller_althold_v2_climb_command_moves_target_up()
{
  When(Method(ArduinoFake(), micros)).AlwaysReturn(0);

  Model model;

  model.state.gyro.clock = 1000;
  model.config.gyro.dlpf = GYRO_DLPF_256;
  model.config.loopSync = 1;
  model.config.mixerSync = 1;
  model.config.mixer.type = FC_MIXER_QUADX;

  model.begin();

  Controller controller(model);
  controller.begin();
    setHealthyAssistedEstimatorState(
      model,
      0);

  model.state.altitude.height =
      2.0f;

  model.state.altitude.vario =
      0.0f;

  model.state.altitude.healthy =
      true;

  // Positive centered-stick displacement.
  model.state.input.ch[ALTHOLD_PILOT_CHANNEL] =
      0.50f;

  model.updateModes(
      uint32_t{1} << MODE_ALTHOLD);

  controller.update();

  const auto& assisted =
      model.state.assistedMode;

  TEST_ASSERT_TRUE(
      assisted.altitudeActive);

  TEST_ASSERT_TRUE(
      assisted.verticalRatePilot > 0.0f);

  // Positive climb command must move the altitude
  // target upward.
  TEST_ASSERT_TRUE(
      assisted.altitudeTarget > 2.0f);

  TEST_ASSERT_TRUE(
      assisted.verticalRateTarget > 0.0f);
}


void test_controller_althold_v2_descent_command_moves_target_down()
{
  When(Method(ArduinoFake(), micros)).AlwaysReturn(0);

  Model model;

  model.state.gyro.clock = 1000;
  model.config.gyro.dlpf = GYRO_DLPF_256;
  model.config.loopSync = 1;
  model.config.mixerSync = 1;
  model.config.mixer.type = FC_MIXER_QUADX;

  model.begin();

  Controller controller(model);
  controller.begin();
    setHealthyAssistedEstimatorState(
      model,
      0);

  model.state.altitude.height =
      2.0f;

  model.state.altitude.vario =
      0.0f;

  model.state.altitude.healthy =
      true;

  model.state.input.ch[ALTHOLD_PILOT_CHANNEL] =
      -0.50f;

  model.updateModes(
      uint32_t{1} << MODE_ALTHOLD);

  controller.update();

  const auto& assisted =
      model.state.assistedMode;

  TEST_ASSERT_TRUE(
      assisted.altitudeActive);

  TEST_ASSERT_TRUE(
      assisted.verticalRatePilot < 0.0f);

  TEST_ASSERT_TRUE(
      assisted.altitudeTarget < 2.0f);

  TEST_ASSERT_TRUE(
      assisted.verticalRateTarget < 0.0f);
}


void test_controller_althold_v2_stops_when_estimator_unhealthy()
{
  When(Method(ArduinoFake(), micros)).AlwaysReturn(0);

  Model model;

  model.state.gyro.clock = 1000;
  model.config.gyro.dlpf = GYRO_DLPF_256;
  model.config.loopSync = 1;
  model.config.mixerSync = 1;
  model.config.mixer.type = FC_MIXER_QUADX;

  model.begin();

  Controller controller(model);
  controller.begin();
    setHealthyAssistedEstimatorState(
      model,
      0);

  model.state.altitude.height =
      1.5f;

  model.state.altitude.vario =
      0.0f;

  model.state.altitude.healthy =
      true;

  model.state.input.ch[ALTHOLD_PILOT_CHANNEL] =
      0.0f;

  model.updateModes(
      uint32_t{1} << MODE_ALTHOLD);

  controller.update();

  TEST_ASSERT_TRUE(
      model.state.assistedMode.altitudeActive);

  TEST_ASSERT_TRUE(
      model.state.assistedMode.altitudeTargetValid);

  // Simulate loss of reliable vertical estimate.
  model.state.altitude.healthy =
      false;

  controller.update();

  TEST_ASSERT_FALSE(
      model.state.assistedMode.altitudeActive);

  TEST_ASSERT_FALSE(
      model.state.assistedMode.altitudeTargetValid);
}


void test_controller_althold_v2_vertical_accel_limit()
{
  When(Method(ArduinoFake(), micros)).AlwaysReturn(0);

  Model model;

  model.state.gyro.clock = 1000;
  model.config.gyro.dlpf = GYRO_DLPF_256;
  model.config.loopSync = 1;
  model.config.mixerSync = 1;
  model.config.mixer.type = FC_MIXER_QUADX;

  model.begin();

  Controller controller(model);
  controller.begin();
    setHealthyAssistedEstimatorState(
      model,
      0);

  model.state.altitude.height =
      2.0f;

  model.state.altitude.vario =
      0.0f;

  model.state.altitude.healthy =
      true;

  model.state.input.ch[ALTHOLD_PILOT_CHANNEL] =
      1.0f;

  model.updateModes(
      uint32_t{1} << MODE_ALTHOLD);

  controller.update();

  const float firstVzTarget =
      model.state.assistedMode.verticalRateTarget;

  controller.update();

  const float secondVzTarget =
      model.state.assistedMode.verticalRateTarget;

  const float change =
      secondVzTarget -
      firstVzTarget;

  TEST_ASSERT_TRUE(
      change >= 0.0f);

  // Vz target must increase gradually rather than
  // instantly jumping to maximum climb rate.
  TEST_ASSERT_TRUE(
      secondVzTarget < 1.5f);

  TEST_ASSERT_TRUE(
      change < 0.1f);
}

void test_controller_althold_v2_target_is_bounded()
{
  When(
      Method(
          ArduinoFake(),
          micros))
      .AlwaysReturn(
          0);

  Model model;

  model.state.gyro.clock =
      1000;

  model.config.gyro.dlpf =
      GYRO_DLPF_256;

  model.config.loopSync =
      1;

  model.config.mixerSync =
      1;

  model.config.mixer.type =
      FC_MIXER_QUADX;

  model.begin();

  Controller controller(
      model);

  controller.begin();

  setHealthyAssistedEstimatorState(
      model,
      0);

  model.state.altitude.height =
      2.0f;

  model.state.altitude.vario =
      0.0f;

  model.state.altitude.healthy =
      true;

  // --------------------------------------------------
  // DESCENT TARGET MUST NOT RUN AWAY
  // --------------------------------------------------

  model.state.input.ch[
      ALTHOLD_PILOT_CHANNEL] =
      -1.0f;

  model.updateModes(
      uint32_t{1} <<
      MODE_ALTHOLD);

  // 10 seconds at nominal 1 kHz would previously
  // integrate roughly -10 m of impossible target.
  for (int i = 0;
       i < 10000;
       ++i)
  {
    controller.update();
  }

  const float descentTarget =
      model.state.assistedMode
          .altitudeTarget;

  // Current altitude = 2 m.
  // Anti-windup window = +/-2 m.
  // Therefore target must never go below 0 m.
  TEST_ASSERT_TRUE(
      descentTarget >=
      -0.001f);

  TEST_ASSERT_TRUE(
      descentTarget <=
      2.001f);


  // --------------------------------------------------
  // RESET ALTHOLD
  // --------------------------------------------------

  model.updateModes(
      0);

  controller.update();

  TEST_ASSERT_FALSE(
      model.state.assistedMode
          .altitudeActive);


  // --------------------------------------------------
  // CLIMB TARGET MUST NOT RUN AWAY
  // --------------------------------------------------

  model.state.input.ch[
      ALTHOLD_PILOT_CHANNEL] =
      1.0f;

  model.updateModes(
      uint32_t{1} <<
      MODE_ALTHOLD);

  for (int i = 0;
       i < 10000;
       ++i)
  {
    controller.update();
  }

  const float climbTarget =
      model.state.assistedMode
          .altitudeTarget;

  // Current altitude = 2 m.
  // Upper edge of the +/-2 m window = 4 m.
  TEST_ASSERT_TRUE(
      climbTarget <=
      4.001f);

  TEST_ASSERT_TRUE(
      climbTarget >=
      1.999f);
}
void test_controller_althold_v2_full_climb_rate_scaling()
{
  When(
      Method(
          ArduinoFake(),
          micros))
      .AlwaysReturn(
          50000);

  Model model;

  model.state.gyro.clock =
      1000;

  model.config.gyro.dlpf =
      GYRO_DLPF_256;

  model.config.loopSync =
      1;

  model.config.mixerSync =
      1;

  model.config.mixer.type =
      FC_MIXER_QUADX;

  model.begin();

  Controller controller(
      model);

  controller.begin();

  // Fresh attitude estimator state.
  model.state.gyro.present =
      true;

  model.state.accel.present =
      true;

  model.state.attitude.healthy =
      true;

  model.state.attitude.lastUpdateUs =
      50000;

  model.state.attitude.quaternion =
      Quaternion(
          1.0f,
          0.0f,
          0.0f,
          0.0f);

  model.state.attitude.euler =
      VectorFloat(
          0.0f,
          0.0f,
          0.0f);

  // Fresh barometer state.
  model.config.baro.dev =
      BARO_BMP280;

  model.state.baro.present =
      true;

  model.state.baro.sampleValid =
      true;

  model.state.baro.lastUpdateUs =
      50000;

  // Healthy altitude estimator.
  model.state.altitude.height =
      2.0f;

  model.state.altitude.vario =
      0.0f;

  model.state.altitude.healthy =
      true;

  // Full climb command.
  model.state.input.ch[
      ALTHOLD_PILOT_CHANNEL] =
      1.0f;

  model.updateModes(
      uint32_t{1} <<
      MODE_ALTHOLD);

  controller.update();

  TEST_ASSERT_TRUE(
      model.state.assistedMode
          .altitudeActive);

  TEST_ASSERT_TRUE(
      model.state.assistedMode
          .altitudeTargetValid);
// Full climb command is 1.5 m/s.
//
// Derive the expected one-cycle target movement from the
// actual controller loop rate instead of assuming 1 kHz.
const float expectedTarget =
    2.0f +
    1.5f /
        static_cast<float>(
            model.state.loopTimer.rate);

TEST_ASSERT_FLOAT_WITHIN(
    0.00002f,
    expectedTarget,
    model.state.assistedMode
        .altitudeTarget);

}
void test_actuator_althold_fault_requires_switch_cycle()
{
  When(
      Method(
          ArduinoFake(),
          micros))
      .AlwaysReturn(
          50000);

  Model model;

  // --------------------------------------------------
  // Required barometer state
  // --------------------------------------------------

  model.config.baro.dev =
      BARO_BMP280;

  model.state.baro.present =
      true;

  model.state.baro.sampleValid =
      true;

  model.state.baro.lastUpdateUs =
      50000;

  // --------------------------------------------------
  // Required altitude-estimator state
  // --------------------------------------------------

  model.state.altitude.healthy =
      true;

  model.state.altitude.height =
      2.0f;

  model.state.altitude.vario =
      0.0f;

  model.state.altitude.lastUpdateUs =
      50000;

  // --------------------------------------------------
  // Required attitude-estimator state
  // AltHold depends on a valid world-frame vertical
  // acceleration projection, so attitude health is part
  // of the AltHold health contract.
  // --------------------------------------------------

  model.state.gyro.present =
      true;

  model.state.accel.present =
      true;

  model.state.attitude.healthy =
      true;

  model.state.attitude.lastUpdateUs =
      50000;

  model.state.attitude.quaternion =
      Quaternion(
          1.0f,
          0.0f,
          0.0f,
          0.0f);

  model.state.attitude.euler =
      VectorFloat(
          0.0f,
          0.0f,
          0.0f);

  // --------------------------------------------------
  // AltHold AUX condition
  // --------------------------------------------------

  auto& condition =
      model.config.conditions[0];

  condition.id =
      MODE_ALTHOLD;

  condition.ch =
      AXIS_AUX_1;

  condition.min =
      1200;

  condition.max =
      1800;

  model.state.input.us[AXIS_AUX_1] =
      1500;

#if defined(ESPFC_ALTHOLD_V2_ACTIVE)
  model.state.input.channelCount =
      std::max<size_t>(
          ALTHOLD_PILOT_CHANNEL + 1,
          static_cast<size_t>(
              AXIS_AUX_1 + 1));

  model.state.input.channelsValid =
      true;

  model.state.input.ch[
      ALTHOLD_PILOT_CHANNEL] =
      0.0f;

  model.state.input.raw[
      ALTHOLD_PILOT_CHANNEL] =
      PWM_RANGE_MID;
#endif

  Actuator actuator(
      model);

  actuator.begin();

  // --------------------------------------------------
  // 1. Healthy estimator + switch ON
  // --------------------------------------------------

  actuator.updateModeMask();

  TEST_ASSERT_TRUE(
      model.isModeActive(
          MODE_ALTHOLD));

  // --------------------------------------------------
  // 2. Estimator failure
  // --------------------------------------------------

  model.state.altitude.healthy =
      false;

  actuator.updateModeMask();

  TEST_ASSERT_FALSE(
      model.isModeActive(
          MODE_ALTHOLD));

  // --------------------------------------------------
  // 3. Estimator recovers while switch stays ON
  //
  // Fault latch must prevent automatic re-entry.
  // --------------------------------------------------

  model.state.altitude.healthy =
      true;

  actuator.updateModeMask();

  TEST_ASSERT_FALSE(
      model.isModeActive(
          MODE_ALTHOLD));

  // --------------------------------------------------
  // 4. Pilot deliberately switches AltHold OFF
  //
  // This clears the fault latch.
  // --------------------------------------------------

  model.state.input.us[AXIS_AUX_1] =
      1000;

  actuator.updateModeMask();

  TEST_ASSERT_FALSE(
      model.isModeActive(
          MODE_ALTHOLD));

  // --------------------------------------------------
  // 5. Pilot deliberately switches AltHold ON again
  // --------------------------------------------------

  model.state.input.us[AXIS_AUX_1] =
      1500;

  actuator.updateModeMask();

  TEST_ASSERT_TRUE(
      model.isModeActive(
          MODE_ALTHOLD));
}




void test_actuator_mode_link_activates_target_without_aux_range()
{
  Model model;

  model.state.input.channelsValid =
      true;

  model.state.input.rxLoss =
      false;

  model.state.input.channelCount =
      AXIS_AUX_2 + 1;

  model.state.input.us[
      AXIS_AUX_1] =
      1500;

  // Source mode: BUZZER from AUX1.
  auto& source =
      model.config.conditions[0];

  source.id =
      MODE_BUZZER;

  source.ch =
      AXIS_AUX_1;

  source.min =
      1200;

  source.max =
      1800;

  // Target mode: BLACKBOX linked to BUZZER. No AUX range is configured.
  auto& linked =
      model.config.conditions[1];

  linked.id =
      MODE_BLACKBOX;

  linked.min =
      900;

  linked.max =
      900;

  linked.logicMode =
      0;

  linked.linkId =
      MODE_BUZZER;

  Actuator actuator(
      model);

  actuator.begin();

  TEST_ASSERT_TRUE(
      model.state.mode.maskPresent &
      (uint32_t{1} <<
       MODE_BLACKBOX));

  actuator.updateModeMask();

  TEST_ASSERT_TRUE(
      model.isModeActive(
          MODE_BUZZER));

  TEST_ASSERT_TRUE(
      model.isModeActive(
          MODE_BLACKBOX));

  // Turning the source mode off also clears the linked target.
  model.state.input.us[
      AXIS_AUX_1] =
      1000;

  actuator.updateModeMask();

  TEST_ASSERT_FALSE(
      model.isModeActive(
          MODE_BUZZER));

  TEST_ASSERT_FALSE(
      model.isModeActive(
          MODE_BLACKBOX));
}


void test_actuator_mode_logic_and_requires_all_ranges()
{
  Model model;

  model.state.input.channelsValid =
      true;

  model.state.input.rxLoss =
      false;

  model.state.input.channelCount =
      AXIS_AUX_2 + 1;

  auto& first =
      model.config.conditions[0];

  first.id =
      MODE_BUZZER;

  first.ch =
      AXIS_AUX_1;

  first.min =
      1200;

  first.max =
      1800;

  first.logicMode =
      1;

  auto& second =
      model.config.conditions[1];

  second.id =
      MODE_BUZZER;

  second.ch =
      AXIS_AUX_2;

  second.min =
      1200;

  second.max =
      1800;

  second.logicMode =
      1;

  model.state.input.us[
      AXIS_AUX_1] =
      1500;

  model.state.input.us[
      AXIS_AUX_2] =
      1000;

  Actuator actuator(
      model);

  actuator.begin();
  actuator.updateModeMask();

  TEST_ASSERT_FALSE(
      model.isModeActive(
          MODE_BUZZER));

  model.state.input.us[
      AXIS_AUX_2] =
      1500;

  actuator.updateModeMask();

  TEST_ASSERT_TRUE(
      model.isModeActive(
          MODE_BUZZER));
}


void test_actuator_mode_link_cannot_target_arm()
{
  Model model;

  model.state.input.channelsValid =
      true;

  model.state.input.rxLoss =
      false;

  model.state.input.channelCount =
      AXIS_AUX_1 + 1;

  model.state.input.us[
      AXIS_AUX_1] =
      1500;

  auto& source =
      model.config.conditions[0];

  source.id =
      MODE_BUZZER;

  source.ch =
      AXIS_AUX_1;

  source.min =
      1200;

  source.max =
      1800;

  auto& linkedArm =
      model.config.conditions[1];

  linkedArm.id =
      MODE_ARMED;

  linkedArm.min =
      900;

  linkedArm.max =
      900;

  linkedArm.linkId =
      MODE_BUZZER;

  Actuator actuator(
      model);

  actuator.begin();

  TEST_ASSERT_FALSE(
      model.state.mode.maskPresent &
      (uint32_t{1} <<
       MODE_ARMED));

  actuator.updateModeMask();

  TEST_ASSERT_TRUE(
      model.isModeActive(
          MODE_BUZZER));

  TEST_ASSERT_FALSE(
      model.isModeActive(
          MODE_ARMED));
}


void test_actuator_mode_link_rejects_linked_source_chain()
{
  Model model;

  model.state.input.channelsValid =
      true;

  model.state.input.rxLoss =
      false;

  model.state.input.channelCount =
      AXIS_AUX_1 + 1;

  model.state.input.us[
      AXIS_AUX_1] =
      1500;

  auto& source =
      model.config.conditions[0];

  source.id =
      MODE_BUZZER;

  source.ch =
      AXIS_AUX_1;

  source.min =
      1200;

  source.max =
      1800;

  auto& middle =
      model.config.conditions[1];

  middle.id =
      MODE_BLACKBOX;

  middle.min =
      900;

  middle.max =
      900;

  middle.linkId =
      MODE_BUZZER;

  auto& chained =
      model.config.conditions[2];

  chained.id =
      MODE_BLACKBOX_ERASE;

  chained.min =
      900;

  chained.max =
      900;

  chained.linkId =
      MODE_BLACKBOX;

  Actuator actuator(
      model);

  actuator.begin();

  TEST_ASSERT_TRUE(
      model.state.mode.maskPresent &
      (uint32_t{1} <<
       MODE_BLACKBOX));

  TEST_ASSERT_FALSE(
      model.state.mode.maskPresent &
      (uint32_t{1} <<
       MODE_BLACKBOX_ERASE));

  actuator.updateModeMask();

  TEST_ASSERT_TRUE(
      model.isModeActive(
          MODE_BUZZER));

  TEST_ASSERT_TRUE(
      model.isModeActive(
          MODE_BLACKBOX));

  TEST_ASSERT_FALSE(
      model.isModeActive(
          MODE_BLACKBOX_ERASE));
}


void test_model_config_antigravity_tail_layout_stays_packed()
{
  const size_t modelNameEnd =
      offsetof(
          ModelConfig,
          modelName) +
      sizeof(
          ModelConfig::modelName);

  TEST_ASSERT_EQUAL_UINT32(
      modelNameEnd,
      offsetof(
          ModelConfig,
          antiGravityGain));

  TEST_ASSERT_EQUAL_UINT32(
      offsetof(
          ModelConfig,
          antiGravityGain) +
          sizeof(uint8_t),
      offsetof(
          ModelConfig,
          antiGravityConfigTag));

  TEST_ASSERT_EQUAL_UINT32(
      offsetof(
          ModelConfig,
          antiGravityConfigTag) +
          sizeof(uint16_t),
      sizeof(
          ModelConfig));

  // Reconstruct the size the legacy class had when modelName was its final
  // member. If this rounded legacy end equals the current size, the new bytes
  // are consuming tail padding rather than growing the EEPROM image.
  const size_t legacyRoundedSize =
      ((modelNameEnd +
        alignof(ModelConfig) -
        1u) /
       alignof(ModelConfig)) *
      alignof(ModelConfig);

  TEST_ASSERT_EQUAL_UINT32(
      legacyRoundedSize,
      sizeof(
          ModelConfig));
}


void test_model_sanitize_migrates_legacy_antigravity_tail()
{
  Model model;

  model.config.antiGravityGain =
      247;

  model.config.antiGravityConfigTag =
      0;

  model.sanitize();

  TEST_ASSERT_EQUAL_UINT8(
      ModelConfig::ANTI_GRAVITY_GAIN_DEFAULT,
      model.config.antiGravityGain);

  TEST_ASSERT_EQUAL_UINT16(
      ModelConfig::ANTI_GRAVITY_CONFIG_TAG,
      model.config.antiGravityConfigTag);
}


void test_model_sanitize_scrubs_legacy_antigravity_mode_rows()
{
  Model model;

  model.config.antiGravityConfigTag =
      0;

  model.config.conditions[0].id =
      MODE_ANTI_GRAVITY;

  model.config.conditions[0].ch =
      AXIS_AUX_1;

  model.config.conditions[0].min =
      1200;

  model.config.conditions[0].max =
      1800;

  model.config.conditions[1].id =
      MODE_ANGLE;

  model.config.conditions[1].linkId =
      MODE_ANTI_GRAVITY;

  model.config.conditions[1].min =
      1200;

  model.config.conditions[1].max =
      1800;

  model.sanitize();

  TEST_ASSERT_EQUAL_UINT8(
      MODE_ARMED,
      model.config.conditions[0].id);

  TEST_ASSERT_EQUAL_INT16(
      900,
      model.config.conditions[0].min);

  TEST_ASSERT_EQUAL_INT16(
      900,
      model.config.conditions[0].max);

  TEST_ASSERT_EQUAL_UINT8(
      0,
      model.config.conditions[0].linkId);

  TEST_ASSERT_EQUAL_UINT8(
      MODE_ARMED,
      model.config.conditions[1].id);

  TEST_ASSERT_EQUAL_INT16(
      900,
      model.config.conditions[1].min);

  TEST_ASSERT_EQUAL_INT16(
      900,
      model.config.conditions[1].max);

  TEST_ASSERT_EQUAL_UINT8(
      0,
      model.config.conditions[1].linkId);
}


void test_model_sanitize_preserves_new_antigravity_mode_rows()
{
  Model model;

  model.config.antiGravityConfigTag =
      ModelConfig::ANTI_GRAVITY_CONFIG_TAG;

  model.config.conditions[0].id =
      MODE_ANTI_GRAVITY;

  model.config.conditions[0].ch =
      AXIS_AUX_2;

  model.config.conditions[0].min =
      1300;

  model.config.conditions[0].max =
      1700;

  model.sanitize();

  TEST_ASSERT_EQUAL_UINT8(
      MODE_ANTI_GRAVITY,
      model.config.conditions[0].id);

  TEST_ASSERT_EQUAL_UINT8(
      AXIS_AUX_2,
      model.config.conditions[0].ch);

  TEST_ASSERT_EQUAL_INT16(
      1300,
      model.config.conditions[0].min);

  TEST_ASSERT_EQUAL_INT16(
      1700,
      model.config.conditions[0].max);
}


void test_model_sanitize_preserves_valid_antigravity_gain()
{
  Model model;

  model.config.antiGravityGain =
      173;

  model.config.antiGravityConfigTag =
      ModelConfig::ANTI_GRAVITY_CONFIG_TAG;

  model.config.featureMask |=
      FEATURE_ANTI_GRAVITY;

  model.sanitize();

  TEST_ASSERT_EQUAL_UINT8(
      173,
      model.config.antiGravityGain);

  TEST_ASSERT_TRUE(
      model.config.featureMask &
      FEATURE_ANTI_GRAVITY);
}


void test_controller_anti_gravity_tracks_manual_throttle_transient()
{
  ArduinoFakeReset();

  When(
      Method(
          ArduinoFake(),
          micros))
      .AlwaysReturn(
          2100000);

  Model model;

  model.config.featureMask |=
      FEATURE_ANTI_GRAVITY;

  model.state.gyro.clock =
      1000;

  model.config.gyro.dlpf =
      GYRO_DLPF_256;

  model.config.loopSync =
      1;

  model.config.mixerSync =
      1;

  model.config.mixer.type =
      FC_MIXER_QUADX;

  model.begin();

  Controller controller(
      model);

  controller.begin();

  model.state.input.channelsValid =
      true;

  model.state.input.rxLoss =
      false;

  model.state.input.rxFailSafe =
      false;

  model.state.input.ch[
      AXIS_THRUST] =
      -1.0f;

  // First cycle primes the detector without creating a synthetic startup spike.
  controller.update();

  TEST_ASSERT_TRUE(
      model.state.antiGravity.enabled);

  TEST_ASSERT_FALSE(
      model.state.antiGravity.active);

  model.state.input.ch[
      AXIS_THRUST] =
      0.0f;

  controller.update();

  TEST_ASSERT_TRUE(
      model.state.antiGravity.enabled);

  TEST_ASSERT_TRUE(
      model.state.antiGravity.derivative >
      0.0f);

  TEST_ASSERT_TRUE(
      model.state.antiGravity.filteredDerivative >=
      0.0f);

  TEST_ASSERT_TRUE(
      model.state.antiGravity.iMultiplier >=
      1.0f);

  TEST_ASSERT_TRUE(
      model.state.antiGravity.pMultiplier >=
      1.0f);

#if defined(ESPFC_ANTI_GRAVITY_ACTIVE)
  TEST_ASSERT_TRUE(
      model.state.antiGravity.ratePidApplied);

  TEST_ASSERT_TRUE(
      model.state.antiGravity.iAccelerator >
      0.0f);
#else
  TEST_ASSERT_FALSE(
      model.state.antiGravity.ratePidApplied);
#endif
}


void test_pid_antigravity_gain_inputs_scale_p_and_add_i()
{
  Espfc::Control::Pid pid;

  pid.rate = 1000.0f;
  pid.Kp = 1.0f;
  pid.Ki = 1.0f;
  pid.Kd = 0.0f;
  pid.Kf = 0.0f;
  pid.pScale = 1.0f;
  pid.iScale = 1.0f;
  pid.iLimitLow = -10.0f;
  pid.iLimitHigh = 10.0f;
  pid.oLimitLow = -10.0f;
  pid.oLimitHigh = 10.0f;
  pid.begin();

  const float output =
      pid.update(
          1.0f,
          0.0f,
          1.0f,
          true,
          2.0f,
          0.5f);

  TEST_ASSERT_FLOAT_WITHIN(
      0.0001f,
      2.0f,
      pid.pTerm);

  TEST_ASSERT_FLOAT_WITHIN(
      0.0001f,
      0.0015f,
      pid.iTerm);

  TEST_ASSERT_FLOAT_WITHIN(
      0.0001f,
      2.0015f,
      output);
}


void test_controller_anti_gravity_can_be_enabled_by_mode()
{
  ArduinoFakeReset();

  When(
      Method(
          ArduinoFake(),
          micros))
      .AlwaysReturn(
          2150000);

  Model model;

  model.config.featureMask &=
      ~FEATURE_ANTI_GRAVITY;

  model.state.gyro.clock =
      1000;

  model.config.gyro.dlpf =
      GYRO_DLPF_256;

  model.config.loopSync =
      1;

  model.config.mixerSync =
      1;

  model.config.mixer.type =
      FC_MIXER_QUADX;

  model.begin();

  Controller controller(
      model);

  controller.begin();

  model.state.input.channelsValid =
      true;

  model.state.input.rxLoss =
      false;

  model.state.input.rxFailSafe =
      false;

  model.state.input.ch[
      AXIS_THRUST] =
      -1.0f;

  model.updateModes(
      uint32_t{1} <<
      MODE_ANTI_GRAVITY);

  controller.update();

  TEST_ASSERT_TRUE(
      model.state.antiGravity.enabled);

  // Entry is primed without a synthetic derivative spike.
  TEST_ASSERT_FALSE(
      model.state.antiGravity.active);

  model.state.input.ch[
      AXIS_THRUST] =
      0.0f;

  controller.update();

  TEST_ASSERT_TRUE(
      model.state.antiGravity.derivative >
      0.0f);
}


#if defined(ESPFC_ALTHOLD_V2_ACTIVE)

void test_controller_anti_gravity_yields_to_assisted_vertical_control()
{
  ArduinoFakeReset();

  When(
      Method(
          ArduinoFake(),
          micros))
      .AlwaysReturn(
          2200000);

  Model model;

  model.config.featureMask |=
      FEATURE_ANTI_GRAVITY;

  model.state.gyro.clock =
      1000;

  model.config.gyro.dlpf =
      GYRO_DLPF_256;

  model.config.loopSync =
      1;

  model.config.mixerSync =
      1;

  model.config.mixer.type =
      FC_MIXER_QUADX;

  model.begin();

  Controller controller(
      model);

  controller.begin();

  model.state.input.channelsValid =
      true;

  model.state.input.rxLoss =
      false;

  model.state.input.rxFailSafe =
      false;

  model.state.input.ch[
      AXIS_THRUST] =
      -1.0f;

  model.updateModes(
      uint32_t{1} <<
      MODE_ALTHOLD);

  controller.update();

  model.state.input.ch[
      AXIS_THRUST] =
      1.0f;

  controller.update();

  TEST_ASSERT_TRUE(
      model.state.antiGravity.enabled);

  TEST_ASSERT_FALSE(
      model.state.antiGravity.active);

  TEST_ASSERT_FLOAT_WITHIN(
      0.0001f,
      1.0f,
      model.state.antiGravity.iMultiplier);

  TEST_ASSERT_FLOAT_WITHIN(
      0.0001f,
      1.0f,
      model.state.antiGravity.pMultiplier);
}

#endif


void test_msp_current_meter_reply_is_ignored_when_source_is_not_msp()
{
  ArduinoFakeReset();

  Model model;

  model.config.ibat.source =
      CURRENT_METER_ADC;

  Connect::MspProcessor processor(
      model);

  Connect::MspMessage reply;

  reply.cmd =
      MSP_ANALOG;

  reply.dir =
      Connect::MSP_TYPE_REPLY;

  reply.state =
      Connect::MSP_STATE_RECEIVED;

  reply.received =
      7;

  const uint8_t payload[7] = {
      120,
      0x01, 0x00,
      0x00, 0x00,
      0x64, 0x00};

  std::copy(
      payload,
      payload + sizeof(payload),
      reply.buffer);

  processor.processReply(
      reply);

  TEST_ASSERT_FALSE(
      model.state.battery
          .mspCurrentValid);

  TEST_ASSERT_EQUAL_INT16(
      0,
      model.state.battery
          .mspCurrentCentiAmps);
}


void test_msp_current_meter_short_reply_is_ignored()
{
  ArduinoFakeReset();

  Model model;

  model.config.ibat.source =
      CURRENT_METER_MSP;

  Connect::MspProcessor processor(
      model);

  Connect::MspMessage reply;

  reply.cmd =
      MSP_ANALOG;

  reply.dir =
      Connect::MSP_TYPE_REPLY;

  reply.state =
      Connect::MSP_STATE_RECEIVED;

  reply.received =
      6;

  processor.processReply(
      reply);

  TEST_ASSERT_FALSE(
      model.state.battery
          .mspCurrentValid);
}


void test_model_sanitize_preserves_msp_current_source()
{
  Model model;

  model.config.ibat.source =
      CURRENT_METER_MSP;

  model.sanitize();

  TEST_ASSERT_EQUAL_INT(
      CURRENT_METER_MSP,
      model.config.ibat.source);

  model.config.ibat.source =
      99;

  model.sanitize();

  TEST_ASSERT_EQUAL_INT(
      CURRENT_METER_NONE,
      model.config.ibat.source);
}


void test_msp_current_meter_reply_updates_battery_state()
{
  ArduinoFakeReset();

  constexpr uint32_t NOW_US =
      2300000;

  When(
      Method(
          ArduinoFake(),
          micros))
      .AlwaysReturn(
          NOW_US);

  Model model;

  model.config.ibat.source =
      CURRENT_METER_MSP;

  Connect::MspProcessor processor(
      model);

  Connect::MspMessage reply;

  reply.cmd =
      MSP_ANALOG;

  reply.dir =
      Connect::MSP_TYPE_REPLY;

  reply.state =
      Connect::MSP_STATE_RECEIVED;

  reply.received =
      7;

  // voltage=12.0V legacy, mAh=321, RSSI=0, current=12.34A.
  const uint8_t payload[7] = {
      120,
      0x41, 0x01,
      0x00, 0x00,
      0xD2, 0x04};

  std::copy(
      payload,
      payload + sizeof(payload),
      reply.buffer);

  processor.processReply(
      reply);

  TEST_ASSERT_TRUE(
      model.state.battery
          .mspCurrentValid);

  TEST_ASSERT_EQUAL_INT16(
      1234,
      model.state.battery
          .mspCurrentCentiAmps);

  TEST_ASSERT_EQUAL_UINT16(
      321,
      model.state.battery
          .mspMahDrawn);

  TEST_ASSERT_EQUAL_UINT32(
      NOW_US,
      model.state.battery
          .mspCurrentLastUpdateUs);
}


void test_voltage_sensor_reads_fresh_msp_current_and_rejects_stale_data()
{
  ArduinoFakeReset();

  constexpr uint32_t NOW_US =
      3000000;

  When(
      Method(
          ArduinoFake(),
          micros))
      .AlwaysReturn(
          NOW_US);

  Model model;

  model.config.ibat.source =
      CURRENT_METER_MSP;

  Sensor::VoltageSensor sensor(
      model);

  sensor.begin();

  model.state.battery
      .mspCurrentCentiAmps =
      750;

  model.state.battery
      .mspMahDrawn =
      42;

  model.state.battery
      .mspCurrentLastUpdateUs =
      NOW_US;

  model.state.battery
      .mspCurrentValid =
      true;

  TEST_ASSERT_EQUAL_INT(
      1,
      sensor.readIbat());

  TEST_ASSERT_EQUAL_INT16(
      750,
      model.state.battery
          .rawCurrent);

  TEST_ASSERT_FLOAT_WITHIN(
      0.001f,
      7.5f,
      model.state.battery
          .currentUnfiltered);

  // Make the same sample older than the 500 ms freshness contract.
  model.state.battery
      .mspCurrentLastUpdateUs =
      NOW_US -
      600000u;

  TEST_ASSERT_EQUAL_INT(
      0,
      sensor.readIbat());

  TEST_ASSERT_FLOAT_WITHIN(
      0.001f,
      0.0f,
      model.state.battery
          .currentUnfiltered);

  TEST_ASSERT_FALSE(
      model.state.battery
          .mspCurrentValid);
}


void test_actuator_angle_fault_requires_switch_cycle()
{
  When(Method(ArduinoFake(), micros)).AlwaysReturn(50000);

  Model model;

  // Required sensor configuration/state.
  model.state.gyro.present =
      true;

  model.state.accel.present =
      true;

  model.state.attitude.healthy =
      true;

  model.state.attitude.lastUpdateUs =
      50000;

  model.state.attitude.quaternion =
      Quaternion(
          1.0f,
          0.0f,
          0.0f,
          0.0f);

  model.state.attitude.euler =
      VectorFloat(
          0.0f,
          0.0f,
          0.0f);

  auto& condition =
      model.config.conditions[0];

  condition.id =
      MODE_ANGLE;

  condition.ch =
      AXIS_AUX_1;

  condition.min =
      1200;

  condition.max =
      1800;

  model.state.input.us[AXIS_AUX_1] =
      1500;

  Actuator actuator(model);

  actuator.begin();

  // Healthy attitude estimator + switch ON.
  actuator.updateModeMask();

  TEST_ASSERT_TRUE(
      model.isModeActive(
          MODE_ANGLE));

  // Simulate attitude-estimator failure.
  model.state.attitude.healthy =
      false;

  actuator.updateModeMask();

  TEST_ASSERT_FALSE(
      model.isModeActive(
          MODE_ANGLE));

  // Estimator recovers while switch remains ON.
  model.state.attitude.healthy =
      true;

  actuator.updateModeMask();

  // It must remain disabled until the pilot cycles
  // the mode switch.
  TEST_ASSERT_FALSE(
      model.isModeActive(
          MODE_ANGLE));

  // Pilot moves switch OFF.
  model.state.input.us[AXIS_AUX_1] =
      1000;

  actuator.updateModeMask();

  TEST_ASSERT_FALSE(
      model.isModeActive(
          MODE_ANGLE));

  // Pilot enables Angle mode deliberately again.
  model.state.input.us[AXIS_AUX_1] =
      1500;

  actuator.updateModeMask();

  TEST_ASSERT_TRUE(
      model.isModeActive(
          MODE_ANGLE));
}
void test_controller_althold_v2_inactive_build_does_not_drive_thrust()
{
  When(
      Method(
          ArduinoFake(),
          micros))
      .AlwaysReturn(
          1000);

  Model model;

  model.state.gyro.clock =
      1000;

  model.config.gyro.dlpf =
      GYRO_DLPF_256;

  model.config.loopSync =
      1;

  model.config.mixerSync =
      1;

  model.config.mixer.type =
      FC_MIXER_QUADX;

  model.begin();

  Controller controller(
      model);

  controller.begin();
    setHealthyAssistedEstimatorState(
      model,
      1000);

  model.state.altitude.height =
      2.0f;

  model.state.altitude.vario =
      0.0f;

  model.state.altitude.healthy =
      true;

  constexpr float MANUAL_THRUST =
      0.35f;

  model.state.input.ch[
      AXIS_THRUST] =
      MANUAL_THRUST;

  model.updateModes(
      uint32_t{1} <<
      MODE_ALTHOLD);

  controller.update();

  // V2 controller state should still update without output authority.
  TEST_ASSERT_TRUE(
      model.state.assistedMode
          .altitudeActive);

  // But actual thrust setpoint must remain manual.
  TEST_ASSERT_FLOAT_WITHIN(
      0.0001f,
      MANUAL_THRUST,
      model.state.setpoint.rate[
          AXIS_THRUST]);

  // And the final controller output must also remain
  // on the manual path.
  TEST_ASSERT_FLOAT_WITHIN(
      0.0001f,
      MANUAL_THRUST,
      model.state.output.ch[
          AXIS_THRUST]);
}

#if defined(ESPFC_ALTHOLD_V2_ACTIVE)

void test_controller_althold_v2_active_path_is_bumpless_and_corrective()
{
  ArduinoFakeReset();

  constexpr uint32_t NOW_US =
      1000000;

  When(
      Method(
          ArduinoFake(),
          micros))
      .AlwaysReturn(
          NOW_US);

  Model model;

  model.state.gyro.clock =
      1000;

  model.config.gyro.dlpf =
      GYRO_DLPF_256;

  model.config.loopSync =
      1;

  model.config.mixerSync =
      1;

  model.config.mixer.type =
      FC_MIXER_QUADX;

  model.begin();

  Controller controller(
      model);

  controller.begin();

  setHealthyAssistedEstimatorState(
      model,
      NOW_US);

  model.state.altitude.height =
      2.0f;

  model.state.altitude.vario =
      0.0f;

  model.state.altitude.healthy =
      true;

  model.state.altitude.lastUpdateUs =
      NOW_US;

  model.state.input.ch[
      ALTHOLD_PILOT_CHANNEL] =
      0.0f;

  constexpr float ENTRY_THRUST =
      0.25f;

  model.state.output.ch[
      AXIS_THRUST] =
      ENTRY_THRUST;

  model.updateModes(
      uint32_t{1} <<
      MODE_ALTHOLD);

  controller.update();

  TEST_ASSERT_TRUE(
      model.state.assistedMode
          .altitudeActive);

  TEST_ASSERT_FLOAT_WITHIN(
      0.0001f,
      model.state.assistedMode
          .verticalRateTarget,
      model.state.setpoint.rate[
          AXIS_THRUST]);

  // Entry target and measured vario are both zero, so the
  // V2 PID should reproduce the pre-existing thrust.
  TEST_ASSERT_FLOAT_WITHIN(
      0.002f,
      ENTRY_THRUST,
      model.state.output.ch[
          AXIS_THRUST]);

  // Simulate being 0.5 m below the captured altitude.
  // Repeated controller cycles should ask for a positive
  // vertical rate and increase thrust.
  model.state.altitude.height =
      1.5f;

  for (int i = 0;
       i < 100;
       ++i)
  {
    controller.update();
  }

  TEST_ASSERT_TRUE(
      model.state.assistedMode
          .verticalRateCorrection >
      0.0f);

  TEST_ASSERT_TRUE(
      model.state.assistedMode
          .verticalRateTarget >
      0.0f);

  TEST_ASSERT_TRUE(
      model.state.output.ch[
          AXIS_THRUST] >
      ENTRY_THRUST);
}

void test_controller_althold_v2_low_manual_throttle_preserves_rate_iterm()
{
  Model model;
  Controller controller(
      model);

  model.config.iterm.lowThrottleZeroIterm =
      true;

  model.state.input.us[
      AXIS_THRUST] =
      1000;

  model.state.innerPid[
      AXIS_ROLL].iTerm =
      0.20f;

  model.state.innerPid[
      AXIS_ROLL].iReset =
      0.0f;

  model.updateModes(
      (uint32_t{1} << MODE_ARMED) |
      (uint32_t{1} << MODE_ALTHOLD));

  controller.resetIterm();

  TEST_ASSERT_FLOAT_WITHIN(
      0.0001f,
      0.20f,
      model.state.innerPid[
          AXIS_ROLL].iTerm);
}

void test_controller_althold_v2_tpa_uses_assisted_thrust()
{
  Model model;
  Controller controller(
      model);

  model.config.controller.tpaScale =
      50;

  model.config.controller.tpaBreakpoint =
      1500;

  // Manual/stateful throttle is deliberately low while AltHold owns thrust.
  model.state.input.us[
      AXIS_THRUST] =
      1000;

  model.state.output.ch[
      AXIS_THRUST] =
      1.0f;

  model.updateModes(
      uint32_t{1} <<
      MODE_ALTHOLD);

  TEST_ASSERT_FLOAT_WITHIN(
      0.001f,
      0.50f,
      controller.getTpaFactor());
}

void test_actuator_althold_v2_allows_stick_deflection_after_entry()
{
  ArduinoFakeReset();

  constexpr uint32_t NOW_US =
      1900000;

  When(
      Method(
          ArduinoFake(),
          micros))
      .AlwaysReturn(
          NOW_US);

  Model model;

  setHealthyAssistedEstimatorState(
      model,
      NOW_US);

  model.state.altitude.healthy =
      true;

  model.state.altitude.lastUpdateUs =
      NOW_US;

  auto& condition =
      model.config.conditions[0];

  condition.id =
      MODE_ALTHOLD;
  condition.ch =
      AXIS_AUX_1;
  condition.min =
      1200;
  condition.max =
      1800;

  model.state.input.us[
      AXIS_AUX_1] =
      1500;

  model.state.input.ch[
      ALTHOLD_PILOT_CHANNEL] =
      0.0f;

  model.state.input.raw[
      ALTHOLD_PILOT_CHANNEL] =
      PWM_RANGE_MID;

  Actuator actuator(
      model);

  actuator.begin();
  actuator.updateModeMask();

  TEST_ASSERT_TRUE(
      model.isModeActive(
          MODE_ALTHOLD));

  // Deflection is a valid climb command after entry, not a mode fault.
  model.state.input.ch[
      ALTHOLD_PILOT_CHANNEL] =
      0.6f;

  model.state.input.raw[
      ALTHOLD_PILOT_CHANNEL] =
      1800;

  actuator.updateModeMask();

  TEST_ASSERT_TRUE(
      model.isModeActive(
          MODE_ALTHOLD));

  TEST_ASSERT_FALSE(
      model.getArmingDisabled(
          ARMING_DISABLED_ALTHOLD));
}

void test_actuator_althold_v2_invalid_pilot_channel_exits_and_latches()
{
  ArduinoFakeReset();

  constexpr uint32_t NOW_US =
      1925000;

  When(
      Method(
          ArduinoFake(),
          micros))
      .AlwaysReturn(
          NOW_US);

  Model model;

  setHealthyAssistedEstimatorState(
      model,
      NOW_US);

  model.state.altitude.healthy =
      true;

  model.state.altitude.lastUpdateUs =
      NOW_US;

  auto& condition =
      model.config.conditions[0];

  condition.id =
      MODE_ALTHOLD;
  condition.ch =
      AXIS_AUX_1;
  condition.min =
      1200;
  condition.max =
      1800;

  model.state.input.us[
      AXIS_AUX_1] =
      1500;

  model.state.input.ch[
      ALTHOLD_PILOT_CHANNEL] =
      0.0f;

  model.state.input.raw[
      ALTHOLD_PILOT_CHANNEL] =
      PWM_RANGE_MID;

  Actuator actuator(
      model);

  actuator.begin();
  actuator.updateModeMask();

  TEST_ASSERT_TRUE(
      model.isModeActive(
          MODE_ALTHOLD));

  // Receiver frame still exists, but the dedicated vertical channel itself
  // has gone out of the accepted RC range.
  model.state.input.raw[
      ALTHOLD_PILOT_CHANNEL] =
      0;

  actuator.updateModeMask();

  TEST_ASSERT_FALSE(
      model.isModeActive(
          MODE_ALTHOLD));

  TEST_ASSERT_TRUE(
      model.getArmingDisabled(
          ARMING_DISABLED_ALTHOLD));

  // A recovered channel must not silently re-enter while the switch stays on.
  model.state.input.raw[
      ALTHOLD_PILOT_CHANNEL] =
      PWM_RANGE_MID;

  actuator.updateModeMask();

  TEST_ASSERT_FALSE(
      model.isModeActive(
          MODE_ALTHOLD));
}

void test_controller_althold_v2_debug_exposes_vertical_control_chain()
{
  ArduinoFakeReset();

  constexpr uint32_t NOW_US =
      1450000;

  When(
      Method(
          ArduinoFake(),
          micros))
      .AlwaysReturn(
          NOW_US);

  Model model;

  model.state.gyro.clock =
      1000;

  model.config.gyro.dlpf =
      GYRO_DLPF_256;

  model.config.loopSync =
      1;

  model.config.mixerSync =
      1;

  model.config.mixer.type =
      FC_MIXER_QUADX;

  model.config.debug.mode =
      DEBUG_AUTOPILOT_PID;

  model.begin();

  Controller controller(
      model);

  controller.begin();

  setHealthyAssistedEstimatorState(
      model,
      NOW_US);

  model.state.altitude.height =
      2.0f;

  model.state.altitude.vario =
      0.15f;

  model.state.altitude.healthy =
      true;

  model.state.altitude.baroAccepted =
      true;

  model.state.altitude.lastUpdateUs =
      NOW_US;

  model.state.input.ch[
      AXIS_THRUST] =
      0.30f;

  model.state.input.ch[
      ALTHOLD_PILOT_CHANNEL] =
      0.0f;

  model.updateModes(
      uint32_t{1} <<
      MODE_ALTHOLD);

  controller.update();

  const uint16_t flags =
      static_cast<uint16_t>(
          model.state.debug[7]);

  TEST_ASSERT_TRUE(
      flags &
      (uint16_t{1} << 0));

  TEST_ASSERT_TRUE(
      flags &
      (uint16_t{1} << 1));

  TEST_ASSERT_TRUE(
      flags &
      (uint16_t{1} << 2));

  TEST_ASSERT_TRUE(
      flags &
      (uint16_t{1} << 3));

  TEST_ASSERT_TRUE(
      flags &
      (uint16_t{1} << 4));

  TEST_ASSERT_TRUE(
      flags &
      (uint16_t{1} << 8));

  TEST_ASSERT_TRUE(
      std::isfinite(
          model.state.innerPid[
              AXIS_THRUST]
              .pTerm));

  TEST_ASSERT_TRUE(
      std::isfinite(
          model.state.innerPid[
              AXIS_THRUST]
              .iTerm));

  TEST_ASSERT_TRUE(
      std::isfinite(
          model.state.output.ch[
              AXIS_THRUST]));
}


#if ESPFC_ALTHOLD_V2_CENTERED_STICK_CHANNEL != 3

void test_controller_althold_v2_uses_dedicated_centered_stick_channel()
{
  ArduinoFakeReset();

  constexpr uint32_t NOW_US =
      1500000;

  When(
      Method(
          ArduinoFake(),
          micros))
      .AlwaysReturn(
          NOW_US);

  Model model;

  model.state.gyro.clock =
      1000;

  model.config.gyro.dlpf =
      GYRO_DLPF_256;

  model.config.loopSync =
      1;

  model.config.mixerSync =
      1;

  model.config.mixer.type =
      FC_MIXER_QUADX;

  model.begin();

  Controller controller(
      model);

  controller.begin();

  setHealthyAssistedEstimatorState(
      model,
      NOW_US);

  model.state.altitude.height =
      2.0f;

  model.state.altitude.vario =
      0.0f;

  model.state.altitude.healthy =
      true;

  model.state.altitude.lastUpdateUs =
      NOW_US;

  // Simulate the project's stateful manual-throttle channel holding a high
  // value while the dedicated raw spring-centered stick is released.
  model.state.input.ch[
      AXIS_THRUST] =
      0.80f;

  model.state.input.ch[
      ALTHOLD_PILOT_CHANNEL] =
      0.0f;

  model.updateModes(
      uint32_t{1} <<
      MODE_ALTHOLD);

  controller.update();

  TEST_ASSERT_FLOAT_WITHIN(
      0.0001f,
      0.0f,
      model.state.assistedMode
          .verticalRatePilot);

  // Moving only the raw centered channel must command climb even though the
  // accumulated manual-throttle channel remains unchanged.
  model.state.input.ch[
      ALTHOLD_PILOT_CHANNEL] =
      0.50f;

  controller.update();

  TEST_ASSERT_TRUE(
      model.state.assistedMode
          .verticalRatePilot >
      0.0f);
}

#endif

void test_actuator_althold_v2_requires_centered_pilot_stick_on_entry()
{
  ArduinoFakeReset();

  constexpr uint32_t NOW_US =
      1800000;

  When(
      Method(
          ArduinoFake(),
          micros))
      .AlwaysReturn(
          NOW_US);

  Model model;

  setHealthyAssistedEstimatorState(
      model,
      NOW_US);

  model.state.altitude.healthy =
      true;

  model.state.altitude.lastUpdateUs =
      NOW_US;

  auto& condition =
      model.config.conditions[0];

  condition.id =
      MODE_ALTHOLD;

  condition.ch =
      AXIS_AUX_1;

  condition.min =
      1200;

  condition.max =
      1800;

  model.state.input.us[
      AXIS_AUX_1] =
      1500;

  // An off-center raw vertical stick must not cause AltHold to engage and
  // immediately command a climb/descent during the mode handoff.
  model.state.input.ch[
      ALTHOLD_PILOT_CHANNEL] =
      0.50f;

  Actuator actuator(
      model);

  actuator.begin();
  actuator.updateModeMask();

  TEST_ASSERT_FALSE(
      model.isModeActive(
          MODE_ALTHOLD));

  TEST_ASSERT_TRUE(
      model.getArmingDisabled(
          ARMING_DISABLED_ALTHOLD));

  // Releasing the spring-centered stick makes the mode eligible without
  // requiring a power cycle.
  model.state.input.ch[
      ALTHOLD_PILOT_CHANNEL] =
      0.0f;

  actuator.updateModeMask();

  TEST_ASSERT_TRUE(
      model.isModeActive(
          MODE_ALTHOLD));

  TEST_ASSERT_FALSE(
      model.getArmingDisabled(
          ARMING_DISABLED_ALTHOLD));
}

#if ESPFC_ALTHOLD_V2_CENTERED_STICK_CHANNEL >= 4
void test_actuator_althold_v2_rejects_missing_pilot_channel()
{
  ArduinoFakeReset();

  constexpr uint32_t NOW_US =
      1850000;

  When(
      Method(
          ArduinoFake(),
          micros))
      .AlwaysReturn(
          NOW_US);

  Model model;

  setHealthyAssistedEstimatorState(
      model,
      NOW_US);

  model.state.altitude.healthy =
      true;

  model.state.altitude.lastUpdateUs =
      NOW_US;

  auto& condition =
      model.config.conditions[0];

  condition.id =
      MODE_ALTHOLD;

  condition.ch =
      AXIS_AUX_1;

  condition.min =
      1200;

  condition.max =
      1800;

  model.state.input.us[
      AXIS_AUX_1] =
      1500;

  model.state.input.ch[
      ALTHOLD_PILOT_CHANNEL] =
      0.0f;

  // Pretend the generic receiver frame itself is valid, but make it too
  // short to contain the configured dedicated AltHold channel.
  model.state.input.channelsValid =
      true;

  model.state.input.channelCount =
      ALTHOLD_PILOT_CHANNEL;

  Actuator actuator(
      model);

  actuator.begin();
  actuator.updateModeMask();

  TEST_ASSERT_FALSE(
      model.isModeActive(
          MODE_ALTHOLD));

  TEST_ASSERT_TRUE(
      model.getArmingDisabled(
          ARMING_DISABLED_ALTHOLD));

  // Recovery while the switch remains ON must not silently re-enter:
  // the dedicated vertical-channel fault is latched until an OFF -> ON cycle.
  model.state.input.channelCount =
      ALTHOLD_PILOT_CHANNEL + 1;

  model.state.input.raw[
      ALTHOLD_PILOT_CHANNEL] =
      PWM_RANGE_MID;

  actuator.updateModeMask();

  TEST_ASSERT_FALSE(
      model.isModeActive(
          MODE_ALTHOLD));

  // Deliberate switch cycle clears the fault latch.
  model.state.input.us[
      AXIS_AUX_1] =
      1000;

  actuator.updateModeMask();

  model.state.input.us[
      AXIS_AUX_1] =
      1500;

  actuator.updateModeMask();

  TEST_ASSERT_TRUE(
      model.isModeActive(
          MODE_ALTHOLD));

  TEST_ASSERT_FALSE(
      model.getArmingDisabled(
          ARMING_DISABLED_ALTHOLD));
}

#endif

void test_actuator_althold_v2_rejects_invalid_raw_pilot_channel()
{
  ArduinoFakeReset();

  constexpr uint32_t NOW_US =
      1875000;

  When(
      Method(
          ArduinoFake(),
          micros))
      .AlwaysReturn(
          NOW_US);

  Model model;

  setHealthyAssistedEstimatorState(
      model,
      NOW_US);

  model.state.altitude.healthy =
      true;

  model.state.altitude.lastUpdateUs =
      NOW_US;

  auto& condition =
      model.config.conditions[0];

  condition.id =
      MODE_ALTHOLD;

  condition.ch =
      AXIS_AUX_1;

  condition.min =
      1200;

  condition.max =
      1800;

  model.state.input.us[
      AXIS_AUX_1] =
      1500;

  model.state.input.ch[
      ALTHOLD_PILOT_CHANNEL] =
      0.0f;

  model.state.input.raw[
      ALTHOLD_PILOT_CHANNEL] =
      0;

  Actuator actuator(
      model);

  actuator.begin();
  actuator.updateModeMask();

  TEST_ASSERT_FALSE(
      model.isModeActive(
          MODE_ALTHOLD));

  TEST_ASSERT_TRUE(
      model.getArmingDisabled(
          ARMING_DISABLED_ALTHOLD));

  model.state.input.raw[
      ALTHOLD_PILOT_CHANNEL] =
      PWM_RANGE_MID;

  actuator.updateModeMask();

  TEST_ASSERT_FALSE(
      model.isModeActive(
          MODE_ALTHOLD));

  // Clear the latched channel fault with a deliberate mode switch cycle.
  model.state.input.us[
      AXIS_AUX_1] =
      1000;

  actuator.updateModeMask();

  model.state.input.us[
      AXIS_AUX_1] =
      1500;

  actuator.updateModeMask();

  TEST_ASSERT_TRUE(
      model.isModeActive(
          MODE_ALTHOLD));
}

#endif

#if defined(ESPFC_LAND_V2_ACTIVE)

void test_controller_land_v2_levels_and_requests_descent()
{
  ArduinoFakeReset();

  constexpr uint32_t NOW_US =
      2000000;

  When(
      Method(
          ArduinoFake(),
          micros))
      .AlwaysReturn(
          NOW_US);

  Model model;

  model.state.gyro.clock =
      1000;

  model.config.gyro.dlpf =
      GYRO_DLPF_256;

  model.config.loopSync =
      1;

  model.config.mixerSync =
      1;

  model.config.mixer.type =
      FC_MIXER_QUADX;

  model.config.debug.mode =
      DEBUG_AUTOPILOT_PID;

  model.begin();

  Controller controller(
      model);

  controller.begin();

  setHealthyAssistedEstimatorState(
      model,
      NOW_US);

  model.state.attitude.euler.set(
      AXIS_ROLL,
      Utils::toRad(
          10.0f));

  model.state.attitude.euler.set(
      AXIS_PITCH,
      Utils::toRad(
          -6.0f));

  model.state.altitude.height =
      3.0f;

  model.state.altitude.vario =
      0.0f;

  model.state.altitude.healthy =
      true;

  model.state.altitude.lastUpdateUs =
      NOW_US;

  model.state.output.ch[
      AXIS_THRUST] =
      0.30f;

  model.state.failsafe.rxEverValid =
      true;

  model.state.failsafe.landingRequested =
      true;

  model.state.failsafe.landingActive =
      true;

  model.state.failsafe.landingOutputBlocked =
      false;

  model.state.failsafe.phase =
      FC_FAILSAFE_LANDING;

  model.updateModes(
      uint32_t{1} <<
      MODE_ARMED);

  controller.update();

  TEST_ASSERT_TRUE(
     model.state.angleV2.active);

  TEST_ASSERT_TRUE(
      model.state.assistedMode
          .altitudeActive);

  // LAND owns the reference: zero attitude and a fixed
  // gentle descent, independent of stale failsafe sticks.
  TEST_ASSERT_TRUE(
     model.state.angleV2.rateTarget[
    AXIS_ROLL] <
      0.0f);

  TEST_ASSERT_TRUE(
      model.state.angleV2.rateTarget[
    AXIS_PITCH] >
      0.0f);

  TEST_ASSERT_FLOAT_WITHIN(
      0.0001f,
      -static_cast<float>(
          ESPFC_LAND_V2_DESCENT_RATE_MS),
      model.state.assistedMode
          .verticalRatePilot);

  TEST_ASSERT_TRUE(
      model.state.assistedMode
          .verticalRateTarget <
      0.0f);

  TEST_ASSERT_FLOAT_WITHIN(
      0.0001f,
      0.0f,
      model.state.setpoint.rate[
          AXIS_YAW]);

  TEST_ASSERT_TRUE(
      std::isfinite(
          model.state.output.ch[
              AXIS_THRUST]));

  const uint16_t debugFlags =
      static_cast<uint16_t>(
          model.state.debug[7]);

  TEST_ASSERT_TRUE(
      debugFlags &
      (uint16_t{1} << 5));

  TEST_ASSERT_TRUE(
      debugFlags &
      (uint16_t{1} << 6));

  TEST_ASSERT_FALSE(
      debugFlags &
      (uint16_t{1} << 7));
}

void test_controller_land_v2_requires_supervisor_authorization()
{
  ArduinoFakeReset();

  constexpr uint32_t NOW_US =
      2950000;

  When(
      Method(
          ArduinoFake(),
          micros))
      .AlwaysReturn(
          NOW_US);

  Model model;

  model.config.mixer.type =
      FC_MIXER_QUADX;

  model.begin();

  Controller controller(
      model);

  controller.begin();

  setHealthyAssistedEstimatorState(
      model,
      NOW_US);

  model.state.altitude.healthy =
      true;

  model.state.altitude.lastUpdateUs =
      NOW_US;

  model.state.failsafe.landingRequested =
      true;

  model.state.failsafe.landingActive =
      false;

  model.state.failsafe.landingOutputBlocked =
      true;

  model.state.failsafe.phase =
      FC_FAILSAFE_LANDING;

  model.updateModes(
      uint32_t{1} <<
      MODE_ARMED);

  model.state.input.ch[
      AXIS_ROLL] =
      0.25f;

  controller.update();

  TEST_ASSERT_FALSE(
     model.state.angleV2.active);

  TEST_ASSERT_FALSE(
      model.state.assistedMode
          .altitudeActive);

  TEST_ASSERT_TRUE(
      std::fabs(
          model.state.setpoint.rate[
              AXIS_ROLL]) >
      0.0001f);
}

void test_failsafe_land_v2_bad_estimator_falls_back_to_disarm()
{
  ArduinoFakeReset();

  constexpr uint32_t NOW_US =
      3000000;

  When(
      Method(
          ArduinoFake(),
          micros))
      .AlwaysReturn(
          NOW_US);

  Model model;

  setHealthyAssistedEstimatorState(
      model,
      NOW_US);

  model.state.altitude.healthy =
      false;

  model.state.altitude.lastUpdateUs =
      NOW_US;

  model.state.failsafe.rxEverValid =
      true;

  model.state.failsafe.landingRequested =
      true;

  model.state.failsafe.landingRequestedUs =
      NOW_US;

  model.state.failsafe.phase =
      FC_FAILSAFE_LANDING;

  model.updateModes(
      uint32_t{1} <<
      MODE_ARMED);

  Actuator actuator(
      model);

  actuator.updateFailsafeLand();

  TEST_ASSERT_FALSE(
      model.isModeActive(
          MODE_ARMED));

  TEST_ASSERT_EQUAL(
      FC_FAILSAFE_LANDED,
      model.state.failsafe.phase);

  TEST_ASSERT_TRUE(
      model.state.failsafe
          .landingFault);

  TEST_ASSERT_TRUE(
      model.state.failsafe
          .landingOutputBlocked);
}

void test_failsafe_land_v2_touchdown_dwell_disarms()
{
  ArduinoFakeReset();

  constexpr uint32_t NOW_US =
      5000000;

  When(
      Method(
          ArduinoFake(),
          micros))
      .AlwaysReturn(
          NOW_US);

  Model model;

  setHealthyAssistedEstimatorState(
      model,
      NOW_US);

  model.state.altitude.healthy =
      true;

  model.state.altitude.lastUpdateUs =
      NOW_US;

  model.state.altitude.height =
      0.10f;

  model.state.altitude.vario =
      0.05f;

  model.state.output.ch[
      AXIS_THRUST] =
      -0.20f;

  model.state.failsafe.rxEverValid =
      true;

  model.state.failsafe.landingRequested =
      true;

  model.state.failsafe.landingRequestedUs =
      NOW_US -
      3000000u;

  model.state.failsafe.landingEntryHeight =
      2.0f;

  model.state.failsafe.landingEntryVario =
      -0.5f;

  model.state.failsafe.landingTouchdownCandidate =
      true;

  model.state.failsafe.landingTouchdownStartedUs =
      NOW_US -
      1000000u;

  model.state.failsafe.phase =
      FC_FAILSAFE_LANDING;

  model.updateModes(
      uint32_t{1} <<
      MODE_ARMED);

  Actuator actuator(
      model);

  actuator.updateFailsafeLand();

  TEST_ASSERT_FALSE(
      model.isModeActive(
          MODE_ARMED));

  TEST_ASSERT_EQUAL(
      FC_FAILSAFE_LANDED,
      model.state.failsafe.phase);

  TEST_ASSERT_FALSE(
      model.state.failsafe
          .landingRequested);

  TEST_ASSERT_TRUE(
      model.state.failsafe
          .landingOutputBlocked);
}

void test_failsafe_land_v2_touchdown_candidate_has_hysteresis()
{
  ArduinoFakeReset();

  constexpr uint32_t NOW_US =
      6500000;

  When(
      Method(
          ArduinoFake(),
          micros))
      .AlwaysReturn(
          NOW_US);

  Model model;

  setHealthyAssistedEstimatorState(
      model,
      NOW_US);

  model.state.altitude.healthy =
      true;

  model.state.altitude.lastUpdateUs =
      NOW_US;

  // Slightly outside the strict entry thresholds, but inside the relaxed
  // hold band. A candidate started on the previous cycle must survive this.
  model.state.altitude.height =
      0.35f;

  model.state.altitude.vario =
      0.20f;

  model.state.output.ch[
      AXIS_THRUST] =
      -0.10f;

  model.state.failsafe.rxEverValid =
      true;

  model.state.failsafe.landingRequested =
      true;

  model.state.failsafe.landingRequestedUs =
      NOW_US -
      3000000u;

  model.state.failsafe.landingEntryHeight =
      2.0f;

  model.state.failsafe.landingTouchdownCandidate =
      true;

  model.state.failsafe.landingTouchdownStartedUs =
      NOW_US -
      500000u;

  model.state.failsafe.phase =
      FC_FAILSAFE_LANDING;

  model.updateModes(
      uint32_t{1} <<
      MODE_ARMED);

  Actuator actuator(
      model);

  actuator.updateFailsafeLand();

  TEST_ASSERT_TRUE(
      model.isModeActive(
          MODE_ARMED));

  TEST_ASSERT_TRUE(
      model.state.failsafe
          .landingTouchdownCandidate);

  TEST_ASSERT_EQUAL(
      FC_FAILSAFE_LANDING,
      model.state.failsafe.phase);
}

void test_failsafe_land_v2_low_entry_thrust_prevents_hover_false_touchdown()
{
  ArduinoFakeReset();

  constexpr uint32_t NOW_US =
      6750000;

  When(
      Method(
          ArduinoFake(),
          micros))
      .AlwaysReturn(
          NOW_US);

  Model model;

  setHealthyAssistedEstimatorState(
      model,
      NOW_US);

  model.state.altitude.healthy =
      true;

  model.state.altitude.lastUpdateUs =
      NOW_US;

  model.state.altitude.height =
      0.10f;

  model.state.altitude.vario =
      0.02f;

  // This aircraft entered LAND while already hovering at a low normalized
  // thrust. The same thrust level near the ground is not a touchdown signal.
  model.state.failsafe.landingEntryThrust =
      -0.90f;

  model.state.output.ch[
      AXIS_THRUST] =
      -0.90f;

  model.state.failsafe.rxEverValid =
      true;

  model.state.failsafe.landingRequested =
      true;

  model.state.failsafe.landingRequestedUs =
      NOW_US -
      3000000u;

  model.state.failsafe.landingEntryHeight =
      2.0f;

  model.state.failsafe.phase =
      FC_FAILSAFE_LANDING;

  model.updateModes(
      uint32_t{1} <<
      MODE_ARMED);

  Actuator actuator(
      model);

  actuator.updateFailsafeLand();

  TEST_ASSERT_TRUE(
      model.isModeActive(
          MODE_ARMED));

  TEST_ASSERT_FALSE(
      model.state.failsafe
          .landingTouchdownCandidate);
}

void test_failsafe_land_v2_near_ground_hover_does_not_disarm()
{
  ArduinoFakeReset();

  constexpr uint32_t NOW_US =
      6000000;

  When(
      Method(
          ArduinoFake(),
          micros))
      .AlwaysReturn(
          NOW_US);

  Model model;

  setHealthyAssistedEstimatorState(
      model,
      NOW_US);

  model.state.altitude.healthy =
      true;

  model.state.altitude.lastUpdateUs =
      NOW_US;

  model.state.altitude.height =
      0.10f;

  model.state.altitude.vario =
      0.02f;

  // At/above the configured hover region: this may be a low hover or a
  // temporarily stalled descent, not confirmed ground contact.
  model.state.output.ch[
      AXIS_THRUST] =
      0.10f;

  model.state.failsafe.rxEverValid =
      true;

  model.state.failsafe.landingRequested =
      true;

  model.state.failsafe.landingRequestedUs =
      NOW_US -
      3000000u;

  model.state.failsafe.landingEntryHeight =
      2.0f;

  model.state.failsafe.landingEntryVario =
      -0.5f;

  model.state.failsafe.phase =
      FC_FAILSAFE_LANDING;

  model.updateModes(
      uint32_t{1} <<
      MODE_ARMED);

  Actuator actuator(
      model);

  actuator.updateFailsafeLand();

  TEST_ASSERT_TRUE(
      model.isModeActive(
          MODE_ARMED));

  TEST_ASSERT_EQUAL(
      FC_FAILSAFE_LANDING,
      model.state.failsafe.phase);

  TEST_ASSERT_FALSE(
      model.state.failsafe
          .landingTouchdownCandidate);
}

void test_failsafe_land_v2_timeout_disarms()
{
  ArduinoFakeReset();

  constexpr float ENTRY_HEIGHT_M =
      1.0f;

  constexpr float RAW_TIMEOUT_S =
      ENTRY_HEIGHT_M /
          static_cast<float>(
              ESPFC_LAND_V2_DESCENT_RATE_MS) +
      10.0f;

  constexpr float TIMEOUT_S =
      RAW_TIMEOUT_S < 15.0f
          ? 15.0f
          : (RAW_TIMEOUT_S > 60.0f
                 ? 60.0f
                 : RAW_TIMEOUT_S);

  constexpr uint32_t TIMEOUT_US =
      static_cast<uint32_t>(
          TIMEOUT_S *
          1000000.0f);

  constexpr uint32_t NOW_US =
      70000000;

  When(
      Method(
          ArduinoFake(),
          micros))
      .AlwaysReturn(
          NOW_US);

  Model model;

  setHealthyAssistedEstimatorState(
      model,
      NOW_US);

  model.state.altitude.healthy =
      true;

  model.state.altitude.lastUpdateUs =
      NOW_US;

  model.state.altitude.height =
      0.80f;

  model.state.altitude.vario =
      -0.40f;

  model.state.failsafe.rxEverValid =
      true;

  model.state.failsafe.landingRequested =
      true;

  // Cross the same bounded timeout that the LAND supervisor derives from
  // the selected production descent rate.
  model.state.failsafe.landingRequestedUs =
      NOW_US -
      TIMEOUT_US;

  model.state.failsafe.landingEntryHeight =
      ENTRY_HEIGHT_M;

  model.state.failsafe.phase =
      FC_FAILSAFE_LANDING;

  model.updateModes(
      uint32_t{1} <<
      MODE_ARMED);

  Actuator actuator(
      model);

  actuator.updateFailsafeLand();

  TEST_ASSERT_FALSE(
      model.isModeActive(
          MODE_ARMED));

  TEST_ASSERT_FALSE(
      model.state.failsafe
          .landingRequested);

  TEST_ASSERT_TRUE(
      model.state.failsafe
          .landingFault);

  TEST_ASSERT_TRUE(
      model.state.failsafe
          .landingOutputBlocked);

  TEST_ASSERT_EQUAL(
      FC_FAILSAFE_LANDED,
      model.state.failsafe.phase);
}

void test_auto_land_arm_request_requires_healthy_altitude_estimator()
{
  ArduinoFakeReset();

  When(
      Method(
          ArduinoFake(),
          micros))
      .AlwaysReturn(
          9000000);

  Model model;

  model.config.failsafe.procedure =
      FAILSAFE_PROCEDURE_AUTO_LAND;

  auto& armCondition =
      model.config.conditions[0];

  armCondition.id =
      MODE_ARMED;

  armCondition.ch =
      AXIS_AUX_1;

  armCondition.min =
      1700;

  armCondition.max =
      2100;

  model.state.input.us[
      AXIS_AUX_1] =
      1800;

  model.state.input.us[
      AXIS_THRUST] =
      1000;

  Actuator actuator(
      model);

  actuator.begin();

  // No valid barometer/altitude estimator has been established.
  actuator.updateModeMask();

  TEST_ASSERT_TRUE(
      model.getArmingDisabled(
          ARMING_DISABLED_ALTHOLD));

  TEST_ASSERT_FALSE(
      model.isModeActive(
          MODE_ARMED));
}

void test_failsafe_land_v2_rx_recovery_stays_committed_to_land()
{
  ArduinoFakeReset();

  constexpr uint32_t NOW_US =
      7000000;

  When(
      Method(
          ArduinoFake(),
          micros))
      .AlwaysReturn(
          NOW_US);

  Model model;

  TelemetryManager telemetry(
      model);

  Input input(
      model,
      telemetry);

  model.state.failsafe.rxEverValid =
      true;

  model.state.failsafe.landingRequested =
      true;

  model.state.failsafe.phase =
      FC_FAILSAFE_LANDING;

  model.state.failsafe.recoveryActive =
      true;

  model.state.failsafe.recoveryStartedUs =
      NOW_US -
      600000u;

  model.state.input.channelsValid =
      true;

  model.state.input.rxLoss =
      true;

  model.state.input.rxFailSafe =
      true;

  model.updateModes(
      uint32_t{1} <<
      MODE_ARMED);

  // Receiver has now been healthy for longer than the 500 ms qualification
  // period, but a failsafe LAND intentionally remains terminal for this
  // armed flight. Pilot input stays gated until touchdown/disarm.
  TEST_ASSERT_TRUE(
      input.failsafe(
          INPUT_RECEIVED));

  TEST_ASSERT_TRUE(
      model.isModeActive(
          MODE_ARMED));

  TEST_ASSERT_TRUE(
      model.state.failsafe
          .landingRequested);

  TEST_ASSERT_FALSE(
      model.state.input.rxLoss);

  TEST_ASSERT_EQUAL(
      FC_FAILSAFE_LANDING,
      model.state.failsafe.phase);
}
#endif

void test_baro_bias_seeds_first_absolute_altitude_sample()
{
  When(
      Method(
          ArduinoFake(),
          micros))
      .AlwaysReturn(
          1000000);

  Model model;

  Espfc::Sensor::BaroSensor sensor(
      model);

  model.state.baro.rate =
      50;

  model.state.baro.altitudeBiasSamples =
      3 *
      model.state.baro.rate;

  // Valid atmospheric pressure well away from sea-level
  // reference, so the old zero-start bias logic would
  // leave a large initial offset.
  model.state.baro.pressure =
      90000.0f;

  model.state.baro.altitudeBias =
      0.0f;

  sensor.updateAltitude();

  TEST_ASSERT_TRUE(
      std::isfinite(
          model.state.baro.altitude));

  // First valid sample must immediately establish the
  // local ground reference.
  TEST_ASSERT_FLOAT_WITHIN(
      0.001f,
      0.0f,
      model.state.baro.altitudeGround);

  TEST_ASSERT_FLOAT_WITHIN(
      0.001f,
      model.state.baro.altitude,
      model.state.baro.altitudeBias);

  TEST_ASSERT_EQUAL_INT32(
      149,
      model.state.baro.altitudeBiasSamples);
}
void test_fusion_rejects_invalid_accel_without_poisoning_state()
{
  // Fusion::update() uses micros().
  When(
      Method(
          ArduinoFake(),
          micros))
      .AlwaysReturn(
          1000);

  // Fusion::begin() writes an initialization message
  // through Logger::info(), and Logger::info() calls
  // millis(). ArduinoFake must therefore provide it.
  When(
      Method(
          ArduinoFake(),
          millis))
      .AlwaysReturn(
          1);

  Model model;

  model.state.loopTimer.setRate(
      500);

  model.state.accel.timer.setRate(
      500);

  model.state.gyro.present =
      true;

  model.state.accel.present =
      true;

  model.config.fusion.mode =
      FUSION_MAHONY;

  model.state.attitude.rate =
      VectorFloat{
          0.0f,
          0.0f,
          0.0f};

  model.state.accel.adc.store(
      VectorFloat{
          0.0f,
          0.0f,
          ACCEL_G});

  Espfc::Control::Fusion fusion(
      model);

  fusion.begin();

  // Establish one known-good AHRS state.
  TEST_ASSERT_EQUAL_INT(
      1,
      fusion.update());

  TEST_ASSERT_TRUE(
      model.state.attitude.healthy);

  const Quaternion goodQ =
      model.state.attitude.quaternion;

  const float nanValue =
      std::numeric_limits<float>
          ::quiet_NaN();

  // Inject corrupt sensor data.
  model.state.accel.adc.store(
      VectorFloat{
          nanValue,
          0.0f,
          ACCEL_G});

  TEST_ASSERT_EQUAL_INT(
      0,
      fusion.update());

  // Last valid quaternion must remain intact.
  TEST_ASSERT_TRUE(
      std::isfinite(
          model.state.attitude
              .quaternion.w));

  TEST_ASSERT_FLOAT_WITHIN(
      0.0001f,
      goodQ.w,
      model.state.attitude
          .quaternion.w);

  TEST_ASSERT_FLOAT_WITHIN(
      0.0001f,
      goodQ.x,
      model.state.attitude
          .quaternion.x);

  TEST_ASSERT_FLOAT_WITHIN(
      0.0001f,
      goodQ.y,
      model.state.attitude
          .quaternion.y);

  TEST_ASSERT_FLOAT_WITHIN(
      0.0001f,
      goodQ.z,
      model.state.attitude
          .quaternion.z);

  // Restore a valid measurement.
  model.state.accel.adc.store(
      VectorFloat{
          0.0f,
          0.0f,
          ACCEL_G});

  // AHRS must continue normally. This proves the bad
  // sample never entered the recursive AHRS state.
  TEST_ASSERT_EQUAL_INT(
      1,
      fusion.update());

  TEST_ASSERT_TRUE(
      model.state.attitude.healthy);
}
void test_rates_betaflight()
{
  InputConfig config;
  config.rateType = RATES_TYPE_BETAFLIGHT;

  config.rate[AXIS_ROLL] = config.rate[AXIS_PITCH] = config.rate[AXIS_YAW] = 70;
  config.expo[AXIS_ROLL] = config.expo[AXIS_PITCH] = config.expo[AXIS_YAW] = 0;
  config.superRate[AXIS_ROLL] = config.superRate[AXIS_PITCH] = config.superRate[AXIS_YAW] = 80;
  config.rateLimit[AXIS_ROLL] = config.rateLimit[AXIS_PITCH] = config.rateLimit[AXIS_YAW] = 1998;

  Rates rates;
  rates.begin(config);

  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, rates.getSetpoint(AXIS_ROLL, 0.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 2.03f, rates.getSetpoint(AXIS_ROLL, 0.5f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 11.92f, rates.getSetpoint(AXIS_ROLL, 1.0f));

  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, rates.getSetpoint(AXIS_PITCH, 0.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 2.03f, rates.getSetpoint(AXIS_PITCH, 0.5f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, -2.03f, rates.getSetpoint(AXIS_PITCH, -0.5f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 11.92f, rates.getSetpoint(AXIS_PITCH, 1.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, -11.92f, rates.getSetpoint(AXIS_PITCH, -1.0f));

  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, rates.getSetpoint(AXIS_YAW, 0.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 2.03f, rates.getSetpoint(AXIS_YAW, 0.5f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 11.92f, rates.getSetpoint(AXIS_YAW, 1.0f));
}

void test_rates_betaflight_expo()
{
  InputConfig config;
  config.rateType = RATES_TYPE_BETAFLIGHT;

  config.rate[AXIS_ROLL] = 70;
  config.expo[AXIS_ROLL] = 10;
  config.superRate[AXIS_ROLL] = 80;
  config.rateLimit[AXIS_ROLL] = 1998;

  Rates rates;
  rates.begin(config);

  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, rates.getSetpoint(AXIS_ROLL, 0.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.24f, rates.getSetpoint(AXIS_ROLL, 0.1f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.52f, rates.getSetpoint(AXIS_ROLL, 0.2f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.87f, rates.getSetpoint(AXIS_ROLL, 0.3f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.30f, rates.getSetpoint(AXIS_ROLL, 0.4f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.86f, rates.getSetpoint(AXIS_ROLL, 0.5f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 2.60f, rates.getSetpoint(AXIS_ROLL, 0.6f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 3.63f, rates.getSetpoint(AXIS_ROLL, 0.7f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 5.16f, rates.getSetpoint(AXIS_ROLL, 0.8f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 7.64f, rates.getSetpoint(AXIS_ROLL, 0.9f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 11.90f, rates.getSetpoint(AXIS_ROLL, 1.0f));
}

void test_rates_raceflight()
{
  InputConfig config;
  config.rateType = RATES_TYPE_RACEFLIGHT;

  config.rate[AXIS_ROLL] = 70;
  config.expo[AXIS_ROLL] = 0;
  config.superRate[AXIS_ROLL] = 80;
  config.rateLimit[AXIS_ROLL] = 1998;

  Rates rates;
  rates.begin(config);

  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, rates.getSetpoint(AXIS_ROLL, 0.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 2.83f, rates.getSetpoint(AXIS_ROLL, 0.2f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 6.45f, rates.getSetpoint(AXIS_ROLL, 0.4f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 8.55f, rates.getSetpoint(AXIS_ROLL, 0.5f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 10.85f, rates.getSetpoint(AXIS_ROLL, 0.6f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 16.03f, rates.getSetpoint(AXIS_ROLL, 0.8f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 21.83f, rates.getSetpoint(AXIS_ROLL, 1.0f));

  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, rates.getSetpoint(AXIS_ROLL, 0.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, -8.55f, rates.getSetpoint(AXIS_ROLL, -0.5f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, -21.83f, rates.getSetpoint(AXIS_ROLL, -1.0f));
}

void test_rates_raceflight_expo()
{
  InputConfig config;
  config.rateType = RATES_TYPE_RACEFLIGHT;

  config.rate[AXIS_ROLL] = 70;
  config.expo[AXIS_ROLL] = 20;
  config.superRate[AXIS_ROLL] = 80;
  config.rateLimit[AXIS_ROLL] = 1998;

  Rates rates;
  rates.begin(config);

  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, rates.getSetpoint(AXIS_ROLL, 0.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 2.29f, rates.getSetpoint(AXIS_ROLL, 0.2f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 5.36f, rates.getSetpoint(AXIS_ROLL, 0.4f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 7.27f, rates.getSetpoint(AXIS_ROLL, 0.5f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 9.46f, rates.getSetpoint(AXIS_ROLL, 0.6f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 14.87f, rates.getSetpoint(AXIS_ROLL, 0.8f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 21.79f, rates.getSetpoint(AXIS_ROLL, 1.0f));

  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, rates.getSetpoint(AXIS_ROLL, 0.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, -7.27f, rates.getSetpoint(AXIS_ROLL, -0.5f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, -21.79f, rates.getSetpoint(AXIS_ROLL, -1.0f));
}

void test_rates_kiss()
{
  InputConfig config;
  config.rateType = RATES_TYPE_KISS;

  config.rate[AXIS_ROLL] = 70;
  config.expo[AXIS_ROLL] = 0;
  config.superRate[AXIS_ROLL] = 80;
  config.rateLimit[AXIS_ROLL] = 1998;

  Rates rates;
  rates.begin(config);

  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, rates.getSetpoint(AXIS_ROLL, 0.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.59f, rates.getSetpoint(AXIS_ROLL, 0.2f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.43f, rates.getSetpoint(AXIS_ROLL, 0.4f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 2.04f, rates.getSetpoint(AXIS_ROLL, 0.5f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 2.81f, rates.getSetpoint(AXIS_ROLL, 0.6f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 5.43f, rates.getSetpoint(AXIS_ROLL, 0.8f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 11.92f, rates.getSetpoint(AXIS_ROLL, 1.0f));

  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, rates.getSetpoint(AXIS_ROLL, 0.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, -2.04f, rates.getSetpoint(AXIS_ROLL, -0.5f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, -11.92f, rates.getSetpoint(AXIS_ROLL, -1.0f));
}

void test_rates_kiss_expo()
{
  InputConfig config;
  config.rateType = RATES_TYPE_KISS;

  config.rate[AXIS_ROLL] = 70;
  config.expo[AXIS_ROLL] = 20;
  config.superRate[AXIS_ROLL] = 80;
  config.rateLimit[AXIS_ROLL] = 1998;

  Rates rates;
  rates.begin(config);

  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, rates.getSetpoint(AXIS_ROLL, 0.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.47f, rates.getSetpoint(AXIS_ROLL, 0.2f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.20f, rates.getSetpoint(AXIS_ROLL, 0.4f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.73f, rates.getSetpoint(AXIS_ROLL, 0.5f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 2.46f, rates.getSetpoint(AXIS_ROLL, 0.6f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 5.04f, rates.getSetpoint(AXIS_ROLL, 0.8f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 11.89f, rates.getSetpoint(AXIS_ROLL, 1.0f));

  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, rates.getSetpoint(AXIS_ROLL, 0.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, -1.73f, rates.getSetpoint(AXIS_ROLL, -0.5f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, -11.89f, rates.getSetpoint(AXIS_ROLL, -1.0f));
}

void test_actuator_stale_gyro_blocks_arm_and_disarms()
{
  ArduinoFakeReset();

  constexpr uint32_t NOW_US =
      500000;

  When(
      Method(
          ArduinoFake(),
          micros))
      .AlwaysReturn(
          NOW_US);

  Model model;

  model.state.gyro.present =
      true;

  model.state.gyro.sampleValid =
      true;

  model.state.gyro.lastUpdateUs =
      NOW_US -
      200000u;

  model.config.output.protocol =
      ESC_PROTOCOL_DSHOT150;

  model.state.input.us[
      AXIS_THRUST] =
      1000;

  Actuator actuator(
      model);

  actuator.begin();

  model.updateModes(
      uint32_t{1} <<
      MODE_ARMED);

  actuator.updateArmingDisabled();

  TEST_ASSERT_TRUE(
      model.getArmingDisabled(
          ARMING_DISABLED_NO_GYRO));

  TEST_ASSERT_FALSE(
      model.isModeActive(
          MODE_ARMED));

  TEST_ASSERT_EQUAL_UINT32(
      DISARM_REASON_SYSTEM,
      model.state.mode.disarmReason);
}

void test_actuator_fresh_gyro_passes_runtime_health_gate()
{
  ArduinoFakeReset();

  constexpr uint32_t NOW_US =
      500000;

  When(
      Method(
          ArduinoFake(),
          micros))
      .AlwaysReturn(
          NOW_US);

  Model model;

  model.state.gyro.present =
      true;

  model.state.gyro.sampleValid =
      true;

  model.state.gyro.lastUpdateUs =
      NOW_US -
      1000u;

  model.config.output.protocol =
      ESC_PROTOCOL_DSHOT150;

  model.state.input.us[
      AXIS_THRUST] =
      1000;

  Actuator actuator(
      model);

  actuator.begin();
  actuator.updateArmingDisabled();

  TEST_ASSERT_FALSE(
      model.getArmingDisabled(
          ARMING_DISABLED_NO_GYRO));
}

void test_actuator_arming_gyro_motor_calbration()
{
  Model model;
  // model.begin();

  Actuator actuator(model);
  actuator.begin();

  TEST_ASSERT_EQUAL_UINT32(0, model.state.mode.armingDisabledFlags);

  actuator.updateArmingDisabled();

  TEST_ASSERT_EQUAL_UINT32(ARMING_DISABLED_NO_GYRO | ARMING_DISABLED_MOTOR_PROTOCOL,
                           model.state.mode.armingDisabledFlags);
}

void test_actuator_arming_failsafe()
{
  ArduinoFakeReset();
  When(Method(ArduinoFake(), micros)).AlwaysReturn(1000);

  Model model;
  model.state.gyro.present = true;
  model.state.gyro.sampleValid = true;
  model.state.gyro.lastUpdateUs = 1000;
  model.config.output.protocol = ESC_PROTOCOL_DSHOT150;
  model.state.failsafe.phase = FC_FAILSAFE_RX_LOSS_DETECTED;
  model.state.gyro.calibrationState = CALIBRATION_UPDATE;
  model.state.input.rxFailSafe = true;
  model.state.input.rxLoss = true;

  // model.begin();

  Actuator actuator(model);
  actuator.begin();

  TEST_ASSERT_EQUAL_UINT32(0, model.state.mode.armingDisabledFlags);

  actuator.updateArmingDisabled();

  TEST_ASSERT_EQUAL_UINT32(ARMING_DISABLED_RX_FAILSAFE | ARMING_DISABLED_FAILSAFE | ARMING_DISABLED_CALIBRATING,
                           model.state.mode.armingDisabledFlags);
}

void test_actuator_arming_throttle()
{
  ArduinoFakeReset();
  When(Method(ArduinoFake(), micros)).AlwaysReturn(1000);

  Model model;
  model.config.output.protocol = ESC_PROTOCOL_DSHOT150;
  model.config.input.minCheck = 1050;
  model.state.input.us[AXIS_THRUST] = 1100;
  model.state.gyro.present = true;
  model.state.gyro.sampleValid = true;
  model.state.gyro.lastUpdateUs = 1000;

  // model.begin();

  Actuator actuator(model);
  actuator.begin();

  TEST_ASSERT_EQUAL_UINT32(0, model.state.mode.armingDisabledFlags);

  actuator.updateArmingDisabled();

  TEST_ASSERT_EQUAL_UINT32(ARMING_DISABLED_THROTTLE, model.state.mode.armingDisabledFlags);
}

void test_mixer_invalid_custom_entries_are_ignored()
{
  Model model;
  Output::Mixer mixer(
      model);

  MixerEntry rules[MIXER_RULE_MAX] = {};

  rules[0] =
      MixerEntry(
          -1,
          0,
          100);

  rules[1] =
      MixerEntry(
          MIXER_SOURCE_ROLL,
          -1,
          100);

  rules[2] =
      MixerEntry(
          MIXER_SOURCE_MAX,
          0,
          100);

  rules[3] =
      MixerEntry(
          MIXER_SOURCE_ROLL,
          0,
          100);

  rules[4] =
      MixerEntry();

  MixerConfig config(
      4,
      rules);

  model.state.output.ch[
      AXIS_ROLL] =
      0.25f;

  float outputs[
      OUTPUT_CHANNELS];

  std::fill_n(
      outputs,
      OUTPUT_CHANNELS,
      99.0f);

  mixer.updateMixer(
      config,
      outputs);

  TEST_ASSERT_FLOAT_WITHIN(
      0.0001f,
      0.25f,
      outputs[0]);

  for (size_t i = 1;
       i < OUTPUT_CHANNELS;
       ++i)
  {
    TEST_ASSERT_FLOAT_WITHIN(
        0.0001f,
        0.0f,
        outputs[i]);
  }
}

void test_mixer_invalid_output_count_fails_closed()
{
  Model model;
  Output::Mixer mixer(
      model);

  MixerEntry rules[MIXER_RULE_MAX] = {};
  rules[0] =
      MixerEntry(
          MIXER_SOURCE_THRUST,
          0,
          100);
  rules[1] =
      MixerEntry();

  MixerConfig invalidConfig(
      255,
      rules);

  model.updateModes(
      uint32_t{1} <<
      MODE_ARMED);

  for (size_t i = 0;
       i < OUTPUT_CHANNELS;
       ++i)
  {
    model.state.output.disarmed[i] =
        1000;

    model.state.output.us[i] =
        1800;
  }

  float outputs[
      OUTPUT_CHANNELS] = {};

  mixer.writeOutput(
      invalidConfig,
      outputs);

  for (size_t i = 0;
       i < OUTPUT_CHANNELS;
       ++i)
  {
    TEST_ASSERT_EQUAL_INT16(
        1000,
        model.state.output.us[i]);
  }
}

void test_mixer_throttle_limit_none()
{
  Model model;
  Output::Mixer mixer(model);

  TEST_ASSERT_FLOAT_WITHIN(0.001f, -1.0f, mixer.limitThrust(-1.0f, THROTTLE_LIMIT_TYPE_NONE, 100));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, mixer.limitThrust(0.0f, THROTTLE_LIMIT_TYPE_NONE, 100));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.5f, mixer.limitThrust(0.5f, THROTTLE_LIMIT_TYPE_NONE, 100));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, mixer.limitThrust(1.0f, THROTTLE_LIMIT_TYPE_NONE, 100));
}

void test_mixer_throttle_limit_scale()
{
  Model model;
  Output::Mixer mixer(model);

  TEST_ASSERT_FLOAT_WITHIN(0.001f, -1.0f, mixer.limitThrust(-1.0f, THROTTLE_LIMIT_TYPE_SCALE, 100));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, mixer.limitThrust(0.0f, THROTTLE_LIMIT_TYPE_SCALE, 100));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.5f, mixer.limitThrust(0.5f, THROTTLE_LIMIT_TYPE_SCALE, 100));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, mixer.limitThrust(1.0f, THROTTLE_LIMIT_TYPE_SCALE, 100));

  TEST_ASSERT_FLOAT_WITHIN(0.001f, -1.0f, mixer.limitThrust(-1.0f, THROTTLE_LIMIT_TYPE_SCALE, 0));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, mixer.limitThrust(0.0f, THROTTLE_LIMIT_TYPE_SCALE, 0));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.5f, mixer.limitThrust(0.5f, THROTTLE_LIMIT_TYPE_SCALE, 0));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, mixer.limitThrust(1.0f, THROTTLE_LIMIT_TYPE_SCALE, 0));

  TEST_ASSERT_FLOAT_WITHIN(0.001f, -1.0f, mixer.limitThrust(-1.0f, THROTTLE_LIMIT_TYPE_SCALE, 110));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, mixer.limitThrust(0.0f, THROTTLE_LIMIT_TYPE_SCALE, 110));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.5f, mixer.limitThrust(0.5f, THROTTLE_LIMIT_TYPE_SCALE, 110));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, mixer.limitThrust(1.0f, THROTTLE_LIMIT_TYPE_SCALE, 110));

  TEST_ASSERT_FLOAT_WITHIN(0.001f, -1.00f, mixer.limitThrust(-1.0f, THROTTLE_LIMIT_TYPE_SCALE, 80));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, -0.20f, mixer.limitThrust(0.0f, THROTTLE_LIMIT_TYPE_SCALE, 80));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.20f, mixer.limitThrust(0.5f, THROTTLE_LIMIT_TYPE_SCALE, 80));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.60f, mixer.limitThrust(1.0f, THROTTLE_LIMIT_TYPE_SCALE, 80));
}

void test_mixer_throttle_limit_clip()
{
  Model model;
  Output::Mixer mixer(model);

  TEST_ASSERT_FLOAT_WITHIN(0.001f, -1.0f, mixer.limitThrust(-1.0f, THROTTLE_LIMIT_TYPE_CLIP, 100));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, mixer.limitThrust(0.0f, THROTTLE_LIMIT_TYPE_CLIP, 100));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.5f, mixer.limitThrust(0.5f, THROTTLE_LIMIT_TYPE_CLIP, 100));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, mixer.limitThrust(1.0f, THROTTLE_LIMIT_TYPE_CLIP, 100));

  TEST_ASSERT_FLOAT_WITHIN(0.001f, -1.0f, mixer.limitThrust(-1.0f, THROTTLE_LIMIT_TYPE_CLIP, 0));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, mixer.limitThrust(0.0f, THROTTLE_LIMIT_TYPE_CLIP, 0));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.5f, mixer.limitThrust(0.5f, THROTTLE_LIMIT_TYPE_CLIP, 0));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, mixer.limitThrust(1.0f, THROTTLE_LIMIT_TYPE_CLIP, 0));

  TEST_ASSERT_FLOAT_WITHIN(0.001f, -1.0f, mixer.limitThrust(-1.0f, THROTTLE_LIMIT_TYPE_CLIP, 110));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, mixer.limitThrust(0.0f, THROTTLE_LIMIT_TYPE_CLIP, 110));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.5f, mixer.limitThrust(0.5f, THROTTLE_LIMIT_TYPE_CLIP, 110));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, mixer.limitThrust(1.0f, THROTTLE_LIMIT_TYPE_CLIP, 110));

  TEST_ASSERT_FLOAT_WITHIN(0.001f, -1.00f, mixer.limitThrust(-1.0f, THROTTLE_LIMIT_TYPE_CLIP, 80));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.00f, mixer.limitThrust(0.0f, THROTTLE_LIMIT_TYPE_CLIP, 80));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.50f, mixer.limitThrust(0.5f, THROTTLE_LIMIT_TYPE_CLIP, 80));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.60f, mixer.limitThrust(1.0f, THROTTLE_LIMIT_TYPE_CLIP, 80));
}

void test_mixer_output_limit_motor()
{
  Model model;
  Output::Mixer mixer(model);
  OutputChannelConfig motor = {.servo = false};

  TEST_ASSERT_FLOAT_WITHIN(0.001f, -1.0f, mixer.limitOutput(-1.0f, motor, 100));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, mixer.limitOutput(0.0f, motor, 100));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.5f, mixer.limitOutput(0.5f, motor, 100));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, mixer.limitOutput(1.0f, motor, 100));

  TEST_ASSERT_FLOAT_WITHIN(0.001f, -1.0f, mixer.limitOutput(-1.0f, motor, 120));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, mixer.limitOutput(0.0f, motor, 120));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.5f, mixer.limitOutput(0.5f, motor, 120));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, mixer.limitOutput(1.0f, motor, 120));

  TEST_ASSERT_FLOAT_WITHIN(0.001f, -1.0f, mixer.limitOutput(-1.0f, motor, 0));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, mixer.limitOutput(0.0f, motor, 0));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.5f, mixer.limitOutput(0.5f, motor, 0));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, mixer.limitOutput(1.0f, motor, 0));

  TEST_ASSERT_FLOAT_WITHIN(0.001f, -1.0f, mixer.limitOutput(-1.0f, motor, 80));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, mixer.limitOutput(0.0f, motor, 80));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.5f, mixer.limitOutput(0.5f, motor, 80));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.6f, mixer.limitOutput(1.0f, motor, 80));
}

void test_mixer_output_limit_servo()
{
  Model model;
  Output::Mixer mixer(model);
  OutputChannelConfig servo = {.servo = true};

  TEST_ASSERT_FLOAT_WITHIN(0.001f, -1.0f, mixer.limitOutput(-1.0f, servo, 100));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, mixer.limitOutput(0.0f, servo, 100));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.5f, mixer.limitOutput(0.5f, servo, 100));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, mixer.limitOutput(1.0f, servo, 100));

  TEST_ASSERT_FLOAT_WITHIN(0.001f, -1.0f, mixer.limitOutput(-1.0f, servo, 120));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, mixer.limitOutput(0.0f, servo, 120));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.5f, mixer.limitOutput(0.5f, servo, 120));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, mixer.limitOutput(1.0f, servo, 120));

  TEST_ASSERT_FLOAT_WITHIN(0.001f, -1.0f, mixer.limitOutput(-1.0f, servo, 0));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, mixer.limitOutput(0.0f, servo, 0));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.5f, mixer.limitOutput(0.5f, servo, 0));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, mixer.limitOutput(1.0f, servo, 0));

  TEST_ASSERT_FLOAT_WITHIN(0.001f, -0.8f, mixer.limitOutput(-1.0f, servo, 80));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, mixer.limitOutput(0.0f, servo, 80));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.5f, mixer.limitOutput(0.5f, servo, 80));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.8f, mixer.limitOutput(1.0f, servo, 80));
}
void test_altitude_baro_vario_used_only_once_per_sample()
{
  // Two altitude-estimator updates:
  //
  // update #1 = new barometer sample
  // update #2 = same barometer timestamp, therefore
  //             NOT a new barometer sample.
When(
    Method(
        ArduinoFake(),
        micros))
    .Return(
        900,   // Stats start - update 1
        1000,  // Altitude estimator time - update 1
        1100,  // Stats end - update 1

        1900,  // Stats start - update 2
        2000,  // Altitude estimator time - update 2
        2100); // Stats end - update 2

  Model model;

  // --------------------------------------------------
  // Estimator rates
  // --------------------------------------------------

  model.state.accel.timer.rate =
      1000;

  model.state.baro.rate =
      50;

  // Altitude.cpp multiplies this by 0.1,
  // therefore effective complementary tau = 1 second.
  model.config.altHold.baroTau =
      10;

  // --------------------------------------------------
  // Valid/fresh attitude projection
  // --------------------------------------------------

  model.state.attitude.healthy =
      true;

  model.state.attitude.lastUpdateUs =
      1000;

  // No vertical acceleration.
  model.state.accel.world.z =
      0.0f;

  // --------------------------------------------------
  // Valid barometer state
  // --------------------------------------------------

  model.state.baro.sampleValid =
      true;

  model.state.baro.lastUpdateUs =
      1000;

  // Startup bias already completed.
  model.state.baro.altitudeBiasSamples =
      -1;

  model.state.baro.altitudeGround =
      0.0f;

  // Give the estimator a clearly non-zero
  // vertical-speed measurement.
  model.state.baro.vario =
      2.0f;

  Control::Altitude altitude(
      model);

  altitude.begin();

  // --------------------------------------------------
  // First update:
  // barometer timestamp is new, therefore Vz may
  // influence the complementary filter.
  // --------------------------------------------------

  altitude.update();
    

  const float firstVario =
      model.state.altitude.vario;

  TEST_ASSERT_TRUE(
      std::isfinite(
          firstVario));

  TEST_ASSERT_TRUE(
      std::fabs(
          firstVario) >
      1.0e-7f);

  // --------------------------------------------------
  // Second update:
  //
  // baro.lastUpdateUs has NOT changed.
  //
  // Correct behaviour:
  //   newBaroSample == false
  //
  // Therefore the old barometer Vz must NOT be injected
  // into the complementary filter a second time.
  //
  // With zero acceleration, predicted Vz equals the
  // previous filter state, so Vz should remain unchanged.
  // --------------------------------------------------

  altitude.update();

  const float secondVario =
      model.state.altitude.vario;

  TEST_ASSERT_TRUE(
      std::isfinite(
          secondVario));

  TEST_ASSERT_FLOAT_WITHIN(
      1.0e-7f,
      firstVario,
      secondVario);
    

}
void test_complementary_variable_dt()
{
  Complementary nominalFilter;
  Complementary delayedFilter;
  Complementary legacyFilter;

  constexpr float SAMPLE_RATE =
      1000.0f;

  constexpr float TAU =
      1.0f;

  constexpr float RATE =
      1.0f;

  constexpr float POSITION =
      0.0f;

  // --------------------------------------------------
  // Nominal 1 ms update
  // --------------------------------------------------

  nominalFilter.begin(
      SAMPLE_RATE,
      TAU,
      0.0f);

  const float nominal =
      nominalFilter.update(
          RATE,
          POSITION,
          0.001f);

  // Expected:
  //
  // alpha =
  //   1 / (1 + 0.001)
  //
  // state =
  //   alpha * (0 + 1 * 0.001)
  //
  // ~= 0.000999001
  TEST_ASSERT_FLOAT_WITHIN(
      1.0e-6f,
      0.000999001f,
      nominal);

  // --------------------------------------------------
  // Simulated delayed estimator cycle: 10 ms
  // --------------------------------------------------

  delayedFilter.begin(
      SAMPLE_RATE,
      TAU,
      0.0f);

  const float delayed =
      delayedFilter.update(
          RATE,
          POSITION,
          0.010f);

  // Expected:
  //
  // alpha =
  //   1 / (1 + 0.010)
  //
  // state =
  //   alpha * (0 + 1 * 0.010)
  //
  // ~= 0.00990099
  TEST_ASSERT_FLOAT_WITHIN(
      1.0e-6f,
      0.00990099f,
      delayed);

  // Most important assertion:
  //
  // A 10 ms integration interval must produce a
  // substantially larger integrated rate contribution
  // than a 1 ms interval.
  TEST_ASSERT_TRUE(
      delayed >
      nominal * 9.0f);

  // --------------------------------------------------
  // Backward-compatible two-argument update()
  //
  // It should still use the nominal sample period.
  // --------------------------------------------------

  legacyFilter.begin(
      SAMPLE_RATE,
      TAU,
      0.0f);

  const float legacy =
      legacyFilter.update(
          RATE,
          POSITION);

  TEST_ASSERT_FLOAT_WITHIN(
      1.0e-6f,
      nominal,
      legacy);
}

// =========================================================
// FINAL ESTIMATOR / FAILSAFE REGRESSION TESTS
// =========================================================

static void prepareAltitudeRegressionModel(
    Model& model,
    int accelRate,
    uint32_t nowUs)
{
  model.state.accel.timer.rate =
      accelRate;

  model.state.baro.rate =
      50;

  // Effective vertical-velocity correction tau = 1 second.
  model.config.altHold.baroTau =
      10;

  // Valid attitude solution.
  model.state.attitude.healthy =
      true;

  model.state.attitude.lastUpdateUs =
      nowUs;

  model.state.attitude.quaternion =
      Quaternion(
          1.0f,
          0.0f,
          0.0f,
          0.0f);

  model.state.attitude.euler =
      VectorFloat(
          0.0f,
          0.0f,
          0.0f);

  // No vertical acceleration unless a test overrides it.
  model.state.accel.world.z =
      0.0f;

  // Valid barometer state.
  model.state.baro.sampleValid =
      true;

  model.state.baro.lastUpdateUs =
      nowUs - 20000u;

  model.state.baro.altitudeBiasSamples =
      -1;

  model.state.baro.altitudeGround =
      0.0f;

  model.state.baro.vario =
      0.0f;
}


// =========================================================
// 1. BAROMETER CORRECTION MUST NOT DEPEND ON IMU RATE
// =========================================================

void test_altitude_correction_independent_of_imu_rate()
{
  constexpr uint32_t NOW_US =
      100000;

  When(
      Method(
          ArduinoFake(),
          micros))
      .AlwaysReturn(
          NOW_US);

  Model model500;
  Model model1000;

  prepareAltitudeRegressionModel(
      model500,
      500,
      NOW_US);

  prepareAltitudeRegressionModel(
      model1000,
      1000,
      NOW_US);

  // Same barometer observation for both estimators.
  model500.state.baro.vario =
      1.0f;

  model1000.state.baro.vario =
      1.0f;

  Control::Altitude altitude500(
      model500);

  Control::Altitude altitude1000(
      model1000);

  altitude500.begin();
  altitude1000.begin();

  altitude500.update(
      true);

  altitude1000.update(
      true);

  TEST_ASSERT_TRUE(
      std::isfinite(
          model500.state.altitude.vario));

  TEST_ASSERT_TRUE(
      std::isfinite(
          model1000.state.altitude.vario));

  // Same barometer rate + same observation + same tau
  // must produce the same correction regardless of
  // whether prediction is running at 500 or 1000 Hz.
  TEST_ASSERT_FLOAT_WITHIN(
      1.0e-6f,
      model500.state.altitude.vario,
      model1000.state.altitude.vario);
}


// =========================================================
// 2. FAILED AHRS CYCLE MUST NOT RE-INTEGRATE OLD WORLD ACCEL
// =========================================================

void test_rejected_fusion_does_not_reintegrate_world_accel()
{
  constexpr uint32_t NOW_US =
      100000;

  When(
      Method(
          ArduinoFake(),
          micros))
      .AlwaysReturn(
          NOW_US);

  Model model;

  model.state.accel.timer.rate =
      1000;

  model.state.baro.rate =
      50;

  model.state.attitude.healthy =
      true;

  model.state.attitude.lastUpdateUs =
      NOW_US;

  // Deliberately non-zero cached world acceleration.
  model.state.accel.world.z =
      4.0f;

  // No barometer correction is needed for this test.
  model.state.baro.sampleValid =
      false;

  Control::Altitude altitude(
      model);

  altitude.begin();

  // fusionValid == false means accel.world belongs to
  // the previous successful AHRS cycle and must not
  // be integrated again.
  altitude.update(
      false);

  TEST_ASSERT_TRUE(
      std::isfinite(
          model.state.altitude.vario));

  TEST_ASSERT_FLOAT_WITHIN(
      1.0e-7f,
      0.0f,
      model.state.altitude.vario);
}


// =========================================================
// 3. REJECTED HEIGHT SAMPLE MUST NOT CORRECT VERTICAL SPEED
// =========================================================

void test_rejected_height_sample_does_not_correct_vario()
{
  constexpr uint32_t NOW_US =
      100000;

  When(
      Method(
          ArduinoFake(),
          micros))
      .AlwaysReturn(
          NOW_US);

  Model model;

  prepareAltitudeRegressionModel(
      model,
      1000,
      NOW_US);

  Control::Altitude altitude(
      model);

  altitude.begin();

  // --------------------------------------------------
  // First sample: establish valid estimator reference.
  // --------------------------------------------------

  model.state.baro.lastUpdateUs =
      80000;

  model.state.baro.altitudeGround =
      0.0f;

  model.state.baro.vario =
      0.0f;

  altitude.update(
      true);

  TEST_ASSERT_TRUE(
      model.state.altitude.baroAccepted);

  const float varioBeforeOutlier =
      model.state.altitude.vario;

  // --------------------------------------------------
  // Second sample:
  // gigantic height + velocity disturbance.
  //
  // Height innovation must reject this observation and
  // the associated velocity observation must not alter Vz.
  // --------------------------------------------------

  model.state.baro.lastUpdateUs =
      90000;

  model.state.baro.altitudeGround =
      1000.0f;

  model.state.baro.vario =
      100.0f;

  altitude.update(
      true);

  TEST_ASSERT_FALSE(
      model.state.altitude.baroAccepted);

  TEST_ASSERT_TRUE(
      std::fabs(
          model.state.altitude
              .baroInnovation) >
      1.5f);

  TEST_ASSERT_FLOAT_WITHIN(
      1.0e-6f,
      varioBeforeOutlier,
      model.state.altitude.vario);
}


// =========================================================
// 4. FILTER RELOAD MUST PRESERVE ALTITUDE ESTIMATOR HISTORY
// =========================================================

void test_altitude_reload_preserves_filter_history()
{
  constexpr uint32_t NOW_US =
      100000;

  When(
      Method(
          ArduinoFake(),
          micros))
      .AlwaysReturn(
          NOW_US);

  Model referenceModel;
  Model reloadModel;

  prepareAltitudeRegressionModel(
      referenceModel,
      1000,
      NOW_US);

  prepareAltitudeRegressionModel(
      reloadModel,
      1000,
      NOW_US);

  referenceModel.state.baro.altitudeGround =
      1.0f;

  reloadModel.state.baro.altitudeGround =
      1.0f;

  referenceModel.state.baro.vario =
      2.0f;

  reloadModel.state.baro.vario =
      2.0f;

  referenceModel.state.baro.lastUpdateUs =
      80000;

  reloadModel.state.baro.lastUpdateUs =
      80000;

  Control::Altitude referenceAltitude(
      referenceModel);

  Control::Altitude reloadedAltitude(
      reloadModel);

  referenceAltitude.begin();
  reloadedAltitude.begin();

  referenceAltitude.update(
      true);

  reloadedAltitude.update(
      true);

  // Reconfigure only one estimator.
  //
  // Correct reload behavior preserves its filter history,
  // therefore it should still match the untouched
  // reference estimator afterward.
  reloadedAltitude.reload(
      MODEL_CHANGE_FILTER);

  referenceModel.state.baro.lastUpdateUs =
      90000;

  reloadModel.state.baro.lastUpdateUs =
      90000;

  referenceAltitude.update(
      true);

  reloadedAltitude.update(
      true);

  TEST_ASSERT_FLOAT_WITHIN(
      1.0e-6f,
      referenceModel.state.altitude.height,
      reloadModel.state.altitude.height);

  TEST_ASSERT_FLOAT_WITHIN(
      1.0e-6f,
      referenceModel.state.altitude.vario,
      reloadModel.state.altitude.vario);
}


// =========================================================
// TEST BAROMETER DEVICE
// =========================================================

class RegressionTestBaroDevice final
    : public Espfc::Device::BaroDevice
{
public:
  explicit RegressionTestBaroDevice(
      float pressure):
      _pressure(pressure)
  {
  }

  int begin(
      Espfc::Device::BusDevice*) override
  {
    return 1;
  }

  int begin(
      Espfc::Device::BusDevice*,
      uint8_t) override
  {
    return 1;
  }

  Espfc::BaroDeviceType getType()
      const override
  {
    return
        BARO_BMP280;
  }

  float readTemperature() override
  {
    return
        25.0f;
  }

  float readPressure() override
  {
    return
        _pressure;
  }

  int getDelay(
      Espfc::BaroDeviceMode)
      const override
  {
    return
        0;
  }

  void setMode(
      Espfc::BaroDeviceMode) override
  {
  }

  bool testConnection() override
  {
    return
        true;
  }

private:
  float _pressure;
};


// =========================================================
// 5. FIRST PRESSURE SAMPLE MUST PRIME FILTER DIRECTLY
// =========================================================

void test_pressure_filter_first_sample_is_primed()
{
  constexpr float TEST_PRESSURE =
      90000.0f;

  Model model;

  model.state.baro.rate =
      50;

  Espfc::Sensor::BaroSensor sensor(
      model);

  RegressionTestBaroDevice device(
      TEST_PRESSURE);

  // Configure the same filter path used by BaroSensor.
  sensor.reload(
      MODEL_CHANGE_FILTER);

  sensor._baro =
      &device;

  sensor._pressurePrimed =
      false;

  TEST_ASSERT_TRUE(
      sensor.readPressure());

  TEST_ASSERT_TRUE(
      sensor._pressurePrimed);

  TEST_ASSERT_TRUE(
      model.state.baro.sampleValid);

  TEST_ASSERT_FLOAT_WITHIN(
      0.001f,
      TEST_PRESSURE,
      model.state.baro.pressureRaw);

  // Critical assertion:
  //
  // The first filtered pressure must equal the first
  // physical pressure observation instead of ramping
  // upward from zero.
  TEST_ASSERT_FLOAT_WITHIN(
      0.001f,
      TEST_PRESSURE,
      model.state.baro.pressure);
}


// =========================================================
// 6. ALTITUDE ESTIMATOR NEEDS ITS OWN FRESHNESS CHECK
// =========================================================

void test_altitude_estimator_has_independent_freshness()
{
  constexpr uint32_t NOW_US =
      500000;

  When(
      Method(
          ArduinoFake(),
          micros))
      .AlwaysReturn(
          NOW_US);

  Model model;

  setHealthyAssistedEstimatorState(
      model,
      NOW_US);

  model.state.altitude.healthy =
      true;

  model.state.altitude.height =
      2.0f;

  model.state.altitude.vario =
      0.0f;

  // Barometer and attitude are fresh, but the altitude
  // estimator itself has not run for 150 ms.
  model.state.altitude.lastUpdateUs =
      NOW_US -
      150000u;

  Actuator actuator(
      model);

  TEST_ASSERT_FALSE(
      actuator.altitudeEstimateHealthy());

  // Once the altitude estimator timestamp is fresh,
  // the same otherwise-valid state should pass.
  model.state.altitude.lastUpdateUs =
      NOW_US;

  TEST_ASSERT_TRUE(
      actuator.altitudeEstimateHealthy());
}


// =========================================================
// 7. DISARMED OUTPUT MUST NOT REQUIRE A CONTROL/PID CYCLE
// =========================================================

void test_disarm_output_propagates_without_control_cycle()
{
  Model model;

  Output::Mixer mixer(
      model);

  // Explicitly disarmed.
  model.updateModes(
      0);

  for (size_t i = 0;
       i < OUTPUT_CHANNELS;
       ++i)
  {
    // Keep every test channel on the motor/disarmed path
    // so servo-neutral special handling does not affect
    // this regression.
    model.config.output.channel[i].servo =
        false;

    model.state.output.disarmed[i] =
        static_cast<int16_t>(
            1000 +
            static_cast<int>(i));

    // Start from deliberately different values.
    model.state.output.us[i] =
        1800;
  }

  // No controller.update() and no normal mixer.update().
  //
  // This must still propagate configured disarmed values.
  mixer.writeDisarmed();

  for (size_t i = 0;
       i < OUTPUT_CHANNELS;
       ++i)
  {
    TEST_ASSERT_EQUAL_INT16(
        model.state.output.disarmed[i],
        model.state.output.us[i]);
  }
}


// =========================================================
// 8. ANGLE-FAULT TRANSITION MUST PRODUCE FINITE,
//    RATE-LIMITED ACTIVE SETPOINTS
// =========================================================

void test_angle_fault_transition_rate_is_finite_and_bounded()
{
  constexpr uint32_t NOW_US =
      50000;

  When(
      Method(
          ArduinoFake(),
          micros))
      .AlwaysReturn(
          NOW_US);

  Model model;

  model.state.gyro.clock =
      1000;

  model.config.gyro.dlpf =
      GYRO_DLPF_256;

  model.config.loopSync =
      1;

  model.config.mixerSync =
      1;

  model.config.mixer.type =
      FC_MIXER_QUADX;

  model.config.level.angleLimit =
      45;

  model.config.level.rateLimit =
      300;

  model.config.input.rateLimit[
      AXIS_ROLL] =
      300;

  model.config.input.rateLimit[
      AXIS_PITCH] =
      300;

  model.config.pid[
      FC_PID_LEVEL] =
      {
          .P = 45u,
          .I = 0u,
          .D = 0u,
          .F = 0
      };

  model.begin();

  Controller controller(
      model);

  controller.begin();

  Actuator actuator(
      model);

  actuator.begin();

  // Valid attitude estimator.
  model.state.gyro.present =
      true;

  model.state.accel.present =
      true;

  model.state.attitude.healthy =
      true;

  model.state.attitude.lastUpdateUs =
      NOW_US;

  model.state.attitude.quaternion =
      Quaternion(
          1.0f,
          0.0f,
          0.0f,
          0.0f);

  model.state.attitude.euler =
      VectorFloat(
          0.10f,
          0.0f,
          0.0f);

  // Configure Angle switch on AUX1.
  auto& condition =
      model.config.conditions[0];

  condition.id =
      MODE_ANGLE;

  condition.ch =
      AXIS_AUX_1;

  condition.min =
      1200;

  condition.max =
      1800;

  model.state.input.us[
      AXIS_AUX_1] =
      1500;

  model.state.input.ch[
      AXIS_ROLL] =
      0.25f;

  // Enter Angle mode.
  actuator.updateModeMask();

  TEST_ASSERT_TRUE(
      model.isModeActive(
          MODE_ANGLE));

  controller.update();

  const float rateBeforeFault =
      model.state.setpoint.rate[
          AXIS_ROLL];

  TEST_ASSERT_TRUE(
      std::isfinite(
          rateBeforeFault));

  // Simulate estimator failure while Angle switch
  // remains enabled.
  model.state.attitude.healthy =
      false;

  actuator.updateModeMask();

  TEST_ASSERT_FALSE(
      model.isModeActive(
          MODE_ANGLE));

  // Active controller now falls back to rate mode.
  controller.update();

  const float rateAfterFault =
      model.state.setpoint.rate[
          AXIS_ROLL];

  TEST_ASSERT_TRUE(
      std::isfinite(
          rateAfterFault));

  const float maxRate =
      Utils::toRad(
          300.0f);

  // The fallback command must remain inside the
  // configured active rate limit.
  TEST_ASSERT_TRUE(
      std::fabs(
          rateAfterFault) <=
      maxRate +
          0.001f);

  // Also ensure the transition itself did not create
  // a non-finite command.
  TEST_ASSERT_TRUE(
      std::isfinite(
          rateAfterFault -
          rateBeforeFault));
}

void test_angle_v2_candidate_keeps_physical_motor_driver_blocked()
{
#if defined(ESPFC_SAFE_BENCH_BUILD)

  ArduinoFakeReset();

  // Mixer::begin() logs initialization information.
  // Logger::info() calls millis(), so ArduinoFake must
  // provide a deterministic value.
  When(
      Method(
          ArduinoFake(),
          millis))
      .AlwaysReturn(
          1);

  Model model;

  model.state.gyro.clock =
      1000;

  model.config.gyro.dlpf =
      GYRO_DLPF_256;

  model.config.loopSync =
      1;

  model.config.mixerSync =
      1;

  model.config.mixer.type =
      FC_MIXER_QUADX;

  model.begin();

  Output::Mixer mixer(
      model);

  mixer.begin();

  // The full mixer may initialize and calculate outputs,
  // but SAFE_BENCH_BUILD must never expose a physical
  // motor ESC driver.
  TEST_ASSERT_NULL(
      model.state.mixer.escMotor);

#endif
}

void test_failsafe_startup_without_rx_does_not_enter_stage2()
{
  ArduinoFakeReset();

  When(
      Method(
          ArduinoFake(),
          micros))
      .AlwaysReturn(
          5000000);

  Model model;

  TelemetryManager telemetry(
      model);

  Input input(
      model,
      telemetry);

  model.state.failsafe.phase =
      FC_FAILSAFE_IDLE;

  model.state.failsafe.rxEverValid =
      false;

  model.state.input.rxLoss =
      true;

  model.state.input.channelsValid =
      false;

  const bool blocked =
      input.failsafe(
          INPUT_IDLE);

  TEST_ASSERT_TRUE(
      blocked);

  TEST_ASSERT_FALSE(
      model.state.failsafe.rxEverValid);

  TEST_ASSERT_EQUAL(
      FC_FAILSAFE_IDLE,
      model.state.failsafe.phase);

  TEST_ASSERT_TRUE(
      model.state.input.rxLoss);

  TEST_ASSERT_FALSE(
      model.state.input.rxFailSafe);
}

void test_failsafe_startup_requires_sustained_rx_recovery()
{
  ArduinoFakeReset();

  When(
    Method(
        ArduinoFake(),
        micros))
    .Return(
        // First failsafe() call:
        // Stats start, failsafe now, Stats end
        1000000,
        1000000,
        1000000,

        // Second failsafe() call:
        // 200 ms after qualification started
        1200000,
        1200000,
        1200000,

        // Third failsafe() call:
        // just over 500 ms after qualification started
        1500001,
        1500001,
        1500001);
  Model model;

  TelemetryManager telemetry(
      model);

  Input input(
      model,
      telemetry);

  model.state.failsafe.phase =
      FC_FAILSAFE_IDLE;

  model.state.failsafe.rxEverValid =
      false;

  model.state.input.channelsValid =
      true;

  model.state.input.rxLoss =
      true;

  // First good frame: qualification begins.
  TEST_ASSERT_TRUE(
      input.failsafe(
          INPUT_RECEIVED));

  TEST_ASSERT_FALSE(
      model.state.failsafe.rxEverValid);

  // Still only 200 ms healthy.
  TEST_ASSERT_TRUE(
      input.failsafe(
          INPUT_RECEIVED));

  TEST_ASSERT_FALSE(
      model.state.failsafe.rxEverValid);

  // More than 500 ms continuously healthy.
  TEST_ASSERT_FALSE(
      input.failsafe(
          INPUT_RECEIVED));

  TEST_ASSERT_TRUE(
      model.state.failsafe.rxEverValid);

  TEST_ASSERT_FALSE(
      model.state.input.rxLoss);

  TEST_ASSERT_EQUAL(
      FC_FAILSAFE_IDLE,
      model.state.failsafe.phase);
}

void test_failsafe_recovery_interrupted_by_loss_stays_blocked()
{
  ArduinoFakeReset();

  When(
      Method(
          ArduinoFake(),
          micros))
      .Return(
          // Begin recovery.
          1000000,
          1000000,
          1000000,

          // INPUT_LOST only 20 ms later.
          1020000,
          1020000,
          1020000,

          // Following idle cycle.
          1030000,
          1030000,
          1030000);

  Model model;

  TelemetryManager telemetry(
      model);

  Input input(
      model,
      telemetry);

  // Receiver had been valid previously, then entered
  // failsafe.
  model.state.failsafe.rxEverValid =
      true;

  model.state.failsafe.phase =
      FC_FAILSAFE_RX_LOSS_DETECTED;

  model.state.input.rxLoss =
      true;

  model.state.input.rxFailSafe =
      true;

  model.state.input.channelsValid =
      true;

  model.state.input.frameTime =
      1000000;

  // First healthy frame starts the 500 ms recovery
  // qualification window.
  TEST_ASSERT_TRUE(
      input.failsafe(
          INPUT_RECEIVED));

  TEST_ASSERT_TRUE(
      model.state.failsafe
          .recoveryActive);

  TEST_ASSERT_EQUAL(
      FC_FAILSAFE_RX_LOSS_MONITORING,
      model.state.failsafe.phase);

  // Link is interrupted only 20 ms later.
  //
  // Recovery timer must reset, but input must remain
  // blocked.
  TEST_ASSERT_TRUE(
      input.failsafe(
          INPUT_LOST));

  TEST_ASSERT_FALSE(
      model.state.failsafe
          .recoveryActive);

  TEST_ASSERT_EQUAL(
      FC_FAILSAFE_RX_LOSS_MONITORING,
      model.state.failsafe.phase);

  // Even the following idle cycle is still blocked.
  TEST_ASSERT_TRUE(
      input.failsafe(
          INPUT_IDLE));

  TEST_ASSERT_EQUAL(
      FC_FAILSAFE_RX_LOSS_MONITORING,
      model.state.failsafe.phase);
}

void test_failsafe_recovery_interrupted_by_invalid_frame_stays_blocked()
{
  ArduinoFakeReset();

  When(
      Method(
          ArduinoFake(),
          micros))
      .Return(
          // Begin recovery.
          2000000,
          2000000,
          2000000,

          // Invalid receiver frame 20 ms later.
          2020000,
          2020000,
          2020000,

          // Following idle cycle.
          2030000,
          2030000,
          2030000);

  Model model;

  TelemetryManager telemetry(
      model);

  Input input(
      model,
      telemetry);

  model.state.failsafe.rxEverValid =
      true;

  model.state.failsafe.phase =
      FC_FAILSAFE_RX_LOSS_DETECTED;

  model.state.input.rxLoss =
      true;

  model.state.input.rxFailSafe =
      true;

  model.state.input.channelsValid =
      true;

  model.state.input.frameTime =
      2000000;

  // Begin post-failsafe recovery.
  TEST_ASSERT_TRUE(
      input.failsafe(
          INPUT_RECEIVED));

  TEST_ASSERT_TRUE(
      model.state.failsafe
          .recoveryActive);

  TEST_ASSERT_EQUAL(
      FC_FAILSAFE_RX_LOSS_MONITORING,
      model.state.failsafe.phase);

  // Next receiver frame exists, but its channels are
  // invalid.
  model.state.input.channelsValid =
      false;

  TEST_ASSERT_TRUE(
      input.failsafe(
          INPUT_RECEIVED));

  TEST_ASSERT_FALSE(
      model.state.failsafe
          .recoveryActive);

  TEST_ASSERT_EQUAL(
      FC_FAILSAFE_RX_LOSS_MONITORING,
      model.state.failsafe.phase);

  // Still blocked on the next cycle.
  TEST_ASSERT_TRUE(
      input.failsafe(
          INPUT_IDLE));

  TEST_ASSERT_EQUAL(
      FC_FAILSAFE_RX_LOSS_MONITORING,
      model.state.failsafe.phase);
}

void test_failsafe_default_procedure_is_drop()
{
  ModelConfig config;

  TEST_ASSERT_EQUAL_UINT8(
      FAILSAFE_PROCEDURE_DROP,
      config.failsafe.procedure);
}

void test_failsafe_auto_land_request_is_recorded_but_disarms()
{
  ArduinoFakeReset();

  When(
      Method(
          ArduinoFake(),
          micros))
      .AlwaysReturn(
          2000000);

  Model model;

  TelemetryManager telemetry(
      model);

  Input input(
      model,
      telemetry);

  model.config.failsafe.procedure =
      FAILSAFE_PROCEDURE_AUTO_LAND;

  model.state.failsafe.rxEverValid =
      true;

  model.state.altitude.healthy =
      true;

  model.state.altitude.height =
      3.25f;

  model.state.altitude.vario =
      -0.15f;

  model.state.output.ch[
      AXIS_THRUST] =
      -0.65f;

  // Simulate an already armed aircraft.
  model.updateModes(
      uint32_t{1} <<
      MODE_ARMED);

  TEST_ASSERT_TRUE(
      model.isModeActive(
          MODE_ARMED));

  input.failsafeStage2();

  // AUTO-LAND was correctly recognized.
  TEST_ASSERT_TRUE(
      model.state.failsafe
          .landingRequested);

TEST_ASSERT_FALSE(
    model.state.failsafe
        .landingEstimatorHealthy);

TEST_ASSERT_FALSE(
    model.state.failsafe
        .landingEligible);

TEST_ASSERT_TRUE(
    model.state.failsafe
        .landingOutputBlocked);

  TEST_ASSERT_EQUAL_UINT32(
      2000000,
      model.state.failsafe
          .landingRequestedUs);

  TEST_ASSERT_FLOAT_WITHIN(
      0.0001f,
      3.25f,
      model.state.failsafe
          .landingEntryHeight);

  TEST_ASSERT_FLOAT_WITHIN(
      0.0001f,
      -0.15f,
      model.state.failsafe
          .landingEntryVario);

  TEST_ASSERT_FLOAT_WITHIN(
      0.0001f,
      -0.65f,
      model.state.failsafe
          .landingEntryThrust);

#if defined(ESPFC_LAND_V2_ACTIVE)
  // The dedicated active-validation build deliberately
  // retains logical arming so Controller/Actuator can run
  // the LAND state machine. Physical ESC attachment is
  // still blocked by ESPFC_SAFE_BENCH_BUILD.
  TEST_ASSERT_TRUE(
      model.isModeActive(
          MODE_ARMED));

  TEST_ASSERT_EQUAL(
      FC_FAILSAFE_LANDING,
      model.state.failsafe.phase);
#else
  // Ordinary builds preserve the existing conservative
  // fallback: unfinished LAND never leaves motors armed.
  TEST_ASSERT_FALSE(
      model.isModeActive(
          MODE_ARMED));

  TEST_ASSERT_EQUAL(
      FC_FAILSAFE_LANDED,
      model.state.failsafe.phase);
#endif
}

void test_failsafe_auto_land_v2_rejects_bad_estimator()
{
  ArduinoFakeReset();

  constexpr uint32_t NOW_US =
      3000000;

  When(
      Method(
          ArduinoFake(),
          micros))
      .AlwaysReturn(
          NOW_US);

  Model model;

  TelemetryManager telemetry(
      model);

  Input input(
      model,
      telemetry);

  Actuator actuator(
      model);

  model.config.failsafe.procedure =
      FAILSAFE_PROCEDURE_AUTO_LAND;

  model.state.failsafe.rxEverValid =
      true;

  // Build an otherwise valid assisted-mode estimator
  // environment first.
  setHealthyAssistedEstimatorState(
      model,
      NOW_US);

  // Then deliberately invalidate only altitude.
  model.state.altitude.healthy =
      false;

  model.state.altitude.lastUpdateUs =
      NOW_US;

  model.updateModes(
      uint32_t{1} <<
      MODE_ARMED);

  input.failsafeStage2();

  // Input only records the LAND request.
  TEST_ASSERT_TRUE(
      model.state.failsafe
          .landingRequested);

  // Actuator performs the actual continuous
  // estimator-health evaluation.
  actuator.updateFailsafeLand();

  TEST_ASSERT_FALSE(
      model.state.failsafe
          .landingEstimatorHealthy);

  TEST_ASSERT_FALSE(
      model.state.failsafe
          .landingEligible);

  TEST_ASSERT_FALSE(
      model.state.failsafe
          .landingActive);

  TEST_ASSERT_FALSE(
      model.state.failsafe
          .landingLevelRequested);

  TEST_ASSERT_FALSE(
      model.state.failsafe
          .landingDescentRequested);

  TEST_ASSERT_TRUE(
      model.state.failsafe
          .landingFault);

  TEST_ASSERT_TRUE(
      model.state.failsafe
          .landingOutputBlocked);

  // Current dry-run fallback still disarms.
  TEST_ASSERT_FALSE(
      model.isModeActive(
          MODE_ARMED));
}

void test_failsafe_drop_does_not_request_land()
{
  Model model;

  TelemetryManager telemetry(
      model);

  Input input(
      model,
      telemetry);

  model.config.failsafe.procedure =
      FAILSAFE_PROCEDURE_DROP;

  model.state.failsafe.rxEverValid =
      true;

  model.updateModes(
      uint32_t{1} <<
      MODE_ARMED);

  input.failsafeStage2();

  TEST_ASSERT_FALSE(
      model.state.failsafe
          .landingRequested);

  TEST_ASSERT_FALSE(
      model.state.failsafe
          .landingEligible);

  TEST_ASSERT_FALSE(
      model.isModeActive(
          MODE_ARMED));

  TEST_ASSERT_EQUAL(
      FC_FAILSAFE_LANDED,
      model.state.failsafe.phase);
}

void test_failsafe_invalid_procedure_sanitizes_to_drop()
{
  Model model;

  model.config.failsafe.procedure =
      255;

  model.sanitize();

  TEST_ASSERT_EQUAL_UINT8(
      FAILSAFE_PROCEDURE_DROP,
      model.config.failsafe.procedure);
}

void test_failsafe_land_v2_healthy_is_non_actuating()
{
  ArduinoFakeReset();

  constexpr uint32_t NOW_US =
      4000000;

  When(
      Method(
          ArduinoFake(),
          micros))
      .AlwaysReturn(
          NOW_US);

  Model model;

  setHealthyAssistedEstimatorState(
      model,
      NOW_US);

  model.state.altitude.healthy =
      true;

  model.state.altitude.lastUpdateUs =
      NOW_US;

  model.state.altitude.height =
      2.0f;

  model.state.altitude.vario =
      -0.1f;

  model.state.failsafe.rxEverValid =
      true;

  model.state.failsafe.landingRequested =
      true;

  constexpr float OUTPUT_THRUST =
      0.37f;

  constexpr float SETPOINT_THRUST =
      0.42f;

  model.state.output.ch[
      AXIS_THRUST] =
      OUTPUT_THRUST;

  model.state.setpoint.rate[
      AXIS_THRUST] =
      SETPOINT_THRUST;

  Actuator actuator(
      model);

  actuator.updateFailsafeLand();

  TEST_ASSERT_TRUE(
      model.state.failsafe
          .landingEstimatorHealthy);

  TEST_ASSERT_TRUE(
      model.state.failsafe
          .landingEligible);

  TEST_ASSERT_TRUE(
      model.state.failsafe
          .landingActive);

  TEST_ASSERT_TRUE(
      model.state.failsafe
          .landingLevelRequested);

  TEST_ASSERT_TRUE(
      model.state.failsafe
          .landingDescentRequested);

  TEST_ASSERT_FALSE(
      model.state.failsafe
          .landingFault);

  TEST_ASSERT_TRUE(
      model.state.failsafe
          .landingOutputBlocked);

  // Non-actuating LAND validation must not change actual thrust.
  TEST_ASSERT_FLOAT_WITHIN(
      0.0001f,
      OUTPUT_THRUST,
      model.state.output.ch[
          AXIS_THRUST]);

  TEST_ASSERT_FLOAT_WITHIN(
      0.0001f,
      SETPOINT_THRUST,
      model.state.setpoint.rate[
          AXIS_THRUST]);
}

void test_box_failsafe_with_valid_rx_still_runs_stage2()
{
  ArduinoFakeReset();

  When(
      Method(
          ArduinoFake(),
          micros))
      .AlwaysReturn(
          5000000);

  Model model;

  TelemetryManager telemetry(
      model);

  Input input(
      model,
      telemetry);

  model.state.failsafe.rxEverValid =
      true;

  model.state.input.channelsValid =
      true;

  model.updateSwitchActive(
      uint32_t{1} <<
      MODE_FAILSAFE);

  model.updateModes(
      uint32_t{1} <<
      MODE_ARMED);

  TEST_ASSERT_TRUE(
      model.isModeActive(
          MODE_ARMED));

  const bool pilotInputBlocked =
      input.failsafe(
          INPUT_RECEIVED);

  // BOXFAILSAFE is manually requested while physical RX may still be valid,
  // but Stage 2 must own the control path and block pilot-input hand-through.
  TEST_ASSERT_TRUE(
      pilotInputBlocked);

  // Stage 2 must still execute.
  TEST_ASSERT_FALSE(
      model.isModeActive(
          MODE_ARMED));

  TEST_ASSERT_EQUAL(
      FC_FAILSAFE_LANDED,
      model.state.failsafe.phase);
}

void test_new_arm_clears_previous_land_latch()
{
  Model model;

  Actuator actuator(
      model);

  model.state.failsafe.landingRequested =
      true;

  model.state.failsafe.landingActive =
      true;

  model.state.failsafe.landingEligible =
      true;

  model.state.failsafe.landingFault =
      true;

  model.state.failsafe.landingRequestedUs =
      123456;

  model.state.failsafe.landingLastUpdateUs =
      123500;

  model.state.failsafe.landingTouchdownCandidate =
      true;

  model.state.failsafe.landingTouchdownStartedUs =
      123400;

  model.state.failsafe.landingEntryHeight =
      4.0f;

  model.state.failsafe.landingEntryVario =
      -0.4f;

  model.state.failsafe.landingEntryThrust =
      -0.7f;

  // Initial state is disarmed.
  // This creates a real DISARMED -> ARMED transition.
  model.updateModes(
      uint32_t{1} <<
      MODE_ARMED);

  actuator.updateArmed();

  TEST_ASSERT_FALSE(
      model.state.failsafe
          .landingRequested);

  TEST_ASSERT_FALSE(
      model.state.failsafe
          .landingEstimatorHealthy);

  TEST_ASSERT_FALSE(
      model.state.failsafe
          .landingEligible);

  TEST_ASSERT_FALSE(
      model.state.failsafe
          .landingActive);

  TEST_ASSERT_FALSE(
      model.state.failsafe
          .landingLevelRequested);

  TEST_ASSERT_FALSE(
      model.state.failsafe
          .landingDescentRequested);

  TEST_ASSERT_FALSE(
      model.state.failsafe
          .landingFault);

  TEST_ASSERT_TRUE(
      model.state.failsafe
          .landingOutputBlocked);

  TEST_ASSERT_EQUAL_UINT32(
      0,
      model.state.failsafe
          .landingRequestedUs);

  TEST_ASSERT_EQUAL_UINT32(
      0,
      model.state.failsafe
          .landingLastUpdateUs);

  TEST_ASSERT_FALSE(
      model.state.failsafe
          .landingTouchdownCandidate);

  TEST_ASSERT_EQUAL_UINT32(
      0,
      model.state.failsafe
          .landingTouchdownStartedUs);

  TEST_ASSERT_FLOAT_WITHIN(
      0.0001f,
      0.0f,
      model.state.failsafe
          .landingEntryHeight);

  TEST_ASSERT_FLOAT_WITHIN(
      0.0001f,
      0.0f,
      model.state.failsafe
          .landingEntryVario);

  TEST_ASSERT_FLOAT_WITHIN(
      0.0001f,
      0.0f,
      model.state.failsafe
          .landingEntryThrust);
}

void test_failsafe_repeated_stage2_preserves_landed_state()
{
  Model model;

  TelemetryManager telemetry(
      model);

  Input input(
      model,
      telemetry);

  model.state.failsafe.rxEverValid =
      true;

  model.config.failsafe.procedure =
      FAILSAFE_PROCEDURE_DROP;

  model.updateModes(
      uint32_t{1} <<
      MODE_ARMED);

  TEST_ASSERT_TRUE(
      model.isModeActive(
          MODE_ARMED));

  // First Stage 2 completes failsafe and disarms.
  input.failsafeStage2();

  TEST_ASSERT_FALSE(
      model.isModeActive(
          MODE_ARMED));

  TEST_ASSERT_EQUAL(
      FC_FAILSAFE_LANDED,
      model.state.failsafe.phase);

  // Stage 2 can be called again by later timeout cycles.
  input.failsafeStage2();

  // Terminal state must remain terminal.
  TEST_ASSERT_EQUAL(
      FC_FAILSAFE_LANDED,
      model.state.failsafe.phase);

  TEST_ASSERT_FALSE(
      model.isModeActive(
          MODE_ARMED));
}

void test_failsafe_land_fault_survives_request_termination()
{
  ArduinoFakeReset();

  When(
      Method(
          ArduinoFake(),
          micros))
      .AlwaysReturn(
          6000000u);

  Model model;

  model.state.failsafe.landingRequested =
      false;

  model.state.failsafe.landingFault =
      true;

  Actuator actuator(
      model);

  actuator.updateFailsafeLand();

  TEST_ASSERT_TRUE(
      model.state.failsafe
          .landingFault);
}

void test_failsafe_land_v2_fault_latches()
{
  ArduinoFakeReset();

  constexpr uint32_t NOW_US =
      6000000;

  When(
      Method(
          ArduinoFake(),
          micros))
      .AlwaysReturn(
          NOW_US);

  Model model;

  setHealthyAssistedEstimatorState(
      model,
      NOW_US);

  model.state.altitude.lastUpdateUs =
      NOW_US;

  model.state.failsafe.rxEverValid =
      true;

  model.state.failsafe.landingRequested =
      true;

  Actuator actuator(
      model);

  // First update: estimator fault.
  model.state.altitude.healthy =
      false;

  actuator.updateFailsafeLand();

  TEST_ASSERT_TRUE(
      model.state.failsafe
          .landingFault);

  TEST_ASSERT_FALSE(
      model.state.failsafe
          .landingEstimatorHealthy);

  // Estimator later recovers.
  model.state.altitude.healthy =
      true;

  actuator.updateFailsafeLand();

  TEST_ASSERT_TRUE(
      model.state.failsafe
          .landingEstimatorHealthy);

  TEST_ASSERT_TRUE(
      model.state.failsafe
          .landingEligible);

  // Historical fault remains latched for this LAND
  // lifecycle.
  TEST_ASSERT_TRUE(
      model.state.failsafe
          .landingFault);
}

void test_actuator_missing_aux_channel_cannot_activate_mode()
{
  Model model;

  // Receiver exposes only the four primary flight channels.
  model.state.input.channelCount =
      AXIS_AUX_1;

  auto& condition =
      model.config.conditions[0];

  condition.id =
      MODE_BUZZER;

  condition.ch =
      AXIS_AUX_1;

  condition.min =
      1200;

  condition.max =
      1800;

  // The unused logical slot still contains a centered-looking value.  It must
  // not be interpreted as a real switch when the receiver has no AUX1.
  model.state.input.us[
      AXIS_AUX_1] =
      1500;

  Actuator actuator(
      model);

  actuator.begin();
  actuator.updateModeMask();

  TEST_ASSERT_FALSE(
      model.isModeActive(
          MODE_BUZZER));
}

void test_actuator_missing_aux_channel_does_not_apply_scaler()
{
  Model model;

  auto& scaler =
      model.config.scaler[0];

  scaler.dimension =
      ACT_INNER_P |
      ACT_AXIS_ROLL;

  scaler.channel =
      AXIS_AUX_1;

  scaler.minScale =
      20;

  scaler.maxScale =
      400;

  model.state.input.channelCount =
      AXIS_AUX_1 + 1;

  model.state.input.ch[
      AXIS_AUX_1] =
      1.0f;

  Actuator actuator(
      model);

  actuator.updateScaler();

  TEST_ASSERT_FLOAT_WITHIN(
      0.0001f,
      4.0f,
      model.state.innerPid[
          AXIS_ROLL].pScale);

  // If AUX1 disappears, the previous 4x multiplier must not remain latched.
  model.state.input.channelCount =
      AXIS_AUX_1;

  actuator.updateScaler();

  TEST_ASSERT_FLOAT_WITHIN(
      0.0001f,
      1.0f,
      model.state.innerPid[
          AXIS_ROLL].pScale);

  // Disabling the scaler must also restore neutral scaling.
  model.state.input.channelCount =
      AXIS_AUX_1 + 1;

  model.config.scaler[0].dimension =
      0;

  model.state.innerPid[
      AXIS_ROLL].pScale =
      3.0f;

  actuator.updateScaler();

  TEST_ASSERT_FLOAT_WITHIN(
      0.0001f,
      1.0f,
      model.state.innerPid[
          AXIS_ROLL].pScale);
}

void test_actuator_angle_v2_scaler_updates_selected_axis()
{
  Model model;

  auto& scaler =
      model.config.scaler[0];

  scaler.dimension =
      ACT_ANGLE_P |
      ACT_AXIS_ROLL;

  scaler.channel =
      AXIS_AUX_1;

  scaler.minScale =
      20;

  scaler.maxScale =
      400;

  // Make AUX1 a real receiver channel.
  model.state.input.channelCount =
      AXIS_AUX_1 + 1;

  // Full positive AUX command maps to maxScale = 4.0.
  model.state.input.ch[
      AXIS_AUX_1] =
      1.0f;

  Actuator actuator(
      model);

  actuator.updateScaler();

  // Angle V2 Roll P scaling must receive the configured scale.
  TEST_ASSERT_FLOAT_WITHIN(
      0.0001f,
      4.0f,
      model.state.angleV2
          .pScale[AXIS_ROLL]);

  // Pitch was not selected.
  TEST_ASSERT_FLOAT_WITHIN(
      0.0001f,
      1.0f,
      model.state.angleV2
          .pScale[AXIS_PITCH]);

  // ACT_ANGLE_P must not modify the inner rate-P scaler.
  TEST_ASSERT_FLOAT_WITHIN(
      0.0001f,
      1.0f,
      model.state.innerPid[
          AXIS_ROLL].pScale);

  // If the AUX channel disappears, transient scaling
  // must return to neutral instead of remaining latched.
  model.state.input.channelCount =
      AXIS_AUX_1;

  actuator.updateScaler();

  TEST_ASSERT_FLOAT_WITHIN(
      0.0001f,
      1.0f,
      model.state.angleV2
          .pScale[AXIS_ROLL]);
}

void test_actuator_no_receiver_cannot_activate_aux_mode()
{
  Model model;

  model.state.input.channelCount =
      0;

  model.state.input.rxLoss =
      true;

  auto& condition =
      model.config.conditions[0];

  condition.id =
      MODE_BUZZER;

  condition.ch =
      AXIS_AUX_1;

  condition.min =
      1200;

  condition.max =
      1800;

  model.state.input.us[
      AXIS_AUX_1] =
      1500;

  Actuator actuator(
      model);

  actuator.begin();
  actuator.updateModeMask();

  TEST_ASSERT_FALSE(
      model.isModeActive(
          MODE_BUZZER));
}

void test_actuator_gps_arming_block_clears_when_feature_disabled()
{
  Model model;
  Actuator actuator(
      model);

  model.config.featureMask |=
      FEATURE_GPS;

  model.state.gps.present =
      false;

  actuator.updateArmingDisabled();

  TEST_ASSERT_TRUE(
      model.getArmingDisabled(
          ARMING_DISABLED_GPS));

  model.config.featureMask &=
      ~FEATURE_GPS;

  actuator.updateArmingDisabled();

  TEST_ASSERT_FALSE(
      model.getArmingDisabled(
          ARMING_DISABLED_GPS));
}

void test_ppm_only_publishes_complete_stable_frames()
{
  ArduinoFakeReset();

  Device::InputPPM ppm;

  When(
      Method(
          ArduinoFake(),
          micros))
      .Return(
          0u,
          4000u,
          5000u,
          6500u,
          8000u,
          9500u,
          13500u,
          14500u,
          16000u,
          17500u,
          19000u,
          23000u,
          24000u,
          25500u,
          27000u,
          28500u,
          32500u,
          33500u,
          35000u,
          36500u,
          42000u);

  ppm.begin(
      -1);

  TEST_ASSERT_EQUAL_UINT32(
      0u,
      ppm.getChannelCount());

  // Frame 1: sync + four valid channels + sync.
  ppm.handle();
  ppm.handle();
  ppm.handle();
  ppm.handle();
  ppm.handle();
  ppm.handle();

  TEST_ASSERT_EQUAL(
      INPUT_IDLE,
      ppm.update());

  // Frame 2: same width, still qualifying.
  ppm.handle();
  ppm.handle();
  ppm.handle();
  ppm.handle();
  ppm.handle();

  TEST_ASSERT_EQUAL(
      INPUT_IDLE,
      ppm.update());

  // Frame 3: stable channel count is now authoritative.
  ppm.handle();
  ppm.handle();
  ppm.handle();
  ppm.handle();
  ppm.handle();

  TEST_ASSERT_EQUAL(
      INPUT_RECEIVED,
      ppm.update());

  TEST_ASSERT_EQUAL_UINT32(
      4u,
      ppm.getChannelCount());

  // A truncated/corrupt frame with only three channels must not be
  // published as a fresh receiver frame.
  ppm.handle();
  ppm.handle();
  ppm.handle();
  ppm.handle();

  TEST_ASSERT_EQUAL(
      INPUT_IDLE,
      ppm.update());

  TEST_ASSERT_EQUAL_UINT32(
      4u,
      ppm.getChannelCount());
}

void test_input_frame_rate_ignores_startup_and_loss_gaps()
{
  ArduinoFakeReset();

  Model model;
  TelemetryManager telemetry(
      model);
  Input input(
      model,
      telemetry);

  input.begin();

  const uint32_t initialDelta =
      model.state.input.frameDelta;

  const uint32_t initialRate =
      model.state.input.frameRate;

  When(
      Method(
          ArduinoFake(),
          micros))
      .Return(
          30000000u,
          30010000u,
          31000000u);

  // The first frame may arrive long after boot. It establishes the baseline
  // but must not be interpreted as a 30-second receiver frame period.
  input.updateFrameRate();

  TEST_ASSERT_TRUE(
      model.state.input.frameTimeValid);

  TEST_ASSERT_EQUAL_UINT32(
      initialDelta,
      model.state.input.frameDelta);

  TEST_ASSERT_EQUAL_UINT32(
      initialRate,
      model.state.input.frameRate);

  input.updateFrameRate();

  const uint32_t stableDelta =
      model.state.input.frameDelta;

  const uint32_t stableRate =
      model.state.input.frameRate;

  TEST_ASSERT_TRUE(
      stableDelta <
      initialDelta);

  TEST_ASSERT_TRUE(
      stableRate >
      0u);

  // A long RX outage is likewise not a slow frame; preserve the established
  // rate estimate and restart timing from the recovery frame.
  input.updateFrameRate();

  TEST_ASSERT_EQUAL_UINT32(
      stableDelta,
      model.state.input.frameDelta);

  TEST_ASSERT_EQUAL_UINT32(
      stableRate,
      model.state.input.frameRate);
}

void test_model_sanitize_preserves_rc_safety_invariants()
{
  Model model;

  model.config.input.minCheck =
      32767;

  model.config.input.maxCheck =
      -32768;

  model.config.input.filterAutoFactor =
      -20;

  model.config.input.filterAutoThrottleFactor =
      -20;

  model.config.input.deadband =
      -10;

  model.config.input.airModeActivateThreshold =
      -40;

  model.config.input.rateType =
      127;

  model.config.input.ppmMode =
      99;

  model.config.input.serialRxProvider =
      255;

  model.config.level.angleLimit =
      -40;

  model.config.level.rateLimit =
      -100;

  model.config.arming.smallAngle =
      255;

  model.config.debug.mode =
      -1;

  model.config.altHold.itermCenter =
      255;

  model.config.altHold.itermRange =
      0;

  model.config.output.minCommand =
      2200;

  model.config.output.maxThrottle =
      800;

  model.config.output.motorIdle =
      30000;

  model.config.output.servoRate =
      1;

  model.config.output.channel[0].min =
      -30000;

  model.config.output.channel[0].max =
      30000;

  model.config.output.channel[0].neutral =
      32000;

  model.sanitize();

  TEST_ASSERT_EQUAL_INT16(
      1050,
      model.config.input.minCheck);

  TEST_ASSERT_EQUAL_INT16(
      1900,
      model.config.input.maxCheck);

  TEST_ASSERT_TRUE(
      model.config.input.filterAutoFactor >= 0);

  TEST_ASSERT_TRUE(
      model.config.input.filterAutoThrottleFactor >= 0);

  TEST_ASSERT_TRUE(
      model.config.input.deadband >= 0);

  TEST_ASSERT_TRUE(
      model.config.input.airModeActivateThreshold >= 0);

  TEST_ASSERT_TRUE(
      model.config.input.rateType >= 0 &&
      model.config.input.rateType <= 4);

  TEST_ASSERT_EQUAL_INT(
      PPM_MODE_NORMAL,
      model.config.input.ppmMode);

  TEST_ASSERT_EQUAL_UINT8(
      SERIALRX_SBUS,
      model.config.input.serialRxProvider);

  TEST_ASSERT_TRUE(
      model.config.level.angleLimit >= 0);

  TEST_ASSERT_TRUE(
      model.config.level.rateLimit > 0);

  TEST_ASSERT_EQUAL_UINT8(
      180,
      model.config.arming.smallAngle);

  TEST_ASSERT_EQUAL_INT(
      DEBUG_NONE,
      model.config.debug.mode);

  TEST_ASSERT_EQUAL_UINT8(
      60,
      model.config.altHold.itermCenter);

  TEST_ASSERT_EQUAL_UINT8(
      10,
      model.config.altHold.itermRange);

  TEST_ASSERT_EQUAL_INT16(
      1000,
      model.config.output.minCommand);

  TEST_ASSERT_EQUAL_INT16(
      2000,
      model.config.output.maxThrottle);

  TEST_ASSERT_TRUE(
      model.config.output.motorIdle >= 0 &&
      model.config.output.motorIdle <= 2000);

  TEST_ASSERT_EQUAL_INT16(
      50,
      model.config.output.servoRate);

  TEST_ASSERT_EQUAL_INT16(
      750,
      model.config.output.channel[0].min);

  TEST_ASSERT_EQUAL_INT16(
      2250,
      model.config.output.channel[0].max);

  TEST_ASSERT_EQUAL_INT16(
      2250,
      model.config.output.channel[0].neutral);
}

void test_fusion_mode_name_rejects_negative_enum()
{
  TEST_ASSERT_EQUAL_STRING(
      "?",
      FusionConfig::getModeName(
          static_cast<FusionMode>(-1)));
}

int main(int argc, char** argv)
{
  UNITY_BEGIN();
  RUN_TEST(test_timer_rate_100hz);
  RUN_TEST(test_timer_rate_100hz_div2);
  RUN_TEST(test_timer_interval_10ms);
  RUN_TEST(test_timer_check);
  RUN_TEST(test_timer_check_micros);
  RUN_TEST(test_model_gyro_init_1k_256dlpf);
  RUN_TEST(test_model_gyro_init_1k_188dlpf);
  RUN_TEST(test_model_inner_pid_init);
  RUN_TEST(test_controller_rates);
RUN_TEST(test_controller_rates_limit);

// Angle V2 controller regression tests
RUN_TEST(
    test_controller_angle_v2_activates_and_slews);

RUN_TEST(
    test_controller_angle_v2_bumpless_entry);

RUN_TEST(
    test_controller_angle_v2_active_path_is_bumpless_and_negative_feedback);

RUN_TEST(test_controller_althold_v2_captures_current_altitude);
RUN_TEST(test_controller_althold_v2_center_stick_holds_target);
RUN_TEST(test_controller_althold_v2_climb_command_moves_target_up);
RUN_TEST(test_controller_althold_v2_descent_command_moves_target_down);
RUN_TEST(test_controller_althold_v2_stops_when_estimator_unhealthy);
RUN_TEST(test_controller_althold_v2_vertical_accel_limit);
RUN_TEST(
    test_controller_althold_v2_target_is_bounded);

// Additional V2 regression tests
RUN_TEST(test_controller_althold_v2_full_climb_rate_scaling);
RUN_TEST(test_actuator_althold_fault_requires_switch_cycle);
RUN_TEST(test_actuator_angle_fault_requires_switch_cycle);
#if defined(ESPFC_SAFE_BENCH_BUILD)

RUN_TEST(
    test_angle_v2_candidate_keeps_physical_motor_driver_blocked);

#endif
  // Final assisted-mode architecture regression tests
#if !defined(ESPFC_ALTHOLD_V2_ACTIVE)
RUN_TEST(
    test_controller_althold_v2_inactive_build_does_not_drive_thrust);
#endif

#if defined(ESPFC_ALTHOLD_V2_ACTIVE)
RUN_TEST(
    test_controller_althold_v2_active_path_is_bumpless_and_corrective);

RUN_TEST(
    test_actuator_althold_v2_requires_centered_pilot_stick_on_entry);

RUN_TEST(
    test_controller_althold_v2_debug_exposes_vertical_control_chain);

#if ESPFC_ALTHOLD_V2_CENTERED_STICK_CHANNEL >= 4
RUN_TEST(
    test_actuator_althold_v2_rejects_missing_pilot_channel);
#endif

RUN_TEST(
    test_actuator_althold_v2_rejects_invalid_raw_pilot_channel);

RUN_TEST(
    test_controller_althold_v2_low_manual_throttle_preserves_rate_iterm);

RUN_TEST(
    test_controller_althold_v2_tpa_uses_assisted_thrust);

RUN_TEST(
    test_actuator_althold_v2_allows_stick_deflection_after_entry);

RUN_TEST(
    test_actuator_althold_v2_invalid_pilot_channel_exits_and_latches);

#if ESPFC_ALTHOLD_V2_CENTERED_STICK_CHANNEL != 3
RUN_TEST(
    test_controller_althold_v2_uses_dedicated_centered_stick_channel);
#endif
#endif

#if defined(ESPFC_LAND_V2_ACTIVE)
RUN_TEST(
    test_controller_land_v2_levels_and_requests_descent);

RUN_TEST(
    test_controller_land_v2_requires_supervisor_authorization);

RUN_TEST(
    test_failsafe_land_v2_bad_estimator_falls_back_to_disarm);

RUN_TEST(
    test_failsafe_land_v2_touchdown_dwell_disarms);

RUN_TEST(
    test_failsafe_land_v2_touchdown_candidate_has_hysteresis);

RUN_TEST(
    test_failsafe_land_v2_low_entry_thrust_prevents_hover_false_touchdown);

RUN_TEST(
    test_failsafe_land_v2_near_ground_hover_does_not_disarm);

RUN_TEST(
    test_failsafe_land_v2_timeout_disarms);

RUN_TEST(
    test_auto_land_arm_request_requires_healthy_altitude_estimator);

RUN_TEST(
    test_failsafe_land_v2_rx_recovery_stays_committed_to_land);
#endif

RUN_TEST(
    test_baro_bias_seeds_first_absolute_altitude_sample);

RUN_TEST(
    test_fusion_rejects_invalid_accel_without_poisoning_state);
  RUN_TEST(
    test_altitude_baro_vario_used_only_once_per_sample);

RUN_TEST(
    test_complementary_variable_dt);

    RUN_TEST(
    test_altitude_correction_independent_of_imu_rate);

RUN_TEST(
    test_rejected_fusion_does_not_reintegrate_world_accel);

RUN_TEST(
    test_rejected_height_sample_does_not_correct_vario);

RUN_TEST(
    test_altitude_reload_preserves_filter_history);

RUN_TEST(
    test_pressure_filter_first_sample_is_primed);

RUN_TEST(
    test_altitude_estimator_has_independent_freshness);

RUN_TEST(
    test_disarm_output_propagates_without_control_cycle);

RUN_TEST(
    test_angle_fault_transition_rate_is_finite_and_bounded);
    
  RUN_TEST(test_rates_betaflight);
  RUN_TEST(test_rates_betaflight_expo);
  RUN_TEST(test_rates_raceflight);
  RUN_TEST(test_rates_raceflight_expo);
  RUN_TEST(test_rates_kiss);
  RUN_TEST(test_rates_kiss_expo);
  RUN_TEST(test_actuator_stale_gyro_blocks_arm_and_disarms);
  RUN_TEST(test_actuator_fresh_gyro_passes_runtime_health_gate);
  RUN_TEST(test_actuator_arming_gyro_motor_calbration);
  RUN_TEST(test_actuator_missing_aux_channel_cannot_activate_mode);
  RUN_TEST(test_actuator_missing_aux_channel_does_not_apply_scaler);
  RUN_TEST(
    test_actuator_angle_v2_scaler_updates_selected_axis);
  RUN_TEST(test_actuator_mode_link_activates_target_without_aux_range);
  RUN_TEST(test_actuator_mode_link_cannot_target_arm);
  RUN_TEST(test_actuator_mode_link_rejects_linked_source_chain);
  RUN_TEST(test_actuator_mode_logic_and_requires_all_ranges);
  RUN_TEST(test_model_config_antigravity_tail_layout_stays_packed);
  RUN_TEST(test_model_sanitize_migrates_legacy_antigravity_tail);
  RUN_TEST(test_model_sanitize_scrubs_legacy_antigravity_mode_rows);
  RUN_TEST(test_model_sanitize_preserves_new_antigravity_mode_rows);
  RUN_TEST(test_model_sanitize_preserves_valid_antigravity_gain);
  RUN_TEST(test_controller_anti_gravity_tracks_manual_throttle_transient);
  RUN_TEST(test_pid_antigravity_gain_inputs_scale_p_and_add_i);
  RUN_TEST(test_controller_anti_gravity_can_be_enabled_by_mode);
  RUN_TEST(test_msp_current_meter_reply_is_ignored_when_source_is_not_msp);
  RUN_TEST(test_msp_current_meter_short_reply_is_ignored);
  RUN_TEST(test_model_sanitize_preserves_msp_current_source);
  RUN_TEST(test_msp_current_meter_reply_updates_battery_state);
  RUN_TEST(test_voltage_sensor_reads_fresh_msp_current_and_rejects_stale_data);
#if defined(ESPFC_ALTHOLD_V2_ACTIVE)
  RUN_TEST(test_controller_anti_gravity_yields_to_assisted_vertical_control);
#endif
  RUN_TEST(test_actuator_no_receiver_cannot_activate_aux_mode);
  RUN_TEST(test_actuator_gps_arming_block_clears_when_feature_disabled);
  RUN_TEST(test_ppm_only_publishes_complete_stable_frames);
  RUN_TEST(test_input_frame_rate_ignores_startup_and_loss_gaps);
  RUN_TEST(test_model_sanitize_preserves_rc_safety_invariants);
  RUN_TEST(test_fusion_mode_name_rejects_negative_enum);
RUN_TEST(
    test_failsafe_startup_without_rx_does_not_enter_stage2);

RUN_TEST(
    test_failsafe_startup_requires_sustained_rx_recovery);
RUN_TEST(
    test_failsafe_recovery_interrupted_by_loss_stays_blocked);

RUN_TEST(
    test_failsafe_recovery_interrupted_by_invalid_frame_stays_blocked);

RUN_TEST(
    test_failsafe_repeated_stage2_preserves_landed_state);

RUN_TEST(
    test_failsafe_land_fault_survives_request_termination);

RUN_TEST(
    test_failsafe_land_v2_fault_latches);
RUN_TEST(
    test_failsafe_default_procedure_is_drop);

RUN_TEST(
    test_failsafe_auto_land_request_is_recorded_but_disarms);

RUN_TEST(
    test_failsafe_auto_land_v2_rejects_bad_estimator);

RUN_TEST(
    test_failsafe_drop_does_not_request_land);

RUN_TEST(
    test_failsafe_invalid_procedure_sanitizes_to_drop);

RUN_TEST(
    test_failsafe_land_v2_healthy_is_non_actuating);

RUN_TEST(
    test_box_failsafe_with_valid_rx_still_runs_stage2);

RUN_TEST(
    test_new_arm_clears_previous_land_latch);

RUN_TEST(
    test_actuator_arming_failsafe);
  RUN_TEST(test_actuator_arming_throttle);
  RUN_TEST(test_mixer_invalid_custom_entries_are_ignored);
  RUN_TEST(test_mixer_invalid_output_count_fails_closed);
  RUN_TEST(test_mixer_throttle_limit_none);
  RUN_TEST(test_mixer_throttle_limit_scale);
  RUN_TEST(test_mixer_throttle_limit_clip);
  RUN_TEST(test_mixer_output_limit_motor);
  RUN_TEST(test_mixer_output_limit_servo);
  

  return UNITY_END();
}
