// Tests for Control::PositionHold (pure math). Closed-loop checks use a point
// mass with attitude lag and a 5 Hz / 200 ms-late GPS (NEO-6M like).

#include <Control/GpsRescue.h>
#include <Control/Navigation.h>
#include <Control/PositionHold.h>
#include <cmath>
#include <cstdint>
#include <unity.h>

using namespace Espfc::Control;

namespace {

constexpr float PI_F = 3.14159265358979323846f;
constexpr float M_PER_UNIT = 1.113195e-2f;

struct Sim
{
  // metres north/east of origin
  float n = 0.0f, e = 0.0f, vn = 0.0f, ve = 0.0f;
  float heading = 0.0f;
  float rollAng = 0.0f, pitchAng = 0.0f; // actual lean (rad), lags the command
  float windN = 0.0f, windE = 0.0f;      // disturbance acceleration m/s^2
  PositionHold ph;

  // GPS emulation: new sample at 5 Hz, delayed by one sample
  float gpsN = 0, gpsE = 0, gpsVn = 0, gpsVe = 0;
  float sampleN = 0, sampleE = 0, sampleVn = 0, sampleVe = 0;
  int tick = 0;

  int32_t lat() const { return 450000000 + static_cast<int32_t>(gpsN / M_PER_UNIT); }
  int32_t lon() const
  {
    const float c = std::cos(45.0f * PI_F / 180.0f);
    return 100000000 + static_cast<int32_t>(gpsE / (M_PER_UNIT * c));
  }

  PositionHoldOutput step(float stickRoll = 0.0f, float stickPitch = 0.0f)
  {
    constexpr float dt = 0.02f; // 50 Hz
    if (tick % 10 == 0)         // 5 Hz GPS, previous sample delivered (200 ms latency)
    {
      gpsN = sampleN; gpsE = sampleE; gpsVn = sampleVn; gpsVe = sampleVe;
      sampleN = n; sampleE = e; sampleVn = vn; sampleVe = ve;
    }
    tick++;

    PositionHoldInput in;
    in.lat = lat();
    in.lon = lon();
    in.velNorth = gpsVn;
    in.velEast = gpsVe;
    in.heading = heading;
    in.stickRoll = stickRoll;
    in.stickPitch = stickPitch;
    in.dt = dt;
    const auto out = ph.update(in);

    float cmdRoll = out.controlling ? out.rollAngle : stickRoll * 0.4f;
    float cmdPitch = out.controlling ? out.pitchAngle : stickPitch * 0.4f;

    constexpr float tau = 0.15f; // attitude loop lag
    rollAng += (cmdRoll - rollAng) * (dt / tau);
    pitchAng += (cmdPitch - pitchAng) * (dt / tau);

    // body accelerations from lean: forward = +pitch, right = +roll
    const float aF = PositionHold::GRAVITY * std::tan(pitchAng);
    const float aR = PositionHold::GRAVITY * std::tan(rollAng);
    const float c = std::cos(heading), s = std::sin(heading);
    float aN = aF * c - aR * s + windN;
    float aE = aF * s + aR * c + windE;
    aN -= 0.15f * vn; // light drag
    aE -= 0.15f * ve;

    vn += aN * dt; ve += aE * dt;
    n += vn * dt;  e += ve * dt;
    return out;
  }

  float posError() const { return std::hypot(n, e); }
};

} // namespace

void test_poshold_sign_target_ahead_heading_north()
{
  PositionHold ph;
  PositionHoldInput in;
  in.lat = 0; in.lon = 0; in.heading = 0.0f;
  ph.update(in); // latch at origin
  in.lat = static_cast<int32_t>(-10.0f / M_PER_UNIT); // aircraft 10 m south of target
  const auto out = ph.update(in);
  TEST_ASSERT_TRUE(out.controlling);
  TEST_ASSERT_TRUE(out.pitchAngle > 0.02f);        // target is ahead -> pitch forward
  TEST_ASSERT_FLOAT_WITHIN(0.005f, 0.0f, out.rollAngle);
}

void test_poshold_sign_rotates_with_heading()
{
  const float headings[] = {0.0f, PI_F / 2, PI_F, -PI_F / 2};
  // target 10 m north of aircraft
  const float expPitchSign[] = {+1, 0, -1, 0};
  const float expRollSign[] = {0, -1, 0, +1}; // facing east: north is on the left
  for (int i = 0; i < 4; i++)
  {
    PositionHold ph;
    PositionHoldInput in;
    in.heading = headings[i];
    ph.update(in);
    in.lat = static_cast<int32_t>(-10.0f / M_PER_UNIT);
    const auto out = ph.update(in);
    if (expPitchSign[i] != 0) TEST_ASSERT_TRUE(out.pitchAngle * expPitchSign[i] > 0.02f);
    else TEST_ASSERT_FLOAT_WITHIN(0.005f, 0.0f, out.pitchAngle);
    if (expRollSign[i] != 0) TEST_ASSERT_TRUE(out.rollAngle * expRollSign[i] > 0.02f);
    else TEST_ASSERT_FLOAT_WITHIN(0.005f, 0.0f, out.rollAngle);
  }
}

void test_poshold_east_target_heading_north_rolls_right()
{
  PositionHold ph;
  PositionHoldInput in;
  in.lat = 450000000; in.lon = 100000000;
  ph.update(in);
  in.lon = 100000000 - 1000; // aircraft ~8 m west of target -> target is east
  const auto out = ph.update(in);
  TEST_ASSERT_TRUE(out.rollAngle > 0.02f);
  TEST_ASSERT_FLOAT_WITHIN(0.005f, 0.0f, out.pitchAngle);
}

void test_poshold_lean_angle_is_capped()
{
  PositionHold ph;
  PositionHoldInput in;
  ph.update(in);
  in.lat = static_cast<int32_t>(-25.0f / M_PER_UNIT);
  in.velNorth = -5.0f; // large velocity error
  const auto out = ph.update(in);
  TEST_ASSERT_TRUE(std::fabs(out.pitchAngle) <= ph.params().maxAngle + 1e-5f);
  TEST_ASSERT_TRUE(std::fabs(out.rollAngle) <= ph.params().maxAngle + 1e-5f);
}

void test_poshold_converges_from_offset_all_headings()
{
  const float headings[] = {0.0f, 0.7f, PI_F / 2, PI_F, -2.0f};
  for (float h : headings)
  {
    Sim s;
    s.heading = h;
    s.ph.update(PositionHoldInput{}); // latch at start (0,0)
    s.n = -6.0f; s.e = 5.0f; s.vn = 0.8f; s.ve = -0.5f;
    s.sampleN = s.gpsN = s.n; s.sampleE = s.gpsE = s.e;
    s.sampleVn = s.gpsVn = s.vn; s.sampleVe = s.gpsVe = s.ve;
    s.ph.reset();
    // latch where the sim says origin is: re-latch at origin explicitly
    PositionHoldInput origin; origin.lat = 450000000; origin.lon = 100000000;
    s.ph.update(origin);
    for (int i = 0; i < 50 * 60; i++) s.step();
    TEST_ASSERT_TRUE_MESSAGE(s.posError() < 0.6f, "did not converge to hold point");
    TEST_ASSERT_TRUE(std::hypot(s.vn, s.ve) < 0.15f);
  }
}

void test_poshold_rejects_constant_wind()
{
  Sim s;
  s.heading = 1.0f;
  PositionHoldInput origin; origin.lat = 450000000; origin.lon = 100000000;
  s.ph.update(origin);
  s.windN = 0.4f; s.windE = -0.3f;
  float maxErr = 0.0f;
  for (int i = 0; i < 50 * 90; i++)
  {
    s.step();
    maxErr = std::max(maxErr, s.posError());
  }
  TEST_ASSERT_TRUE_MESSAGE(maxErr < 8.0f, "excursion too large in wind");
  TEST_ASSERT_TRUE_MESSAGE(s.posError() < 1.0f, "I term did not remove wind offset");
}

void test_poshold_pilot_override_then_brake_and_relatch()
{
  Sim s;
  PositionHoldInput origin; origin.lat = 450000000; origin.lon = 100000000;
  s.ph.update(origin);

  // pilot pushes forward (north) for 2 s: no position control, target follows
  PositionHoldOutput out;
  for (int i = 0; i < 100; i++) out = s.step(0.0f, 0.5f);
  TEST_ASSERT_FALSE(out.controlling);
  TEST_ASSERT_TRUE(out.phase == PosHoldPhase::PILOT);
  const float flown = s.n;
  TEST_ASSERT_TRUE(flown > 0.5f);

  // release: brake, then hold at the stopping point (not back at the origin)
  for (int i = 0; i < 50 * 20; i++) out = s.step();
  TEST_ASSERT_TRUE(out.phase == PosHoldPhase::HOLD);
  TEST_ASSERT_TRUE(s.n > flown * 0.5f); // did NOT snap back to the original latch
  TEST_ASSERT_TRUE(std::hypot(s.vn, s.ve) < 0.2f);
}

void test_poshold_filter_recovers_after_persistent_rejections()
{
  PositionHold ph;
  PositionHoldInput in;
  in.lat = 450000000;
  in.lon = 100000000;
  in.gpsTimestampMs = 1000;
  in.horizontalAccuracy = 1.0f;

  auto out = ph.filterGps(in);
  TEST_ASSERT_TRUE(out.gpsFilterAccepted);
  TEST_ASSERT_EQUAL_UINT32(1, out.acceptedSamples);

  // A 20 m step is outside the 12 m innovation gate. Five consecutive
  // rejections must re-seed the filter instead of locking it out forever.
  in.lat += static_cast<int32_t>(20.0f / M_PER_UNIT);
  for (int i = 0; i < 5; ++i)
  {
    in.gpsTimestampMs += 200;
    out = ph.filterGps(in);
  }

  TEST_ASSERT_FALSE(out.gpsFilterAccepted);
  TEST_ASSERT_EQUAL_UINT32(1, out.acceptedSamples);
  TEST_ASSERT_EQUAL_UINT32(5, out.rejectedSamples);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, out.rawFilteredDistance);

  // The next good sample must be accepted from the recovered reference.
  in.lat += static_cast<int32_t>(0.2f / M_PER_UNIT);
  in.gpsTimestampMs += 200;
  out = ph.filterGps(in);
  TEST_ASSERT_TRUE(out.gpsFilterAccepted);
  TEST_ASSERT_EQUAL_UINT32(2, out.acceptedSamples);
}

void test_poshold_far_from_target_relatches_instead_of_chasing()
{
  PositionHold ph;
  PositionHoldInput in;
  ph.update(in);
  in.lat = static_cast<int32_t>(100.0f / M_PER_UNIT); // GPS jumped 100 m
  const auto out = ph.update(in);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, out.pitchAngle);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, out.rollAngle);
}


void test_navigation_local_ned_for_rescue()
{
  NavigationInput in{};
  in.homeValid = true;
  in.gpsValid = true;
  in.attitudeValid = true;
  in.altitudeValid = true;
  in.homeLatitudeE7 = 450000000;
  in.homeLongitudeE7 = 100000000;
  in.latitudeE7 = 450001000;
  in.longitudeE7 = 100000000;
  in.altitude = 12.0f;
  in.homeAltitude = 10.0f;
  in.velocityNorth = 1.0f;
  in.velocityEast = -0.5f;
  in.velocityDown = -0.2f;

  const auto out = Navigation::update(in);
  TEST_ASSERT_TRUE(out.positionValid);
  TEST_ASSERT_FLOAT_WITHIN(0.2f, 11.13f, out.north);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, out.east);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, -2.0f, out.down);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 2.0f, out.altitudeAboveHome);
}

void test_gps_rescue_state_machine_reaches_return()
{
  GpsRescue r;
  GpsRescueInput in{};
  GpsRescueParams p{};
  in.requested = true;
  in.armed = true;
  in.homeValid = true;
  in.gpsValid = true;
  in.attitudeHealthy = true;
  in.altitudeHealthy = true;
  in.positionValid = true;
  in.velocityValid = true;
  in.sats = 10;
  in.horizontalAccuracy = 1.5f;
  in.gpsAgeS = 0.1f;
  in.north = 50.0f;
  in.altitudeAboveHome = 2.0f;
  in.yawHeading = 3.14159265358979323846f;
  in.dt = 0.02f;

  auto out = r.update(in, p);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(
          GpsRescuePhase::CLIMB),
      static_cast<uint8_t>(
          out.phase));
  TEST_ASSERT_TRUE(out.controlling);
  TEST_ASSERT_TRUE(out.verticalRate > 0.0f);

  in.altitudeAboveHome = 12.0f;
  out = r.update(in, p);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(
          GpsRescuePhase::ALIGN),
      static_cast<uint8_t>(
          out.phase));

  out = r.update(in, p);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(
          GpsRescuePhase::RETURN),
      static_cast<uint8_t>(
          out.phase));
}

void test_gps_rescue_falls_back_to_land_on_gps_loss()
{
  GpsRescue r;
  GpsRescueInput in{};
  GpsRescueParams p{};
  in.requested = true;
  in.armed = true;
  in.homeValid = true;
  in.gpsValid = true;
  in.attitudeHealthy = true;
  in.altitudeHealthy = true;
  in.positionValid = true;
  in.velocityValid = true;
  in.sats = 10;
  in.horizontalAccuracy = 1.0f;
  in.gpsAgeS = 0.1f;
  in.north = 20.0f;
  in.altitudeAboveHome = 12.0f;
  in.yawHeading = 3.14159265358979323846f;
  in.dt = 0.02f;

  r.update(in, p);
  in.gpsValid = false;

  // One bad sample only starts the confirmation window.
  auto out = r.update(in, p);
  TEST_ASSERT_TRUE(out.active);
  TEST_ASSERT_TRUE(out.faultEvaluationPending);
  TEST_ASSERT_FALSE(out.requestLand);
  TEST_ASSERT_FALSE(out.controlling);
  TEST_ASSERT_NOT_EQUAL(
      static_cast<uint8_t>(GpsRescuePhase::ABORT),
      static_cast<uint8_t>(out.phase));

  // A transient recovery inside the window cancels the handoff.
  in.gpsValid = true;
  out = r.update(in, p);
  TEST_ASSERT_FALSE(out.faultEvaluationPending);
  TEST_ASSERT_FALSE(out.requestLand);

  // A persistent failure is confirmed only after the evaluation window.
  in.gpsValid = false;
  out = r.update(in, p);
  for (int i = 0; i < 20 && !out.requestLand; ++i)
  {
    out = r.update(in, p);
  }

  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(GpsRescuePhase::ABORT),
      static_cast<uint8_t>(out.phase));
  TEST_ASSERT_FALSE(out.controlling);
  TEST_ASSERT_TRUE(out.requestLand);
}

void test_gps_rescue_limits_horizontal_command()
{
  GpsRescue r;
  GpsRescueInput in{};
  GpsRescueParams p{};
  p.maxAngle = 0.3f;
  p.maxAcceleration = 1.0f;

  in.requested = true;
  in.armed = true;
  in.homeValid = true;
  in.gpsValid = true;
  in.attitudeHealthy = true;
  in.altitudeHealthy = true;
  in.positionValid = true;
  in.velocityValid = true;
  in.sats = 10;
  in.horizontalAccuracy = 1.0f;
  in.gpsAgeS = 0.1f;
  in.north = 40.0f;
  in.altitudeAboveHome = 12.0f;
  in.yawHeading = 3.14159265358979323846f;
  in.dt = 0.02f;

  r.update(in, p);
  const auto out = r.update(in, p);
  TEST_ASSERT_TRUE(
      std::fabs(out.pitchAngle) <=
      p.maxAngle + 1e-5f);
  TEST_ASSERT_TRUE(
      std::fabs(out.rollAngle) <=
      p.maxAngle + 1e-5f);
}


void test_gps_rescue_climb_holds_horizontal_position()
{
  GpsRescue r;
  GpsRescueInput in{};
  GpsRescueParams p{};

  in.requested = true;
  in.armed = true;
  in.homeValid = true;
  in.gpsValid = true;
  in.attitudeHealthy = true;
  in.altitudeHealthy = true;
  in.positionValid = true;
  in.velocityValid = true;
  in.sats = 10;
  in.horizontalAccuracy = 1.0f;
  in.gpsAgeS = 0.1f;
  in.north = 50.0f;
  in.east = 0.0f;
  in.velocityNorth = 2.0f;
  in.altitudeAboveHome = 2.0f;
  in.yawHeading = 0.0f;
  in.dt = 0.02f;

  const auto out = r.update(in, p);

  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(GpsRescuePhase::CLIMB),
      static_cast<uint8_t>(out.phase));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, out.targetVelocityNorth);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, out.targetVelocityEast);
  TEST_ASSERT_TRUE(out.pitchAngle < 0.0f);
}

void test_gps_rescue_stops_return_after_no_progress()
{
  GpsRescue r;
  GpsRescueInput in{};
  GpsRescueParams p{};

  in.requested = true;
  in.armed = true;
  in.homeValid = true;
  in.gpsValid = true;
  in.attitudeHealthy = true;
  in.altitudeHealthy = true;
  in.positionValid = true;
  in.velocityValid = true;
  in.sats = 10;
  in.horizontalAccuracy = 1.0f;
  in.gpsAgeS = 0.1f;
  in.north = 50.0f;
  in.altitudeAboveHome = 20.0f;
  in.yawHeading = 3.14159265358979323846f;
  in.dt = 0.02f;

  r.update(in, p);
  in.altitudeAboveHome = 30.0f;
  r.update(in, p);
  r.update(in, p);
  r.update(in, p);

  GpsRescueOutput out{};
  for (int i = 0; i < 500 && out.phase != GpsRescuePhase::ABORT; ++i)
  {
    out = r.update(in, p);
  }

  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(GpsRescuePhase::ABORT),
      static_cast<uint8_t>(out.phase));
  TEST_ASSERT_TRUE(
      (out.faultFlags & GPS_RESCUE_FAULT_NO_PROGRESS) != 0);
  TEST_ASSERT_TRUE(out.requestLand);
}

void test_gps_rescue_inside_min_distance_skips_return_leg()
{
  GpsRescue r;
  GpsRescueInput in{};
  GpsRescueParams p{};
  p.minDistance = 15.0f;
  p.landDistance = 4.0f;

  in.requested = true;
  in.armed = true;
  in.homeValid = true;
  in.gpsValid = true;
  in.attitudeHealthy = true;
  in.altitudeHealthy = true;
  in.positionValid = true;
  in.velocityValid = true;
  in.sats = 10;
  in.horizontalAccuracy = 1.0f;
  in.gpsAgeS = 0.1f;
  in.north = 8.0f;
  in.altitudeAboveHome = 20.0f;
  in.yawHeading = 0.0f;
  in.dt = 0.02f;

  const auto out = r.update(in, p);

  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(GpsRescuePhase::APPROACH),
      static_cast<uint8_t>(out.phase));
}

void setUp() {}
void tearDown() {}

int main(int, char**)
{
  UNITY_BEGIN();
  RUN_TEST(test_poshold_sign_target_ahead_heading_north);
  RUN_TEST(test_poshold_sign_rotates_with_heading);
  RUN_TEST(test_poshold_east_target_heading_north_rolls_right);
  RUN_TEST(test_poshold_lean_angle_is_capped);
  RUN_TEST(test_poshold_converges_from_offset_all_headings);
  RUN_TEST(test_poshold_rejects_constant_wind);
  RUN_TEST(test_poshold_pilot_override_then_brake_and_relatch);
  RUN_TEST(test_poshold_far_from_target_relatches_instead_of_chasing);
  RUN_TEST(test_poshold_filter_recovers_after_persistent_rejections);
  RUN_TEST(test_navigation_local_ned_for_rescue);
  RUN_TEST(test_gps_rescue_state_machine_reaches_return);
  RUN_TEST(test_gps_rescue_falls_back_to_land_on_gps_loss);
  RUN_TEST(test_gps_rescue_limits_horizontal_command);
  RUN_TEST(test_gps_rescue_climb_holds_horizontal_position);
  RUN_TEST(test_gps_rescue_stops_return_after_no_progress);
  RUN_TEST(test_gps_rescue_inside_min_distance_skips_return_leg);
  return UNITY_END();
}
