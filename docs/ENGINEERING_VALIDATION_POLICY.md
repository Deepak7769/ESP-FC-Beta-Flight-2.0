# Engineering validation and upstream-reference policy

## Active and frozen hardware targets

- **Required development targets on `main`:** `esp32`, `esp32s3`, `esp32c3`, `esp8266`, `rp2040`, and `rp2350`. A repair is *not green* until the native test job, both safe-bench validation artifact jobs, and all six active hardware builds succeed.
- **Frozen ESP32-S2:** revision `d95f2c9941968929545c5a2743974adb45361a21` only (successful Actions run [42269919345](https://github.com/Deepak7769/ESP-FC-Beta-Flight-2.0/actions/runs/42269919345)). Rebuild only through `esp32s2-frozen.yml` with `workflow_dispatch`. The manual workflow explicitly checks out that commit, never current `main`.
- **Not an S2 repair:** leaving S2 out of the normal build matrix does not fix current-main S2 DRAM allocation or imply that later PID, Airmode, Angle, Anti-Gravity, AltHold, or Land repairs reach S2.
- Do not call any firmware flight-safe merely because its unit tests and compiler pass. The motor-driving builds require independent bench interlocks, motor-order/direction checks, real sensor/ESC timing validation, failsafe-injection tests, and progressively constrained flight testing.

## Authoritative practical implementation references

Use current upstream **source code as the primary behavioral reference**, with official documentation to establish operator-facing behavior. Differences in units, mixers, hardware resources, scheduling, and estimator health must be analyzed; do not copy constants or algorithms blindly.

| Repair domain | Practical upstream source or official design reference | Comparison required |
| --- | --- | --- |
| FPV PID, I-term relax, Anti-Gravity | [Betaflight PID implementation](https://github.com/betaflight/betaflight/blob/master/src/main/flight/pid.c), [PID definitions](https://github.com/betaflight/betaflight/blob/master/src/main/flight/pid.h) | Loop time and rate units; numerical PID gains; filtering phase, anti-windup, I-term relax, throttle transients |
| Multirotor rate control and allocation | [PX4 multicopter controller architecture](https://docs.px4.io/main/en/flight_stack/controller_diagrams) | Position/attitude/rate ownership, saturation propagation, integrator bounds, actuator authority |
| AltHold and vertical control | [ArduPilot Copter Altitude Hold](https://ardupilot.org/copter/docs/altholdmode.html) | Altitude→velocity→acceleration cascade, stick deadband, estimator validity, climb/descent acceleration caps |
| LAND and ground detection | [ArduPilot Copter LAND](https://ardupilot.org/copter/docs/land-mode.html) | Controlled descent, landed detection using multiple independent cues, timed disarming only upon grounded confirmation, estimator-fault behavior |

These links are **behavioral reference material**, not a certification of any code here. Record upstream revision/commit when comparing exact implementation constants. If a reference is unsuitable for ESP-FC, document why and what alternative was chosen.

## Required workflow for each control repair

1. Define the specific defect from current ESP-FC source, including a reproducible input/state trace. Capture baseline output.
2. Consult at least one real relevant upstream flight-controller implementation (Betaflight for FPV dynamics; PX4/ArduPilot for navigation, altitude and landing). Record reference path and behavioral assumptions.
3. Verify state ownership, input freshness, coordinate frames, units, sign conventions, control periods, saturation constraints, motor authority, arming/failsafe gates, and mode transitions.
4. Make one narrow, reviewable implementation change; avoid new dynamic allocation or unexpected RTOS work in the flight loop.
5. Add native regression tests for normal operation, boundary conditions, invalid data, mode exit/entry, and output saturation. For output-control changes include actuator-disabled bench tests.
6. Commit to `main`, then inspect the **entire** GitHub Actions run, including all six targets, not just native tests. If red, stop subsequent feature work and remediate. Never describe a failed commit as green because a later combined commit succeeds.
7. Record known limitations and require real-hardware validation before motor-driving firmware is considered a flight candidate.

## Current priorities

Baseline → timing/PID/mixer validation → Angle V2 → Anti-Gravity → AltHold V2 → Land V2.
Treat GPS Rescue/Position Hold interfaces as dependencies requiring checks when Angle/Altitude mode ownership changes.
External HC-SR04 is out of scope. Avoid changing unrelated sensor handling as an implied prerequisite.

## Measured gyro sample interval review (timing/PID repair)

- **Reference:** [PX4 MulticopterRateControl.cpp](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/mc_rate_control/MulticopterRateControl.cpp) uses gyro `timestamp_sample` for a measured, bounded `dt`; [PX4 rate_control.cpp](https://github.com/PX4/PX4-Autopilot/blob/main/src/lib/rate_control/rate_control.cpp) uses that `dt` for integrator updates. [ArduPilot AC_PID.cpp](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AC_PID/AC_PID.cpp) also explicitly takes `dt`.
- **Our adaptation:** `GyroSensor::read` timestamps successful gyro samples; `Controller` subtracts successive successful timestamps, not the timer divider that also advances after failed reads. Initialize to nominal interval, and constrain accepted elapsed intervals to 0.5–4.0 times the configured PID period. This bound is intentionally specific to our configured control period, rather than copying PX4's absolute 0.125–20ms limits.
- **PID math:** I uses elapsed seconds; D and derivative feedforward use its inverse; P does not depend on elapsed time. Filter coefficients remain configured at nominal sample rate until a separate, measured-frequency filter review.
- **Tests:** integration and derivative/feedforward scaling, missing gyro-cycle interval, nominal recovery, long-stall clamp, repeated timestamp, non-finite time fallback. Hardware CPU latency, actual gyro scheduler jitter, and motor/ESC waveforms still require instrumentation before flight.

## Angle V2 navigation rate-target ownership

- **Observed defect:** `updateAssistedModes()` enables `angleV2.active` for POSHOLD/GPS Rescue, but `outerLoop()` previously used those attitude rate targets only with `MODE_ANGLE` or LAND; otherwise it remapped pilot sticks. This severed the navigation controller from the inner rate loop.
- **Reference:** [PX4 multicopter attitude-control source](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/mc_att_control) passes attitude-controller body-rate setpoints to multicopter rate control regardless of a pilot Angle auxiliary switch. Unlike PX4's quaternion control, ESP-FC uses a fixed roll/pitch Angle V2 outer loop, so only ownership routing is mirrored.
- **Change:** POSHOLD requested and GPS Rescue active share Angle V2 rate-target ownership. When no valid Angle V2 target is available, rates fall back to zero (not stale commands); leaving navigation returns manual rates. Tests bypass estimator setup to specifically verify outer-loop routing and transition ownership.

## Anti-Gravity additive I gain unit correction

- **Reference:** [Betaflight `src/main/flight/pid.c`](https://github.com/betaflight/betaflight/blob/master/src/main/flight/pid.c) computes `itermAccelerator = antiGravityThrottleD * ANTIGRAVITY_KI` and adds it to its per-axis `Ki` in the integrated `I` update. [Betaflight `pid.h`](https://github.com/betaflight/betaflight/blob/master/src/main/flight/pid.h) defines `ANTIGRAVITY_KI = 0.34f`.
- **Unit mapping:** This firmware's `ITERM_SCALE` multiplies the Betaflight I coefficient by `Utils::toDeg(1.0f) * 0.001f`, because rate-error inputs are rad/s and output units normalized. The additive Anti-Gravity gain must receive the same factor. `ANTIGRAVITY_KP` affects a dimensionless P multiplier and is intentionally unchanged.
- **Regression:** Compare computed `AntiGravityState::iAccelerator / scaledDerivative` against the converted coefficient under transient manual throttle. This does **not** change standard firmware's existing feature gate that restricts rate-PID Anti-Gravity authority to specially enabled builds.

## AltHold V2-to-manual throttle handoff

- **Practical reference:** [ArduPilot Copter AltHold](https://ardupilot.org/copter/docs/altholdmode.html) defines altitude-controlled vertical speed from centered pilot throttle and highlights mode-switch mismatch; [ArduPilot mode implementation](https://github.com/ArduPilot/ardupilot/tree/master/ArduCopter) distinguishes pilot-throttle and automated vertical-control ownership. Our short throttle-output slew is **an ESP-FC-specific mitigation**, not a copied ArduPilot algorithm.
- **Observed defect:** previous authoritative output could be +0.25 while resumed manual stick mapped to -0.8, causing an abrupt -1.05 normalized thrust step in one rate-PID cycle.
- **Change:** on healthy armed AltHold exit only, rate-limit vertical output to 4 normalized units/s using bounded measured PID dt. The limiter terminates once pilot thrust is reached. It is canceled by disarm, lost/invalid RX, landing request, re-entered AltHold, or GPS-origin no-drop handoff. The physical mixer must always enforce immediate disarmed output independently.
- **Tests:** first-cycle throttle slew, eventual direct takeover, switch-to-disarm bypass, receiver-loss bypass. The slew constant must be reviewed against actual mass/thrust response and transition logs before flight use.

## LAND V2 timeout cannot disarm an unconfirmed airborne vehicle

- **Observed fault:** a hard 60-second LAND timeout previously called `Model::disarm()` while the altitude estimator could still show significant height and descent speed. At the configured 0.1 m/s descent rate, a 20m approach requires ~200s before contact, so this can cause an airborne motor cut.
- **Real firmware reference:** [ArduPilot `mode_land.cpp`](https://github.com/ArduPilot/ardupilot/blob/master/ArduCopter/mode_land.cpp) disarms after its landed detector reports completion and motors have reached ground idle; [ArduPilot `land_detector.cpp`](https://github.com/ArduPilot/ardupilot/blob/master/ArduCopter/land_detector.cpp) uses motor/thrust, attitude and motion evidence rather than flight time alone.
- **Change:** retain elapsed-time timeout as a fault diagnostic only, proportionally extend its maximum reporting interval to 900s, and continue controlled descent while the estimator is healthy. Automatic disarm remains gated by existing touchdown candidate/dwell confirmation. Tests cover expiration while still descending, high-altitude entry, and verified touchdown after an earlier timeout.
- **Outstanding:** the existing local baro-height near-ground check still assumes height relative to the original launch altitude and can be wrong at different terrain. A later landing-detector repair must remove that assumption and validate contact with independent cues. Estimator faults and power depletion still need an explicit hardware-tested emergency policy.

## CPU-overload LAND resource-latch recovery

- **Defect:** `ResourceManager::applyPolicy()` latched `_cpuLandActive` until controller reinitialization, so MSP, telemetry, blackbox and navigation remained shut off across disarm and subsequent rearming.
- **Practical references:** [ArduPilot Scheduler](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Scheduler/AP_Scheduler.cpp) tracks load/overruns continuously; [PX4 Commander](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/commander/Commander.cpp) provides explicit arm/disarm lifecycle transitions. Our CPU-LAND load-shedding policy is ESP-FC-specific; the requirement is to reset emergency *flight* state after disarm, never to invent an equivalent PX4/ArduPilot CPU-LAND mode.
- **Change:** if the craft is no longer armed, clear CPU-LAND latch, reset hysteresis/peak/deadline history, and reevaluate normal access policy. Preserve the latch while armed even if the measured CPU drops. Test forced overload and post-disarm recovery with a compile-time test friend (no production test API).

### CPU-LAND feature selection compilation fix

The first CPU-LAND recovery regression failed because `ResourceManager.cpp` did not include `Control/AssistedModeV2.h`, where `ESPFC_LAND_V2_ACTIVE` is derived from `ESPFC_ASSISTED_V2_ACTIVE` or `ESPFC_LAND_V2_ACTIVE_TEST`. Consequently the `#if defined(ESPFC_LAND_V2_ACTIVE)` inside `enterCpuLand()` compiled out, while the test TU saw LAND active. The module now includes the authoritative feature-policy header. CI validates that emergency LAND can activate under sustained CPU overload and release resources after disarm. **Caution:** this enables previously inert CPU-LAND behavior and must undergo armed/gyro-health/estimator fault-injection bench validation before physical flight.

## Anti-Gravity derivative uses successful gyro-sample timing

- **Defect:** Anti-Gravity used nominal `loopTimer.rate` for its throttle derivative, even though the rate PID already bounded `dt` from the elapsed time between successful gyro samples. A missed 1kHz cycle could overstate throttle-change rate, I acceleration and P boost.
- **Reference:** [Betaflight Anti-Gravity throttle derivative](https://github.com/betaflight/betaflight/blob/master/src/main/flight/pid.c) uses the actual PID update frequency for its throttle derivative. ESP-FC's measured bounded rate-PID `dt` is the appropriate equivalent when scheduler intervals are irregular.
- **Change:** derive Anti-Gravity throttle derivative frequency from the same measured/bounded `innerPid[AXIS_ROLL].dt` used by the rate loop. Native test verifies a 4ms valid-gyro gap reduces a fixed 0.5 throttle-step derivative from the nominal 31.25 to 7.8125. Filter cutoff remains nominal and requires separate jitter/bench characterization.

## LAND motor-balance regression for observed bench asymmetry

- The reported front/rear motor split when LAND is engaged at level attitude is tested through the complete Angle V2 → rate PID → Quad X mixer path. With fresh, level attitude and zero gyro input, stale roll/pitch pilot sticks must be ignored, PID roll/pitch outputs zero, and all motor outputs match. A real pitch angle error must still generate an opposing differential.
- Reference: [Betaflight multirotor motor mixer](https://github.com/betaflight/betaflight/blob/master/src/main/flight/mixer.c), which allocates collective thrust and roll/pitch/yaw corrections separately. This test is an isolated software check, not evidence that actual ESC signals or gyro calibration are correct. Investigate real output traces, board orientation, sensor bias and motor order if bench differences persist.

## Directional anti-windup from Quad X mixer authority

- **Bug:** a near-limit motor previously set `outputSaturated=true` for all roll/pitch/yaw integrators via `Model::setOutputSaturation`, even when the affected axis wanted to move that motor *away* from the limit. This artificially prevents an otherwise available attitude/rate correction.
- **Practical references:** [Betaflight motor mixer](https://github.com/betaflight/betaflight/blob/master/src/main/flight/mixer.c) considers torque mixing and output clipping; [PX4 control allocation](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/control_allocator/ControlAllocator.cpp) reports unallocated torque and actuator saturation to the rate control system. ESP-FC cannot directly copy those allocation methods because it has different mixer tables and PID normalization.
- **Change:** For built-in Quad X / Quad X 1234 only, mark an axis's PID output saturated when its rate error requests a motor change in a direction already blocked by the actual motor limit; account for yaw reversal. Preserve the old conservative global saturation for other mixer layouts, and keep vertical saturation feedback directional. Test opposing roll/pitch demand under identical motor saturation and verify that unaffected integrators remain available. This is still a *software* approximation; actual ESC authority must be measured on a motor-off bench and in blackbox flight data.

## LAND uses current terrain clearance when available

- **Defect:** `Actuator::updateFailsafeLand()` assumed `altitude.height <= 0.30m` measured ground clearance. The estimator is takeoff-relative, so a vehicle hovering over a slope or elevated terrain can appear 'near ground' without contact; conversely, touchdown above launch elevation can go unrecognized.
- **Real firmware source:** [ArduPilot land detector](https://github.com/ArduPilot/ardupilot/blob/master/ArduCopter/land_detector.cpp) combines landed-state checks with healthy rangefinder AGL clearance, motor limits, and motion evidence. ESP-FC currently retains its own baro/vario/thrust/dwell logic and does not copy ArduPilot's full landed detector.
- **Change:** when a generic rangefinder is present, accept only a valid reading younger than 100ms to establish near-ground entry/hold thresholds. A stale or invalid present sensor fails the clearance check closed; no takeover by launch-relative barometric altitude. When no sensor exists, retain legacy barometric check to avoid silently disabling auto-disarm for the current hardware fleet. This implementation neither introduces nor depends on HC-SR04 hardware.
- **Tests:** freshly reported 1.5m AGL vetoes a false barometric touchdown; a stale reading cannot bypass it; a 0.1m valid AGL reading allows verified touchdown at elevated terrain. Physical sensor angle, slant range, failure modes and false-contact behavior remain unverified and require hardware tests.

## Reject non-finite estimator states before assisted control

- **Observed gap:** the Angle V2 freshness predicate checked the estimator's boolean health and timestamp but not its roll/pitch/yaw values or tilt cosine. The AltHold and navigation gates similarly trusted `altitude.healthy` without finite checks on height, vario, or vertical acceleration. Non-finite values could propagate into PID arithmetic and motor commands.
- **Reference:** [PX4 multicopter AttitudeControl](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/mc_att_control/AttitudeControl/AttitudeControl.cpp) validates attitude targets for finite/normalizable quaternions before accepting them. This fix adds ESP-FC-specific scalar validity checks before granting Angle/AltHold/GPS Rescue ownership; it does not claim to replace full sensor innovation/freshness health logic.
- **Tests:** deliberately inject a NaN into roll, tilt cosine, altitude, vertical speed and vertical acceleration while health flags remain set. Demand that Angle and/or AltHold authority be withdrawn and restored only with finite inputs. Motor-driving acceptance still requires real IMU/baro corruption and recovery fault-injection trials.

## Align supervisor LAND estimator health with controller validity

- **Observed mismatch:** Controller's assisted-mode selector now refuses non-finite Euler yaw, tilt cosine or vertical acceleration, while the Actuator supervisor's `attitudeEstimateHealthy` and `altitudeEstimateHealthy` still accepted them under healthy/fresh state flags. Consequently the supervisor could grant LAND authority while the controller could not issue a valid leveling/descent target.
- **Reference:** [PX4 multicopter attitude controller](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/mc_att_control/AttitudeControl/AttitudeControl.cpp) validates attitude data before accepting control targets. This ESP-FC change preserves the actuator's existing quaternion norm bounds and adds the same scalar finite checks as the assisted controller.
- **Regression:** A separately tested healthy estimator must pass; NaN yaw and tilt cosine must reject attitude and altitude, NaN vertical acceleration must reject altitude only, and restoring valid numbers restores eligibility. This is independent of and does not solve automatic LAND's currently separate estimator-loss policy.

### Native test isolation after actuator health regression

The new actuator estimator-health test passed, but the following `test_timer_check_micros` aborted due to ArduinoFake call-history/stub contamination from the earlier `micros()` AlwaysReturn setup. The test now explicitly resets ArduinoFake at exit so exact-call-count timer tests can run independently. Do not interpret this test-harness failure as a passing whole workflow; recheck all native environments and six targets.
