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
