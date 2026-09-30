# Assisted V2 integration contract

This document defines the radio/channel contract and active-controller policy for
Angle V2, AltHold V2, and failsafe LAND V2.

## Channel contract

ESP-FC uses logical AETR input indices after receiver mapping:

| Logical index | Meaning | Assisted V2 use |
| ---: | --- | --- |
| 0 | Roll | normal pilot roll |
| 1 | Pitch | normal pilot pitch |
| 2 | Yaw | normal pilot yaw |
| 3 | Throttle | accumulated/manual throttle |
| 4 | AUX1 | arm/mode switch as configured |
| 5 | AUX2 | available/legacy project switch |
| 6 | AUX3 | **raw spring-centered vertical stick for AltHold** |

The standard `esp32` production build and the production-policy native
validation environment compile with:

```
-DESPFC_ALTHOLD_V2_CENTERED_STICK_CHANNEL=6
```

For the custom LoRa/iBUS radio, iBUS channel 7 (zero-based channel index 6)
therefore carries the raw left-stick Y position mapped to 1000..2000 us with
approximately 1500 us at physical stick center.

The ordinary throttle channel remains independent. It may keep the existing
stateful/accumulated throttle behavior for manual flight. AltHold must not use
that accumulated value as a climb-rate request.

Expected receiver output while the vertical stick is released:

```
CH3 / logical throttle (index 3): previous accumulated manual throttle
CH7 / logical AUX3    (index 6):  1500 us
```

Expected AUX3 values:

```
full down   -> about 1000 us -> descent request
center      -> about 1500 us -> zero pilot climb/descent request
full up     -> about 2000 us -> climb request
```

The FC applies its own deadband and climb/descent scaling. The transmitter
should therefore send the raw centered stick rather than another integrated
or rate-shaped throttle value.

## Failsafe LAND ownership

Once Stage-2 AUTO_LAND begins, LAND remains authoritative for that armed flight.
A recovered receiver is still qualified, but pilot throttle is not handed back
during the descent. LAND ends only through estimator failure fallback,
touchdown confirmation, or the bounded LAND timeout, all of which disarm.

This avoids a thrust discontinuity when the custom transmitter's manual
throttle channel contains an accumulated value unrelated to the current
spring-stick position.

## Build policy

The standard `esp32` environment is the active motor-driving target for this
project. It defines:

```
-DESPFC_ASSISTED_V2_ACTIVE
-DESPFC_ASSISTED_V2_OUTPUT_ACK
-DESPFC_ALTHOLD_V2_CENTERED_STICK_CHANNEL=6
-DESPFC_ANTI_GRAVITY_ACTIVE
```

ANGLE V2 is always compiled and remains the authoritative Angle controller.
`ESPFC_ASSISTED_V2_ACTIVE` promotes AltHold V2 and LAND V2 to their
authoritative controller paths. A motor-driving Assisted V2 build still requires
`ESPFC_ASSISTED_V2_OUTPUT_ACK`, which prevents accidental actuator authority
in an unacknowledged build.

Anti-Gravity is compiled through its active rate-PID path in the same production
target. Runtime feature/mode state still decides whether Anti-Gravity actually
modifies roll/pitch P/I terms, and Anti-Gravity yields whenever Assisted V2 owns
vertical thrust.

The repository keeps non-actuating validation targets:

- `esp32_antigravity_bench`: active Anti-Gravity controller math with
  `ESPFC_SAFE_BENCH_BUILD`, so ESC/servo drivers are not attached.
- `esp32_assisted_v2_candidate`: active AltHold/LAND production-policy math
  with `ESPFC_SAFE_BENCH_BUILD`.
- `native_assisted_v2_active`: unit-test mirror of the standard ESP32
  authority policy, including active Anti-Gravity, with no physical hardware.

GitHub Actions publishes the two ESP32 SAFE_BENCH validation firmware artifacts
and separately builds the standard `esp32` target, so CI checks both the
non-actuating validation paths and the actual production compile policy.

## Blackbox validation views

Use the existing debug modes to identify which layer generated an unexpected
command during non-actuating hardware validation.

### `AUTOPILOT_ALTITUDE`

| debug field | value |
| --- | --- |
| debug[0] | fused altitude, cm |
| debug[1] | altitude target, cm |
| debug[2] | fused vertical rate, cm/s |
| debug[3] | vertical-rate target, cm/s |
| debug[4] | pilot vertical-rate request, cm/s |
| debug[5] | barometer innovation, cm |
| debug[6] | altitude estimator healthy (0/1) |
| debug[7] | barometer sample accepted (0/1) |

### `AUTOPILOT_PID`

| debug field | value |
| --- | --- |
| debug[0] | vertical-rate target, cm/s |
| debug[1] | measured vertical rate, cm/s |
| debug[2] | vertical-rate error, cm/s |
| debug[3] | vertical PID P term x1000 |
| debug[4] | vertical PID I term x1000 |
| debug[5] | vertical PID D term x1000 |
| debug[6] | requested normalized thrust x1000 |
| debug[7] | packed Assisted V2 status bits |

`debug[7]` status bits:

| bit | meaning |
| ---: | --- |
| 0 | ALTHOLD mode active |
| 1 | assisted altitude controller active |
| 2 | altitude estimator healthy |
| 3 | barometer sample accepted |
| 4 | Assisted V2 currently owns vertical output |
| 5 | LAND requested |
| 6 | LAND supervisor active |
| 7 | LAND output blocked |
| 8 | receiver channels valid |

These fields make the complete command chain observable in both the active
production controller and the non-actuating SAFE_BENCH validation builds.

## Configuration prerequisites

Before Assisted V2 can be considered available at runtime, the FC still
requires its saved configuration to provide the actual hardware settings:
a detected gyro/accelerometer, a detected and calibrated barometer, a valid
motor protocol, correct receiver channel mapping, and mode conditions for
ARM/ANGLE/ALTHOLD as desired.

AUTO_LAND is not the source-code default. The default remains DROP, so a saved
configuration must explicitly select AUTO_LAND before LAND V2 can become the
Stage-2 failsafe procedure.

## LAND termination

LAND V2 uses a build-selectable positive descent-rate constant whose sign is
applied downward by the controller. The project production default is currently
**0.10 m/s downward**:

```
ESPFC_LAND_V2_DESCENT_RATE_MS = 0.10f
```

Touchdown confirmation uses near-ground altitude, low vertical speed, descent
evidence, reduced commanded thrust, and a dwell period. The independent timeout
uses the same selected descent-rate constant:

```
timeout = clamp(entry_height / selected_descent_rate + 10 s, 15 s, 60 s)
```

If the estimator becomes invalid or the timeout expires, LAND disarms through
the failsafe path rather than remaining armed indefinitely.
