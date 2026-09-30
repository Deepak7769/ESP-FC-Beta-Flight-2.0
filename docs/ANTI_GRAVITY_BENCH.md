# Anti-Gravity active controller and bench validation

Anti-Gravity now uses one controller implementation for both production and
validation. The standard `esp32` production environment defines
`ESPFC_ANTI_GRAVITY_ACTIVE`, so the calculated Anti-Gravity demand can modify
the motor-driving roll/pitch rate PID when the runtime Anti-Gravity feature or
mode is enabled.

## Runtime ownership

Anti-Gravity is active only when all of the following are true:

- the firmware was compiled with `ESPFC_ANTI_GRAVITY_ACTIVE`;
- the Anti-Gravity feature or mode is enabled;
- receiver input is healthy;
- manual thrust owns the vertical output.

When AltHold V2 or LAND V2 owns vertical thrust, Anti-Gravity yields and its
gain demand returns to neutral. Yaw receives neither Anti-Gravity P boost nor
I acceleration.

The controller uses the existing PT2-filtered throttle-transient detector.
Roll and pitch receive the calculated P multiplier and additive I-term
accelerator. The configured `antiGravityGain` remains the runtime gain input.

## Production build

The standard project target:

```
esp32
```

defines `ESPFC_ANTI_GRAVITY_ACTIVE` together with the active Assisted V2
controller flags. This is the motor-driving production path; there is no
SAFE_BENCH requirement around Anti-Gravity authority.

Turning the feature/mode off still leaves the controller compiled but inactive,
so `ratePidApplied` remains false and the normal rate PID is used unchanged.

## Non-actuating bench build

The validation target remains:

```
esp32_antigravity_bench
```

It defines both `ESPFC_ANTI_GRAVITY_ACTIVE` and
`ESPFC_SAFE_BENCH_BUILD`. The same Anti-Gravity math runs, but
`ESPFC_SAFE_BENCH_BUILD` prevents the motor and servo drivers from being
initialized or attached.

GitHub Actions publishes the matching validation firmware as:

```
esp32_antigravity_validation_<commit>
```

The `native_assisted_v2_active` unit-test environment also compiles the
production Anti-Gravity authority path without SAFE_BENCH, alongside the active
Angle/AltHold/LAND controller policy.

## DEBUG_ANTI_GRAVITY

| field | meaning |
| ---: | --- |
| debug[0] | raw throttle derivative x100 |
| debug[1] | filtered throttle derivative x100 |
| debug[2] | pitch-equivalent I gain multiplier x1000 |
| debug[3] | pitch P gain multiplier x1000 |
| debug[4] | Anti-Gravity rate-PID application active (0/1) |
| debug[5] | gain-scaled filtered derivative x100 |
| debug[6] | additive I accelerator x1000 |

For validation, `debug[4]` should become 1 only during an accepted
Anti-Gravity transient and should remain 0 whenever Anti-Gravity is disabled,
receiver input is unhealthy, or Assisted V2 owns vertical thrust.
