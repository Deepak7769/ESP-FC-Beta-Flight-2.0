# ESP-FC diagnosis — 10 November 2026

Firmware: `5e17dca91caa1e8459f089dc8a848935285a950e`, ESP32, build Nov 10 2026 13:29:49. Confirmed directly by live CLI. User-supplied Actions run: https://github.com/Deepak7769/ESP-FC-Beta-Flight-2.0/actions/runs/42824368242

Client: Betaflight 2026.6.1 (`14a057ffc58417c5128199fc1233284982a64be3`). Exact tagged source inspected in `../betaflight-app-2026.6.1`; firmware inspected in isolated detached worktree `../diagnosis-5e17dca`. Original checkout was not changed.

## 1. Confirmed cause of widespread missing configuration controls

Firmware `lib/Espfc/src/Connect/MspProcessor.cpp:515` answers MSP_FC_VERSION with only three bytes: 26, 6, patch. The exact client decoder at `src/js/msp/MSPHelper.js:852` treats major >=10 as a calendar version: it discards the next two bytes and reads a length-prefixed version string. The firmware omits that string, so the version becomes empty.

Client `src/js/serial_backend.js:1199` initializes Features and Beepers only if the firmware version is nonempty. The initial Features value stays numeric 0 (`src/js/fc.js:244`). Subsequent MSP_FEATURE_CONFIG decoding calls `.setMask(...)` on that numeric value (`MSPHelper.js:794`), causing an exception. MSP_BEEPER_CONFIG has the equivalent uninitialized-object problem. Tabs that load these commands cannot finish initialization.

Reproduction: `node docs/diagnostics/reproduce-version.cjs <Betaflight-2026.6.1-source-directory>`. Supply a checkout of Betaflight configurator tag `2026.6.1`. It executes the version decoder extracted from the exact tagged client against the current payload and verifies the empty result. Legacy three-byte versions and calendar versions with the missing string restored are successful controls. This is an offline reproduction, not an instrumented browser exception trace.

Required firmware correction: supply the length-prefixed calendar version text after the three bytes, consistent with the client's wire contract. Do not merely raise the advertised MSP API version. Reconnect after correcting the handshake so Features and Beepers are instantiated, then retest every affected tab.

## 2. Live CPU timing

| Measurement | Observed |
|---|---:|
| CPU frequency | 240 MHz |
| Gyro device clock | 8,000 Hz |
| Scheduled gyro / PID / mixer rates | 2,000 / 2,000 / 2,000 Hz |
| Accelerometer rate | 500 Hz |
| Gyro read time | 203 us per read, 40.7% |
| Accelerometer read time | 196 us per read, 9.8% |
| Gyro filtering | 4.6% |
| Outer / inner PID | 5.5% / 3.0% |
| Measured flight-task load | 75.9% |

Setup also displayed 76%. Serial UI traffic is not sufficient to explain this: the same heavy flight-task load appears in CLI statistics, where normal MSP polling is paused. Sensor reads account for 50.5 percentage points of task occupancy. I2C reads block the flight task; this occupancy is not a measurement of arithmetic utilization across both ESP32 cores.

The firmware defaults allow 2 kHz I2C sampling (`TargetESP32.h:72`) and select a device 8 kHz clock at the default DLPF mode. `Model.h:794` clamps the scheduled rate to 2 kHz. A conservative 1 kHz I2C gyro/PID schedule is the appropriate performance change to evaluate. Changing only PID divider leaves 2,000 gyro I2C transactions per second. Measure again after any rate change; no reduced-load number has yet been verified. No rates were changed during this inspection.

## 3. Sensor identity discrepancy

Live `status` reports **MPU6500/I2C**, not MPU6050. Sensor attitude values were updating. MPU6500 detection accepts WHO_AM_I 0x70 or 0x75; MPU6050 accepts 0x68 or 0x72. The device label is therefore meaningful evidence from the detection path, but the actual register value was not captured. Do not force the MPU6050 driver based only on the module's printed name.

## 4. Receiver condition and defaults

Observed Receiver page: receiver mode `0`, blank channel map, initially zero stick thresholds; later partial values appeared, but initialization remained incomplete. RC bars showed roll/pitch/yaw 1500 and throttle 1000 with an explicit failsafe warning. CLI confirmed RX_FAILSAFE, while the input frame-rate estimate was 43 Hz. A nonzero frame-rate estimate does not establish valid usable channels.

The later visible configuration dump showed RX_SERIAL enabled and RX_PPM disabled, UART3 serial RX, and GPIO35 as the separate input pin. Firmware defaults select SBUS (`ModelConfig.h:457`). Therefore the PPM-mailbox changes in this commit do not repair a receiver operating in serial mode. Actual receiver hardware, protocol, signal validity and wiring remain unverified; the broken page must first be restored to inspect them reliably. Do not switch to PPM or change pins without establishing the receiver type.

## 5. Other independent defects / inhibits

- **Blank loop-rate telemetry:** MSP_STATUS_EX uses `stats.loopTime()`. Its only tick is inside `Mixer::update()` after the early `outputReady` guard (`Output/Mixer.cpp:102–120`). With default motor protocol DISABLED and outputs ready 0, no tick occurs, so status can report zero despite the live CLI showing a 2 kHz controller schedule. Move cadence accounting to the successful control iteration, independently of motor output readiness, with appropriate distinctions between gyro, PID and mixer rate.
- **Blank MCU:** firmware reports MCU id 255 (name supplied separately), but exact client ignores MSP2_MCU_INFO for API <1.47 (`MSPHelper.js:1321`). Firmware advertises 1.46. This is another mixed-version contract; adding the command alone cannot make the client consume it.
- **Storage:** live status showed storage healthy 0, outputs ready 0 and REBOOT_REQUIRED. `Model::load()` marks storage healthy only after a successful configuration load; first boot/missing/incompatible configuration can produce this status. `Actuator.cpp:554` maps unhealthy storage to REBOOT_REQUIRED. This is not proof of defective physical flash. Inspect storage logs and validate save/reboot/readback after resolving the protocol issue.
- **Mode-range metadata:** firmware MSP_MODE_RANGES_EXTRA writes 3 as the first byte, describing it as bytes per entry (`MspProcessor.cpp:718`), but the matching client interprets that byte as entry count. Firmware sends ACTUATOR_CONDITIONS entries (8), so the client consumes only 3. Correct the header count and verify all mode rows survive read/write.
- **Motor inhibit:** MOTOR_PROTOCOL and outputs ready 0 agree with the dumped default DISABLED output protocol. This does not establish an ESC driver failure.

## 6. Configurator coverage

| Area | Evidence / outcome |
|---|---|
| Setup | Inspected live: attitude updates, CPU 76%, blank firmware version/MCU/loop rate, RXLOSS and other inhibits |
| Receiver | Inspected live: incomplete initialization, blank map, mode 0, failsafe channels |
| Configuration | Inspected live: blank gyro frequency, empty Other Features and beeper controls |
| PID Tuning | Page rendered live with sliders and PID values; editing/saving and all subtabs were not tested |
| CLI | Live stats, status and get input returned; later visible dump captured configuration context |
| Failsafe | Source audit: loads MSP_FEATURE_CONFIG, so affected by the same handshake initialization bug |
| GPS | Source audit: loads MSP_FEATURE_CONFIG, same failure path |
| Motors | Source audit: loads MSP_FEATURE_CONFIG, same failure path; no motor tests performed |
| Blackbox / onboard logging | Source audit: loads MSP_FEATURE_CONFIG, same failure path |
| Ports, Power, Sensors, Modes, Adjustments, Presets, VTX, tethered logging, Autotune, viewer | Not fully exercised live; must not be described as passed or completely audited |

The live inspection could not complete every tab. The app window changed and the computer-use helper subsequently returned `foreground window did not report a process id`; further UI input was stopped. Source inspection completed the diagnosis of the shared failure path. No firmware was flashed, no settings saved, and no motor commands were issued by this investigation.

## Repair and verification order

1. Correct firmware-version encoding and retest full handshake with exact client 2026.6.1. Confirm Features/Beeper objects initialize and Receiver/Configuration finish loading.
2. Correct the I2C scheduling default, loop-rate accounting, MCU/API contract and mode-range entry count as separate changes with focused regression coverage.
3. Identify the actual sensor and receiver hardware/protocol. Confirm live channels and loss/recovery behavior.
4. Inspect storage load logs, then verify configuration persistence across save and reboot before treating REBOOT_REQUIRED as solved.
5. Revisit every configurator tab and record visible controls, readback, write/save/reboot behavior and intentionally unsupported features. Hardware output tests require a separate controlled test setup.

This report diagnoses the current build. It does not claim a repaired or flight-validated firmware binary.
