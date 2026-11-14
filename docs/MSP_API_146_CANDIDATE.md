# API 1.46 review candidate

This source uses the MSP API 1.46 subset. Its decoder/serializer regression now
runs against Betaflight App 2026.6.1, commit
14a057ffc58417c5128199fc1233284982a64be3 (the version visible in the reference
videos). The version packet is also checked against Configurator 10.10.0,
commit c97deaf7146837b2378e683a7f5b429fea34d5a9; that does not establish complete
legacy UI validation. The reference firmware wire definitions
are Betaflight 4.5.2, commit 024f8e13d4e642eb6a380308685b9ea3aa3ef1a2.
These references define byte layouts; they do not certify this firmware as
Betaflight or establish full feature parity. Complete supported-tab interaction
with a physical controller has not been run for this candidate.

## Identity and packets

MSP_API_VERSION returns 0,1,46. MSP_FC_VARIANT retains the BTFL family envelope
required by the unmodified Configurator handshake. The project name is ESP-FC;
MSP_FC_VERSION is the three-byte ESP-FC version 26,6,1 followed by the
length-prefixed text `2026.6.1`. Calendar-version clients require this text;
omitting it prevents Features/Beeper initialization. It is not Betaflight 4.5.2.
STATUS/STATUS_EX use the API 1.46 26-bit arming-disable namespace. Private newer
inhibits map to REBOOT_REQUIRED; internal ARM_SWITCH bit 29 maps to wire bit 25.
The CPU-temperature field terminates STATUS at 24 bytes with no extended modes.

| Packet | Candidate payload bytes |
| --- | ---: |
| MSP_FC_VERSION | 12 |
| MSP_PID | 30 (ten slots, stored indices preserved) |
| MSP_SET_PID | 15 legacy or 30 full; unsupported nonzero slots rejected |
| MSP_SENSOR_ALIGNMENT | 7 |
| MSP_SET_SENSOR_ALIGNMENT | 3 legacy or 6; first gyro only |
| MSP_MODE_RANGES_EXTRA | 25 (count byte 8, eight three-byte entries) |
| MSP_RX_MAP | 8 |
| MSP_RX_CONFIG | 38 |
| MSP_RC_TUNING | 23 |
| MSP_FILTER_CONFIG | 49 |
| MSP_BATTERY_CONFIG | 13 |
| MSP_GPS_RESCUE | 26 |

The three API 1.47/1.48 RX, throttle-hover and filter tail fields are absent.
SET packets are staged, decoded completely, validated, then committed once.
Truncated optional groups, trailing bytes, invalid selectors and invalid
composite values fail before commit. Reload, notifications and reboot-required
side effects follow successful commit. This does not turn ignored historical
reserved fields into implemented flight features.

API 1.46 receiver smoothing uses the feedforward cutoff at offset 26, not
API 1.47's separate throttle cutoff. The adapter updates the real derivative
filter and reloads its coefficients. Provider changes require reboot; UART
framing is not switched live. Throttle smoothing remains configurable in CLI.

Accelerometer and active-gyro hardware IDs are translated explicitly to the
legacy client lists. For example, internal MPU6500=4 becomes accelerometer=8
and gyro=7. Unsupported selections fail instead of selecting a different chip.
SPL06 and QMC5883P have no names in the legacy lists: their selectors report
255 and an unchanged GET/SET preserves the stored device. Configure these
devices in ESP-FC CLI; they must not be mislabeled as LPS/LIS2MDL. The legacy
MPU6000/MS5611/AK8975 enums do not imply implemented detection drivers.

RAW_IMU acceleration is 2048 counts/g. Gyro packets invert the supported
clients' `4/16.4` conversion so a 120 degrees/second internal observation
displays as 120 degrees/second, not about 29.3. This changes reporting only.
ATTITUDE retains deci-degree roll/pitch and degree yaw with its existing sign
convention. Packet tests inject known state; they are not physical IMU tests.

## Diagnostic access and bus policy

ESP32 and ESP32-S3 retain a 4 kHz SPI ceiling. Software I2C uses at most 1 kHz
at 400 kHz bus clock, with a lower polling budget at slower bus clocks. This
does not disable filters or flight modes. Stuck-clock faults propagate from
EspWire; empty reads cannot underflow; SDA recovery has a finite pulse count.
GPIO fault injection checks software bounds, not real bus timing or electrical
recovery. Actual MPU6050-I2C and MPU6500-SPI connection tests remain required.

Disarmed MSP/CLI remains serviceable during load shedding, using the existing
bounded serial parser. Armed load shedding and CPU-LAND priority are unchanged.
PID loop statistics are updated on successful control iterations independently
of ESC readiness, so a disabled motor protocol does not falsely report 0 Hz.

## Capability limits

There is one stored PID configuration and one stored rate configuration.
Profile banks, names, copy/reset/select operations, Adjustments and Autotune
are not implemented. The legacy client can still display four rate selectors;
API 1.46 has no rate-profile-count field. A visible selector does not prove
that a profile exists. Permanent AIRMODE feature-bit selection is unsupported;
the implemented AIRMODE uses the AUX mode. OSD, optical flow, reversible 3D,
arbitrary motor reordering and DShot direction commands are not implemented.

Other legacy fields remain reserved/no-op, including independent yaw deadband,
FPV camera angle, selected advanced tuning fields, PID-controller selection
and RTC setting. These are not validated as functioning flight features.
Stock presets are not certified for the ESP-FC CLI/configuration model.
RACEFLIGHT curve preview with expo differs between the pinned client's formula
and the Betaflight-derived firmware formula; do not infer exact curve parity
from a successful rate-packet roundtrip. Physical graphs/model rendering,
serial reconnection, actual reboot persistence and actuator behavior remain
bench acceptance items.

The videos identify firmware `7bf8293`, built September 25, 2026, advertising
API 1.48. That commit is unavailable in this repository's accessible history.
The videos prove visible behavior, not every setting's original implementation
or persistence. Regressions attributed to later commits use source diffs.

## Modes and capabilities

Stored MODE_* values remain unchanged. Wire IDs in stored-enum order are
0,28,1,200,13,27,26,31,4,201,46. IDs 200 and 201 are explicit ESP-FC extensions
for AltHold and Position Hold. They are not official Betaflight permanent IDs;
retired Betaflight BARO/GPS HOLD IDs 3 and 11 are no longer reused.
Configurator 10.10.0 builds AUX rows from BOXNAMES and BOXIDS. Its connection,
link-save and reconnect behavior still needs physical integration validation.

BOXNAMES, BOXIDS, status bit ordering, setter translation and runtime mode
eligibility share compiled capabilities. AltHold, Position Hold and Rescue are
hidden/rejected without authoritative AltHold+LAND. Anti-Gravity is hidden and
its feature bit rejected without its active implementation. Softserial and VTX
build options reflect their target implementations. ESP-NOW's RX_SPI feature
is available only where the actual input dispatch is compiled. Output protocols
are PWM through DSHOT600; PROSHOT is rejected before it can be silently mapped.
The separately frozen ESP32-S2 workflow is unchanged.

## Rescue profile selection

MSP_GPS_RESCUE and MSP_SET_GPS_RESCUE use the API 1.46 26-byte structure.
Speeds and vertical rates are centimetres/second; altitude and distances are
metres; throttle settings use the 1000..2000 logical motor-command range.
Stock altitude modes MAX/FIXED/CURRENT map explicitly to tracked-flight maximum,
fixed altitude, and current altitude plus initial climb. FIXED can attain a
lower return altitude before aligning. Return altitude is maintained in RETURN.
The stock min-sats, minimum start distance, angle, ground speed, vertical rates,
hover feedforward, throttle bounds, sanity option and arming-without-fix policy
feed the runtime paths. Packet/record roundtrips are covered; controller tuning
and actuator behavior still need hardware validation.

A standard SET selects extension.stockRescueEnabled. A legacy custom 18-byte
MSP2_ESPFC_GPS_RESCUE_CONFIG SET selects the compact ESP-FC profile. The compact
GET returns an error while the stock profile is selected: it cannot faithfully
represent every 16-bit stock field. Legacy AT_LEAST altitude mode cannot be
represented by a stock GET and returns an error rather than a false mapping.
CLI dump includes the stock profile fields and battery extension settings.

ESP-FC retains its own safety semantics: navigation/heading faults preserve
thrust during the 250ms confirmation and request LAND; they do not request DROP.
Too-close Rescue initiation falls back to LAND rather than flying outward.
Sanity OFF does not disable essential sensor/numeric validity gates. The stock
Rescue PID-tuning messages and every other Configurator feature are not implied
by implementing the main Rescue configuration message.

## Serial, VTX and battery

One implemented UART consumer owns each port. GPS, receiver, VTX and Blackbox
each have at most one owner. Incompatible combined masks are errors; CRSF's
integrated telemetry remains part of its receiver provider. Blackbox opens with
blackboxBaud. Soft ports currently accept MSP only.

SmartAudio is an ESP32-family one-wire transport candidate. TX completion must
include the UART shift register before releasing the pin. The nonblocking state
machine preserves partial writes, bounds work, retries, changes baud after
backoff, validates response CRC/length/opcode, and publishes acknowledged
settings. V1 DAC, V2 indexed and V2.1 advertised dBm power encodings are distinct.
Band/channel, power and two-state low-power-disarm are supported. Pit-mode
control, direct-frequency control and custom VTX tables are not implemented;
unsupported SETs fail, and observed pit/frequency mode is reported as not ready.
A real VTX and logic-analyser trace are still required.

Battery cells require eight stable fast-voltage observations, not the slow
filter startup ramp. Cell count stays fixed until a confirmed disarmed disconnect.
Minimum/maximum/warning thresholds and capacity roundtrip and persist. Capacity
can drive consumption percentage when consumption is valid. Critical-voltage
flight failsafe and chemistry inference are not added by this change.

## Persistence breaking change

Schema 6 uses a 16-byte header, target ID, payload size, ABI fingerprint and
CRC32 covering the header metadata and target-ABI-specific ModelConfig payload.
It is deliberately not a portable cross-target serialization format. Booleans
are checked before materializing the object; the whole candidate is validated
before assignment. Save checks commit and backend readback. Invalid boot data
leaves default RAM settings and a visible storage fault that inhibits arming.

Legacy A5 v2/v3/v4/v5 records are rejected without rewrite. Automatic historical
migration is NOT implemented. Before using this candidate on an existing device,
export the previous CLI settings using the existing firmware. Restore reviewed
settings, resolve validation errors, and explicitly save with the new firmware.
Do not assume every old dump can be replayed without ordering/validation changes.
CRC fail-closed detection is not atomic retention of the previous record during
sector erase. Interrupted-write and recovery tests are required per backend.

## Outputs and logging

Output readiness requires a supported nonempty mixer, bounded valid rules,
required unique pins, correct fixed-layout motor/servo roles, successful driver
begin and every required attach. Partial failure cleans up drivers and inhibits
arming. A non-actuating bench build records that state explicitly. RMT/SDK
failures and RP DMA exhaustion propagate; physical peripheral-failure tests are
still required. Pin policy is board-family conservative and is not a substitute
for verifying the actual module schematic and PWM allocation.

Blackbox lifecycle advances outside optional sample encoding. Serial drain work
is capped at 32 bytes per lifecycle service, with a wrap-safe close timeout and
forced-close diagnostics. Closing inhibits rearming until serviced. Mandatory
flash close service runs only disarmed, because the flash backend performs
synchronous partition writes. Flash write latency/durability is not certified by
host tests. blackboxUnsentBytes is the software-buffer count, not a complete
measurement of bytes still resident in a hardware UART. Logging remains opt-in.

No production build enables ESPFC_LAND_NO_RANGE_AUTODISARM_ACK. Native positive
ACK tests are simulation only. F01 physical no-range touchdown qualification
remains open. F48 external-Arduino rangefinder transport remains unsupported.

## Reproduce host validation without network access

Install/cache ArduinoFake and Unity through the normal project setup first.
Then point --deps to their common directory. These commands fetch nothing:

```sh
python bin/native_review.py --deps .pio/libdeps/native --profile land-ack --out /tmp/efc-review-land-ack
python bin/native_review.py --deps .pio/libdeps/native --profile production --out /tmp/efc-review-production test/test_fc/test_fc.cpp test/test_closure/test_closure.cpp
python bin/native_review.py --deps .pio/libdeps/native --profile base --out /tmp/efc-review-base test/test_fc/test_fc.cpp test/test_closure/test_closure.cpp
```

These checks use host GCC, ArduinoFake and Unity, not target SDKs. Run the normal
PlatformIO target matrix and actual Configurator/device checks before treating
this as a release. Where LeakSanitizer cannot operate under the host tracing
runtime, ASAN_OPTIONS=detect_leaks=0 permits address checking only; that is not
a leak check.

