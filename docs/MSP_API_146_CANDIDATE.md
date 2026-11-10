# API 1.46 review candidate

This source candidate targets Betaflight Configurator 10.10.0, commit
c97deaf7146837b2378e683a7f5b429fea34d5a9. The reference firmware wire definitions
are Betaflight 4.5.2, commit 024f8e13d4e642eb6a380308685b9ea3aa3ef1a2.
These references define byte layouts; they do not certify this firmware as
Betaflight or establish full feature parity. Complete supported-tab interaction
with a physical controller has not been run for this candidate.

## Identity and packets

MSP_API_VERSION returns 0,1,46. MSP_FC_VARIANT retains the BTFL family envelope
required by the unmodified Configurator handshake. The project name is ESP-FC;
MSP_FC_VERSION is the three-byte ESP-FC version 26,6,1. It is not Betaflight 4.5.2.
STATUS/STATUS_EX use the API 1.46 26-bit arming-disable namespace. Private newer
inhibits map to REBOOT_REQUIRED; internal ARM_SWITCH bit 29 maps to wire bit 25.
The CPU-temperature field terminates STATUS at 24 bytes with no extended modes.

| Packet | Candidate payload bytes |
| --- | ---: |
| MSP_FC_VERSION | 3 |
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

