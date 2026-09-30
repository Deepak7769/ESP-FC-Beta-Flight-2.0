# MSP current-meter companion bridge

ESP-FC supports Betaflight-compatible `CURRENT_METER_MSP` as a telemetry-only
current source. The intended use is a companion processor that reads an INA219
and publishes current/consumption to the flight controller over an MSP serial
link.

This path has no flight-control authority. A missing, stale, or malformed
current sample is rejected and current telemetry falls back to zero.

## Default transport: companion push

The default build does **not** poll the companion. This allows an RX-only link
into the flight controller and avoids making the FC an MSP master on ports that
may also be used by Configurator.

The companion periodically sends an MSPv1 `MSP_ANALOG` reply. A 10 Hz update
rate is suitable and matches the cadence used by Betaflight's MSP current-meter
path.

The payload fields used by ESP-FC are:

| Offset | Size | Field | Unit |
| ---: | ---: | --- | --- |
| 0 | 1 | legacy voltage | 0.1 V; ignored by the current bridge |
| 1 | 2 | mAh drawn | mAh |
| 3 | 2 | RSSI | ignored by the current bridge |
| 5 | 2 | current | 0.01 A |

Modern `MSP_ANALOG` replies may include additional bytes after these first
seven bytes. ESP-FC accepts the extended form and only consumes the fields
listed above.

## FC configuration

Select current-meter source `MSP` / source ID `4`. The source ID follows
Betaflight's `currentMeterSource_e` values:

- `0`: NONE
- `1`: ADC
- `4`: MSP

Unsupported source IDs sanitize back to `NONE`.

The serial port receiving the companion frames must have the MSP function
enabled. USB VCP remains available for Configurator independently.

The current-meter stream must **not** be interleaved onto a port that is already
carrying an RC receiver protocol such as iBUS, SBUS, or CRSF. ESP-FC gives
`SERIAL_FUNCTION_RX_SERIAL` ownership of that stream, so MSP frames on the same
wire would be ignored or would disturb receiver framing. Use a dedicated MSP
serial input for this bridge. On hardware where the companion-to-FC UART is
already the RC link, keep current telemetry disabled until a separate MSP path
or a receiver-native telemetry transport is provided.

## Freshness contract

A received current sample is valid for 500 ms. If no new sample arrives within
that interval:

- the MSP-current validity flag is cleared;
- unfiltered and filtered reported current are reset to 0 A;
- stale current is not held indefinitely.

Consumption (`mAh`) is reported from the most recent accepted companion sample.

## Optional bidirectional polling

Builds with a genuinely bidirectional dedicated MSP link can define:

```
ESPFC_MSP_CURRENT_METER_POLLING
```

That enables FC-originated `MSP_ANALOG` requests at 10 Hz on non-USB MSP
ports. The normal project build leaves this undefined so unsolicited companion
replies are sufficient.

## Diagnostics

`DEBUG_CURRENT_SENSOR` exposes the current-meter values already used by the
existing battery diagnostic path. The companion bridge also feeds
`MSP_ANALOG`, `MSP_BATTERY_STATE`, and `MSP_CURRENT_METERS` so compatible
ground software can display the received current and consumption.
