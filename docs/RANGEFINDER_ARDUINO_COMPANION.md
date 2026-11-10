# External Arduino + HC-SR04: unsupported companion transport (F48)

**Status:** unsupported on current `main` (2026-11-10). This is an explicit
capability classification, **not** a claim that companion communication works.

## What exists

- `RANGEFINDER_NONE` (default): no rangefinder; `state.rangefinder.present`
  remains false. Barometer-based altitude is *not* a ground clearance reading.
- `RANGEFINDER_HCSR04`: direct flight-controller GPIO trigger and echo.
  It calls the target GPIO HAL and **must not** be selected for an HC-SR04
  connected solely to a separate Arduino. The HC-SR04 echo can be 5 V; do
  not directly connect it to a 3.3 V ESP GPIO without proper level shifting.
- `RangefinderSensor::applyMeasurement`: sensor freshness logic for a
  backed measurement; it does **not** accept serial companion packets.

No Arduino transmit sketch, framing, receiver driver, CRC, parser, serial port
ownership, or monotonic-clock synchronization is implemented.

## Required hardware/firmware work before enabling companion support

1. Define a versioned binary frame contract with magic, version, length,
   sequence, measured distance, quality/validity, sample age, and CRC.
2. Implement paired Arduino sender and FC receiver, bounded parser, serial
   ownership and timeout/error reporting. Handle truncated/reordered frames.
3. Carry the **physical capture time**, not the UART arrival time, into the
   flight controller's monotonic clock domain; reject expired (>100 ms)
   measurements and replayed sequences.
4. Reject implausible distances, invalid quality, loss of serial connection
   and stale timestamps. Never infer range data from barometer estimates.
5. Prove correct behavior with bench hardware and recorded packet loss before
   enabling LAND ground-contact authority. Keep the existing no-range
   auto-disarm build gate closed until real-aircraft evidence exists.

For the user's external-Arduino configuration, select `RANGEFINDER_NONE`;
this avoids redundant HC-SR04 trigger/echo processing on the ESP32. It also
means range-aided LAND touchdown detection is **unavailable**. A later,
separately reviewed implementation must be tracked as a new feature rather
than silently claiming F48's companion functionality is complete.
