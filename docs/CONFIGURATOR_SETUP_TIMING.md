# Setup, calibration and 4 kHz timing acceptance (ESP32)

Source baseline: commit ea4d51c, Betaflight App 2026.6.1 and MSP API 1.46.
These are separate issues: the firmware sends control/attitude packets, while the
Configurator itself owns its 3D renderer and the calibration instruction panels.

## Calibration and model

* In Betaflight App 2026.6.1, calibration instructions/buttons are in **Sensors**,
  not Setup. Use Accelerometer > Calibrate on a level, stationary FC. For the
  QMC5883P, open Sensors > Magnetometer > calibration workflow.
* Firmware implements MSP_ACC_CALIBRATION (gyro/accelerometer),
  MSP_MAG_CALIBRATION, MSP_MIXER_CONFIG (QUADX is 3), MSP_RAW_IMU and
  MSP_ATTITUDE. The pinned client packet test checks the quad model selector
  and synthetic attitude. It does **not** prove rendering.
* If the model area is blank, inspect the App JavaScript console and Network
  requests for `resources/models/quad_x.gltf` / WebGL errors. Verify the
  mixer is Quad X and that Setup roll/pitch/yaw numerics change as the board is
  rotated. A failed GLTF fetch or unavailable WebGL is a client-side problem;
  changing firmware PID/motor configuration will not repair it.
* Confirm actual calibration command acceptance, progress and persistence on
  hardware before relying on Angle, AltHold or Rescue. Keep props removed.

## Timing diagnosis

* On SPI MPU6500, gyro/PID is requested at 4 kHz (250 us/period). A QMC5883P
  I2C read observed at 304 us is long enough to overrun an individual period
  when it executes in the same task. A 100 Hz magnetometer plus a 50 Hz
  barometer can alias with the previous 10 ms stats window.
* `stats` now includes a display-only smoothed CPU percentage, last completed
  1-second maximum control period, last completed 1-second maximum task
  duration, and cumulative FreeRTOS timer-notification deadline misses.
* The Configurator displays the smoothed value; `ResourceManager` continues
  consuming unsmoothed CPU load, preserving its existing overload response.
* The worst-case counters expose the problem; smoothing is **not** a scheduling
  fix. The 4 kHz real-time I2C scheduling problem remains until an offloaded
  or bounded bus design is validated on real MPU6500/SPI + BMP280/QMC5883P
  hardware, including bus ownership and sensor freshness. Do not extrapolate
  to MPU6050/I2C, where the gyro shares the I2C bus.

## Acceptance measurements (motor power disconnected)

1. After cold reboot and 30 seconds idle, record `status`, then `stats`
   repeatedly for at least 10 seconds. Record cumulative missed deadlines at
   start/end; the *difference*, not its total, matters.
2. Open Setup (3D preview) and Sensors (calibration) separately. Repeat with
   live MSP telemetry and graph polling. Check 3D rendering in the client.
3. Repeat with only MPU6500/SPI, then with BMP280, then with QMC5883P added.
   Record control period maxima and task maxima for each configuration.
4. Repeat the diagnostic process for MPU6050/I2C separately (nominal rate is
   limited to 1 kHz). Do not force 4 kHz on the software I2C gyro.
5. No props-on test or flight authorization is implied by passing CI or by a
   lower displayed CPU percentage. Verify RC/failsafe, attitude direction,
   motors (without propellers), and LAND/Rescue bench states independently.
