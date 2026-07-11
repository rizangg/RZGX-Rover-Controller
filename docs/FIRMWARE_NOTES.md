# Firmware Notes

Last updated: 2026-07-11

Firmware file:

`firmware/RZGX_Rover_Controller/RZGX_Rover_Controller.ino`

Current stable version on `main`: `v3.30`

Current stable mark: `Stable 05`

Current experimental branch build: `v3.32 EXP`

Suggested branch name:

`experimental/v3.32-pan-tilt-safety`

## Firmware Role

The ESP32-S3 acts as a surface vehicle controller plus MSP DisplayPort OSD source.

It is not a real Betaflight flight controller and it is not intended for autonomous rover navigation.

Main jobs:

- Reply to DJI O3/O4 MSP polling.
- Push MSP DisplayPort frames for custom OSD.
- Parse GPS NMEA.
- Parse ELRS CRSF.
- Generate manual 50 Hz steering and ESC PWM pulse output.
- Gate ESC output behind arming and safety checks.
- Read battery voltage from ADC through a divider.
- Read MPU6050 IMU roll/pitch.
- Host a lightweight WiFi configurator.
- Persist settings to ESP32 flash.

## Current Stable Validation

`v3.30` was marked Stable 05 after more than three field test runs on 2026-07-03.

Validated behavior:

- Each test run lasted more than 10 minutes.
- OSD remained consistent.
- GPS/home point behavior worked as intended.
- `Home Stability ms = 4000` delayed automatic home recording correctly.
- Manual home point update gesture worked.
- Home OSD now shows home icon plus distance only.
- ELRS healthy-state center text was removed; LQ remains the normal link indicator.

## Experimental v3.32 Notes

`v3.32 EXP` is an experimental build intended for a separate GitHub branch, not a replacement for the Stable 05 baseline yet.

Changes since Stable 05:

- Adds optional camera pan servo output on GPIO11 from CH4.
- Adds optional camera tilt servo output on GPIO12 from CH3.
- Adds WiFi configurator controls for pan/tilt enable, reverse, trim, scale, min, and max.
- Adds post-drive stats display after minimum drive time.
- Adds stricter WiFi safety gate behavior when a configurator client is connected.
- Adds ELRS failsafe recovery lock: after failsafe recovery, throttle must return neutral before ESC output is allowed again.
- Adds ESP32 task watchdog.

Field notes:

- Pan and tilt have been tested and move proportionally according to intended design.
- The latest experimental firmware has been smooth in several field runs, but is not yet marked stable.
- WiFi configurator CH5 visual state turns red when arming is active.
- Safety consent behavior around WiFi/arming still needs review before this branch can be promoted.
- ELRS telemetry lost/connected voice warnings were observed after an ELRS 4.0.1 update while control remained normal and OSD did not show failsafe.

## Important Serial Ports

| Purpose | ESP32 serial | Pins | Baud |
| --- | --- | --- | ---: |
| DJI MSP | `Serial1` | RX GPIO16, TX GPIO15 | 115200 |
| GPS NMEA | `Serial2` | RX GPIO18, TX GPIO17 | 115200 |
| ELRS CRSF | `Serial0` | RX GPIO4, TX GPIO5 | 420000 |
| USB debug/config | `Serial` | USB CDC | 115200 |

Arduino IDE requirement:

- USB CDC On Boot: Enabled.

## MSP Behavior

Observed DJI O3/O4 polling commands include:

- `3` MSP_FC_VERSION
- `10` MSP_NAME
- `92` MSP_FILTER_CONFIG
- `94` MSP_PID_ADVANCED
- `101` MSP_STATUS
- `105` MSP_RC
- `110` MSP_ANALOG
- `111` MSP_RC_TUNING
- `112` MSP_PID
- `130` MSP_BATTERY_STATE
- `150` MSP_STATUS_EX
- `182` MSP_DISPLAYPORT, after DisplayPort is active

The firmware replies to a deliberately small MSP surface. This minimal surface is part of the stable baseline and should not be expanded casually.

## OSD Rendering

The current OSD is drawn by MSP DisplayPort packets:

- Heartbeat.
- Clear screen.
- Write strings.
- Draw screen.

Current important OSD items:

- Craft name.
- Drive mode: `TRAIL` / `CRAWL`.
- LAT and LON.
- GPS satellites and speed.
- Battery average cell voltage.
- ELRS link quality.
- STR and GAS percentages.
- Home distance only, without directional arrow.
- WiFi status.
- IMU roll/pitch.
- Drive timer.
- Safety warnings: `ELRS FAILSAFE`, `RETURN NOW`, `FUEL EMPTY`, `START FIRST`, `STANDBY`, `DISARM FIRST`, `GAS TO CENTER`, `DRIVE`.

The home arrow was removed intentionally. GPS alone gives course over ground, not vehicle nose heading, and MPU6050 has no magnetometer. Showing only distance is safer than showing a direction that can become misleading after the vehicle turns around.

## GPS Home Point

Stable 05 adds configurable home acquisition:

- `Home Min Satellites`
  - Default: `8`
  - Minimum: `7`
  - Maximum: `15`
- `Home Stability ms`
  - Default: `4000`
  - Range: `0-30000`

Automatic home point recording waits until the satellite threshold is continuously eligible for the stability duration.

Manual home point reset uses the same satellite threshold and is triggered by the configured gesture while disarmed.

Manual home point update gesture:

- CH5 arming OFF.
- GPS fix valid.
- Satellite count at or above the configured threshold.
- GAS full forward held for 4 seconds.
- Successful update shows `HOME POINT UPDATED`.

Current GPS hardware notes:

- Current module: Rush FPV M10 mini.
- Previous module: Blitz M10 GPS.
- Best observed satellite count in the Stable 05 field tests: `11`.
- GPS performance improved from the previous day despite no wiring changes.

## IMU Calibration

The IMU can be zeroed to the current vehicle attitude.

Supported methods:

- WiFi configurator: `Set Current Position as Level`.
- Gesture: CH5 arming OFF, GAS full reverse held for 4 seconds.

Successful calibration shows `IMU CALIBRATED` on the OSD.

## Engine And Throttle Safety

Current logic:

- CH5 controls arming.
- Steering output is live when link/input are valid.
- ESC output is blocked until the engine is armed.
- ESC also requires a disarmed-since-boot and neutral safety flow.
- If throttle is moved while not armed, OSD shows `START FIRST`.
- On arming, OSD can briefly show `DRIVE`.
- `FUEL EMPTY` locks ESC output to neutral until reboot.
- ELRS failsafe forces safe behavior and shows `ELRS FAILSAFE`.

Current defaults:

- ESC min: `1000 us`
- ESC max: `2000 us`
- ESC neutral gate: `1450-1550 us`
- Steering min: `1000 us`
- Steering max: `2000 us`
- Steering reversed default: `true`
- Throttle reversed default: `false`
- Steering trim default: `0 us`
- Drive mode default: `TRAIL`
- `CRAWL` throttle cap: `50%`

## Manual PWM

The firmware uses manual pulse generation instead of LEDC PWM.

Pins:

- Steering: GPIO13.
- ESC: GPIO14.

Frame:

- 50 Hz style frame, `20000 us`.
- Pulse widths roughly `1000-2000 us`.

## WiFi Config Portal

Default AP settings:

- SSID: `RZGXRover`
- Password: `RZGXRover`
- Captive portal/IP: `http://10.10.4.1/`

WiFi starts after the configured delay and may remain active while a phone or browser is connected.

Main endpoints:

- `/`
- `/api/config`
- `/api/status`
- `/api/save`
- `/api/defaults`
- `/api/imu/calibrate`
- `/api/imu/reset`

## Config Storage

The firmware uses ESP32 `Preferences` with namespace:

- `surface`

Important stored settings include:

- Craft name.
- Steering/ESC direction, endpoints, neutral gate, trim.
- Reverse assist.
- Battery cell count and warning thresholds.
- GPS home minimum satellites and stability time.
- IMU rotation and zero position.
- Optional OSD visibility toggles.

## Firmware Edit Notes

When making changes:

- Keep MSP and DisplayPort behavior boring and stable.
- Do not reintroduce home direction arrows without a reliable heading source such as a magnetometer or another heading solution.
- Do not remove manual PWM unless hardware is retested.
- Avoid blocking delays in the main loop.
- Keep safety warnings mandatory even when optional OSD elements are hidden.
- If OSD disappears, first compare against Stable 05 `v3.30`.
