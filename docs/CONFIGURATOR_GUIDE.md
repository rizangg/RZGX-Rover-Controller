# RZGX Rover Configurator Guide

Last updated: 2026-07-03

Main stable baseline: `v3.30` / Stable 05

This branch/package also documents experimental `v3.32 EXP` pan/tilt and safety-gate controls.

## WiFi Configurator

The firmware hosts the primary configurator directly from the ESP32.

Connect to:

- SSID: `RZGXRover`
- Password: `RZGXRover`
- URL: `http://10.10.4.1/`

The WiFi configurator is the preferred interface for normal field configuration.

## Basic Use

1. Power the vehicle/ESP32.
2. Wait for the WiFi portal to become available.
3. Connect phone/laptop to `RZGXRover`.
4. Open `http://10.10.4.1/`.
5. Confirm firmware version shows `V3.30` on stable, or `V3.32 EXP` on this experimental branch.
6. Adjust settings.
7. Save.

If the phone has saved the SSID/password, it may auto-connect to the ESP32 AP. In that case `WIFI ON` may appear in OSD even if the configurator was not opened manually.

## Main Settings

### Craft Name

Default:

`RZGXRIDE`

Rules:

- Max 16 characters.
- Allowed: `A-Z`, `0-9`, space, `_`, `-`.
- Lowercase becomes uppercase.
- Empty name falls back to default.

### Steering

Configurable:

- Direction normal/reversed.
- Trim.
- Min/max endpoint.

Current default steering direction is reversed because the tested MN128 setup needed it.

### ESC / GAS

Configurable:

- Throttle direction.
- ESC min/max.
- Neutral low/high.
- Reverse assist.
- Reverse delay.

Safety behavior:

- CH5 must arm before ESC output is active.
- Throttle must be neutral for safe arming.
- `FUEL EMPTY` locks ESC to neutral until reboot.

### Battery Warning

Configurable:

- Cell count.
- Low Warning V/cell: displays `RETURN NOW`.
- Fuel Empty V/cell: displays `FUEL EMPTY` and locks ESC neutral until reboot.

The rendered OSD battery value shows average cell voltage. DJI native UI receives total pack voltage.

### GPS Home Point

Stable 05 settings:

- `Home Min Satellites`
  - Default: `8`
  - Minimum: `7`
  - Maximum: `15`
- `Home Stability ms`
  - Default: `4000`
  - Range: `0-30000`

Automatic home point acquisition waits until the configured satellite threshold remains valid for the configured stability time.

Manual home point update uses the same minimum satellite threshold.

Manual home point update gesture:

- CH5 arming OFF.
- GPS fix valid.
- Satellite count meets the configured minimum.
- Hold GAS forward at full input for 4 seconds.
- The OSD shows `HOME POINT UPDATED` after the new home point is accepted.

### IMU

Configurable:

- IMU rotation.
- Set current position as level.
- Reset IMU calibration.

The OSD can show roll and pitch.

IMU zero/level gesture:

- CH5 arming OFF.
- IMU data valid.
- Hold GAS reverse at full input for 4 seconds.
- The OSD shows `IMU CALIBRATED` after the current vehicle attitude is saved as level.

The same level calibration can also be triggered from the WiFi configurator with `Set Current Position as Level`.

### Experimental Camera Pan/Tilt

Experimental `v3.32 EXP` settings:

- Camera pan output enable/disable.
- Camera pan reverse.
- Camera pan scale, trim, min, and max.
- Camera tilt output enable/disable.
- Camera tilt reverse.
- Camera tilt scale, trim, min, and max.

Experimental safety note:

- If a WiFi configurator client is connected while arming is active, vehicle drive output may be blocked and the OSD may show `WIFI STILL CONNECTED`.
- The CH5 arming indicator may turn red to show that arming is active during a WiFi session.
- This behavior is intentionally conservative, but the UI still needs more review before the branch is promoted to stable.

## OSD Visibility

Optional OSD items can be toggled:

- Craft.
- Drive Mode.
- Coordinates.
- GPS.
- Battery.
- Link Quality.
- STR/GAS.
- Home.
- WiFi.
- IMU.
- Drive Time.

`ELRS OK` was removed from the optional OSD list because link quality already indicates the healthy-link state.

Safety warnings are mandatory and stay enabled even when optional OSD items are hidden.

## System Monitor

The WiFi page shows compact system status:

- CPU estimate.
- RAM usage.
- Uptime.
- Battery voltage.
- Channel values.
- O4 MSP status.
- IMU/failsafe diagnostics.

These are field diagnostics, not a full profiler.

## Local Configurator

The older local configurator is still present:

`outputs/surface-osd-configurator-preview/index.html`

It may lag behind the WiFi configurator. Prefer the ESP32 WiFi configurator unless local configurator work is explicitly requested.

## Upload / COM Port Notes

If upload fails because COM is busy:

1. Close Arduino Serial Monitor.
2. Disconnect any local configurator session.
3. Retry upload.

Compiling does not require the ESP32 to be connected. Uploading does.
