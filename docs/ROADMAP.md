# Roadmap

Last updated: 2026-07-03

This roadmap is intentionally practical. The project already works at the important proof-of-concept level.

## Stable 05 Notes

Current safe firmware baseline: `v3.30`.

Completed and validated in Stable 05:

- Home direction arrow was removed intentionally.
- Home OSD now shows home icon plus distance only.
- Configurable `Home Min Satellites` and `Home Stability ms` were added.
- Automatic home recording waits for the configured satellite threshold and stability time.
- Manual home reset gesture uses the configured satellite threshold.
- `ELRS OK` center text was removed; LQ remains the healthy-link indicator.

Rationale:

- Betaflight and INAV home arrows depend on a valid vehicle heading/yaw source.
- RZGX Rover Controller currently has GPS and MPU6050 roll/pitch, but no compass/magnetometer.
- GPS course over ground cannot reliably represent the vehicle nose direction after turns or reversing.
- Showing distance only is safer and more honest than showing a misleading arrow.

## Immediate Next Patch Candidates

### Post-Drive Statistics

Add a post-drive statistics page shown after disarm or after the drive session ends.

Initial rover-oriented stats:

- Drive time.
- Trip distance.
- Maximum speed.
- Average speed.
- Maximum distance from home.
- Minimum pack voltage.
- Minimum average cell voltage.
- Minimum ELRS LQ.
- Minimum GPS satellites.
- Maximum GPS satellites.
- Maximum roll/pitch angle or G-force from IMU, if useful after testing.
- Stop/disarm reason, such as manual disarm, ELRS failsafe, or fuel empty lock.

Design notes:

- Treat this as a lightweight readout, not a mandatory always-on OSD element.
- Prefer showing it only when safe, such as disarmed/standby state.
- Keep the first implementation simple: one compact page before considering multi-page stats.

### ELRS Telemetry Back To Radio

Enable the ESP32 TX path back to the ELRS receiver so the receiver can forward selected telemetry to compatible transmitters such as Radiomaster Boxer and potentially Radiomaster MT12.

Candidate telemetry:

- Main RC battery voltage.
- Average cell voltage.
- GPS fix/satellite count.
- ELRS/failsafe status.
- Armed/drive state.
- Drive mode: Trail/Crawl.

Implementation notes:

- Verify the receiver expects CRSF telemetry frames on its RX pin.
- Keep this lower priority than steering/throttle safety and MSP OSD stability.
- Start with one low-rate telemetry value, likely battery voltage, before adding more fields.

### RZGX Splash Screen / Logo

If a custom RZGX Rover logo is created, evaluate how much of it can be displayed on DJI O3/O4 OSD.

Constraints:

- DJI Goggles 2/newer style HD OSD is character-based and does not support arbitrary custom fonts in the normal MSP DisplayPort path.
- A true bitmap logo or custom glyph font is probably not practical on Goggles 3 through the current firmware path.
- A text-based splash screen is feasible: for example `RZGX ROVER`, `RZGX`, driver name, version, or a simple ASCII/block-style mark using supported characters.

Design direction:

- Use an original RZGX Rover design, not INAV or Betaflight assets.
- Prefer a small boot/standby splash that does not delay safety-critical OSD.
- Treat this as branding polish after OSD reliability and safety behavior remain stable.

### Configurable Battery Cell Count And Low-Voltage Warning

Status: implemented before Stable 05.

Current behavior:

- Battery cell count is configurable.
- `RETURN NOW` warning threshold is configurable per cell.
- `FUEL EMPTY` threshold is configurable per cell.
- `FUEL EMPTY` locks ESC output to neutral until reboot.
- Rendered OSD shows average cell voltage.
- DJI native UI receives total pack voltage.

Future polish:

- Add ADC calibration fields if measured voltage drift becomes significant.
- Add optional export/import config JSON.

### GPS Configuration Polish

Stable 05 already includes:

- Configurable `Home Min Satellites`, default 8, min 7, max 15.
- Configurable `Home Stability ms`, default 4000 ms.

Possible future GPS settings:

- GPS weak warning.
- GPS lost timeout.
- Ignore GPS jitter below a small distance.
- Optional home point acquisition debug indicator.

## Highest Priority

### 1. Safety/Failsafe Polish

Add and verify:

- ELRS signal loss detection.
- On signal loss: steering neutral and ESC output disabled or neutral.
- Clear OSD warning for RX loss.
- Configurable failsafe behavior in RZGX Rover Configurator.

### 2. Battery Voltage Calibration

Current OSD uses battery voltage, but the ADC scaling should be turned into a configurable value.

Add later if needed:

- ADC pin documentation.
- Divider ratio.
- Calibration offset.
- Configurator field for calibration.
- Low voltage warning in OSD.

### 3. Configuration Backup

Add to configurator:

- Export config JSON.
- Import config JSON.
- Restore defaults confirmation.

This will make moving PC/account easier.

## Next Nice Improvements

### OSD Layout Editor

Later:

- Drag OSD elements in preview.
- Save element positions.
- Toggle elements on/off.
- Store OSD layout in ESP32 flash.

Keep first version simple:

- Fixed position presets.
- Maybe `Compact`, `Trail`, `Bench Test`.

### Smaller ESP32 Board

The project may be moved to a smaller ESP32-S3 board later.

Must confirm:

- Enough UART pins.
- USB CDC works.
- 3.3V regulator capacity.
- Physical pin availability for GPIO4, 5, 13, 14, 15, 16, 17, 18 or equivalent remap.
- Common ground and clean power.

### Open Source Preparation

Before public release:

- Add license.
- Add clean README.
- Remove private paths from public docs.
- Avoid publishing private GPS coordinates.
- Add wiring diagram image.
- Add photos of safe wiring, if desired.
- Add release zip with firmware and configurator.
- Add "not for autonomous driving" and safety disclaimers.

## Deferred Features

Not needed now:

- Autonomous RTH.
- Rover navigation.
- Mission mode.
- Obstacle avoidance.
- Full Betaflight compatibility.
- Full INAV Rover feature parity.

Reason:

- Current goal is manual RC car control plus DJI O4 OSD telemetry.

## Guiding Principle

Keep the system boring and reliable:

1. OSD stable.
2. Steering stable.
3. Throttle safe.
4. Config clear.
5. Add features only after safety behavior remains predictable.
