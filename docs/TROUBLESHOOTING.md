# Troubleshooting

Last updated: 2026-07-03

Baseline: Stable 05 / `v3.30`

## Upload Fails: COM Port Busy

Symptom:

```text
Could not open COM7, the port is busy or does not exist.
Access is denied.
```

Fix:

1. Close Arduino Serial Monitor.
2. Disconnect any local configurator/Web Serial page.
3. Close other serial terminals.
4. Retry upload.

Compile-only does not need the ESP32 connected. Upload does.

## OSD Does Not Appear

Check in this order:

1. DJI O3/O4 air unit is powered.
2. Goggles are powered and linked.
3. ESP32 common ground is connected to air unit ground.
4. Air unit RX is connected to ESP32 GPIO15 TX.
5. Air unit TX is connected to ESP32 GPIO16 RX.
6. Firmware label is `V3.30` or newer stable.
7. Power-cycle ESP32 and air unit.
8. Leave the system running for at least 30 seconds.

Known project finding:

- The older DJI O4 unit behaved inconsistently even on F405/F722 Betaflight flight controllers.
- DJI O3 showed more consistent OSD behavior, but needs better cooling.
- Do not blame ESP32 firmware first if the same O4 also fails to show Betaflight OSD.

## OSD Appears Then Disappears

Check:

- Air unit health.
- O4/O3 power stability.
- Common ground.
- Six-pin JST connector fit.
- UART wire quality.
- ESP32 brownout/reset.
- DisplayPort behavior was not changed from Stable 05.

Stable 05 rule:

- Compare against `v3.30`.
- Avoid changing MSP reply surface and DisplayPort cadence in the same patch.

## GPS Lock Is Slow

Check:

1. GPS TX goes to ESP32 GPIO18 RX.
2. GPS GND is common with ESP32.
3. GPS baud is `115200`.
4. GPS has open sky view.
5. GPS patch antenna faces upward.
6. GPS is not directly beside air unit, ELRS antenna, BEC inductor, fan motor, or ESC/power wiring.

Current GPS modules tested:

- Rush FPV M10 mini.
- Blitz M10 GPS.

Stable 05 observation:

- In a roughly 50 m x 50 m open complex field, GPS performance improved compared with the previous day.
- Maximum observed satellite count during the stable test set was 11.
- GPS locked from 0 to 4 satellites faster, but still needed time to reach the configured home threshold.

Good signs:

- GPS bytes increase.
- NMEA sentences increase.
- Parsed count increases.
- Satellites eventually appear.
- Fix becomes `1`.

## Home Point Does Not Record

Stable 05 requires:

- GPS fix valid.
- Satellite count >= `Home Min Satellites`.
- Eligibility remains valid for `Home Stability ms`.

Defaults:

- `Home Min Satellites = 8`.
- `Home Stability ms = 4000`.

If needed:

- Lower `Home Min Satellites`, but firmware clamps the minimum to `7`.
- Increase `Home Stability ms` if GPS position appears unstable.
- Move the GPS farther from RF/noise sources.
- Try again in a more open area.

## Manual Home Point Gesture Does Not Work

Check:

- CH5 is disarmed/off.
- GPS fix is valid.
- Satellite count >= configured `Home Min Satellites`.
- Gesture is held long enough.

Manual update uses the configured satellite threshold but does not need to wait for auto-home stability time after the gesture is accepted.

## ELRS Shows No Movement

Check:

1. ELRS receiver is powered.
2. RadioMaster Boxer is bound.
3. ELRS TX is connected to ESP32 GPIO4 RX.
4. Common ground is connected.
5. CRSF baud is `420000`.
6. CH1/CH2/CH5 values change in the WiFi configurator.

Current wiring:

- ELRS TX -> ESP32 GPIO4.
- ELRS RX -> ESP32 GPIO5 is optional for future telemetry back to radio.

## ELRS Failsafe Appears

Check:

- Radio battery.
- Receiver power.
- Antenna placement.
- Distance/obstructions.
- Whether the radio was left on for a long time before driving.

The OSD no longer shows `ELRS OK` during healthy operation. Use LQ as the normal link indicator. `ELRS FAILSAFE` remains mandatory as a safety warning.

## Steering Or ESC Does Not Move

Check:

1. Servo/ESC have 5V from ESC/BEC or external regulator.
2. Servo/ESC ground is common with ESP32.
3. Steering signal wire goes to GPIO13.
4. ESC signal wire goes to GPIO14.
5. ELRS channel values move in configurator.
6. CH5 arm is ON for ESC output.
7. Throttle is neutral before arming.
8. Wheels are lifted before testing.

Known solution:

- Manual servo pulse output is the working path for this car.

## Throttle Does Not Work But Steering Works

Likely safety gate:

- CH5 not armed.
- Throttle not neutral during arming.
- ESC neutral gate settings too narrow.
- `FUEL EMPTY` active; reboot is required to clear it.

Try:

1. Disarm.
2. Put throttle stick neutral.
3. Arm with CH5.
4. Move throttle slowly.
5. If needed, widen ESC neutral gate slightly.

## WiFi Configurator Is Open Unexpectedly

If a phone has saved `RZGXRover`, it may auto-connect to the ESP32 AP.

This can make `WIFI ON` appear even if the page was not intentionally opened.

## Config Changes Do Not Stick

Check:

1. Firmware reports `V3.30`.
2. Click `Save`.
3. Reopen `/api/config` or refresh the page.
4. Check that values persisted after reboot.
