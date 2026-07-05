# Stable 05

Firmware baseline: `v3.30`

Marked stable after repeated field test runs on 2026-07-03.

Validated status:
- More than three test runs completed successfully.
- Each test run lasted more than 10 minutes.
- MSP DisplayPort OSD remained consistent through the test runs.
- GPS lock behavior improved compared with the previous day, with a maximum observed satellite count of 11.
- Home point acquisition with the 4 second stability delay worked as designed.
- Manual home point update by gesture worked as designed.
- Home display uses home icon plus distance only; the directional arrow was intentionally removed because vehicle heading is not reliable without a compass/magnetometer.
- `ELRS OK` center text was removed because link quality already represents the healthy-link state.

Stable 05 changes since Stable 04:
- Firmware version label is `V3.30`.
- WiFi AP is `RZGXRover` with password `RZGXRover`.
- OSD label `THR` is now `GAS`.
- Home OSD is simplified to `[HOME ICON] distance`.
- Configurable GPS home point threshold:
  - `Home Min Satellites`, default 8, allowed 7-15.
  - `Home Stability ms`, default 4000 ms.
- Automatic home point recording waits until the minimum satellite count remains eligible for the configured stability time.
- Manual home point reset uses the configured minimum satellite threshold.
- Optional OSD visibility settings no longer include `ELRS Status`.
- Safety warnings remain mandatory and are not governed by optional OSD visibility toggles.

Known GPS notes:
- The current module is Rush FPV M10 mini.
- The previous module was Blitz M10 GPS.
- Field tests were performed in an open complex field of roughly 50 m x 50 m.
- GPS reached satellites faster than the previous day even on cold start, but still needed time to reach the configured home threshold.
- Maximum observed satellite count during the stable test set was 11.
- Future hardware tests may compare shielding, GPS placement, and power noise.

MSP stability rule:
- The minimal MSP reply surface introduced earlier remains the safe baseline.
- Avoid large MSP behavior changes unless isolated and tested separately.
- OSD loss on the older DJI O4 unit has been linked to that unit's inconsistent internal behavior; DJI O3 has shown more consistent MSP OSD behavior but requires stronger cooling.

Stable rule:
- Treat `v3.30` as the current safe baseline.
- Preserve this firmware backup before starting the next incremental patch.
