# Hardware Wiring

Last updated: 2026-07-03

Baseline: Stable 05 / `v3.30`

## Working Hardware

- ESP32-S3 DevKit.
- DJI O3 or DJI O4 Air Unit.
- DJI Goggles 3.
- GPS module with NMEA output.
- ELRS receiver bound to RadioMaster Boxer.
- RC car/crawler ESC and steering servo.
- MPU6050 IMU.
- Battery voltage divider on GPIO1.
- External 5V rail/BEC for servo, ESC side, GPS/ELRS where appropriate.

## UART Wiring

### DJI O3/O4 MSP UART

Baud: `115200`

| ESP32-S3 | Air Unit |
| --- | --- |
| GPIO15 TX | O3/O4 RX |
| GPIO16 RX | O3/O4 TX |
| GND | GND |

Firmware constants:

- `MSP_TX_PIN = 15`
- `MSP_RX_PIN = 16`
- `MSP_BAUD = 115200`

### GPS UART

Baud: `115200`

| ESP32-S3 | GPS |
| --- | --- |
| GPIO18 RX | GPS TX |
| GPIO17 TX | GPS RX, optional |
| 5V or 3.3V rail | GPS VCC, according to module rating |
| GND | GPS GND |

Firmware constants:

- `GPS_RX_PIN = 18`
- `GPS_TX_PIN = 17`
- `GPS_BAUD = 115200`

Current GPS modules tested:

- Rush FPV M10 mini.
- Blitz M10 GPS.

### ELRS CRSF UART

Baud: `420000`

| ESP32-S3 | ELRS receiver |
| --- | --- |
| GPIO4 RX | ELRS TX |
| GPIO5 TX | ELRS RX, optional future telemetry/backchannel |
| 5V or 3.3V rail | ELRS VCC, according to receiver rating |
| GND | ELRS GND |

Firmware constants:

- `CRSF_RX_PIN = 4`
- `CRSF_TX_PIN = 5`
- `CRSF_BAUD = 420000`

Current control only needs ELRS TX into ESP32 GPIO4. GPIO5 is reserved for future telemetry back to the radio.

## PWM Output Wiring

### Steering Servo

| Servo wire | Connection |
| --- | --- |
| Signal, usually yellow/white/orange | ESP32 GPIO13 |
| Power, usually red | ESC/BEC 5V or external servo rail |
| Ground, usually black/brown | Common ground |

Firmware constant:

- `STEERING_PWM_PIN = 13`

### ESC Throttle

| ESC receiver plug wire | Connection |
| --- | --- |
| Signal, usually yellow/white/orange | ESP32 GPIO14 |
| BEC power, usually red | Servo 5V rail as needed |
| Ground, usually black/brown | Common ground |

Firmware constant:

- `ESC_PWM_PIN = 14`

## Battery Voltage Divider

GPIO1 senses main RC battery voltage through a resistor divider.

Current divider:

- Battery positive -> 100 kOhm -> GPIO1 sense node.
- GPIO1 sense node -> 47 kOhm -> GND.
- Battery negative -> common GND.

Firmware constants:

- `BATTERY_ADC_PIN = 1`
- `BATTERY_DIVIDER_TOP_KOHM = 100`
- `BATTERY_DIVIDER_BOTTOM_KOHM = 47`

Important:

- GPIO1 is a voltage sense input, not a power input.
- ESP32 power still comes from USB, 5V pin, or a proper 5V regulator.

## IMU Wiring

MPU6050 I2C:

| ESP32-S3 | MPU6050 |
| --- | --- |
| GPIO8 | SDA |
| GPIO9 | SCL |
| 3.3V or 5V | VCC, according to module rating |
| GND | GND |
| GND | AD0, if address 0x68 is desired |

Firmware constants:

- `IMU_SDA_PIN = 8`
- `IMU_SCL_PIN = 9`
- `IMU_I2C_ADDRESS = 0x68`

`INT`, `XDA`, and `XCL` are not required for the current firmware.

## Power And Ground

Critical rule:

- All grounds must be common.

Common ground group:

- ESP32 GND.
- DJI O3/O4 GND.
- GPS GND.
- ELRS GND.
- ESC/BEC GND.
- Servo GND.
- Battery voltage divider GND.
- IMU GND.

Power notes:

- Do not power servo or ESC from ESP32 3.3V.
- Servo/ESC power should come from ESC/BEC or a proper external regulator.
- GPS and ELRS should be powered according to their rated input voltage.
- DJI O3/O4 should use its own correct supply.
- A large electrolytic capacitor near the ESP32 5V/GND rail helped reduce reset behavior during servo/throttle movement in testing.

## GPS Placement Notes

GPS performance is sensitive to RF noise and sky view.

Practical placement:

- Keep GPS away from O3/O4 air unit, ELRS antenna, BEC inductors, fan motors, ESC wiring, and high-current power wiring.
- Keep the patch antenna facing the sky.
- Avoid covering the top of the patch antenna with foil/metal.
- If shielding is tested, shield the cable only and connect shield to ground on one side.

## Safe Bench Test Order

1. ESP32 connected to PC only.
2. Upload firmware.
3. Check WiFi configurator can open.
4. Connect GPS and confirm GPS bytes/sats change.
5. Connect O3/O4 and confirm OSD appears.
6. Connect ELRS and confirm CH values change in configurator.
7. Lift wheels or remove drive power.
8. Connect servo signal GPIO13.
9. Test steering.
10. Connect ESC signal GPIO14.
11. Test arm gate and throttle slowly.
