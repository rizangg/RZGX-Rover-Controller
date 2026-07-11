/*
  DJI O4 + Goggles 3 MSP Emulator v3.32-exp-safety-gate
  Board: ESP32-S3 DevKit

  Wiring:
    ESP32 GPIO15 TX -> DJI O4 RX
    ESP32 GPIO16 RX <- DJI O4 TX
    ESP32 GPIO17 TX -> GPS RX, optional
    ESP32 GPIO18 RX <- GPS TX
    ESP32 GPIO4  RX <- ELRS RX TX, CRSF read-only
    ESP32 GPIO5  TX -> ELRS RX RX, optional telemetry later
    ESP32 GPIO13    -> Steering servo PWM from CH1
    ESP32 GPIO14    -> ESC signal PWM, gated by arm switch
    ESP32 GPIO11    -> Camera pan servo PWM from CH4, optional
    ESP32 GPIO12    -> Camera tilt servo PWM from CH3, optional
    ESP32 GPIO1     -> Battery voltage divider ADC sense
    ESP32 GPIO8     <-> MPU6050 SDA
    ESP32 GPIO9     -> MPU6050 SCL
    GND             <-> GND

  Arduino IDE:
    USB CDC On Boot: Enabled
    Serial Monitor: 115200 baud

  Stable 01 notes:
    GPS and ELRS are powered from external 5V header rails.
    USB power does not energize GPS or ELRS.
    Battery voltage divider is installed on GPIO01 ADC input.

  This version replies to DJI MSP polling and reads GPS NMEA on Serial2.
  This version keeps DisplayPort push enabled and uses the same update
  pattern as the reference sketch: heartbeat, clear, strings, draw.
*/

#include <Arduino.h>
#if __has_include(<esp_arduino_version.h>)
#include <esp_arduino_version.h>
#endif
#include <Preferences.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Wire.h>
#include <math.h>
#include <esp_task_wdt.h>

HardwareSerial DJISerial(1);
HardwareSerial GPSSerial(2);
HardwareSerial CRSFSerial(0);
Preferences configStore;
WebServer wifiServer(80);
DNSServer dnsServer;

static const char FIRMWARE_VERSION[] = "V3.32 EXP";

static const uint8_t MSP_RX_PIN = 16;
static const uint8_t MSP_TX_PIN = 15;
static const uint32_t MSP_BAUD = 115200;

static const uint8_t GPS_RX_PIN = 18;
static const uint8_t GPS_TX_PIN = 17;
static const uint32_t GPS_BAUD = 115200;

static const uint8_t CRSF_RX_PIN = 4;
static const uint8_t CRSF_TX_PIN = 5;
static const uint32_t CRSF_BAUD = 420000;

static const bool WIFI_CONFIG_ENABLED = true;
static const char WIFI_AP_SSID[] = "RZGXRover";
// WPA SoftAP passwords must be 8+ chars.
static const char WIFI_AP_PASSWORD[] = "RZGXRover";
static const uint8_t DNS_PORT = 53;
static const IPAddress WIFI_AP_IP(10, 10, 4, 1);
static const IPAddress WIFI_AP_GATEWAY(10, 10, 4, 1);
static const IPAddress WIFI_AP_SUBNET(255, 255, 255, 0);

static const size_t DJI_RX_BUFFER_SIZE = 512;
static const size_t DJI_TX_BUFFER_SIZE = 1024;
static const size_t GPS_RX_BUFFER_SIZE = 512;
static const size_t CRSF_RX_BUFFER_SIZE = 512;

static const uint8_t ESC_PWM_PIN = 14;
static const uint8_t STEERING_PWM_PIN = 13;
static const uint8_t HEAD_PAN_PWM_PIN = 11;
static const uint8_t HEAD_TILT_PWM_PIN = 12;
static const uint16_t ESC_PWM_FREQ_HZ = 50;
static const uint8_t ESC_PWM_RESOLUTION_BITS = 16;
static const uint16_t ESC_PWM_MIN_US = 1000;
static const uint16_t ESC_PWM_NEUTRAL_US = 1500;
static const uint16_t ESC_PWM_MAX_US = 2000;
static const uint16_t ESC_THROTTLE_NEUTRAL_LOW_US = 1450;
static const uint16_t ESC_THROTTLE_NEUTRAL_HIGH_US = 1550;
static const uint16_t ENGINE_NOT_READY_THROTTLE_DELTA_US = 80;
static const uint16_t STEERING_PWM_MIN_US = 1000;
static const uint16_t STEERING_PWM_MAX_US = 2000;
static const uint32_t SERVO_PWM_FRAME_US = 20000;
static const bool DEFAULT_STEERING_OUTPUT_REVERSED = true;
static const bool DEFAULT_THROTTLE_OUTPUT_REVERSED = false;
static const bool DEFAULT_DJI_ARMED_TO_O4 = false;
static const int16_t DEFAULT_STEERING_TRIM_US = 0;
static const uint16_t DEFAULT_STEERING_MIN_US = 1000;
static const uint16_t DEFAULT_STEERING_MAX_US = 2000;
static const uint16_t DEFAULT_ESC_MIN_US = 1000;
static const uint16_t DEFAULT_ESC_MAX_US = 2000;
static const uint16_t DEFAULT_ESC_NEUTRAL_LOW_US = 1450;
static const uint16_t DEFAULT_ESC_NEUTRAL_HIGH_US = 1550;
static const bool DEFAULT_ESC_REVERSE_ASSIST = false;
static const uint16_t DEFAULT_ESC_REVERSE_DELAY_MS = 250;
static const bool DEFAULT_HEAD_PAN_ENABLED = false;
static const bool DEFAULT_HEAD_TILT_ENABLED = false;
static const bool DEFAULT_HEAD_PAN_REVERSED = false;
static const bool DEFAULT_HEAD_TILT_REVERSED = false;
static const int16_t DEFAULT_HEAD_PAN_TRIM_US = 0;
static const int16_t DEFAULT_HEAD_TILT_TRIM_US = 0;
static const uint8_t DEFAULT_HEAD_PAN_SCALE_PERCENT = 60;
static const uint8_t DEFAULT_HEAD_TILT_SCALE_PERCENT = 100;
static const uint16_t DEFAULT_HEAD_SERVO_MIN_US = 1000;
static const uint16_t DEFAULT_HEAD_SERVO_MAX_US = 2000;
static const uint16_t HEAD_SERVO_CENTER_US = 1500;
static const uint16_t DRIVE_MODE_CHANNEL_INDEX = 5; // CH6
static const uint16_t DRIVE_MODE_THRESHOLD_US = 1500;
static const uint8_t DRIVE_MODE_CRAWL_THROTTLE_PERCENT = 50;
static const uint8_t DRIVE_MODE_TRAIL_THROTTLE_PERCENT = 100;
static const uint8_t BATTERY_ADC_PIN = 1;
static const uint16_t BATTERY_DIVIDER_TOP_KOHM = 100;
static const uint16_t BATTERY_DIVIDER_BOTTOM_KOHM = 47;
static const uint8_t BATTERY_ADC_SAMPLES = 8;
static const uint16_t BATTERY_ADC_MIN_VALID_MV = 250;
static const uint16_t BATTERY_SAMPLE_INTERVAL_MS = 200;
static const uint8_t DEFAULT_BATTERY_CELL_COUNT = 2;
static const uint16_t DEFAULT_BATTERY_WARN_CELL_CV = 310;
static const uint16_t DEFAULT_BATTERY_FUEL_EMPTY_CELL_CV = 300;
static const uint16_t BATTERY_WARN_TRIGGER_MS = 3000;
static const uint16_t BATTERY_WARN_CLEAR_MS = 2000;
static const uint8_t BATTERY_WARN_HYSTERESIS_CV = 10;
static const uint16_t BATTERY_FUEL_EMPTY_TRIGGER_MS = 1200;
static const uint8_t DEFAULT_HOME_POINT_MIN_SATS = 8;
static const uint8_t HOME_POINT_MIN_SATS_MIN = 7;
static const uint8_t HOME_POINT_MIN_SATS_MAX = 15;
static const uint16_t DEFAULT_HOME_STABILITY_MS = 4000;
static const uint16_t HOME_STABILITY_MS_MIN = 0;
static const uint16_t HOME_STABILITY_MS_MAX = 30000;
static const uint16_t HOME_POINT_GESTURE_HOLD_MS = 4000;
static const uint16_t HOME_POINT_NOTICE_MS = 2200;
static const bool DEFAULT_OSD_SHOW_CRAFT = true;
static const bool DEFAULT_OSD_SHOW_DRIVE_MODE = true;
static const bool DEFAULT_OSD_SHOW_COORDS = true;
static const bool DEFAULT_OSD_SHOW_GPS = true;
static const bool DEFAULT_OSD_SHOW_BATTERY = true;
static const bool DEFAULT_OSD_SHOW_LINK_QUALITY = true;
static const bool DEFAULT_OSD_SHOW_CONTROLS = true;
static const bool DEFAULT_OSD_SHOW_HOME = true;
static const bool DEFAULT_OSD_SHOW_WIFI = true;
static const bool DEFAULT_OSD_SHOW_IMU = true;
static const bool DEFAULT_OSD_SHOW_DRIVE_TIME = true;

static const uint8_t IMU_SDA_PIN = 8;
static const uint8_t IMU_SCL_PIN = 9;
static const uint8_t IMU_I2C_ADDRESS = 0x68;
static const uint32_t IMU_I2C_FREQUENCY_HZ = 400000;
static const uint16_t IMU_I2C_TIMEOUT_MS = 5;
static const uint16_t IMU_SAMPLE_INTERVAL_MS = 20;
static const uint16_t IMU_RETRY_INTERVAL_MS = 1000;
static const uint8_t IMU_MAX_CONSECUTIVE_ERRORS = 3;
static const float IMU_COMPLEMENTARY_ALPHA = 0.98f;
static const uint8_t DEFAULT_IMU_ROTATION = 0;
static const uint8_t IMU_GESTURE_EXTREME_PERCENT = 90;
static const uint16_t IMU_GESTURE_REVERSE_HOLD_MS = 4000;
static const uint16_t IMU_CALIBRATION_NOTICE_MS = 2200;
static const uint32_t DRIVE_STATS_MIN_DRIVE_MS = 180000;
static const uint16_t DRIVE_STATS_SAMPLE_INTERVAL_MS = 500;
static const uint16_t DRIVE_STATS_DISPLAY_MS = 45000;
static const uint16_t DRIVE_STATS_TRIP_MIN_STEP_CM = 20;
static const uint16_t DRIVE_STATS_TRIP_MAX_STEP_CM = 800;

static bool steeringOutputReversed = DEFAULT_STEERING_OUTPUT_REVERSED;
static bool throttleOutputReversed = DEFAULT_THROTTLE_OUTPUT_REVERSED;
static bool djiArmedToO4 = DEFAULT_DJI_ARMED_TO_O4;
static int16_t steeringTrimUs = DEFAULT_STEERING_TRIM_US;
static uint16_t steeringMinUs = DEFAULT_STEERING_MIN_US;
static uint16_t steeringMaxUs = DEFAULT_STEERING_MAX_US;
static uint16_t escMinUs = DEFAULT_ESC_MIN_US;
static uint16_t escMaxUs = DEFAULT_ESC_MAX_US;
static uint16_t escNeutralLowUs = DEFAULT_ESC_NEUTRAL_LOW_US;
static uint16_t escNeutralHighUs = DEFAULT_ESC_NEUTRAL_HIGH_US;
static bool escReverseAssistEnabled = DEFAULT_ESC_REVERSE_ASSIST;
static uint16_t escReverseDelayMs = DEFAULT_ESC_REVERSE_DELAY_MS;
static bool headPanEnabled = DEFAULT_HEAD_PAN_ENABLED;
static bool headTiltEnabled = DEFAULT_HEAD_TILT_ENABLED;
static bool headPanReversed = DEFAULT_HEAD_PAN_REVERSED;
static bool headTiltReversed = DEFAULT_HEAD_TILT_REVERSED;
static int16_t headPanTrimUs = DEFAULT_HEAD_PAN_TRIM_US;
static int16_t headTiltTrimUs = DEFAULT_HEAD_TILT_TRIM_US;
static uint8_t headPanScalePercent = DEFAULT_HEAD_PAN_SCALE_PERCENT;
static uint8_t headTiltScalePercent = DEFAULT_HEAD_TILT_SCALE_PERCENT;
static uint16_t headPanMinUs = DEFAULT_HEAD_SERVO_MIN_US;
static uint16_t headPanMaxUs = DEFAULT_HEAD_SERVO_MAX_US;
static uint16_t headTiltMinUs = DEFAULT_HEAD_SERVO_MIN_US;
static uint16_t headTiltMaxUs = DEFAULT_HEAD_SERVO_MAX_US;
static uint8_t imuRotation = DEFAULT_IMU_ROTATION;
static float imuRollZeroDeg = 0.0f;
static float imuPitchZeroDeg = 0.0f;
static uint8_t batteryCellCount = DEFAULT_BATTERY_CELL_COUNT;
static uint16_t batteryWarnCellCv = DEFAULT_BATTERY_WARN_CELL_CV;
static uint16_t batteryFuelEmptyCellCv = DEFAULT_BATTERY_FUEL_EMPTY_CELL_CV;
static uint8_t homeMinSats = DEFAULT_HOME_POINT_MIN_SATS;
static uint16_t homeStabilityMs = DEFAULT_HOME_STABILITY_MS;
static bool osdShowCraft = DEFAULT_OSD_SHOW_CRAFT;
static bool osdShowDriveMode = DEFAULT_OSD_SHOW_DRIVE_MODE;
static bool osdShowCoords = DEFAULT_OSD_SHOW_COORDS;
static bool osdShowGps = DEFAULT_OSD_SHOW_GPS;
static bool osdShowBattery = DEFAULT_OSD_SHOW_BATTERY;
static bool osdShowLinkQuality = DEFAULT_OSD_SHOW_LINK_QUALITY;
static bool osdShowControls = DEFAULT_OSD_SHOW_CONTROLS;
static bool osdShowHome = DEFAULT_OSD_SHOW_HOME;
static bool osdShowWifi = DEFAULT_OSD_SHOW_WIFI;
static bool osdShowImu = DEFAULT_OSD_SHOW_IMU;
static bool osdShowDriveTime = DEFAULT_OSD_SHOW_DRIVE_TIME;

#if !defined(ESP_ARDUINO_VERSION_MAJOR) || ESP_ARDUINO_VERSION_MAJOR < 3
static const uint8_t ESC_PWM_CHANNEL = 0;
static const uint8_t STEERING_PWM_CHANNEL = 1;
#endif

static const char DEFAULT_CRAFT_NAME[] = "RZGXRIDE";
static const uint8_t CRAFT_NAME_MAX_LEN = 16;
static char craftName[CRAFT_NAME_MAX_LEN + 1] = "RZGXRIDE";

static const uint16_t MSP_API_VERSION = 1;
static const uint16_t MSP_FC_VARIANT = 2;
static const uint16_t MSP_FC_VERSION = 3;
static const uint16_t MSP_BOARD_INFO = 4;
static const uint16_t MSP_BUILD_INFO = 5;
static const uint16_t MSP_NAME = 10;
static const uint16_t MSP_OSD_CONFIG = 84;
static const uint16_t MSP_FILTER_CONFIG = 92;
static const uint16_t MSP_PID_ADVANCED = 94;
static const uint16_t MSP_STATUS = 101;
static const uint16_t MSP_RC = 105;
static const uint16_t MSP_RAW_GPS = 106;
static const uint16_t MSP_COMP_GPS = 107;
static const uint16_t MSP_ATTITUDE = 108;
static const uint16_t MSP_ANALOG = 110;
static const uint16_t MSP_RC_TUNING = 111;
static const uint16_t MSP_PID = 112;
static const uint16_t MSP_BATTERY_STATE = 130;
static const uint16_t MSP_STATUS_EX = 150;
static const uint16_t MSP_DISPLAYPORT = 182;
static const uint16_t MSP_VTX_CONFIG = 88;
static const uint16_t MSP_SET_OSD_CANVAS = 188;
static const uint16_t MSP_OSD_CANVAS = 189;
static const uint16_t MSP_FEATURE_CONFIG = 36;
static const uint16_t MSP_RX_CONFIG = 44;
static const uint16_t MSP_MODE_RANGES = 34;
static const uint16_t MSP_BOXIDS = 119;

static const uint16_t MSP_SENSOR_ACC = 1 << 0;
static const uint16_t MSP_SENSOR_BARO = 1 << 1;
static const uint16_t MSP_SENSOR_MAG = 1 << 2;
static const uint16_t MSP_SENSOR_GPS = 1 << 3;
static const uint16_t MSP_SENSOR_GYRO = 1 << 5;
static const uint16_t MSP_SENSOR_MASK_FAKE_FC = MSP_SENSOR_ACC | MSP_SENSOR_BARO | MSP_SENSOR_MAG | MSP_SENSOR_GPS | MSP_SENSOR_GYRO;
static const uint16_t MSP_SENSOR_MASK_DJI_MINIMAL = MSP_SENSOR_ACC | MSP_SENSOR_BARO | MSP_SENSOR_MAG | MSP_SENSOR_GYRO;

static const uint8_t CRSF_ADDRESS_FLIGHT_CONTROLLER = 0xC8;
static const uint8_t CRSF_FRAMETYPE_LINK_STATISTICS = 0x14;
static const uint8_t CRSF_FRAMETYPE_RC_CHANNELS_PACKED = 0x16;
static const uint8_t CRSF_MAX_FRAME_SIZE = 64;
static const uint16_t CRSF_CHANNEL_VALUE_MIN = 172;
static const uint16_t CRSF_CHANNEL_VALUE_MID = 992;
static const uint16_t CRSF_CHANNEL_VALUE_MAX = 1811;

static const uint8_t MSP_DP_HEARTBEAT = 0;
static const uint8_t MSP_DP_RELEASE = 1;
static const uint8_t MSP_DP_CLEAR_SCREEN = 2;
static const uint8_t MSP_DP_WRITE_STRING = 3;
static const uint8_t MSP_DP_DRAW_SCREEN = 4;

static const uint8_t SYM_HOMEFLAG = 0x11;
static const uint8_t SYM_LAT = 0x89;
static const uint8_t SYM_LON = 0x98;
static const uint8_t SYM_LINK_QUALITY = 0x7B;
static const uint8_t SYM_M = 0x0C;
static const uint8_t SYM_SAT_L = 0x1E;
static const uint8_t SYM_SAT_R = 0x1F;
static const uint8_t SYM_ARROW_SOUTH = 0x60;
static const uint8_t SYM_ARROW_EAST = 0x64;
static const uint8_t SYM_ARROW_NORTH = 0x68;
static const uint8_t SYM_ARROW_WEST = 0x6C;
static const uint8_t SYM_BATT_FULL = 0x90;
static const uint8_t SYM_VOLT = 0x06;
static const uint8_t SYM_SPEED = 0x70;
static const uint8_t SYM_KPH = 0x9E;

static const bool DISPLAYPORT_TEST_ENABLED = true;
static const bool DISPLAYPORT_SEND_BOTH_DIRECTIONS = false;
static const bool VTX_MSP_PROBE_ENABLED = false;
static const bool VERBOSE_EVERY_REQUEST = false;
static const bool FAKE_ARMED_MODE = false;
static const bool DJI_NATIVE_CLASSIC_STATUS = true;
static const bool DJI_NATIVE_GPS_HOME_ENABLED = false;
static const bool USB_CONFIG_MODE = true;
static const bool USB_DIAGNOSTIC_MODE = false;
static const bool USB_DEBUG_DEFAULT = false;
static const uint16_t DISPLAYPORT_INTERVAL_MS = 200;
static const uint32_t WIFI_START_DELAY_MS = 60000;
static const uint32_t WIFI_IDLE_AUTO_OFF_MS = 60000;
static const uint16_t WIFI_SERVICE_INTERVAL_MS = 10;
static const uint8_t WATCHDOG_TIMEOUT_SECONDS = 5;
static const uint32_t PERF_WINDOW_US = 1000000;
static const uint32_t O4_MSP_LINK_TIMEOUT_MS = 2000;
static bool sendMspReplies = true;
static bool txBeaconEnabled = false;
static bool usbDebugEnabled = USB_DEBUG_DEFAULT;
static const uint8_t ARMING_DISABLE_FLAGS_COUNT_FAKE = 25;
static const uint8_t VIDEO_SYSTEM_HD_FAKE = 3;
static const uint8_t OSD_HD_COLS_FAKE = 53;
static const uint8_t OSD_HD_ROWS_FAKE = 20;
static const uint8_t OSD_ITEM_COUNT_FAKE = 80;
static const uint8_t OSD_STAT_COUNT_FAKE = 31;
static const uint8_t OSD_TIMER_COUNT_FAKE = 2;
static const uint8_t OSD_WARNING_COUNT_FAKE = 19;
static const uint16_t OSD_ITEM_DISABLED = 0;

static const char WIFI_CONFIG_HTML[] PROGMEM = R"rawliteral(
<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>RZGX Rover Configurator</title>
  <style>
    :root{color-scheme:dark;--bg:#090d0d;--panel:#111818;--line:#2a3838;--text:#e9f3ef;--muted:#9db0aa;--accent:#66e3bc}
    *{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--text);font:13px system-ui,Segoe UI,sans-serif}
    main{max-width:720px;margin:0 auto;padding:14px}.top{display:flex;justify-content:space-between;gap:10px;align-items:center;margin-bottom:12px}
    h1{font-size:20px;margin:0}.sub{color:var(--muted);font-size:12px}.pill{border:1px solid var(--line);border-radius:999px;padding:5px 9px;color:var(--accent);font-weight:700}
    .panel{background:var(--panel);border:1px solid var(--line);border-radius:8px;margin:10px 0;overflow:hidden}.head{padding:10px 12px;border-bottom:1px solid var(--line);font-weight:800}
    .body{padding:12px;display:grid;gap:12px}.grid{display:grid;grid-template-columns:1fr 1fr;gap:10px}.full{grid-column:1/-1}
    label{display:grid;gap:5px;color:var(--muted);font-size:12px}input,select{width:100%;min-height:34px;border:1px solid var(--line);border-radius:6px;background:#0c1111;color:var(--text);padding:7px}
    button{min-height:36px;border:0;border-radius:6px;background:var(--accent);color:#04110d;font-weight:850;padding:8px 12px}button.secondary{background:#1b2525;color:var(--text);border:1px solid var(--line)}
    .section-title{padding-top:8px;border-top:1px solid var(--line);color:var(--text);font-weight:850}.live{color:var(--accent);font-weight:750}
    .trim-tools{grid-column:1/-1;display:grid;grid-template-columns:repeat(5,minmax(0,1fr));gap:6px}.trim-tools button{min-height:32px;font-size:12px;padding:6px}
    .head{display:flex;justify-content:space-between;gap:10px;align-items:center}.link{font-size:11px;border-radius:999px;border:1px solid var(--line);padding:3px 7px;color:var(--muted)}.link.ok{color:var(--accent)}.link.lost{color:#ff9b8c}
    .system-body{padding:0;display:block}.system-stats{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));border-bottom:1px solid var(--line)}
    .stat{min-height:48px;padding:8px;display:grid;place-items:center;gap:3px;text-align:center;border-right:1px solid var(--line);font-size:11px;color:var(--muted)}.stat:last-child{border-right:0}.stat strong{color:var(--text);font-size:12px}
    .channel-grid{display:grid;grid-template-columns:minmax(0,1fr) 64px minmax(0,1fr) 64px}
    .cell{min-height:48px;padding:8px;display:flex;align-items:center;border-right:1px solid var(--line);border-bottom:1px solid var(--line);font-size:11px}.cell:nth-child(4n){border-right:0}.cell:nth-last-child(-n+4){border-bottom:0}
    .label{color:var(--text);font-weight:800}.value{justify-content:flex-end;color:var(--accent);font-weight:850}
    .checks{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:8px}.check{display:flex;gap:8px;align-items:center;border:1px solid var(--line);border-radius:6px;background:#0c1111;padding:8px;color:var(--text);font-size:12px}.check input{width:auto;min-height:auto;accent-color:var(--accent)}
  </style>
</head>
<body>
<main>
  <div class="top">
    <div><h1>RZGX Rover Configurator</h1><div class="sub">ESP32 WiFi portal</div></div>
    <div class="pill" id="fw">v--</div>
  </div>
  <div class="panel"><div class="head">System <span class="link" id="o4msp">O4 MSP --</span></div><div class="system-body">
    <div class="system-stats">
      <div class="stat"><span>CPU</span><strong id="cpu">--</strong></div>
      <div class="stat"><span>RAM</span><strong id="ram">--</strong></div>
      <div class="stat"><span>UP</span><strong id="up">--</strong></div>
      <div class="stat"><span>BAT</span><strong id="bat">-</strong></div>
    </div>
    <div class="channel-grid">
      <div class="cell label">CH1 STR</div><div class="cell value" id="ch1">--</div>
      <div class="cell label">CH2 GAS</div><div class="cell value" id="ch2">--</div>
      <div class="cell label">CH3 TILT</div><div class="cell value" id="ch3">--</div>
      <div class="cell label">CH4 PAN</div><div class="cell value" id="ch4">--</div>
      <div class="cell label">CH5 ARM</div><div class="cell value" id="ch5">--</div>
      <div class="cell label">CH6 MODE</div><div class="cell value" id="ch6">--</div>
      <div class="cell label">CH7 UNASSIGNED</div><div class="cell value" id="ch7">--</div>
      <div class="cell label">CH8 UNASSIGNED</div><div class="cell value" id="ch8">--</div>
    </div>
  </div></div>
  <form id="form" class="panel">
    <div class="head">Output Configuration</div>
    <div class="body grid">
      <label class="full">Craft Name<input id="craftName" name="craftName" maxlength="16"></label>
      <label>Steering<select id="steeringReversed" name="steeringReversed"><option value="0">Normal</option><option value="1">Reversed</option></select></label>
      <label>Throttle<select id="throttleReversed" name="throttleReversed"><option value="0">Normal</option><option value="1">Reversed</option></select></label>
      <label class="full">O4 Armed Trigger<select id="djiArmedToO4" name="djiArmedToO4"><option value="0">Off</option><option value="1">On</option></select></label>
      <label class="full">Steering Trim<input id="steeringTrimUs" name="steeringTrimUs" type="number" min="-300" max="300" step="5"></label>
      <div class="trim-tools">
        <button type="button" class="secondary" data-trim="-25">-25</button>
        <button type="button" class="secondary" data-trim="-5">-5</button>
        <button type="button" class="secondary" id="centerTrim">Center</button>
        <button type="button" class="secondary" data-trim="5">+5</button>
        <button type="button" class="secondary" data-trim="25">+25</button>
      </div>
      <label>STR Min<input id="steeringMinUs" name="steeringMinUs" type="number" min="800" max="1600" step="5"></label>
      <label>STR Max<input id="steeringMaxUs" name="steeringMaxUs" type="number" min="1400" max="2200" step="5"></label>
      <label>ESC Min<input id="escMinUs" name="escMinUs" type="number" min="800" max="1600" step="5"></label>
      <label>ESC Max<input id="escMaxUs" name="escMaxUs" type="number" min="1400" max="2200" step="5"></label>
      <label>Neutral Low<input id="escNeutralLowUs" name="escNeutralLowUs" type="number" min="1200" max="1600" step="5"></label>
      <label>Neutral High<input id="escNeutralHighUs" name="escNeutralHighUs" type="number" min="1400" max="1800" step="5"></label>
      <label>Reverse Assist<select id="escReverseAssist" name="escReverseAssist"><option value="0">Off</option><option value="1">On</option></select></label>
      <label>Reverse Delay ms<input id="escReverseDelayMs" name="escReverseDelayMs" type="number" min="0" max="1500" step="10"></label>
      <div class="section-title full">Head Servo</div>
      <label>Pan Output<select id="headPanEnabled" name="headPanEnabled"><option value="0">Off</option><option value="1">On</option></select></label>
      <label>Pan Direction<select id="headPanReversed" name="headPanReversed"><option value="0">Normal</option><option value="1">Reversed</option></select></label>
      <label>Pan Scale %<input id="headPanScalePercent" name="headPanScalePercent" type="number" min="0" max="100" step="5"></label>
      <label>Pan Trim<input id="headPanTrimUs" name="headPanTrimUs" type="number" min="-300" max="300" step="5"></label>
      <label>Pan Min<input id="headPanMinUs" name="headPanMinUs" type="number" min="800" max="1600" step="5"></label>
      <label>Pan Max<input id="headPanMaxUs" name="headPanMaxUs" type="number" min="1400" max="2200" step="5"></label>
      <label>Tilt Output<select id="headTiltEnabled" name="headTiltEnabled"><option value="0">Off</option><option value="1">On</option></select></label>
      <label>Tilt Direction<select id="headTiltReversed" name="headTiltReversed"><option value="0">Normal</option><option value="1">Reversed</option></select></label>
      <label>Tilt Scale %<input id="headTiltScalePercent" name="headTiltScalePercent" type="number" min="0" max="100" step="5"></label>
      <label>Tilt Trim<input id="headTiltTrimUs" name="headTiltTrimUs" type="number" min="-300" max="300" step="5"></label>
      <label>Tilt Min<input id="headTiltMinUs" name="headTiltMinUs" type="number" min="800" max="1600" step="5"></label>
      <label>Tilt Max<input id="headTiltMaxUs" name="headTiltMaxUs" type="number" min="1400" max="2200" step="5"></label>
      <div class="sub full">Pan follows CH4 on GPIO11. Tilt follows CH3 on GPIO12. Outputs default to Off for safe setup.</div>
      <div class="section-title full">Battery Warning</div>
      <label>Cell Count<select id="batteryCellCount" name="batteryCellCount"><option value="1">1S</option><option value="2">2S</option><option value="3">3S</option><option value="4">4S</option><option value="5">5S</option><option value="6">6S</option></select></label>
      <label>Low Warning V/cell<input id="batteryWarnCellV" name="batteryWarnCellV" type="number" min="0" max="4.20" step="0.01"></label>
      <label>Fuel Empty V/cell<input id="batteryFuelEmptyCellV" name="batteryFuelEmptyCellV" type="number" min="0" max="4.20" step="0.01"></label>
      <div class="sub full">Low Warning shows RETURN NOW. Fuel Empty locks ESC neutral until reboot. 0.00 disables each threshold.</div>
      <div class="section-title full">GPS Home Point</div>
      <label>Home Min Satellites<input id="homeMinSats" name="homeMinSats" type="number" min="7" max="15" step="1"></label>
      <label>Home Stability ms<input id="homeStabilityMs" name="homeStabilityMs" type="number" min="0" max="30000" step="500"></label>
      <div class="section-title full">IMU Configuration</div>
      <label class="full">IMU Rotation<select id="imuRotation" name="imuRotation"><option value="0">0 degrees</option><option value="1">90 degrees clockwise</option><option value="2">180 degrees</option><option value="3">270 degrees clockwise</option></select></label>
      <div class="sub full">Place the vehicle level before calibration. Changing rotation clears the previous zero position.</div>
      <div class="live full" id="imuLive">IMU ROL -- / PIT --</div>
      <button type="button" class="secondary" id="imuCalibrate">Set Current Position as Level</button><button type="button" class="secondary" id="imuReset">Reset IMU Calibration</button>
      <div class="section-title full">OSD Visibility</div>
      <div class="checks full">
        <label class="check"><input type="checkbox" id="osdShowCraft" name="osdShowCraft">Craft</label>
        <label class="check"><input type="checkbox" id="osdShowDriveMode" name="osdShowDriveMode">Drive Mode</label>
        <label class="check"><input type="checkbox" id="osdShowCoords" name="osdShowCoords">Coordinates</label>
        <label class="check"><input type="checkbox" id="osdShowGps" name="osdShowGps">GPS</label>
        <label class="check"><input type="checkbox" id="osdShowBattery" name="osdShowBattery">Battery</label>
        <label class="check"><input type="checkbox" id="osdShowLinkQuality" name="osdShowLinkQuality">Link Quality</label>
        <label class="check"><input type="checkbox" id="osdShowControls" name="osdShowControls">STR/GAS</label>
        <label class="check"><input type="checkbox" id="osdShowHome" name="osdShowHome">Home</label>
        <label class="check"><input type="checkbox" id="osdShowWifi" name="osdShowWifi">WiFi</label>
        <label class="check"><input type="checkbox" id="osdShowImu" name="osdShowImu">IMU</label>
        <label class="check"><input type="checkbox" id="osdShowDriveTime" name="osdShowDriveTime">Drive Time</label>
      </div>
      <div class="sub full">Safety warnings stay enabled even when optional OSD items are hidden.</div>
      <button type="submit">Save</button><button type="button" class="secondary" id="defaults">Defaults</button><button type="button" class="secondary" id="wifiOff">Disconnect WiFi</button>
    </div>
  </form>
  <div class="sub" id="msg">Connect to RZGXRover, then open 10.10.4.1</div>
</main>
<script>
const $=id=>document.getElementById(id);
const osdKeys=['osdShowCraft','osdShowDriveMode','osdShowCoords','osdShowGps','osdShowBattery','osdShowLinkQuality','osdShowControls','osdShowHome','osdShowWifi','osdShowImu','osdShowDriveTime'];
function pctRam(d){return d.heapSizeBytes?Math.max(0,Math.min(100,Math.round(((d.heapSizeBytes-d.freeHeapBytes)/d.heapSizeBytes)*100)))+'%':'--'}
function up(ms){let s=Math.floor((ms||0)/1000),m=Math.floor(s/60);s%=60;return m?m+'m '+s+'s':s+'s'}
function setConfig(d){$('fw').textContent='v'+(d.version||'--');['craftName','steeringTrimUs','steeringMinUs','steeringMaxUs','escMinUs','escMaxUs','escNeutralLowUs','escNeutralHighUs','escReverseDelayMs','headPanScalePercent','headPanTrimUs','headPanMinUs','headPanMaxUs','headTiltScalePercent','headTiltTrimUs','headTiltMinUs','headTiltMaxUs','homeMinSats','homeStabilityMs'].forEach(k=>{if(d[k]!=null)$(k).value=d[k]});$('steeringReversed').value=d.steeringReversed?1:0;$('throttleReversed').value=d.throttleReversed?1:0;$('djiArmedToO4').value=d.djiArmedToO4?1:0;$('escReverseAssist').value=d.escReverseAssist?1:0;$('headPanEnabled').value=d.headPanEnabled?1:0;$('headPanReversed').value=d.headPanReversed?1:0;$('headTiltEnabled').value=d.headTiltEnabled?1:0;$('headTiltReversed').value=d.headTiltReversed?1:0;$('imuRotation').value=d.imuRotation??0;$('batteryCellCount').value=d.batteryCellCount??2;$('batteryWarnCellV').value=((d.batteryWarnCellCv??310)/100).toFixed(2);$('batteryFuelEmptyCellV').value=((d.batteryFuelEmptyCellCv??300)/100).toFixed(2);osdKeys.forEach(k=>{if($(k))$(k).checked=d[k]!==false})}
function bat(d){return d.batteryVoltageValid?(Number(d.batteryVoltageCv||0)/100).toFixed(2)+'V':'-'}
function setStatus(d){$('fw').textContent='v'+(d.version||'--');$('cpu').textContent=(d.cpuLoadPct??0)+'%';$('ram').textContent=pctRam(d);$('up').textContent=up(d.uptimeMs);$('bat').textContent=bat(d);$('ch1').textContent=d.rcCh1Us??'--';$('ch2').textContent=d.rcCh2Us??'--';$('ch3').textContent=d.rcCh3Us??'--';$('ch4').textContent=d.rcCh4Us??'--';$('ch5').textContent=(d.rcCh5Us??0)>1500?'ON':'OFF';$('ch5').style.color=d.wifiArmingLockout?'#ff756d':'';$('ch6').textContent=d.driveMode||'--';$('ch7').textContent=d.rcCh7Us??'--';$('ch8').textContent=d.rcCh8Us??'--';$('o4msp').textContent=d.o4MspLink?'O4 MSP OK':'O4 MSP LOST';$('o4msp').className='link '+(d.o4MspLink?'ok':'lost');$('imuLive').textContent=d.imuDataValid?'IMU ROL '+Number(d.imuRollDeg||0).toFixed(1)+' / PIT '+Number(d.imuPitchDeg||0).toFixed(1)+' | FS '+(d.elrsFailsafeCount??0)+' / age '+(d.crsfRcFrameAgeMs??0)+'ms':'IMU unavailable | FS '+(d.elrsFailsafeCount??0)}
async function load(){const d=await (await fetch('/api/config')).json();setConfig(d);setStatus(d)}
async function tick(){try{setStatus(await (await fetch('/api/status')).json())}catch(e){}}
$('form').addEventListener('submit',async e=>{e.preventDefault();const body=new URLSearchParams(new FormData(e.target));osdKeys.forEach(k=>body.set(k,$(k).checked?'1':'0'));const r=await fetch('/api/save',{method:'POST',body});$('msg').textContent=await r.text();load()});
$('defaults').addEventListener('click',async()=>{if(confirm('Restore defaults?')){$('msg').textContent=await (await fetch('/api/defaults',{method:'POST'})).text();load()}});
$('imuCalibrate').addEventListener('click',async()=>{if(confirm('Use the current vehicle position as roll 0 and pitch 0?')){$('msg').textContent=await (await fetch('/api/imu/calibrate',{method:'POST'})).text();load()}});
$('imuReset').addEventListener('click',async()=>{if(confirm('Clear the saved IMU zero position?')){$('msg').textContent=await (await fetch('/api/imu/reset',{method:'POST'})).text();load()}});
$('wifiOff').addEventListener('click',async()=>{if(confirm('Disconnect ESP32 WiFi now?')){try{await fetch('/api/wifi/off',{method:'POST'})}catch(e){}$('msg').textContent='WiFi disconnect requested'}});
function setTrim(v){const i=$('steeringTrimUs');i.value=Math.max(-300,Math.min(300,Math.round((Number(v)||0)/5)*5));i.dispatchEvent(new Event('input',{bubbles:true}));i.dispatchEvent(new Event('change',{bubbles:true}))}
document.querySelectorAll('[data-trim]').forEach(b=>b.addEventListener('click',e=>{e.preventDefault();setTrim((Number($('steeringTrimUs').value)||0)+Number(b.dataset.trim||0))}));
$('centerTrim').addEventListener('click',e=>{e.preventDefault();setTrim(0)});
load();setInterval(tick,1000);
</script>
</body>
</html>
)rawliteral";

struct Payload {
  uint8_t data[240];
  uint8_t len;

  void clear() {
    len = 0;
  }

  void u8(uint8_t v) {
    if (len < sizeof(data)) data[len++] = v;
  }

  void u16(uint16_t v) {
    u8(v & 0xFF);
    u8((v >> 8) & 0xFF);
  }

  void i16(int16_t v) {
    u16((uint16_t)v);
  }

  void u32(uint32_t v) {
    u8(v & 0xFF);
    u8((v >> 8) & 0xFF);
    u8((v >> 16) & 0xFF);
    u8((v >> 24) & 0xFF);
  }

  void i32(int32_t v) {
    u32((uint32_t)v);
  }

  void bytes(const char *s, uint8_t n) {
    for (uint8_t i = 0; i < n; i++) u8((uint8_t)s[i]);
  }

  void cstr(const char *s) {
    while (*s) u8((uint8_t)*s++);
  }
};

enum ParserState {
  WAIT_DOLLAR,
  WAIT_PROTO,
  WAIT_DIR,
  WAIT_V2_FLAGS,
  WAIT_V2_CMD_L,
  WAIT_V2_CMD_H,
  WAIT_V2_SIZE_L,
  WAIT_V2_SIZE_H,
  WAIT_SIZE,
  WAIT_CMD,
  WAIT_PAYLOAD,
  WAIT_CHECKSUM
};

enum CrsfParserState {
  CRSF_WAIT_ADDR,
  CRSF_WAIT_LEN,
  CRSF_WAIT_FRAME
};

static ParserState parserState = WAIT_DOLLAR;
static bool rxMspV2 = false;
static uint8_t rxFlags = 0;
static uint8_t rxSize = 0;
static uint16_t rxSizeV2 = 0;
static uint16_t rxCmd = 0;
static uint8_t rxChecksum = 0;
static uint8_t rxPayload[128];
static uint8_t rxIndex = 0;

static uint32_t commandCount = 0;
static uint32_t lastMspRequestMs = 0;
static uint32_t rawByteCount = 0;
static uint32_t badChecksumCount = 0;
static uint32_t dollarCount = 0;
static uint32_t mspV1HeaderCount = 0;
static uint32_t mspV2HeaderCount = 0;
static uint32_t lastHeartbeatMs = 0;
static uint32_t lastDisplayPortMs = 0;
static uint32_t lastVtxMspProbeMs = 0;
static uint32_t lastTxBeaconMs = 0;
static uint32_t displayPortFramesSent = 0;
static uint16_t displayPortRegrabCount = 0;
static uint16_t displayPortReacquireCount = 0;
static uint16_t displayPortRequestCount = 0;
static bool displayPortClearRequested = false;
static uint32_t vtxMspProbeCount = 0;
static uint32_t txBeaconCount = 0;
static uint32_t replyPacketCount = 0;
static uint32_t replyByteCount = 0;
static uint8_t sniffAfterDollar = 0;
static uint8_t sniffBytes[10];
static uint8_t sniffIndex = 0;
static uint8_t sniffPrinted = 0;
static uint8_t rawSample[48];
static uint8_t rawSampleLen = 0;
static uint8_t lastByte1 = 0;
static uint8_t lastByte2 = 0;
static uint32_t commandHistogram[256];
static char usbConfigLine[512];
static uint8_t usbConfigLineLen = 0;
static bool usbConfigStreamEnabled = false;
static uint32_t lastUsbConfigRxMs = 0;
static uint32_t lastUsbConfigStreamMs = 0;
static uint32_t perfWindowStartUs = 0;
static uint32_t perfWindowBusyUs = 0;
static uint32_t perfWindowLoops = 0;
static uint32_t perfWindowMaxLoopUs = 0;
static uint16_t cpuLoadPct = 0;
static uint32_t loopAvgUs = 0;
static uint32_t loopMaxUs = 0;
static uint32_t loopHz = 0;
static uint32_t freeHeapBytes = 0;
static uint32_t minFreeHeapBytes = 0;
static uint32_t heapSizeBytes = 0;
static bool wifiConfigPortalStarted = false;
static uint32_t wifiConfigPortalStartMs = 0;
static uint32_t lastWifiActivityMs = 0;
static uint32_t lastWifiServiceMs = 0;
static bool wifiAutoOffDone = false;
static bool wifiArmingLockout = false;

static CrsfParserState crsfParserState = CRSF_WAIT_ADDR;
static uint8_t crsfFrameSize = 0;
static uint8_t crsfFrameIndex = 0;
static uint8_t crsfFrame[CRSF_MAX_FRAME_SIZE];
static uint32_t crsfByteCount = 0;
static uint32_t crsfFrameCount = 0;
static uint32_t crsfRcFrameCount = 0;
static uint32_t crsfLinkFrameCount = 0;
static uint32_t crsfBadCrcCount = 0;
static uint32_t lastCrsfFrameMs = 0;
static uint32_t lastCrsfRcFrameMs = 0;
static uint8_t crsfLinkQuality = 0;
static int8_t crsfSnr = 0;
static bool lastFailsafeState = false;
static uint32_t elrsFailsafeCount = 0;
static uint32_t elrsFailsafeStartMs = 0;
static uint32_t elrsFailsafeLastDurationMs = 0;
static bool steeringPwmAttached = false;
static bool steeringOutputLive = false;
static bool steeringPulseHigh = false;
static uint16_t steeringPulseUs = 1500;
static uint32_t steeringFrameStartUs = 0;
static uint32_t steeringPulseEndUs = 0;
static bool escPwmAttached = false;
static bool escSawDisarmedSinceBoot = false;
static bool escNeutralGateSatisfied = false;
static bool escFailsafeRecoveryLocked = false;
static bool escOutputLive = false;
static bool escPulseHigh = false;
static uint16_t escPulseUs = 1500;
static uint32_t escFrameStartUs = 0;
static uint32_t escPulseEndUs = 0;
static bool headPanPwmAttached = false;
static bool headTiltPwmAttached = false;
static bool headPanOutputLive = false;
static bool headTiltOutputLive = false;
static uint16_t headPanPulseUs = HEAD_SERVO_CENTER_US;
static uint16_t headTiltPulseUs = HEAD_SERVO_CENTER_US;
static bool escReverseDelayActive = false;
static uint32_t escReverseDelayStartMs = 0;
static bool engineReadyBlinkActive = false;
static uint32_t engineReadyBlinkStartMs = 0;

static uint32_t gpsByteCount = 0;
static uint32_t gpsSentenceCount = 0;
static uint32_t gpsParsedCount = 0;
static uint32_t lastGpsFixMs = 0;
static char gpsLine[128];
static uint8_t gpsLineLen = 0;
static bool homeSet = false;
static double homeLatDeg = 0.0;
static double homeLonDeg = 0.0;
static bool currentGpsPositionValid = false;
static double currentGpsLatDeg = 0.0;
static double currentGpsLonDeg = 0.0;
static uint32_t homePointNoticeStartMs = 0;
static uint32_t homePointGestureHoldStartMs = 0;
static bool homePointGestureDone = false;
static uint32_t homeEligibleSinceMs = 0;

static bool batteryVoltageValid = false;
static uint16_t batteryVoltage_dV = 0;         // 0.1V units
static uint16_t batteryVoltage_cV = 0;         // 0.01V units
static uint16_t batteryAdcMilliVolts = 0;
static uint32_t lastBatterySampleMs = 0;
static bool batteryWarningActive = false;
static uint32_t batteryWarnLowSinceMs = 0;
static uint32_t batteryWarnClearSinceMs = 0;
static bool batteryFuelEmptyActive = false;
static uint32_t batteryFuelEmptyLowSinceMs = 0;
static uint16_t amperage_cA = 0;               // 0.01A units
static uint16_t mahDrawn = 0;
static uint16_t rssi = 1023;

static bool imuAvailable = false;
static bool imuDataValid = false;
static uint8_t imuConsecutiveErrors = 0;
static uint32_t lastImuProbeMs = 0;
static uint32_t lastImuSampleMs = 0;
static uint32_t lastImuSampleUs = 0;
static float imuRollDeg = 0.0f;
static float imuPitchDeg = 0.0f;

enum ImuCalibrationGestureState : uint8_t {
  IMU_GESTURE_IDLE,
  IMU_GESTURE_REVERSE_HOLD,
  IMU_GESTURE_DONE
};

static ImuCalibrationGestureState imuGestureState = IMU_GESTURE_IDLE;
static uint32_t imuGestureHoldStartMs = 0;
static uint32_t imuCalibrationNoticeStartMs = 0;
static bool driveTimerSessionActive = false;
static uint32_t driveTimerStartMs = 0;
static uint32_t driveTimerElapsedMs = 0;
static bool driveStatsSessionActive = false;
static uint32_t driveStatsLastSampleMs = 0;
static uint32_t driveStatsSpeedSampleCount = 0;
static uint32_t driveStatsSpeedSumKmh = 0;
static uint16_t driveStatsMaxSpeedKmh = 0;
static uint8_t driveStatsMaxSats = 0;
static float driveStatsMaxRollAbsDeg = 0.0f;
static float driveStatsMaxPitchAbsDeg = 0.0f;
static bool driveStatsLastGpsValid = false;
static double driveStatsLastLatDeg = 0.0;
static double driveStatsLastLonDeg = 0.0;
static uint32_t driveStatsTripDistanceCm = 0;
static bool driveStatsDisplayActive = false;
static uint32_t driveStatsDisplayStartMs = 0;
static uint32_t driveStatsLastDurationMs = 0;
static uint16_t driveStatsLastAvgSpeedKmh = 0;
static uint16_t driveStatsLastMaxSpeedKmh = 0;
static uint16_t driveStatsLastTripMeters = 0;
static uint8_t driveStatsLastMaxSats = 0;
static uint16_t driveStatsLastMaxRollDeg = 0;
static uint16_t driveStatsLastMaxPitchDeg = 0;

static uint8_t gpsFix = 0;
static uint8_t gpsSats = 0;
static int32_t gpsLat = 0;                     // degrees * 10,000,000
static int32_t gpsLon = 0;                     // degrees * 10,000,000
static int16_t gpsAltMeters = 0;
static uint16_t gpsSpeedCms = 0;
static uint16_t gpsGroundCourseDeg10 = 0;
static uint16_t homeDistanceMeters = 0;
static int16_t homeDirectionDeg = 0;

static uint16_t rcChannels[8] = {
  1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000
};

double nmeaCoordToDeg(const char *value, const char *hemi) {
  if (!value || !hemi || value[0] == '\0' || hemi[0] == '\0') return 0.0;

  const double raw = atof(value);
  const int degrees = (int)(raw / 100.0);
  const double minutes = raw - (degrees * 100.0);
  double result = degrees + (minutes / 60.0);

  if (hemi[0] == 'S' || hemi[0] == 'W') result = -result;
  return result;
}

int32_t degToMspCoord(double deg) {
  return (int32_t)llround(deg * 10000000.0);
}

double degToRad(double deg) {
  return deg * PI / 180.0;
}

double radToDeg(double rad) {
  return rad * 180.0 / PI;
}

void updateBatteryVoltage(bool force = false) {
  const uint32_t now = millis();
  if (!force && now - lastBatterySampleMs < BATTERY_SAMPLE_INTERVAL_MS) return;
  lastBatterySampleMs = now;

  uint32_t adcMvSum = 0;
  for (uint8_t i = 0; i < BATTERY_ADC_SAMPLES; i++) {
    adcMvSum += analogReadMilliVolts(BATTERY_ADC_PIN);
  }

  const uint32_t adcMv = adcMvSum / BATTERY_ADC_SAMPLES;
  batteryAdcMilliVolts = (adcMv > 65535) ? 65535 : (uint16_t)adcMv;

  if (adcMv < BATTERY_ADC_MIN_VALID_MV) {
    batteryVoltageValid = false;
    batteryVoltage_dV = 0;
    batteryVoltage_cV = 0;
    return;
  }

  const uint32_t batteryMv =
      (adcMv * (uint32_t)(BATTERY_DIVIDER_TOP_KOHM + BATTERY_DIVIDER_BOTTOM_KOHM) +
       (BATTERY_DIVIDER_BOTTOM_KOHM / 2)) /
      BATTERY_DIVIDER_BOTTOM_KOHM;
  const uint32_t batteryCentivolts = batteryMv / 10;

  batteryVoltageValid = true;
  batteryVoltage_cV = (batteryCentivolts > 65535) ? 65535 : (uint16_t)batteryCentivolts;
  batteryVoltage_dV = (uint16_t)((batteryVoltage_cV + 5) / 10);
}

uint16_t batteryAverageCellVoltageCv() {
  if (!batteryVoltageValid || batteryCellCount == 0) return 0;
  return (uint16_t)((batteryVoltage_cV + (batteryCellCount / 2)) / batteryCellCount);
}

bool batteryLowWarningEnabled() {
  return batteryWarnCellCv > 0;
}

bool batteryFuelEmptyEnabled() {
  return batteryFuelEmptyCellCv > 0;
}

void resetBatteryWarningState() {
  batteryWarningActive = false;
  batteryWarnLowSinceMs = 0;
  batteryWarnClearSinceMs = 0;
}

void updateBatteryWarning() {
  const uint32_t now = millis();
  if (!batteryVoltageValid || !batteryLowWarningEnabled()) {
    resetBatteryWarningState();
    return;
  }

  const uint16_t cellVoltageCv = batteryAverageCellVoltageCv();
  if (cellVoltageCv <= batteryWarnCellCv) {
    if (batteryWarnLowSinceMs == 0) batteryWarnLowSinceMs = now;
    batteryWarnClearSinceMs = 0;
    if (now - batteryWarnLowSinceMs >= BATTERY_WARN_TRIGGER_MS) {
      batteryWarningActive = true;
    }
    return;
  }

  batteryWarnLowSinceMs = 0;
  if (!batteryWarningActive) return;

  const uint16_t clearCv = batteryWarnCellCv + BATTERY_WARN_HYSTERESIS_CV;
  if (cellVoltageCv >= clearCv) {
    if (batteryWarnClearSinceMs == 0) batteryWarnClearSinceMs = now;
    if (now - batteryWarnClearSinceMs >= BATTERY_WARN_CLEAR_MS) {
      resetBatteryWarningState();
    }
  } else {
    batteryWarnClearSinceMs = 0;
  }
}

void updateBatteryFuelEmpty() {
  if (batteryFuelEmptyActive) return;

  const uint32_t now = millis();
  if (!batteryVoltageValid || !batteryFuelEmptyEnabled()) {
    batteryFuelEmptyLowSinceMs = 0;
    return;
  }

  const uint16_t cellVoltageCv = batteryAverageCellVoltageCv();
  if (cellVoltageCv <= batteryFuelEmptyCellCv) {
    if (batteryFuelEmptyLowSinceMs == 0) batteryFuelEmptyLowSinceMs = now;
    if (now - batteryFuelEmptyLowSinceMs >= BATTERY_FUEL_EMPTY_TRIGGER_MS) {
      batteryFuelEmptyActive = true;
    }
  } else {
    batteryFuelEmptyLowSinceMs = 0;
  }
}

bool imuWriteRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(IMU_I2C_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool imuReadRegisters(uint8_t reg, uint8_t *data, uint8_t length) {
  Wire.beginTransmission(IMU_I2C_ADDRESS);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;

  const uint8_t received = Wire.requestFrom(IMU_I2C_ADDRESS, length);
  if (received != length) {
    while (Wire.available()) Wire.read();
    return false;
  }

  for (uint8_t i = 0; i < length; i++) {
    data[i] = (uint8_t)Wire.read();
  }
  return true;
}

int16_t imuReadI16(const uint8_t *data) {
  return (int16_t)(((uint16_t)data[0] << 8) | data[1]);
}

bool initializeImuDevice() {
  uint8_t whoAmI = 0;
  if (!imuReadRegisters(0x75, &whoAmI, 1)) return false;
  if (whoAmI != 0x68 && whoAmI != 0x69) return false;

  if (!imuWriteRegister(0x6B, 0x01)) return false; // Wake, use X gyro PLL.
  if (!imuWriteRegister(0x1A, 0x03)) return false; // DLPF around 44 Hz.
  if (!imuWriteRegister(0x1B, 0x00)) return false; // Gyro +/-250 deg/s.
  if (!imuWriteRegister(0x1C, 0x00)) return false; // Accelerometer +/-2 g.

  imuConsecutiveErrors = 0;
  imuDataValid = false;
  lastImuSampleUs = 0;
  return true;
}

void setupImu() {
  Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN, IMU_I2C_FREQUENCY_HZ);
  Wire.setTimeOut(IMU_I2C_TIMEOUT_MS);
  lastImuProbeMs = millis();
  imuAvailable = initializeImuDevice();
}

void updateImu() {
  const uint32_t nowMs = millis();
  if (!imuAvailable) {
    if (nowMs - lastImuProbeMs < IMU_RETRY_INTERVAL_MS) return;
    lastImuProbeMs = nowMs;
    imuAvailable = initializeImuDevice();
    return;
  }

  if (nowMs - lastImuSampleMs < IMU_SAMPLE_INTERVAL_MS) return;
  lastImuSampleMs = nowMs;

  uint8_t raw[14];
  if (!imuReadRegisters(0x3B, raw, sizeof(raw))) {
    if (++imuConsecutiveErrors >= IMU_MAX_CONSECUTIVE_ERRORS) {
      imuAvailable = false;
      imuDataValid = false;
      lastImuProbeMs = nowMs;
    }
    return;
  }

  imuConsecutiveErrors = 0;
  const float ax = imuReadI16(&raw[0]) / 16384.0f;
  const float ay = imuReadI16(&raw[2]) / 16384.0f;
  const float az = imuReadI16(&raw[4]) / 16384.0f;
  const float gx = imuReadI16(&raw[8]) / 131.0f;
  const float gy = imuReadI16(&raw[10]) / 131.0f;

  const float rollAccel = atan2f(ay, az) * 180.0f / PI;
  const float pitchAccel = atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / PI;
  const uint32_t nowUs = micros();

  if (!imuDataValid || lastImuSampleUs == 0) {
    imuRollDeg = rollAccel;
    imuPitchDeg = pitchAccel;
    imuDataValid = true;
  } else {
    float dt = (nowUs - lastImuSampleUs) / 1000000.0f;
    if (dt < 0.001f || dt > 0.1f) dt = IMU_SAMPLE_INTERVAL_MS / 1000.0f;
    imuRollDeg = IMU_COMPLEMENTARY_ALPHA * (imuRollDeg + gx * dt) +
                 (1.0f - IMU_COMPLEMENTARY_ALPHA) * rollAccel;
    imuPitchDeg = IMU_COMPLEMENTARY_ALPHA * (imuPitchDeg + gy * dt) +
                  (1.0f - IMU_COMPLEMENTARY_ALPHA) * pitchAccel;
  }
  lastImuSampleUs = nowUs;
}

void mappedImuAngles(float &rollDeg, float &pitchDeg) {
  switch (imuRotation & 0x03) {
    case 1:
      rollDeg = -imuPitchDeg;
      pitchDeg = imuRollDeg;
      break;
    case 2:
      rollDeg = -imuRollDeg;
      pitchDeg = -imuPitchDeg;
      break;
    case 3:
      rollDeg = imuPitchDeg;
      pitchDeg = -imuRollDeg;
      break;
    default:
      rollDeg = imuRollDeg;
      pitchDeg = imuPitchDeg;
      break;
  }
}

float displayedImuRollDeg() {
  float rollDeg;
  float pitchDeg;
  mappedImuAngles(rollDeg, pitchDeg);
  return rollDeg - imuRollZeroDeg;
}

float displayedImuPitchDeg() {
  float rollDeg;
  float pitchDeg;
  mappedImuAngles(rollDeg, pitchDeg);
  return pitchDeg - imuPitchZeroDeg;
}

void resetImuCalibration() {
  imuRollZeroDeg = 0.0f;
  imuPitchZeroDeg = 0.0f;
}

bool calibrateImuCurrentLevel() {
  if (!imuDataValid || engineSwitchOn()) return false;

  mappedImuAngles(imuRollZeroDeg, imuPitchZeroDeg);
  saveSurfaceConfig();
  imuCalibrationNoticeStartMs = millis();
  return true;
}

bool imuCalibrationNoticeActive() {
  return imuCalibrationNoticeStartMs != 0 &&
         millis() - imuCalibrationNoticeStartMs < IMU_CALIBRATION_NOTICE_MS;
}

bool homePointNoticeActive() {
  return homePointNoticeStartMs != 0 &&
         millis() - homePointNoticeStartMs < HOME_POINT_NOTICE_MS;
}

bool gpsPositionUsable(double latDeg, double lonDeg) {
  return gpsFix && isfinite(latDeg) && isfinite(lonDeg) &&
         (fabs(latDeg) >= 0.000001 || fabs(lonDeg) >= 0.000001);
}

bool gpsHomePointEligible() {
  return gpsFix && gpsSats >= homeMinSats && currentGpsPositionValid;
}

bool setHomePoint(double latDeg, double lonDeg, bool notify) {
  if (!gpsPositionUsable(latDeg, lonDeg)) return false;

  homeSet = true;
  homeLatDeg = latDeg;
  homeLonDeg = lonDeg;
  homeDistanceMeters = 0;
  homeDirectionDeg = 0;
  homeEligibleSinceMs = 0;
  if (notify) {
    homePointNoticeStartMs = millis();
  }
  return true;
}

double distanceMetersBetween(double fromLatDeg, double fromLonDeg, double toLatDeg, double toLonDeg) {
  const double earthRadiusMeters = 6371000.0;
  const double lat1 = degToRad(fromLatDeg);
  const double lat2 = degToRad(toLatDeg);
  const double dLat = degToRad(toLatDeg - fromLatDeg);
  const double dLon = degToRad(toLonDeg - fromLonDeg);
  const double a = sin(dLat / 2.0) * sin(dLat / 2.0) +
                   cos(lat1) * cos(lat2) * sin(dLon / 2.0) * sin(dLon / 2.0);
  const double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
  const double distance = earthRadiusMeters * c;
  return (isfinite(distance) && distance >= 0.0) ? distance : -1.0;
}

void updateHomeFromGps(double latDeg, double lonDeg) {
  if (!gpsPositionUsable(latDeg, lonDeg)) return;

  currentGpsPositionValid = true;
  currentGpsLatDeg = latDeg;
  currentGpsLonDeg = lonDeg;

  if (!homeSet) {
    const uint32_t now = millis();
    if (gpsSats >= homeMinSats) {
      if (homeEligibleSinceMs == 0) homeEligibleSinceMs = now;
      if (now - homeEligibleSinceMs >= homeStabilityMs) {
        setHomePoint(latDeg, lonDeg, true);
      }
    } else {
      homeEligibleSinceMs = 0;
    }
  }

  if (!homeSet) {
    return;
  }

  const double lat1 = degToRad(homeLatDeg);
  const double lat2 = degToRad(latDeg);
  const double dLon = degToRad(lonDeg - homeLonDeg);
  const double distance = distanceMetersBetween(homeLatDeg, homeLonDeg, latDeg, lonDeg);
  if (!isfinite(distance) || distance < 0.0) return;

  const double y = sin(dLon) * cos(lat2);
  const double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);
  double bearing = radToDeg(atan2(y, x));
  if (!isfinite(bearing)) bearing = 0.0;
  if (bearing < 0.0) bearing += 360.0;

  homeDistanceMeters = (uint16_t)constrain((int)lround(distance), 0, 65535);
  homeDirectionDeg = (int16_t)lround(bearing);
}

bool getNmeaField(const char *line, uint8_t wantedIndex, char *out, uint8_t outSize) {
  uint8_t fieldIndex = 0;
  uint8_t outIndex = 0;

  if (outSize == 0) return false;
  out[0] = '\0';

  for (uint16_t i = 0; line[i] != '\0'; i++) {
    const char c = line[i];

    if (c == ',' || c == '*') {
      if (fieldIndex == wantedIndex) {
        out[outIndex] = '\0';
        return true;
      }
      fieldIndex++;
      outIndex = 0;
      continue;
    }

    if (fieldIndex == wantedIndex && outIndex < outSize - 1) {
      out[outIndex++] = c;
    }
  }

  if (fieldIndex == wantedIndex) {
    out[outIndex] = '\0';
    return true;
  }
  return false;
}

void parseGga(const char *line) {
  char fixQuality[4];
  char sats[4];
  char lat[16];
  char ns[3];
  char lon[16];
  char ew[3];
  char alt[12];

  if (!getNmeaField(line, 2, lat, sizeof(lat))) return;
  if (!getNmeaField(line, 3, ns, sizeof(ns))) return;
  if (!getNmeaField(line, 4, lon, sizeof(lon))) return;
  if (!getNmeaField(line, 5, ew, sizeof(ew))) return;
  if (!getNmeaField(line, 6, fixQuality, sizeof(fixQuality))) return;
  if (!getNmeaField(line, 7, sats, sizeof(sats))) return;
  if (!getNmeaField(line, 9, alt, sizeof(alt))) return;

  gpsFix = (atoi(fixQuality) > 0) ? 1 : 0;
  gpsSats = (uint8_t)constrain(atoi(sats), 0, 99);
  gpsAltMeters = (int16_t)constrain((int)lround(atof(alt)), -32768, 32767);

  if (gpsFix) {
    const double latDeg = nmeaCoordToDeg(lat, ns);
    const double lonDeg = nmeaCoordToDeg(lon, ew);
    gpsLat = degToMspCoord(latDeg);
    gpsLon = degToMspCoord(lonDeg);
    lastGpsFixMs = millis();
    updateHomeFromGps(latDeg, lonDeg);
  }

  gpsParsedCount++;
}

void parseRmc(const char *line) {
  char status[3];
  char lat[16];
  char ns[3];
  char lon[16];
  char ew[3];
  char speedKnots[12];
  char courseDeg[12];

  if (!getNmeaField(line, 2, status, sizeof(status))) return;
  if (!getNmeaField(line, 3, lat, sizeof(lat))) return;
  if (!getNmeaField(line, 4, ns, sizeof(ns))) return;
  if (!getNmeaField(line, 5, lon, sizeof(lon))) return;
  if (!getNmeaField(line, 6, ew, sizeof(ew))) return;
  if (!getNmeaField(line, 7, speedKnots, sizeof(speedKnots))) return;
  if (!getNmeaField(line, 8, courseDeg, sizeof(courseDeg))) return;

  gpsFix = (status[0] == 'A') ? 1 : gpsFix;
  gpsSpeedCms = (uint16_t)constrain((int)lround(atof(speedKnots) * 51.4444), 0, 65535);
  gpsGroundCourseDeg10 = (uint16_t)constrain((int)lround(atof(courseDeg) * 10.0), 0, 3599);

  if (gpsFix) {
    const double latDeg = nmeaCoordToDeg(lat, ns);
    const double lonDeg = nmeaCoordToDeg(lon, ew);
    gpsLat = degToMspCoord(latDeg);
    gpsLon = degToMspCoord(lonDeg);
    lastGpsFixMs = millis();
    updateHomeFromGps(latDeg, lonDeg);
  }

  gpsParsedCount++;
}

void parseGpsLine(char *line) {
  gpsSentenceCount++;

  if (strncmp(line, "$GNGGA", 6) == 0 || strncmp(line, "$GPGGA", 6) == 0) {
    parseGga(line);
  } else if (strncmp(line, "$GNRMC", 6) == 0 || strncmp(line, "$GPRMC", 6) == 0) {
    parseRmc(line);
  }
}

void parseGpsByte(uint8_t b) {
  if (b == '\r') return;

  if (b == '\n') {
    if (gpsLineLen > 0) {
      gpsLine[gpsLineLen] = '\0';
      parseGpsLine(gpsLine);
      gpsLineLen = 0;
    }
    return;
  }

  if (gpsLineLen < sizeof(gpsLine) - 1) {
    gpsLine[gpsLineLen++] = (char)b;
  } else {
    gpsLineLen = 0;
  }
}

const char *commandName(uint8_t cmd) {
  switch (cmd) {
    case MSP_API_VERSION: return "MSP_API_VERSION";
    case MSP_FC_VARIANT: return "MSP_FC_VARIANT";
    case MSP_FC_VERSION: return "MSP_FC_VERSION";
    case MSP_BOARD_INFO: return "MSP_BOARD_INFO";
    case MSP_BUILD_INFO: return "MSP_BUILD_INFO";
    case MSP_NAME: return "MSP_NAME";
    case MSP_OSD_CONFIG: return "MSP_OSD_CONFIG";
    case MSP_VTX_CONFIG: return "MSP_VTX_CONFIG";
    case MSP_FILTER_CONFIG: return "MSP_FILTER_CONFIG";
    case MSP_PID_ADVANCED: return "MSP_PID_ADVANCED";
    case MSP_STATUS: return "MSP_STATUS";
    case MSP_RC: return "MSP_RC";
    case MSP_RAW_GPS: return "MSP_RAW_GPS";
    case MSP_COMP_GPS: return "MSP_COMP_GPS";
    case MSP_ATTITUDE: return "MSP_ATTITUDE";
    case MSP_ANALOG: return "MSP_ANALOG";
    case MSP_RC_TUNING: return "MSP_RC_TUNING";
    case MSP_PID: return "MSP_PID";
    case MSP_BATTERY_STATE: return "MSP_BATTERY_STATE";
    case MSP_STATUS_EX: return "MSP_STATUS_EX";
    case MSP_DISPLAYPORT: return "MSP_DISPLAYPORT";
    case MSP_SET_OSD_CANVAS: return "MSP_SET_OSD_CANVAS";
    case MSP_OSD_CANVAS: return "MSP_OSD_CANVAS";
    default: return "UNKNOWN";
  }
}

void sendMspReply(uint8_t cmd, const uint8_t *payload, uint8_t size) {
  if (!sendMspReplies) return;

  uint8_t checksum = 0;

  DJISerial.write('$');
  DJISerial.write('M');
  DJISerial.write('>');
  replyByteCount += 3;

  DJISerial.write(size);
  checksum ^= size;
  replyByteCount++;

  DJISerial.write(cmd);
  checksum ^= cmd;
  replyByteCount++;

  for (uint8_t i = 0; i < size; i++) {
    DJISerial.write(payload[i]);
    checksum ^= payload[i];
    replyByteCount++;
  }

  DJISerial.write(checksum);
  replyByteCount++;
  replyPacketCount++;
}

void sendMspV1PacketWithDirection(uint8_t direction, uint8_t cmd, const uint8_t *payload, uint8_t size) {
  uint8_t checksum = 0;

  DJISerial.write('$');
  DJISerial.write('M');
  DJISerial.write(direction);

  DJISerial.write(size);
  checksum ^= size;

  DJISerial.write(cmd);
  checksum ^= cmd;

  for (uint8_t i = 0; i < size; i++) {
    DJISerial.write(payload[i]);
    checksum ^= payload[i];
  }

  DJISerial.write(checksum);
  replyByteCount += (uint32_t)size + 6;
  replyPacketCount++;
}

uint8_t crc8DvbS2(uint8_t crc, uint8_t data) {
  crc ^= data;
  for (uint8_t i = 0; i < 8; i++) {
    crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5) : (uint8_t)(crc << 1);
  }
  return crc;
}

uint16_t crsfChannelToPwm(uint16_t value) {
  const uint16_t constrainedValue = constrain(value, CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX);
  return (uint16_t)map(constrainedValue, CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX, 1000, 2000);
}

uint16_t unpackCrsfChannel(const uint8_t *payload, uint8_t channelIndex) {
  const uint16_t bitOffset = channelIndex * 11;
  uint16_t value = 0;

  for (uint8_t bit = 0; bit < 11; bit++) {
    const uint16_t sourceBit = bitOffset + bit;
    if (payload[sourceBit / 8] & (1 << (sourceBit % 8))) {
      value |= (1 << bit);
    }
  }

  return value;
}

bool crsfLinkActive() {
  return (millis() - lastCrsfRcFrameMs) < 500;
}

bool engineSwitchOn() {
  return rcChannels[4] > 1500;
}

bool engineArmed() {
  return crsfLinkActive() && engineSwitchOn();
}

bool failsafeActive() {
  return !crsfLinkActive();
}

bool wifiClientConnected() {
  return wifiConfigPortalStarted && WiFi.softAPgetStationNum() > 0;
}

bool wifiBlocksDrive() {
  return wifiClientConnected() && engineSwitchOn();
}

uint32_t crsfRcFrameAgeMs() {
  if (lastCrsfRcFrameMs == 0) return 0;
  return millis() - lastCrsfRcFrameMs;
}

uint32_t elrsFailsafeActiveMs() {
  if (!failsafeActive() || elrsFailsafeStartMs == 0) return 0;
  return millis() - elrsFailsafeStartMs;
}

void updateElrsFailsafeDiagnostics() {
  const bool active = failsafeActive();
  const uint32_t now = millis();

  if (active && !lastFailsafeState) {
    elrsFailsafeStartMs = now;
    if (lastCrsfRcFrameMs != 0) {
      elrsFailsafeCount++;
      escFailsafeRecoveryLocked = true;
    }
  } else if (!active && lastFailsafeState) {
    if (elrsFailsafeStartMs != 0) {
      elrsFailsafeLastDurationMs = now - elrsFailsafeStartMs;
    }
    elrsFailsafeStartMs = 0;
  }

  lastFailsafeState = active;
}

void updateWifiArmingLockout() {
  if (!engineSwitchOn()) {
    wifiArmingLockout = false;
    return;
  }
  if (wifiClientConnected()) {
    wifiArmingLockout = true;
  }
}

bool o4MspLinkActive() {
  if (lastMspRequestMs == 0) return false;
  return millis() - lastMspRequestMs <= O4_MSP_LINK_TIMEOUT_MS;
}

uint32_t o4MspLinkAgeMs() {
  if (lastMspRequestMs == 0) return 0;
  return millis() - lastMspRequestMs;
}

bool djiShouldReportArmed() {
  return djiArmedToO4 && (engineArmed() || (failsafeActive() && escNeutralGateSatisfied));
}

void updateDriveTimer() {
  const uint32_t now = millis();

  if (engineArmed() && !driveTimerSessionActive) {
    driveTimerSessionActive = true;
    driveTimerStartMs = now;
    driveTimerElapsedMs = 0;
  }

  if (!driveTimerSessionActive) return;

  driveTimerElapsedMs = now - driveTimerStartMs;

  // A lost ELRS link does not end the armed session. The timer stops only
  // after the link returns and CH5 is deliberately switched off.
  if (crsfLinkActive() && !engineSwitchOn()) {
    driveTimerSessionActive = false;
  }
}

void resetDriveStatsSession() {
  driveStatsLastSampleMs = 0;
  driveStatsSpeedSampleCount = 0;
  driveStatsSpeedSumKmh = 0;
  driveStatsMaxSpeedKmh = 0;
  driveStatsMaxSats = 0;
  driveStatsMaxRollAbsDeg = 0.0f;
  driveStatsMaxPitchAbsDeg = 0.0f;
  driveStatsLastGpsValid = false;
  driveStatsLastLatDeg = 0.0;
  driveStatsLastLonDeg = 0.0;
  driveStatsTripDistanceCm = 0;
}

void updateDriveStats() {
  const uint32_t now = millis();

  if (driveTimerSessionActive) {
    if (!driveStatsSessionActive) {
      driveStatsSessionActive = true;
      driveStatsDisplayActive = false;
      resetDriveStatsSession();
    }

    if (now - driveStatsLastSampleMs >= DRIVE_STATS_SAMPLE_INTERVAL_MS) {
      driveStatsLastSampleMs = now;
      const uint16_t speedKmh = (uint16_t)lround(gpsSpeedCms * 0.036);
      if (gpsFix) {
        driveStatsSpeedSumKmh += speedKmh;
        driveStatsSpeedSampleCount++;
        if (speedKmh > driveStatsMaxSpeedKmh) driveStatsMaxSpeedKmh = speedKmh;
        if (currentGpsPositionValid) {
          if (driveStatsLastGpsValid) {
            const double stepMeters = distanceMetersBetween(driveStatsLastLatDeg, driveStatsLastLonDeg, currentGpsLatDeg, currentGpsLonDeg);
            if (stepMeters >= (DRIVE_STATS_TRIP_MIN_STEP_CM / 100.0) && stepMeters <= (DRIVE_STATS_TRIP_MAX_STEP_CM / 100.0)) {
              driveStatsTripDistanceCm += (uint32_t)lround(stepMeters * 100.0);
            }
          }
          driveStatsLastLatDeg = currentGpsLatDeg;
          driveStatsLastLonDeg = currentGpsLonDeg;
          driveStatsLastGpsValid = true;
        }
      }
      if (gpsSats > driveStatsMaxSats) driveStatsMaxSats = gpsSats;
      if (imuDataValid) {
        driveStatsMaxRollAbsDeg = max(driveStatsMaxRollAbsDeg, fabsf(displayedImuRollDeg()));
        driveStatsMaxPitchAbsDeg = max(driveStatsMaxPitchAbsDeg, fabsf(displayedImuPitchDeg()));
      }
    }
    return;
  }

  if (driveStatsSessionActive && crsfLinkActive() && !engineSwitchOn() && driveTimerElapsedMs >= DRIVE_STATS_MIN_DRIVE_MS) {
    driveStatsLastDurationMs = driveTimerElapsedMs;
    driveStatsLastAvgSpeedKmh = driveStatsSpeedSampleCount ? (uint16_t)((driveStatsSpeedSumKmh + driveStatsSpeedSampleCount / 2) / driveStatsSpeedSampleCount) : 0;
    driveStatsLastMaxSpeedKmh = driveStatsMaxSpeedKmh;
    driveStatsLastTripMeters = (uint16_t)constrain((uint32_t)((driveStatsTripDistanceCm + 50) / 100), 0UL, 65535UL);
    driveStatsLastMaxSats = driveStatsMaxSats;
    driveStatsLastMaxRollDeg = (uint16_t)lroundf(driveStatsMaxRollAbsDeg);
    driveStatsLastMaxPitchDeg = (uint16_t)lroundf(driveStatsMaxPitchAbsDeg);
    driveStatsDisplayActive = true;
    driveStatsDisplayStartMs = now;
  }

  if (crsfLinkActive() && !engineSwitchOn()) {
    driveStatsSessionActive = false;
  }

  if (driveStatsDisplayActive && (failsafeActive() || now - driveStatsDisplayStartMs >= DRIVE_STATS_DISPLAY_MS)) {
    driveStatsDisplayActive = false;
  }
}

uint32_t currentDriveTimeMs() {
  return driveTimerElapsedMs;
}

bool throttleCenteredForEscStart() {
  return rcChannels[1] >= escNeutralLowUs && rcChannels[1] <= escNeutralHighUs;
}

bool throttleMovedFromCenter() {
  return abs((int)rcChannels[1] - (int)ESC_PWM_NEUTRAL_US) > ENGINE_NOT_READY_THROTTLE_DELTA_US;
}

bool escPulseIsNeutral(uint16_t pulseUs) {
  return pulseUs >= escNeutralLowUs && pulseUs <= escNeutralHighUs;
}

bool escPulseIsReverse(uint16_t pulseUs) {
  return throttleOutputReversed ? pulseUs > escNeutralHighUs : pulseUs < escNeutralLowUs;
}

bool escPulseIsForward(uint16_t pulseUs) {
  return throttleOutputReversed ? pulseUs < escNeutralLowUs : pulseUs > escNeutralHighUs;
}

void resetEscReverseAssist() {
  escReverseDelayActive = false;
  escReverseDelayStartMs = 0;
}

uint16_t reversedPulseUs(uint16_t pulseUs, uint16_t minUs, uint16_t maxUs) {
  const uint16_t safePulseUs = constrain(pulseUs, minUs, maxUs);
  return minUs + maxUs - safePulseUs;
}

uint16_t steeringOutputPulseUs() {
  uint16_t pulseUs = constrain(rcChannels[0], STEERING_PWM_MIN_US, STEERING_PWM_MAX_US);
  if (steeringOutputReversed) {
    pulseUs = reversedPulseUs(pulseUs, STEERING_PWM_MIN_US, STEERING_PWM_MAX_US);
  }
  const int32_t trimmedPulseUs = (int32_t)pulseUs + steeringTrimUs;
  return constrain(trimmedPulseUs, steeringMinUs, steeringMaxUs);
}

uint16_t scaledHeadServoPulseUs(uint16_t channelUs, bool reversed, int16_t trimUs, uint8_t scalePercent, uint16_t minUs, uint16_t maxUs) {
  const int32_t rawDeltaUs = (int32_t)constrain(channelUs, 1000, 2000) - HEAD_SERVO_CENTER_US;
  int32_t scaledDeltaUs = (rawDeltaUs * scalePercent) / 100;
  if (reversed) scaledDeltaUs = -scaledDeltaUs;
  return constrain((int32_t)HEAD_SERVO_CENTER_US + scaledDeltaUs + trimUs, minUs, maxUs);
}

uint16_t headPanOutputPulseUs() {
  return scaledHeadServoPulseUs(rcChannels[3], headPanReversed, headPanTrimUs, headPanScalePercent, headPanMinUs, headPanMaxUs);
}

uint16_t headTiltOutputPulseUs() {
  return scaledHeadServoPulseUs(rcChannels[2], headTiltReversed, headTiltTrimUs, headTiltScalePercent, headTiltMinUs, headTiltMaxUs);
}

bool driveModeCrawlActive() {
  if (!crsfLinkActive()) return false;
  return rcChannels[DRIVE_MODE_CHANNEL_INDEX] < DRIVE_MODE_THRESHOLD_US;
}

uint8_t driveModeThrottlePercent() {
  return driveModeCrawlActive() ? DRIVE_MODE_CRAWL_THROTTLE_PERCENT : DRIVE_MODE_TRAIL_THROTTLE_PERCENT;
}

const char *driveModeName() {
  return driveModeCrawlActive() ? "CRAWL" : "TRAIL";
}

uint16_t throttleOutputPulseUs() {
  uint16_t pulseUs = constrain(rcChannels[1], ESC_PWM_MIN_US, ESC_PWM_MAX_US);
  if (throttleOutputReversed) {
    pulseUs = reversedPulseUs(pulseUs, ESC_PWM_MIN_US, ESC_PWM_MAX_US);
  }
  const int32_t cappedDeltaUs = ((int32_t)pulseUs - ESC_PWM_NEUTRAL_US) * driveModeThrottlePercent() / 100;
  pulseUs = constrain((int32_t)ESC_PWM_NEUTRAL_US + cappedDeltaUs, ESC_PWM_MIN_US, ESC_PWM_MAX_US);
  return constrain(pulseUs, escMinUs, escMaxUs);
}

void attachSteeringPwm() {
  if (steeringPwmAttached) return;

  pinMode(STEERING_PWM_PIN, OUTPUT);
  digitalWrite(STEERING_PWM_PIN, LOW);
  steeringPulseHigh = false;
  steeringFrameStartUs = micros() - SERVO_PWM_FRAME_US;
  steeringPwmAttached = true;
}

void detachSteeringPwm() {
  digitalWrite(STEERING_PWM_PIN, LOW);
  pinMode(STEERING_PWM_PIN, INPUT);
  steeringPulseHigh = false;
  steeringPwmAttached = false;
}

void writeSteeringPulseUs(uint16_t pulseUs) {
  steeringPulseUs = constrain(pulseUs, steeringMinUs, steeringMaxUs);
  attachSteeringPwm();
}

void updateSteeringPwm() {
  if (crsfLinkActive()) {
    writeSteeringPulseUs(steeringOutputPulseUs());
    steeringOutputLive = true;
  } else {
    detachSteeringPwm();
    steeringOutputLive = false;
  }
}

void attachHeadPanPwm() {
  if (headPanPwmAttached) return;
  pinMode(HEAD_PAN_PWM_PIN, OUTPUT);
  digitalWrite(HEAD_PAN_PWM_PIN, LOW);
  headPanPwmAttached = true;
}

void detachHeadPanPwm() {
  digitalWrite(HEAD_PAN_PWM_PIN, LOW);
  pinMode(HEAD_PAN_PWM_PIN, INPUT);
  headPanPwmAttached = false;
}

void attachHeadTiltPwm() {
  if (headTiltPwmAttached) return;
  pinMode(HEAD_TILT_PWM_PIN, OUTPUT);
  digitalWrite(HEAD_TILT_PWM_PIN, LOW);
  headTiltPwmAttached = true;
}

void detachHeadTiltPwm() {
  digitalWrite(HEAD_TILT_PWM_PIN, LOW);
  pinMode(HEAD_TILT_PWM_PIN, INPUT);
  headTiltPwmAttached = false;
}

void updateHeadServoPwm() {
  if (crsfLinkActive() && headPanEnabled) {
    headPanPulseUs = headPanOutputPulseUs();
    attachHeadPanPwm();
    headPanOutputLive = true;
  } else {
    detachHeadPanPwm();
    headPanOutputLive = false;
  }

  if (crsfLinkActive() && headTiltEnabled) {
    headTiltPulseUs = headTiltOutputPulseUs();
    attachHeadTiltPwm();
    headTiltOutputLive = true;
  } else {
    detachHeadTiltPwm();
    headTiltOutputLive = false;
  }
}

void attachEscPwm() {
  if (escPwmAttached) return;

  pinMode(ESC_PWM_PIN, OUTPUT);
  digitalWrite(ESC_PWM_PIN, LOW);
  escPulseHigh = false;
  escFrameStartUs = micros() - SERVO_PWM_FRAME_US;
  escPwmAttached = true;
}

void detachEscPwm() {
  digitalWrite(ESC_PWM_PIN, LOW);
  pinMode(ESC_PWM_PIN, INPUT);
  escPulseHigh = false;
  escPwmAttached = false;
}

void writeEscPulseUs(uint16_t pulseUs) {
  escPulseUs = constrain(pulseUs, escMinUs, escMaxUs);
  attachEscPwm();
}

void servicePwmOutputs() {
  static uint32_t frameStartUs = 0;
  if (!steeringPwmAttached && !escPwmAttached && !headPanPwmAttached && !headTiltPwmAttached) {
    frameStartUs = micros();
    return;
  }

  const uint32_t nowUs = micros();
  if ((uint32_t)(nowUs - frameStartUs) < SERVO_PWM_FRAME_US) {
    return;
  }
  frameStartUs = nowUs;

  const uint8_t pins[] = { STEERING_PWM_PIN, ESC_PWM_PIN, HEAD_PAN_PWM_PIN, HEAD_TILT_PWM_PIN };
  const bool active[] = { steeringPwmAttached, escPwmAttached, headPanPwmAttached, headTiltPwmAttached };
  const uint16_t widths[] = { steeringPulseUs, escPulseUs, headPanPulseUs, headTiltPulseUs };
  uint16_t sortedWidths[4];
  uint8_t sortedCount = 0;

  for (uint8_t i = 0; i < 4; i++) {
    if (!active[i]) continue;
    digitalWrite(pins[i], HIGH);
    sortedWidths[sortedCount++] = widths[i];
  }

  for (uint8_t i = 0; i < sortedCount; i++) {
    for (uint8_t j = i + 1; j < sortedCount; j++) {
      if (sortedWidths[j] < sortedWidths[i]) {
        const uint16_t tmp = sortedWidths[i];
        sortedWidths[i] = sortedWidths[j];
        sortedWidths[j] = tmp;
      }
    }
  }

  uint16_t elapsedPulseUs = 0;
  for (uint8_t step = 0; step < sortedCount; step++) {
    const uint16_t targetUs = sortedWidths[step];
    if (targetUs > elapsedPulseUs) {
      delayMicroseconds(targetUs - elapsedPulseUs);
      elapsedPulseUs = targetUs;
    }
    for (uint8_t i = 0; i < 4; i++) {
      if (active[i] && widths[i] == targetUs) {
        digitalWrite(pins[i], LOW);
      }
    }
  }
}

void updateEscPwm() {
  const bool armed = engineArmed();
  const bool wifiBlocked = wifiBlocksDrive() || wifiArmingLockout;
  bool nextOutputLive = false;

  if (batteryFuelEmptyActive && escSawDisarmedSinceBoot && escNeutralGateSatisfied) {
    resetEscReverseAssist();
    writeEscPulseUs(ESC_PWM_NEUTRAL_US);
    nextOutputLive = true;
    engineReadyBlinkActive = false;
  } else if (batteryFuelEmptyActive) {
    resetEscReverseAssist();
    detachEscPwm();
    engineReadyBlinkActive = false;
  } else if (failsafeActive() && escNeutralGateSatisfied) {
    resetEscReverseAssist();
    writeEscPulseUs(ESC_PWM_NEUTRAL_US);
    nextOutputLive = true;
    engineReadyBlinkActive = false;
  } else if (wifiBlocked && escSawDisarmedSinceBoot && escNeutralGateSatisfied) {
    resetEscReverseAssist();
    writeEscPulseUs(ESC_PWM_NEUTRAL_US);
    nextOutputLive = true;
    engineReadyBlinkActive = false;
  } else if (wifiBlocked) {
    resetEscReverseAssist();
    detachEscPwm();
    engineReadyBlinkActive = false;
  } else if (!armed) {
    escSawDisarmedSinceBoot = true;
    escNeutralGateSatisfied = false;
    escFailsafeRecoveryLocked = false;
    resetEscReverseAssist();
    engineReadyBlinkActive = false;
    detachEscPwm();
  } else if (!escSawDisarmedSinceBoot) {
    resetEscReverseAssist();
    detachEscPwm();
  } else {
    if (!escNeutralGateSatisfied && throttleCenteredForEscStart()) {
      escNeutralGateSatisfied = true;
    }

    if (escFailsafeRecoveryLocked) {
      if (throttleCenteredForEscStart()) {
        escFailsafeRecoveryLocked = false;
      } else if (escNeutralGateSatisfied) {
        resetEscReverseAssist();
        writeEscPulseUs(ESC_PWM_NEUTRAL_US);
        nextOutputLive = true;
      } else {
        resetEscReverseAssist();
        detachEscPwm();
      }
    }

    if (!escFailsafeRecoveryLocked && escNeutralGateSatisfied) {
      const uint16_t desiredPulseUs = throttleOutputPulseUs();

      if (escReverseAssistEnabled && escPulseIsReverse(desiredPulseUs)) {
        if (!escReverseDelayActive) {
          escReverseDelayActive = true;
          escReverseDelayStartMs = millis();
        }

        if (millis() - escReverseDelayStartMs < escReverseDelayMs) {
          writeEscPulseUs(ESC_PWM_NEUTRAL_US);
        } else {
          writeEscPulseUs(desiredPulseUs);
        }
      } else {
        resetEscReverseAssist();
        writeEscPulseUs(desiredPulseUs);
      }

      nextOutputLive = true;
    } else if (!escFailsafeRecoveryLocked) {
      resetEscReverseAssist();
      detachEscPwm();
    }
  }

  if (nextOutputLive && !escOutputLive) {
    engineReadyBlinkActive = true;
    engineReadyBlinkStartMs = millis();
  }
  escOutputLive = nextOutputLive;
}

void handleCrsfRcChannels(const uint8_t *payload, uint8_t payloadLen) {
  if (payloadLen < 22) return;

  for (uint8_t i = 0; i < 8; i++) {
    rcChannels[i] = crsfChannelToPwm(unpackCrsfChannel(payload, i));
  }

  lastCrsfRcFrameMs = millis();
  crsfRcFrameCount++;
}

void handleCrsfLinkStatistics(const uint8_t *payload, uint8_t payloadLen) {
  if (payloadLen >= 3) {
    crsfLinkQuality = constrain(payload[2], 0, 100);
  }
  if (payloadLen >= 4) {
    crsfSnr = (int8_t)payload[3];
  }

  crsfLinkFrameCount++;
}

void handleCrsfFrame(const uint8_t *frame, uint8_t frameSize) {
  if (frameSize < 2) return;

  uint8_t crc = 0;
  for (uint8_t i = 0; i < frameSize - 1; i++) {
    crc = crc8DvbS2(crc, frame[i]);
  }

  if (crc != frame[frameSize - 1]) {
    crsfBadCrcCount++;
    return;
  }

  const uint8_t frameType = frame[0];
  const uint8_t *payload = &frame[1];
  const uint8_t payloadLen = frameSize - 2;

  crsfFrameCount++;
  lastCrsfFrameMs = millis();

  if (frameType == CRSF_FRAMETYPE_RC_CHANNELS_PACKED) {
    handleCrsfRcChannels(payload, payloadLen);
  } else if (frameType == CRSF_FRAMETYPE_LINK_STATISTICS) {
    handleCrsfLinkStatistics(payload, payloadLen);
  }
}

void parseCrsfByte(uint8_t b) {
  switch (crsfParserState) {
    case CRSF_WAIT_ADDR:
      if (b == CRSF_ADDRESS_FLIGHT_CONTROLLER) {
        crsfParserState = CRSF_WAIT_LEN;
      }
      break;

    case CRSF_WAIT_LEN:
      if (b >= 2 && b <= CRSF_MAX_FRAME_SIZE) {
        crsfFrameSize = b;
        crsfFrameIndex = 0;
        crsfParserState = CRSF_WAIT_FRAME;
      } else {
        crsfParserState = CRSF_WAIT_ADDR;
      }
      break;

    case CRSF_WAIT_FRAME:
      crsfFrame[crsfFrameIndex++] = b;
      if (crsfFrameIndex >= crsfFrameSize) {
        handleCrsfFrame(crsfFrame, crsfFrameSize);
        crsfParserState = CRSF_WAIT_ADDR;
      }
      break;
  }
}

void sendMspV2Reply(uint16_t cmd, const uint8_t *payload, uint16_t size) {
  if (!sendMspReplies) return;

  uint8_t crc = 0;
  const uint8_t flags = 0;

  DJISerial.write('$');
  DJISerial.write('X');
  DJISerial.write('>');
  replyByteCount += 3;

  DJISerial.write(flags);
  crc = crc8DvbS2(crc, flags);
  replyByteCount++;

  DJISerial.write(cmd & 0xFF);
  crc = crc8DvbS2(crc, cmd & 0xFF);
  DJISerial.write((cmd >> 8) & 0xFF);
  crc = crc8DvbS2(crc, (cmd >> 8) & 0xFF);
  replyByteCount += 2;

  DJISerial.write(size & 0xFF);
  crc = crc8DvbS2(crc, size & 0xFF);
  DJISerial.write((size >> 8) & 0xFF);
  crc = crc8DvbS2(crc, (size >> 8) & 0xFF);
  replyByteCount += 2;

  for (uint16_t i = 0; i < size; i++) {
    DJISerial.write(payload[i]);
    crc = crc8DvbS2(crc, payload[i]);
    replyByteCount++;
  }

  DJISerial.write(crc);
  replyByteCount++;
  replyPacketCount++;
}

void sendEmptyReply(uint8_t cmd) {
  sendMspReply(cmd, nullptr, 0);
}

void sendDisplayPortPayload(const uint8_t *payload, uint8_t size) {
  sendMspV1PacketWithDirection('>', (uint8_t)MSP_DISPLAYPORT, payload, size);
  if (DISPLAYPORT_SEND_BOTH_DIRECTIONS) {
    sendMspV1PacketWithDirection('<', (uint8_t)MSP_DISPLAYPORT, payload, size);
  }
  displayPortFramesSent++;
}

void dpHeartbeat() {
  const uint8_t payload[] = { MSP_DP_HEARTBEAT };
  sendDisplayPortPayload(payload, sizeof(payload));
}

void dpClearScreen() {
  const uint8_t payload[] = { MSP_DP_CLEAR_SCREEN };
  sendDisplayPortPayload(payload, sizeof(payload));
}

void dpDrawScreen() {
  const uint8_t payload[] = { MSP_DP_DRAW_SCREEN };
  sendDisplayPortPayload(payload, sizeof(payload));
}

void dpWriteString(uint8_t row, uint8_t col, const char *text) {
  uint8_t payload[34];
  uint8_t len = 0;

  payload[len++] = MSP_DP_WRITE_STRING;
  payload[len++] = row;
  payload[len++] = col;
  payload[len++] = 0; // normal font / normal attribute

  while (*text && len < sizeof(payload)) {
    payload[len++] = (uint8_t)*text++;
  }

  sendDisplayPortPayload(payload, len);
}

void sendVtxMspProbeFrame() {
  uint8_t payload[15];
  payload[0] = 4;      // VTXDEV_MSP in Betaflight
  payload[1] = 1;      // band index
  payload[2] = 1;      // channel index
  payload[3] = 1;      // power index
  payload[4] = 0;      // pit mode
  payload[5] = 0xE1;   // 5865 MHz low byte
  payload[6] = 0x16;   // 5865 MHz high byte
  payload[7] = 1;      // ready
  payload[8] = 0;      // low power disarm
  payload[9] = 0;      // pit mode freq low
  payload[10] = 0;     // pit mode freq high
  payload[11] = 1;     // vtx table available
  payload[12] = 5;     // bands
  payload[13] = 8;     // channels
  payload[14] = 3;     // power levels

  sendMspV2Reply(MSP_VTX_CONFIG, payload, sizeof(payload));
  vtxMspProbeCount++;
}

int channelPercent(uint16_t pwm, bool centerBased) {
  if (centerBased) {
    return constrain((int)lround(((int)pwm - 1500) / 5.0), -100, 100);
  }
  return constrain((int)lround(((int)pwm - 1000) / 10.0), 0, 100);
}

int logicalThrottlePercent() {
  int percent = channelPercent(rcChannels[1], true);
  return throttleOutputReversed ? -percent : percent;
}

void resetImuCalibrationGesture() {
  imuGestureState = IMU_GESTURE_IDLE;
  imuGestureHoldStartMs = 0;
}

bool imuGestureConditionHeld(bool condition, uint16_t holdMs) {
  const uint32_t now = millis();
  if (!condition) {
    imuGestureHoldStartMs = 0;
    return false;
  }
  if (imuGestureHoldStartMs == 0) imuGestureHoldStartMs = now;
  return now - imuGestureHoldStartMs >= holdMs;
}

void updateImuCalibrationGesture() {
  const int throttlePercent = logicalThrottlePercent();
  const bool reverseHeld = throttlePercent <= -(int)IMU_GESTURE_EXTREME_PERCENT;
  const bool safe = imuDataValid && crsfLinkActive() && !engineSwitchOn();

  if (!safe || !reverseHeld) {
    resetImuCalibrationGesture();
    return;
  }

  if (imuGestureState == IMU_GESTURE_DONE) {
    return;
  }

  if (imuGestureState == IMU_GESTURE_IDLE) {
    imuGestureState = IMU_GESTURE_REVERSE_HOLD;
  }

  if (imuGestureConditionHeld(reverseHeld, IMU_GESTURE_REVERSE_HOLD_MS)) {
    if (calibrateImuCurrentLevel()) {
      imuGestureState = IMU_GESTURE_DONE;
      imuGestureHoldStartMs = 0;
    } else {
      resetImuCalibrationGesture();
    }
  }
}

void resetHomePointGesture() {
  homePointGestureHoldStartMs = 0;
}

void updateHomePointResetGesture() {
  const int throttlePercent = logicalThrottlePercent();
  const bool forwardHeld = throttlePercent >= (int)IMU_GESTURE_EXTREME_PERCENT;
  const bool safe = crsfLinkActive() && !engineSwitchOn() && gpsHomePointEligible();

  if (!forwardHeld) {
    homePointGestureDone = false;
  }

  if (!safe || !forwardHeld) {
    resetHomePointGesture();
    return;
  }

  if (homePointGestureDone) {
    return;
  }

  const uint32_t now = millis();
  if (homePointGestureHoldStartMs == 0) {
    homePointGestureHoldStartMs = now;
    return;
  }

  if (now - homePointGestureHoldStartMs >= HOME_POINT_GESTURE_HOLD_MS) {
    if (setHomePoint(currentGpsLatDeg, currentGpsLonDeg, true)) {
      homePointGestureDone = true;
      homePointGestureHoldStartMs = 0;
    }
  }
}

uint16_t djiSensorMask() {
  if (DJI_NATIVE_GPS_HOME_ENABLED) {
    return MSP_SENSOR_MASK_FAKE_FC;
  }

  return MSP_SENSOR_MASK_DJI_MINIMAL;
}

bool usbDebugReady() {
  return USB_DIAGNOSTIC_MODE && usbDebugEnabled && Serial && Serial.availableForWrite() >= 64;
}

uint8_t centerCol(const char *text) {
  const size_t len = strlen(text);
  return (len >= OSD_HD_COLS_FAKE) ? 0 : (uint8_t)((OSD_HD_COLS_FAKE - len) / 2);
}

uint8_t rightCol(const char *text) {
  const size_t len = strlen(text);
  return (len >= OSD_HD_COLS_FAKE) ? 0 : (uint8_t)(OSD_HD_COLS_FAKE - len);
}

void formatGpsCoord(char *out, size_t outSize, uint8_t symbol, int32_t coord) {
  if (!gpsFix) {
    snprintf(out, outSize, "%c --", symbol);
    return;
  }

  const char sign = (coord < 0) ? '-' : ' ';
  const uint32_t absCoord = (uint32_t)labs(coord);
  const uint32_t whole = absCoord / 10000000UL;
  const uint32_t frac = (absCoord % 10000000UL) / 100UL;
  snprintf(out, outSize, "%c%c%lu.%05lu", symbol, sign, (unsigned long)whole, (unsigned long)frac);
}

uint8_t signedArrowSymbol(int value, uint8_t positiveSymbol, uint8_t negativeSymbol) {
  if (value > 0) return positiveSymbol;
  if (value < 0) return negativeSymbol;
  return '-';
}

void sendDisplayPortTestFrame() {
  char line[32];
  char statLine[48];
  const int steering = channelPercent(rcChannels[0], true);
  const int throttle = channelPercent(rcChannels[1], true);
  const int speedKmh = (int)lround(gpsSpeedCms * 0.036);
  const uint16_t osdLinkQuality = crsfLinkActive() ? crsfLinkQuality : 0;
  const bool armed = engineArmed();
  const uint32_t now = millis();

  dpHeartbeat();
  dpClearScreen();
  displayPortClearRequested = false;

  if (driveStatsDisplayActive && !armed && !failsafeActive()) {
    const uint32_t statSeconds = driveStatsLastDurationMs / 1000;
    const uint32_t statMinutes = statSeconds / 60;
    const uint32_t statRemainSeconds = statSeconds % 60;
    const uint8_t statCol = 5;

    dpWriteString(2, centerCol("DRIVE STATS"), "DRIVE STATS");

    snprintf(statLine, sizeof(statLine), "DRIVE TIME        : %lu MINUTES %02lu SECONDS",
             (unsigned long)statMinutes,
             (unsigned long)statRemainSeconds);
    dpWriteString(5, statCol, statLine);

    snprintf(statLine, sizeof(statLine), "AVG SPEED         : %u KMH", driveStatsLastAvgSpeedKmh);
    dpWriteString(6, statCol, statLine);

    snprintf(statLine, sizeof(statLine), "MAX SPEED         : %u KMH", driveStatsLastMaxSpeedKmh);
    dpWriteString(7, statCol, statLine);

    snprintf(statLine, sizeof(statLine), "TRIP DISTANCE     : %u METERS", driveStatsLastTripMeters);
    dpWriteString(8, statCol, statLine);

    snprintf(statLine, sizeof(statLine), "MAX SATELLITE     : %u SATELLITES", driveStatsLastMaxSats);
    dpWriteString(9, statCol, statLine);

    snprintf(statLine, sizeof(statLine), "MAX ROLL          : %u DEGREE", driveStatsLastMaxRollDeg);
    dpWriteString(10, statCol, statLine);

    snprintf(statLine, sizeof(statLine), "MAX PITCH         : %u DEGREE", driveStatsLastMaxPitchDeg);
    dpWriteString(11, statCol, statLine);

    dpDrawScreen();
    return;
  }

  if (osdShowCraft) {
    dpWriteString(0, centerCol(craftName), craftName);
  }
  if (osdShowDriveMode) {
    dpWriteString(1, centerCol(driveModeName()), driveModeName());
  }

  if (osdShowCoords) {
    formatGpsCoord(line, sizeof(line), SYM_LAT, gpsLat);
    dpWriteString(0, 1, line);

    formatGpsCoord(line, sizeof(line), SYM_LON, gpsLon);
    dpWriteString(0, rightCol(line), line);
  }

  if (osdShowGps) {
    snprintf(line, sizeof(line), "%c%c %u", SYM_SAT_L, SYM_SAT_R, gpsSats);
    dpWriteString(10, 1, line);
    snprintf(line, sizeof(line), "%c %d%c", SYM_SPEED, speedKmh, SYM_KPH);
    dpWriteString(11, 1, line);
  }
  if (osdShowWifi && wifiConfigPortalStarted) {
    dpWriteString(10, rightCol("WIFI ON"), "WIFI ON");
  }

  if (osdShowImu && imuDataValid) {
    snprintf(line, sizeof(line), "ROL %+d", (int)lroundf(displayedImuRollDeg()));
    dpWriteString(11, rightCol(line), line);
    snprintf(line, sizeof(line), "PIT %+d", (int)lroundf(displayedImuPitchDeg()));
    dpWriteString(12, rightCol(line), line);
  }

  if (osdShowDriveTime) {
    const uint32_t driveSeconds = currentDriveTimeMs() / 1000;
    if (driveSeconds < 3600) {
      snprintf(line, sizeof(line), "DRV %02lu:%02lu",
               (unsigned long)(driveSeconds / 60),
               (unsigned long)(driveSeconds % 60));
    } else {
      snprintf(line, sizeof(line), "DRV %lu:%02lu:%02lu",
               (unsigned long)(driveSeconds / 3600),
               (unsigned long)((driveSeconds / 60) % 60),
               (unsigned long)(driveSeconds % 60));
    }
    dpWriteString(13, rightCol(line), line);
  }

  if (osdShowBattery) {
    if (batteryVoltageValid) {
      const uint16_t cellVoltage_cV = batteryAverageCellVoltageCv();
      snprintf(line, sizeof(line), "%c %u.%02u%c", SYM_BATT_FULL, cellVoltage_cV / 100, cellVoltage_cV % 100, SYM_VOLT);
    } else {
      snprintf(line, sizeof(line), "%c -", SYM_BATT_FULL);
    }
    dpWriteString(13, 1, line);
  }

  if (osdShowLinkQuality) {
    snprintf(line, sizeof(line), "%c %u%%", SYM_LINK_QUALITY, osdLinkQuality);
    dpWriteString(14, 1, line);
  }

  if (imuCalibrationNoticeActive()) {
    const uint32_t noticeElapsedMs = now - imuCalibrationNoticeStartMs;
    if ((noticeElapsedMs / 250) % 2 == 0) {
      dpWriteString(9, centerCol("IMU CALIBRATED"), "IMU CALIBRATED");
    }
  } else if (failsafeActive()) {
    dpWriteString(9, centerCol("ELRS FAILSAFE"), "ELRS FAILSAFE");
  } else if (wifiArmingLockout) {
    if ((now / 250) % 2 == 0) {
      dpWriteString(9, centerCol("WIFI STILL CONNECTED"), "WIFI STILL CONNECTED");
    }
  } else if (batteryFuelEmptyActive) {
    if ((now / 250) % 2 == 0) {
      dpWriteString(9, centerCol("FUEL EMPTY"), "FUEL EMPTY");
    }
  } else if (batteryWarningActive) {
    if ((now / 250) % 2 == 0) {
      dpWriteString(9, centerCol("RETURN NOW"), "RETURN NOW");
    }
  } else if (homePointNoticeActive()) {
    const uint32_t noticeElapsedMs = now - homePointNoticeStartMs;
    if ((noticeElapsedMs / 250) % 2 == 0) {
      dpWriteString(9, centerCol("HOME POINT UPDATED"), "HOME POINT UPDATED");
    }
  } else if (!armed) {
    if (crsfLinkActive() && throttleMovedFromCenter()) {
      if ((now / 250) % 2 == 0) {
        dpWriteString(9, centerCol("START FIRST"), "START FIRST");
      }
    } else {
      dpWriteString(9, centerCol("STANDBY"), "STANDBY");
    }
  } else if (!escSawDisarmedSinceBoot) {
    dpWriteString(9, centerCol("DISARM FIRST"), "DISARM FIRST");
  } else if (!escNeutralGateSatisfied) {
    dpWriteString(9, centerCol("GAS TO CENTER"), "GAS TO CENTER");
  } else if (engineReadyBlinkActive) {
    const uint32_t blinkElapsedMs = now - engineReadyBlinkStartMs;
    if (blinkElapsedMs < 1200) {
      const bool blinkVisible = blinkElapsedMs < 300 || (blinkElapsedMs >= 600 && blinkElapsedMs < 900);
      if (blinkVisible) {
        dpWriteString(9, centerCol("DRIVE"), "DRIVE");
      }
    } else {
      engineReadyBlinkActive = false;
    }
  }

  if (osdShowControls) {
    snprintf(line, sizeof(line), "STR %c %d%%", signedArrowSymbol(steering, SYM_ARROW_EAST, SYM_ARROW_WEST), abs(steering));
    dpWriteString(16, 1, line);
    dpWriteString(16, rightCol(FIRMWARE_VERSION), FIRMWARE_VERSION);

    snprintf(line, sizeof(line), "GAS %c %d%%", signedArrowSymbol(throttle, SYM_ARROW_NORTH, SYM_ARROW_SOUTH), abs(throttle));
    dpWriteString(17, 1, line);
  }

  if (osdShowHome) {
    snprintf(line, sizeof(line), "%c %u%c", SYM_HOMEFLAG, homeDistanceMeters, SYM_M);
    dpWriteString(18, centerCol(line), line);
  }

  dpDrawScreen();
}

void requestDisplayPortRegrab(const char *reason) {
  displayPortClearRequested = true;
  displayPortRegrabCount++;
  displayPortReacquireCount++;
  if (usbDebugReady()) {
    Serial.print("DisplayPort regrab: ");
    Serial.println(reason);
  }
}

void buildBetaflightLikeOsdConfig(Payload &p) {
  const uint8_t osdFlags =
      (1 << 0) |  // OSD feature enabled
      (1 << 5) |  // OSD device detected
      (1 << 6);   // MSP DisplayPort device

  p.u8(osdFlags);
  p.u8(VIDEO_SYSTEM_HD_FAKE);
  p.u8(0);     // units, metric
  p.u8(0);     // RSSI alarm
  p.u16(0);    // capacity alarm
  p.u8(0);     // old timer alarm low byte
  p.u8(0);     // OSD item count, no BF-managed elements
  p.u16(0);    // altitude alarm
  p.u8(0);     // OSD stat count
  p.u8(0);     // OSD timer count
  p.u16(0);                    // enabled warnings, low word
  p.u8(0);                     // OSD warning count
  p.u32(0);                    // enabled warnings, full word
  p.u8(1);                     // profile count
  p.u8(1);                     // active profile
}

void buildReply(uint8_t cmd, Payload &p) {
  p.clear();

  switch (cmd) {
    case MSP_API_VERSION:
      p.u8(0);   // MSP protocol version
      p.u8(1);   // API major
      p.u8(48);  // API minor
      break;

    case MSP_FC_VARIANT:
      p.bytes("BTFL", 4);
      break;

    case MSP_FC_VERSION:
      p.u8(4);
      p.u8(5);
      p.u8(0);
      break;

    case MSP_BOARD_INFO:
      p.bytes("RZG1", 4);
      p.u16(0);
      p.u8(0);
      break;

    case MSP_BUILD_INFO:
      p.bytes("Jun 11 2026", 11);
      p.bytes("18:00:00", 8);
      p.bytes("RZG0010", 7);
      break;

    case MSP_NAME:
      p.cstr(craftName);
      break;

    case MSP_STATUS:
      p.u16(1000);        // cycle time
      p.u16(0);           // i2c errors
      p.u16(djiSensorMask());
      p.u32(djiShouldReportArmed() ? 1 : 0);
      p.u8(0);            // current profile
      if (!DJI_NATIVE_CLASSIC_STATUS) {
        p.u16(5);           // average system load
        p.u16(1000);        // gyro cycle time
        p.u8(0);            // extra flight mode flag byte count
        p.u8(ARMING_DISABLE_FLAGS_COUNT_FAKE);
        p.u32(0);           // arming disabled flags
        p.u8(0);            // config state flags
        p.u16(0);           // CPU temperature, API >= 1.46
      }
      break;

    case MSP_STATUS_EX:
      p.u16(1000);        // cycle time
      p.u16(0);           // i2c errors
      p.u16(djiSensorMask());
      p.u32(djiShouldReportArmed() ? 1 : 0);
      p.u8(0);            // current profile
      p.u16(5);           // system load
      p.u8(3);            // PID profile count
      p.u8(0);            // rate profile
      p.u8(0);            // extra flight mode flag byte count
      p.u8(ARMING_DISABLE_FLAGS_COUNT_FAKE);
      p.u32(0);           // arming disabled flags
      p.u8(0);            // config state flags
      p.u16(0);           // CPU temperature, API >= 1.46
      break;

    case MSP_ANALOG:
      p.u8((uint8_t)batteryVoltage_dV);
      p.u16(mahDrawn);
      p.u16(rssi);
      p.u16(amperage_cA);
      p.u16(batteryVoltage_cV);
      break;

    case MSP_BATTERY_STATE:
      p.u8(batteryCellCount);   // configured RC pack cell count
      p.u16(0);                 // capacity mAh, unknown
      p.u8((uint8_t)batteryVoltage_dV);
      p.u16(mahDrawn);
      p.u16(amperage_cA);
      p.u8(0);                  // battery state
      p.u16(batteryVoltage_cV);
      break;

    case MSP_RAW_GPS:
      if (DJI_NATIVE_GPS_HOME_ENABLED) {
        p.u8(gpsFix);
        p.u8(gpsSats);
        p.i32(gpsLat);
        p.i32(gpsLon);
        p.i16(gpsAltMeters);
        p.u16(gpsSpeedCms);
        p.u16(gpsGroundCourseDeg10);
        p.u16(gpsFix ? 120 : 9999); // PDOP, added in MSP API 1.44
      } else {
        p.u8(0);
        p.u8(0);
        p.i32(0);
        p.i32(0);
        p.i16(0);
        p.u16(0);
        p.u16(0);
        p.u16(9999);
      }
      break;

    case MSP_COMP_GPS:
      if (DJI_NATIVE_GPS_HOME_ENABLED) {
        p.u16(homeDistanceMeters);
        p.i16(homeDirectionDeg);
        p.u8((gpsFix && homeSet) ? 1 : 0);
      } else {
        p.u16(0);
        p.i16(0);
        p.u8(0);
      }
      break;

    case MSP_ATTITUDE:
      p.i16(0); // roll, degrees * 10
      p.i16(0); // pitch, degrees * 10
      p.i16(0); // yaw, degrees
      break;

    case MSP_RC:
      for (uint8_t i = 0; i < 8; i++) p.u16(rcChannels[i]);
      break;

    case MSP_VTX_CONFIG:
      p.u8(0);     // unknown VTX device
      p.u8(0);     // band
      p.u8(0);     // channel
      p.u8(0);     // power
      p.u8(0);     // pit mode
      p.u16(0);    // frequency
      p.u8(0);     // device ready
      p.u8(0);     // low power disarm
      p.u16(0);    // pit mode frequency
      p.u8(0);     // VTX table available
      p.u8(0);     // bands
      p.u8(0);     // channels
      p.u8(0);     // power levels
      break;

    case MSP_FILTER_CONFIG:
      break;

    case MSP_PID_ADVANCED:
      break;

    case MSP_RC_TUNING:
      break;

    case MSP_PID:
    case MSP_FEATURE_CONFIG:
    case MSP_RX_CONFIG:
    case MSP_MODE_RANGES:
    case MSP_BOXIDS:
      break;

    case MSP_OSD_CONFIG:
      buildBetaflightLikeOsdConfig(p);
      break;

    case MSP_OSD_CANVAS:
      p.u8(OSD_HD_COLS_FAKE);
      p.u8(OSD_HD_ROWS_FAKE);
      break;

    default:
      break;
  }
}

bool isMspReplySupported(uint16_t cmd) {
  switch (cmd) {
    case MSP_API_VERSION:
    case MSP_FC_VARIANT:
    case MSP_FC_VERSION:
    case MSP_BOARD_INFO:
    case MSP_BUILD_INFO:
    case MSP_NAME:
    case MSP_STATUS:
    case MSP_STATUS_EX:
    case MSP_ANALOG:
    case MSP_BATTERY_STATE:
    case MSP_RAW_GPS:
    case MSP_COMP_GPS:
    case MSP_ATTITUDE:
    case MSP_RC:
    case MSP_VTX_CONFIG:
    case MSP_FILTER_CONFIG:
    case MSP_PID_ADVANCED:
    case MSP_RC_TUNING:
    case MSP_PID:
    case MSP_FEATURE_CONFIG:
    case MSP_RX_CONFIG:
    case MSP_MODE_RANGES:
    case MSP_BOXIDS:
    case MSP_OSD_CONFIG:
    case MSP_OSD_CANVAS:
      return true;
    default:
      return false;
  }
}

void handleMspRequest(uint16_t cmd, const uint8_t *payload, uint16_t size, bool mspV2) {
  (void)payload;
  (void)size;

  commandCount++;
  lastMspRequestMs = millis();
  if (cmd < 256) commandHistogram[cmd]++;

  if (VERBOSE_EVERY_REQUEST && usbDebugReady()) {
    Serial.print("REQ ");
    Serial.print(cmd);
    Serial.print(" ");
    Serial.println(commandName(cmd));
  }

  if (cmd == MSP_SET_OSD_CANVAS) {
    if (mspV2) {
      sendMspV2Reply(cmd, nullptr, 0);
    } else {
      sendEmptyReply((uint8_t)cmd);
    }
    return;
  }

  if (cmd == MSP_DISPLAYPORT) {
    displayPortRequestCount++;
    displayPortClearRequested = true;
    return;
  }

  if (!isMspReplySupported(cmd)) {
    return;
  }

  Payload reply;
  buildReply((uint8_t)cmd, reply);
  if (mspV2) {
    sendMspV2Reply(cmd, reply.data, reply.len);
  } else {
    sendMspReply((uint8_t)cmd, reply.data, reply.len);
  }
}

void sniffByteAfterDollar(uint8_t b) {
  if (sniffAfterDollar == 0) return;

  if (sniffIndex < sizeof(sniffBytes)) {
    sniffBytes[sniffIndex++] = b;
  }

  sniffAfterDollar--;
  if (sniffAfterDollar == 0 && sniffPrinted < 20 && usbDebugReady()) {
    sniffPrinted++;
    Serial.print("SNIF $");
    for (uint8_t i = 0; i < sniffIndex; i++) {
      Serial.print(" ");
      if (sniffBytes[i] < 16) Serial.print("0");
      Serial.print(sniffBytes[i], HEX);
    }
    Serial.println();
  }
}

void parseMspByte(uint8_t b) {
  sniffByteAfterDollar(b);

  if (rawSampleLen < sizeof(rawSample)) {
    rawSample[rawSampleLen++] = b;
  }

  if (lastByte2 == '$' && lastByte1 == 'M' && b == '<') {
    mspV1HeaderCount++;
  }
  if (lastByte2 == '$' && lastByte1 == 'X' && b == '<') {
    mspV2HeaderCount++;
  }
  lastByte2 = lastByte1;
  lastByte1 = b;

  switch (parserState) {
    case WAIT_DOLLAR:
      if (b == '$') {
        dollarCount++;
        sniffAfterDollar = sizeof(sniffBytes);
        sniffIndex = 0;
        parserState = WAIT_PROTO;
      }
      break;

    case WAIT_PROTO:
      if (b == 'M') {
        rxMspV2 = false;
        parserState = WAIT_DIR;
      } else if (b == 'X') {
        rxMspV2 = true;
        parserState = WAIT_DIR;
      } else {
        parserState = WAIT_DOLLAR;
      }
      break;

    case WAIT_DIR:
      if (b != '<') {
        parserState = WAIT_DOLLAR;
      } else if (rxMspV2) {
        parserState = WAIT_V2_FLAGS;
      } else {
        parserState = WAIT_SIZE;
      }
      break;

    case WAIT_V2_FLAGS:
      rxFlags = b;
      rxChecksum = crc8DvbS2(0, b);
      parserState = WAIT_V2_CMD_L;
      break;

    case WAIT_V2_CMD_L:
      rxCmd = b;
      rxChecksum = crc8DvbS2(rxChecksum, b);
      parserState = WAIT_V2_CMD_H;
      break;

    case WAIT_V2_CMD_H:
      rxCmd |= ((uint16_t)b << 8);
      rxChecksum = crc8DvbS2(rxChecksum, b);
      parserState = WAIT_V2_SIZE_L;
      break;

    case WAIT_V2_SIZE_L:
      rxSizeV2 = b;
      rxChecksum = crc8DvbS2(rxChecksum, b);
      parserState = WAIT_V2_SIZE_H;
      break;

    case WAIT_V2_SIZE_H:
      rxSizeV2 |= ((uint16_t)b << 8);
      rxChecksum = crc8DvbS2(rxChecksum, b);
      rxIndex = 0;
      if (rxSizeV2 > sizeof(rxPayload)) {
        parserState = WAIT_DOLLAR;
      } else {
        parserState = (rxSizeV2 == 0) ? WAIT_CHECKSUM : WAIT_PAYLOAD;
      }
      break;

    case WAIT_SIZE:
      rxSize = b;
      rxChecksum = b;
      rxIndex = 0;
      parserState = WAIT_CMD;
      break;

    case WAIT_CMD:
      rxCmd = b;
      rxChecksum ^= b;
      parserState = (rxSize == 0) ? WAIT_CHECKSUM : WAIT_PAYLOAD;
      break;

    case WAIT_PAYLOAD:
      if (rxIndex < sizeof(rxPayload)) {
        rxPayload[rxIndex++] = b;
        if (rxMspV2) {
          rxChecksum = crc8DvbS2(rxChecksum, b);
        } else {
          rxChecksum ^= b;
        }
      }
      if ((!rxMspV2 && rxIndex >= rxSize) || (rxMspV2 && rxIndex >= rxSizeV2)) parserState = WAIT_CHECKSUM;
      break;

    case WAIT_CHECKSUM:
      if (rxChecksum == b) {
        handleMspRequest(rxCmd, rxPayload, rxMspV2 ? rxSizeV2 : rxSize, rxMspV2);
      } else {
        badChecksumCount++;
        if (usbDebugReady()) {
          Serial.print("Bad checksum for cmd ");
          Serial.println(rxCmd);
        }
      }
      parserState = WAIT_DOLLAR;
      break;
  }
}

void handleUsbCommand(char c) {
  if (c == '0') {
    sendMspReplies = false;
    if (usbDebugReady()) Serial.println("USB CMD: MSP replies OFF");
  } else if (c == '1') {
    sendMspReplies = true;
    if (usbDebugReady()) Serial.println("USB CMD: MSP replies ON");
  } else if (c == 't' || c == 'T') {
    txBeaconEnabled = !txBeaconEnabled;
    if (usbDebugReady()) {
      Serial.print("USB CMD: TX beacon ");
      Serial.println(txBeaconEnabled ? "ON" : "OFF");
    }
  } else if (c == 'r' || c == 'R') {
    requestDisplayPortRegrab("manual");
  } else if (c == 'd' || c == 'D') {
    usbDebugEnabled = !usbDebugEnabled;
    if (usbDebugReady()) {
      Serial.print("USB CMD: debug ");
      Serial.println(usbDebugEnabled ? "ON" : "OFF");
    }
  } else if (c == 'h' || c == 'H' || c == '?') {
    if (usbDebugReady()) Serial.println("USB commands: 0=reply off, 1=reply on, r=regrab OSD, t=TX beacon toggle, d=debug toggle");
  }
}

int8_t craftNameHexNibble(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

void setCraftNameFromString(const char *value) {
  uint8_t writeIndex = 0;

  for (uint8_t readIndex = 0; value[readIndex] != '\0' && writeIndex < CRAFT_NAME_MAX_LEN; readIndex++) {
    char c = value[readIndex];
    if (c == '%' && value[readIndex + 1] != '\0' && value[readIndex + 2] != '\0') {
      const int8_t hi = craftNameHexNibble(value[readIndex + 1]);
      const int8_t lo = craftNameHexNibble(value[readIndex + 2]);
      if (hi >= 0 && lo >= 0) {
        c = (char)((hi << 4) | lo);
        readIndex += 2;
      }
    } else if (c == '+') {
      c = ' ';
    }

    if (c >= 'a' && c <= 'z') c = (char)(c - 32);

    const bool safeLetter = c >= 'A' && c <= 'Z';
    const bool safeNumber = c >= '0' && c <= '9';
    const bool safeSpace = c == ' ';
    if (safeSpace && writeIndex == 0) continue;
    if (safeLetter || safeNumber || safeSpace || c == '_' || c == '-') {
      craftName[writeIndex++] = c;
    }
  }

  while (writeIndex > 0 && craftName[writeIndex - 1] == ' ') {
    writeIndex--;
  }

  if (writeIndex == 0) {
    for (uint8_t i = 0; DEFAULT_CRAFT_NAME[i] != '\0' && i < CRAFT_NAME_MAX_LEN; i++) {
      craftName[i] = DEFAULT_CRAFT_NAME[i];
      writeIndex = i + 1;
    }
  }

  craftName[writeIndex] = '\0';
}

void clampSurfaceConfig() {
  steeringTrimUs = constrain(steeringTrimUs, -300, 300);
  steeringMinUs = constrain(steeringMinUs, 800, 1600);
  steeringMaxUs = constrain(steeringMaxUs, 1400, 2200);
  if (steeringMinUs >= steeringMaxUs) {
    steeringMinUs = DEFAULT_STEERING_MIN_US;
    steeringMaxUs = DEFAULT_STEERING_MAX_US;
  }

  escMinUs = constrain(escMinUs, 800, 1600);
  escMaxUs = constrain(escMaxUs, 1400, 2200);
  if (escMinUs >= escMaxUs) {
    escMinUs = DEFAULT_ESC_MIN_US;
    escMaxUs = DEFAULT_ESC_MAX_US;
  }

  escNeutralLowUs = constrain(escNeutralLowUs, 1200, 1600);
  escNeutralHighUs = constrain(escNeutralHighUs, 1400, 1800);
  if (escNeutralLowUs >= escNeutralHighUs) {
    escNeutralLowUs = DEFAULT_ESC_NEUTRAL_LOW_US;
    escNeutralHighUs = DEFAULT_ESC_NEUTRAL_HIGH_US;
  }

  escReverseDelayMs = constrain(escReverseDelayMs, 0, 1500);
  headPanTrimUs = constrain(headPanTrimUs, -300, 300);
  headTiltTrimUs = constrain(headTiltTrimUs, -300, 300);
  headPanScalePercent = constrain(headPanScalePercent, 0, 100);
  headTiltScalePercent = constrain(headTiltScalePercent, 0, 100);
  headPanMinUs = constrain(headPanMinUs, 800, 1600);
  headPanMaxUs = constrain(headPanMaxUs, 1400, 2200);
  if (headPanMinUs >= headPanMaxUs) {
    headPanMinUs = DEFAULT_HEAD_SERVO_MIN_US;
    headPanMaxUs = DEFAULT_HEAD_SERVO_MAX_US;
  }
  headTiltMinUs = constrain(headTiltMinUs, 800, 1600);
  headTiltMaxUs = constrain(headTiltMaxUs, 1400, 2200);
  if (headTiltMinUs >= headTiltMaxUs) {
    headTiltMinUs = DEFAULT_HEAD_SERVO_MIN_US;
    headTiltMaxUs = DEFAULT_HEAD_SERVO_MAX_US;
  }
  imuRotation &= 0x03;
  if (!isfinite(imuRollZeroDeg)) imuRollZeroDeg = 0.0f;
  if (!isfinite(imuPitchZeroDeg)) imuPitchZeroDeg = 0.0f;
  imuRollZeroDeg = constrain(imuRollZeroDeg, -180.0f, 180.0f);
  imuPitchZeroDeg = constrain(imuPitchZeroDeg, -180.0f, 180.0f);
  batteryCellCount = constrain(batteryCellCount, 1, 6);
  if (batteryWarnCellCv > 0) {
    batteryWarnCellCv = constrain(batteryWarnCellCv, 250, 420);
  }
  if (batteryFuelEmptyCellCv > 0) {
    batteryFuelEmptyCellCv = constrain(batteryFuelEmptyCellCv, 250, 420);
  }
  homeMinSats = constrain(homeMinSats, HOME_POINT_MIN_SATS_MIN, HOME_POINT_MIN_SATS_MAX);
  homeStabilityMs = constrain(homeStabilityMs, HOME_STABILITY_MS_MIN, HOME_STABILITY_MS_MAX);
}

void setImuRotation(uint8_t rotation) {
  rotation &= 0x03;
  if (imuRotation == rotation) return;
  imuRotation = rotation;
  resetImuCalibration();
}

void resetSurfaceConfigToDefaults() {
  setCraftNameFromString(DEFAULT_CRAFT_NAME);
  steeringOutputReversed = DEFAULT_STEERING_OUTPUT_REVERSED;
  throttleOutputReversed = DEFAULT_THROTTLE_OUTPUT_REVERSED;
  djiArmedToO4 = DEFAULT_DJI_ARMED_TO_O4;
  steeringTrimUs = DEFAULT_STEERING_TRIM_US;
  steeringMinUs = DEFAULT_STEERING_MIN_US;
  steeringMaxUs = DEFAULT_STEERING_MAX_US;
  escMinUs = DEFAULT_ESC_MIN_US;
  escMaxUs = DEFAULT_ESC_MAX_US;
  escNeutralLowUs = DEFAULT_ESC_NEUTRAL_LOW_US;
  escNeutralHighUs = DEFAULT_ESC_NEUTRAL_HIGH_US;
  escReverseAssistEnabled = DEFAULT_ESC_REVERSE_ASSIST;
  escReverseDelayMs = DEFAULT_ESC_REVERSE_DELAY_MS;
  headPanEnabled = DEFAULT_HEAD_PAN_ENABLED;
  headTiltEnabled = DEFAULT_HEAD_TILT_ENABLED;
  headPanReversed = DEFAULT_HEAD_PAN_REVERSED;
  headTiltReversed = DEFAULT_HEAD_TILT_REVERSED;
  headPanTrimUs = DEFAULT_HEAD_PAN_TRIM_US;
  headTiltTrimUs = DEFAULT_HEAD_TILT_TRIM_US;
  headPanScalePercent = DEFAULT_HEAD_PAN_SCALE_PERCENT;
  headTiltScalePercent = DEFAULT_HEAD_TILT_SCALE_PERCENT;
  headPanMinUs = DEFAULT_HEAD_SERVO_MIN_US;
  headPanMaxUs = DEFAULT_HEAD_SERVO_MAX_US;
  headTiltMinUs = DEFAULT_HEAD_SERVO_MIN_US;
  headTiltMaxUs = DEFAULT_HEAD_SERVO_MAX_US;
  imuRotation = DEFAULT_IMU_ROTATION;
  resetImuCalibration();
  batteryCellCount = DEFAULT_BATTERY_CELL_COUNT;
  batteryWarnCellCv = DEFAULT_BATTERY_WARN_CELL_CV;
  batteryFuelEmptyCellCv = DEFAULT_BATTERY_FUEL_EMPTY_CELL_CV;
  homeMinSats = DEFAULT_HOME_POINT_MIN_SATS;
  homeStabilityMs = DEFAULT_HOME_STABILITY_MS;
  osdShowCraft = DEFAULT_OSD_SHOW_CRAFT;
  osdShowDriveMode = DEFAULT_OSD_SHOW_DRIVE_MODE;
  osdShowCoords = DEFAULT_OSD_SHOW_COORDS;
  osdShowGps = DEFAULT_OSD_SHOW_GPS;
  osdShowBattery = DEFAULT_OSD_SHOW_BATTERY;
  osdShowLinkQuality = DEFAULT_OSD_SHOW_LINK_QUALITY;
  osdShowControls = DEFAULT_OSD_SHOW_CONTROLS;
  osdShowHome = DEFAULT_OSD_SHOW_HOME;
  osdShowWifi = DEFAULT_OSD_SHOW_WIFI;
  osdShowImu = DEFAULT_OSD_SHOW_IMU;
  osdShowDriveTime = DEFAULT_OSD_SHOW_DRIVE_TIME;
  resetBatteryWarningState();
}

void loadSurfaceConfig() {
  resetSurfaceConfigToDefaults();

  configStore.begin("surface", true);
  steeringOutputReversed = configStore.getBool("strRev", steeringOutputReversed);
  throttleOutputReversed = configStore.getBool("thrRev", throttleOutputReversed);
  djiArmedToO4 = configStore.getBool("djiArmO4", djiArmedToO4);
  steeringTrimUs = configStore.getShort("strTrim", steeringTrimUs);
  steeringMinUs = configStore.getUShort("strMin", steeringMinUs);
  steeringMaxUs = configStore.getUShort("strMax", steeringMaxUs);
  escMinUs = configStore.getUShort("escMin", escMinUs);
  escMaxUs = configStore.getUShort("escMax", escMaxUs);
  escNeutralLowUs = configStore.getUShort("escNLow", escNeutralLowUs);
  escNeutralHighUs = configStore.getUShort("escNHigh", escNeutralHighUs);
  escReverseAssistEnabled = configStore.getBool("escRevAst", escReverseAssistEnabled);
  escReverseDelayMs = configStore.getUShort("escRevDly", escReverseDelayMs);
  headPanEnabled = configStore.getBool("panEn", headPanEnabled);
  headTiltEnabled = configStore.getBool("tiltEn", headTiltEnabled);
  headPanReversed = configStore.getBool("panRev", headPanReversed);
  headTiltReversed = configStore.getBool("tiltRev", headTiltReversed);
  headPanTrimUs = configStore.getShort("panTrim", headPanTrimUs);
  headTiltTrimUs = configStore.getShort("tiltTrim", headTiltTrimUs);
  headPanScalePercent = configStore.getUChar("panScale", headPanScalePercent);
  headTiltScalePercent = configStore.getUChar("tiltScale", headTiltScalePercent);
  headPanMinUs = configStore.getUShort("panMin", headPanMinUs);
  headPanMaxUs = configStore.getUShort("panMax", headPanMaxUs);
  headTiltMinUs = configStore.getUShort("tiltMin", headTiltMinUs);
  headTiltMaxUs = configStore.getUShort("tiltMax", headTiltMaxUs);
  imuRotation = configStore.getUChar("imuRot", imuRotation);
  imuRollZeroDeg = configStore.getFloat("imuRZero", imuRollZeroDeg);
  imuPitchZeroDeg = configStore.getFloat("imuPZero", imuPitchZeroDeg);
  batteryCellCount = configStore.getUChar("batCells", batteryCellCount);
  batteryWarnCellCv = configStore.getUShort("batWarnCv", batteryWarnCellCv);
  batteryFuelEmptyCellCv = configStore.getUShort("batFuelCv", batteryFuelEmptyCellCv);
  homeMinSats = configStore.getUChar("homeMinSat", homeMinSats);
  homeStabilityMs = configStore.getUShort("homeStab", homeStabilityMs);
  osdShowCraft = configStore.getBool("osdCraft", osdShowCraft);
  osdShowDriveMode = configStore.getBool("osdMode", osdShowDriveMode);
  osdShowCoords = configStore.getBool("osdCoords", osdShowCoords);
  osdShowGps = configStore.getBool("osdGps", osdShowGps);
  osdShowBattery = configStore.getBool("osdBat", osdShowBattery);
  osdShowLinkQuality = configStore.getBool("osdLq", osdShowLinkQuality);
  osdShowControls = configStore.getBool("osdCtrl", osdShowControls);
  osdShowHome = configStore.getBool("osdHome", osdShowHome);
  osdShowWifi = configStore.getBool("osdWifi", osdShowWifi);
  osdShowImu = configStore.getBool("osdImu", osdShowImu);
  osdShowDriveTime = configStore.getBool("osdTime", osdShowDriveTime);
  String storedCraftName = configStore.getString("craft", craftName);
  setCraftNameFromString(storedCraftName.c_str());
  configStore.end();

  clampSurfaceConfig();
}

void saveSurfaceConfig() {
  clampSurfaceConfig();

  configStore.begin("surface", false);
  configStore.putBool("strRev", steeringOutputReversed);
  configStore.putBool("thrRev", throttleOutputReversed);
  configStore.putBool("djiArmO4", djiArmedToO4);
  configStore.putShort("strTrim", steeringTrimUs);
  configStore.putUShort("strMin", steeringMinUs);
  configStore.putUShort("strMax", steeringMaxUs);
  configStore.putUShort("escMin", escMinUs);
  configStore.putUShort("escMax", escMaxUs);
  configStore.putUShort("escNLow", escNeutralLowUs);
  configStore.putUShort("escNHigh", escNeutralHighUs);
  configStore.putBool("escRevAst", escReverseAssistEnabled);
  configStore.putUShort("escRevDly", escReverseDelayMs);
  configStore.putBool("panEn", headPanEnabled);
  configStore.putBool("tiltEn", headTiltEnabled);
  configStore.putBool("panRev", headPanReversed);
  configStore.putBool("tiltRev", headTiltReversed);
  configStore.putShort("panTrim", headPanTrimUs);
  configStore.putShort("tiltTrim", headTiltTrimUs);
  configStore.putUChar("panScale", headPanScalePercent);
  configStore.putUChar("tiltScale", headTiltScalePercent);
  configStore.putUShort("panMin", headPanMinUs);
  configStore.putUShort("panMax", headPanMaxUs);
  configStore.putUShort("tiltMin", headTiltMinUs);
  configStore.putUShort("tiltMax", headTiltMaxUs);
  configStore.putUChar("imuRot", imuRotation);
  configStore.putFloat("imuRZero", imuRollZeroDeg);
  configStore.putFloat("imuPZero", imuPitchZeroDeg);
  configStore.putUChar("batCells", batteryCellCount);
  configStore.putUShort("batWarnCv", batteryWarnCellCv);
  configStore.putUShort("batFuelCv", batteryFuelEmptyCellCv);
  configStore.putUChar("homeMinSat", homeMinSats);
  configStore.putUShort("homeStab", homeStabilityMs);
  configStore.putBool("osdCraft", osdShowCraft);
  configStore.putBool("osdMode", osdShowDriveMode);
  configStore.putBool("osdCoords", osdShowCoords);
  configStore.putBool("osdGps", osdShowGps);
  configStore.putBool("osdBat", osdShowBattery);
  configStore.putBool("osdLq", osdShowLinkQuality);
  configStore.putBool("osdCtrl", osdShowControls);
  configStore.putBool("osdHome", osdShowHome);
  configStore.putBool("osdWifi", osdShowWifi);
  configStore.putBool("osdImu", osdShowImu);
  configStore.putBool("osdTime", osdShowDriveTime);
  configStore.putString("craft", craftName);
  configStore.end();
}

bool configTextEqualsIgnoreCase(const char *a, const char *b) {
  while (*a && *b) {
    char ca = *a++;
    char cb = *b++;
    if (ca >= 'A' && ca <= 'Z') ca = (char)(ca + 32);
    if (cb >= 'A' && cb <= 'Z') cb = (char)(cb + 32);
    if (ca != cb) return false;
  }
  return *a == '\0' && *b == '\0';
}

bool parseConfigBool(const char *value) {
  return strcmp(value, "1") == 0 || configTextEqualsIgnoreCase(value, "true") || configTextEqualsIgnoreCase(value, "yes") || configTextEqualsIgnoreCase(value, "on");
}

uint16_t parseCellVoltageCv(const char *value) {
  if (!value || value[0] == '\0') return DEFAULT_BATTERY_WARN_CELL_CV;
  if (strchr(value, '.')) {
    const float volts = atof(value);
    if (volts <= 0.0f) return 0;
    return (uint16_t)lroundf(volts * 100.0f);
  }
  return (uint16_t)atoi(value);
}

void updatePerformanceStats(uint32_t loopStartUs, uint32_t loopEndUs) {
  const uint32_t loopElapsedUs = loopEndUs - loopStartUs;

  if (perfWindowStartUs == 0) {
    perfWindowStartUs = loopStartUs;
  }

  perfWindowBusyUs += loopElapsedUs;
  perfWindowLoops++;
  if (loopElapsedUs > perfWindowMaxLoopUs) {
    perfWindowMaxLoopUs = loopElapsedUs;
  }

  const uint32_t windowElapsedUs = loopEndUs - perfWindowStartUs;
  if (windowElapsedUs >= PERF_WINDOW_US) {
    loopAvgUs = perfWindowLoops > 0 ? perfWindowBusyUs / perfWindowLoops : 0;
    loopMaxUs = perfWindowMaxLoopUs;
    loopHz = windowElapsedUs > 0 ? (uint32_t)(((uint64_t)perfWindowLoops * 1000000ULL) / windowElapsedUs) : 0;
    cpuLoadPct = (uint16_t)constrain((uint32_t)(((uint64_t)loopAvgUs * 100ULL) / SERVO_PWM_FRAME_US), 0UL, 100UL);
    freeHeapBytes = ESP.getFreeHeap();
    minFreeHeapBytes = ESP.getMinFreeHeap();
    heapSizeBytes = ESP.getHeapSize();

    perfWindowStartUs = loopEndUs;
    perfWindowBusyUs = 0;
    perfWindowLoops = 0;
    perfWindowMaxLoopUs = 0;
  }
}

void sendConfigJson(const char *event) {
  if (!Serial) return;
  if (Serial.availableForWrite() < 64) return;

  const bool includeConfig = strcmp(event, "status") != 0;

  Serial.print("{\"type\":\"");
  Serial.print(event);
  Serial.print("\",\"version\":\"");
  Serial.print(FIRMWARE_VERSION);
  Serial.print("\"");
  if (includeConfig) {
    Serial.print(",\"craftName\":\"");
    Serial.print(craftName);
    Serial.print("\",\"steeringReversed\":");
    Serial.print(steeringOutputReversed ? "true" : "false");
    Serial.print(",\"throttleReversed\":");
    Serial.print(throttleOutputReversed ? "true" : "false");
    Serial.print(",\"djiArmedToO4\":");
    Serial.print(djiArmedToO4 ? "true" : "false");
    Serial.print(",\"steeringTrimUs\":");
    Serial.print(steeringTrimUs);
    Serial.print(",\"steeringMinUs\":");
    Serial.print(steeringMinUs);
    Serial.print(",\"steeringMaxUs\":");
    Serial.print(steeringMaxUs);
    Serial.print(",\"escMinUs\":");
    Serial.print(escMinUs);
    Serial.print(",\"escMaxUs\":");
    Serial.print(escMaxUs);
    Serial.print(",\"escNeutralLowUs\":");
    Serial.print(escNeutralLowUs);
    Serial.print(",\"escNeutralHighUs\":");
    Serial.print(escNeutralHighUs);
    Serial.print(",\"escReverseAssist\":");
    Serial.print(escReverseAssistEnabled ? "true" : "false");
    Serial.print(",\"escReverseDelayMs\":");
    Serial.print(escReverseDelayMs);
    Serial.print(",\"headPanEnabled\":");
    Serial.print(headPanEnabled ? "true" : "false");
    Serial.print(",\"headPanReversed\":");
    Serial.print(headPanReversed ? "true" : "false");
    Serial.print(",\"headPanScalePercent\":");
    Serial.print(headPanScalePercent);
    Serial.print(",\"headPanTrimUs\":");
    Serial.print(headPanTrimUs);
    Serial.print(",\"headPanMinUs\":");
    Serial.print(headPanMinUs);
    Serial.print(",\"headPanMaxUs\":");
    Serial.print(headPanMaxUs);
    Serial.print(",\"headTiltEnabled\":");
    Serial.print(headTiltEnabled ? "true" : "false");
    Serial.print(",\"headTiltReversed\":");
    Serial.print(headTiltReversed ? "true" : "false");
    Serial.print(",\"headTiltScalePercent\":");
    Serial.print(headTiltScalePercent);
    Serial.print(",\"headTiltTrimUs\":");
    Serial.print(headTiltTrimUs);
    Serial.print(",\"headTiltMinUs\":");
    Serial.print(headTiltMinUs);
    Serial.print(",\"headTiltMaxUs\":");
    Serial.print(headTiltMaxUs);
    Serial.print(",\"imuRotation\":");
    Serial.print(imuRotation);
    Serial.print(",\"imuRollZeroDeg\":");
    Serial.print(imuRollZeroDeg, 2);
    Serial.print(",\"imuPitchZeroDeg\":");
    Serial.print(imuPitchZeroDeg, 2);
    Serial.print(",\"batteryCellCount\":");
    Serial.print(batteryCellCount);
    Serial.print(",\"batteryWarnCellCv\":");
    Serial.print(batteryWarnCellCv);
    Serial.print(",\"batteryFuelEmptyCellCv\":");
    Serial.print(batteryFuelEmptyCellCv);
    Serial.print(",\"homeMinSats\":");
    Serial.print(homeMinSats);
    Serial.print(",\"homeStabilityMs\":");
    Serial.print(homeStabilityMs);
    Serial.print(",\"osdShowCraft\":");
    Serial.print(osdShowCraft ? "true" : "false");
    Serial.print(",\"osdShowDriveMode\":");
    Serial.print(osdShowDriveMode ? "true" : "false");
    Serial.print(",\"osdShowCoords\":");
    Serial.print(osdShowCoords ? "true" : "false");
    Serial.print(",\"osdShowGps\":");
    Serial.print(osdShowGps ? "true" : "false");
    Serial.print(",\"osdShowBattery\":");
    Serial.print(osdShowBattery ? "true" : "false");
    Serial.print(",\"osdShowLinkQuality\":");
    Serial.print(osdShowLinkQuality ? "true" : "false");
    Serial.print(",\"osdShowControls\":");
    Serial.print(osdShowControls ? "true" : "false");
    Serial.print(",\"osdShowHome\":");
    Serial.print(osdShowHome ? "true" : "false");
    Serial.print(",\"osdShowWifi\":");
    Serial.print(osdShowWifi ? "true" : "false");
    Serial.print(",\"osdShowImu\":");
    Serial.print(osdShowImu ? "true" : "false");
    Serial.print(",\"osdShowDriveTime\":");
    Serial.print(osdShowDriveTime ? "true" : "false");
  }
  Serial.print(",\"rcCh1Us\":");
  Serial.print(rcChannels[0]);
  Serial.print(",\"rcCh2Us\":");
  Serial.print(rcChannels[1]);
  Serial.print(",\"rcCh3Us\":");
  Serial.print(rcChannels[2]);
  Serial.print(",\"rcCh4Us\":");
  Serial.print(rcChannels[3]);
  Serial.print(",\"rcCh5Us\":");
  Serial.print(rcChannels[4]);
  Serial.print(",\"rcCh6Us\":");
  Serial.print(rcChannels[DRIVE_MODE_CHANNEL_INDEX]);
  Serial.print(",\"rcCh7Us\":");
  Serial.print(rcChannels[6]);
  Serial.print(",\"rcCh8Us\":");
  Serial.print(rcChannels[7]);
  Serial.print(",\"driveMode\":\"");
  Serial.print(driveModeName());
  Serial.print("\",\"driveModeThrottlePct\":");
  Serial.print(driveModeThrottlePercent());
  Serial.print(",\"o4MspLink\":");
  Serial.print(o4MspLinkActive() ? "true" : "false");
  Serial.print(",\"o4MspAgeMs\":");
  Serial.print(o4MspLinkAgeMs());
  Serial.print(",\"djiArmed\":");
  Serial.print(djiShouldReportArmed() ? "true" : "false");
  Serial.print(",\"djiArmedToO4\":");
  Serial.print(djiArmedToO4 ? "true" : "false");
  Serial.print(",\"failsafeActive\":");
  Serial.print(failsafeActive() ? "true" : "false");
  Serial.print(",\"failsafeRecoveryLocked\":");
  Serial.print(escFailsafeRecoveryLocked ? "true" : "false");
  Serial.print(",\"wifiArmingLockout\":");
  Serial.print(wifiArmingLockout ? "true" : "false");
  Serial.print(",\"elrsFailsafeCount\":");
  Serial.print(elrsFailsafeCount);
  Serial.print(",\"elrsFailsafeActiveMs\":");
  Serial.print(elrsFailsafeActiveMs());
  Serial.print(",\"elrsFailsafeLastMs\":");
  Serial.print(elrsFailsafeLastDurationMs);
  Serial.print(",\"crsfRcFrameAgeMs\":");
  Serial.print(crsfRcFrameAgeMs());
  Serial.print(",\"crsfLinkQuality\":");
  Serial.print(crsfLinkQuality);
  Serial.print(",\"crsfSnr\":");
  Serial.print((int)crsfSnr);
  Serial.print(",\"batteryVoltageValid\":");
  Serial.print(batteryVoltageValid ? "true" : "false");
  Serial.print(",\"batteryVoltageCv\":");
  Serial.print(batteryVoltage_cV);
  Serial.print(",\"batteryAverageCellCv\":");
  Serial.print(batteryAverageCellVoltageCv());
  Serial.print(",\"batteryWarningActive\":");
  Serial.print(batteryWarningActive ? "true" : "false");
  Serial.print(",\"batteryFuelEmptyActive\":");
  Serial.print(batteryFuelEmptyActive ? "true" : "false");
  Serial.print(",\"batteryFuelEmptyCellCv\":");
  Serial.print(batteryFuelEmptyCellCv);
  Serial.print(",\"homeSet\":");
  Serial.print(homeSet ? "true" : "false");
  Serial.print(",\"homePointEligible\":");
  Serial.print(gpsHomePointEligible() ? "true" : "false");
  Serial.print(",\"homeMinSats\":");
  Serial.print(homeMinSats);
  Serial.print(",\"homeStabilityMs\":");
  Serial.print(homeStabilityMs);
  Serial.print(",\"batteryAdcMv\":");
  Serial.print(batteryAdcMilliVolts);
  Serial.print(",\"imuAvailable\":");
  Serial.print(imuAvailable ? "true" : "false");
  Serial.print(",\"imuDataValid\":");
  Serial.print(imuDataValid ? "true" : "false");
  Serial.print(",\"imuRollDeg\":");
  Serial.print(displayedImuRollDeg(), 1);
  Serial.print(",\"imuPitchDeg\":");
  Serial.print(displayedImuPitchDeg(), 1);
  Serial.print(",\"driveTimeMs\":");
  Serial.print(currentDriveTimeMs());
  Serial.print(",\"steeringOutUs\":");
  Serial.print(steeringPulseUs);
  Serial.print(",\"escOutUs\":");
  Serial.print(escPulseUs);
  Serial.print(",\"steeringLive\":");
  Serial.print(steeringOutputLive ? "true" : "false");
  Serial.print(",\"escLive\":");
  Serial.print(escOutputLive ? "true" : "false");
  Serial.print(",\"headPanOutUs\":");
  Serial.print(headPanPulseUs);
  Serial.print(",\"headTiltOutUs\":");
  Serial.print(headTiltPulseUs);
  Serial.print(",\"headPanLive\":");
  Serial.print(headPanOutputLive ? "true" : "false");
  Serial.print(",\"headTiltLive\":");
  Serial.print(headTiltOutputLive ? "true" : "false");
  Serial.print(",\"driveStatsActive\":");
  Serial.print(driveStatsDisplayActive ? "true" : "false");
  Serial.print(",\"mspRequests\":");
  Serial.print(commandCount);
  Serial.print(",\"rawBytes\":");
  Serial.print(rawByteCount);
  Serial.print(",\"mspV1Headers\":");
  Serial.print(mspV1HeaderCount);
  Serial.print(",\"mspV2Headers\":");
  Serial.print(mspV2HeaderCount);
  Serial.print(",\"badChecksums\":");
  Serial.print(badChecksumCount);
  Serial.print(",\"dpFrames\":");
  Serial.print(displayPortFramesSent);
  Serial.print(",\"dpReacquire\":");
  Serial.print(displayPortReacquireCount);
  Serial.print(",\"dpRequests\":");
  Serial.print(displayPortRequestCount);
  Serial.print(",\"mspOsdConfigRequests\":");
  Serial.print(commandHistogram[MSP_OSD_CONFIG]);
  Serial.print(",\"mspSetOsdCanvasRequests\":");
  Serial.print(commandHistogram[MSP_SET_OSD_CANVAS]);
  Serial.print(",\"mspOsdCanvasRequests\":");
  Serial.print(commandHistogram[MSP_OSD_CANVAS]);
  Serial.print(",\"wifiStarted\":");
  Serial.print(wifiConfigPortalStarted ? "true" : "false");
  Serial.print(",\"wifiAutoOff\":");
  Serial.print(wifiAutoOffDone ? "true" : "false");
  Serial.print(",\"wifiClients\":");
  Serial.print(wifiConfigPortalStarted ? WiFi.softAPgetStationNum() : 0);
  Serial.print(",\"wifiIdleMs\":");
  Serial.print(wifiConfigPortalStarted ? millis() - lastWifiActivityMs : 0);
  Serial.print(",\"wifiStartDelayRemainingMs\":");
  Serial.print(wifiConfigPortalStarted ? 0 : (millis() < WIFI_START_DELAY_MS ? WIFI_START_DELAY_MS - millis() : 0));
  Serial.print(",\"cpuLoadPct\":");
  Serial.print(cpuLoadPct);
  Serial.print(",\"loopAvgUs\":");
  Serial.print(loopAvgUs);
  Serial.print(",\"loopMaxUs\":");
  Serial.print(loopMaxUs);
  Serial.print(",\"loopHz\":");
  Serial.print(loopHz);
  Serial.print(",\"freeHeapBytes\":");
  Serial.print(freeHeapBytes);
  Serial.print(",\"minFreeHeapBytes\":");
  Serial.print(minFreeHeapBytes);
  Serial.print(",\"heapSizeBytes\":");
  Serial.print(heapSizeBytes);
  Serial.print(",\"uptimeMs\":");
  Serial.print(millis());
  Serial.println("}");
}

String buildWifiConfigJson(bool includeConfig) {
  String json;
  json.reserve(includeConfig ? 2400 : 1400);
  json += "{\"type\":\"";
  json += includeConfig ? "config" : "status";
  json += "\",\"version\":\"";
  json += FIRMWARE_VERSION;
  json += "\"";

  if (includeConfig) {
    json += ",\"craftName\":\"";
    json += craftName;
    json += "\",\"steeringReversed\":";
    json += steeringOutputReversed ? "true" : "false";
    json += ",\"throttleReversed\":";
    json += throttleOutputReversed ? "true" : "false";
    json += ",\"djiArmedToO4\":";
    json += djiArmedToO4 ? "true" : "false";
    json += ",\"steeringTrimUs\":";
    json += steeringTrimUs;
    json += ",\"steeringMinUs\":";
    json += steeringMinUs;
    json += ",\"steeringMaxUs\":";
    json += steeringMaxUs;
    json += ",\"escMinUs\":";
    json += escMinUs;
    json += ",\"escMaxUs\":";
    json += escMaxUs;
    json += ",\"escNeutralLowUs\":";
    json += escNeutralLowUs;
    json += ",\"escNeutralHighUs\":";
    json += escNeutralHighUs;
    json += ",\"escReverseAssist\":";
    json += escReverseAssistEnabled ? "true" : "false";
    json += ",\"escReverseDelayMs\":";
    json += escReverseDelayMs;
    json += ",\"headPanEnabled\":";
    json += headPanEnabled ? "true" : "false";
    json += ",\"headPanReversed\":";
    json += headPanReversed ? "true" : "false";
    json += ",\"headPanScalePercent\":";
    json += headPanScalePercent;
    json += ",\"headPanTrimUs\":";
    json += headPanTrimUs;
    json += ",\"headPanMinUs\":";
    json += headPanMinUs;
    json += ",\"headPanMaxUs\":";
    json += headPanMaxUs;
    json += ",\"headTiltEnabled\":";
    json += headTiltEnabled ? "true" : "false";
    json += ",\"headTiltReversed\":";
    json += headTiltReversed ? "true" : "false";
    json += ",\"headTiltScalePercent\":";
    json += headTiltScalePercent;
    json += ",\"headTiltTrimUs\":";
    json += headTiltTrimUs;
    json += ",\"headTiltMinUs\":";
    json += headTiltMinUs;
    json += ",\"headTiltMaxUs\":";
    json += headTiltMaxUs;
    json += ",\"imuRotation\":";
    json += imuRotation;
    json += ",\"imuRollZeroDeg\":";
    json += String(imuRollZeroDeg, 2);
    json += ",\"imuPitchZeroDeg\":";
    json += String(imuPitchZeroDeg, 2);
    json += ",\"batteryCellCount\":";
    json += batteryCellCount;
    json += ",\"batteryWarnCellCv\":";
    json += batteryWarnCellCv;
    json += ",\"batteryFuelEmptyCellCv\":";
    json += batteryFuelEmptyCellCv;
    json += ",\"homeMinSats\":";
    json += homeMinSats;
    json += ",\"homeStabilityMs\":";
    json += homeStabilityMs;
    json += ",\"osdShowCraft\":";
    json += osdShowCraft ? "true" : "false";
    json += ",\"osdShowDriveMode\":";
    json += osdShowDriveMode ? "true" : "false";
    json += ",\"osdShowCoords\":";
    json += osdShowCoords ? "true" : "false";
    json += ",\"osdShowGps\":";
    json += osdShowGps ? "true" : "false";
    json += ",\"osdShowBattery\":";
    json += osdShowBattery ? "true" : "false";
    json += ",\"osdShowLinkQuality\":";
    json += osdShowLinkQuality ? "true" : "false";
    json += ",\"osdShowControls\":";
    json += osdShowControls ? "true" : "false";
    json += ",\"osdShowHome\":";
    json += osdShowHome ? "true" : "false";
    json += ",\"osdShowWifi\":";
    json += osdShowWifi ? "true" : "false";
    json += ",\"osdShowImu\":";
    json += osdShowImu ? "true" : "false";
    json += ",\"osdShowDriveTime\":";
    json += osdShowDriveTime ? "true" : "false";
  }

  json += ",\"rcCh1Us\":";
  json += rcChannels[0];
  json += ",\"rcCh2Us\":";
  json += rcChannels[1];
  json += ",\"rcCh3Us\":";
  json += rcChannels[2];
  json += ",\"rcCh4Us\":";
  json += rcChannels[3];
  json += ",\"rcCh5Us\":";
  json += rcChannels[4];
  json += ",\"rcCh6Us\":";
  json += rcChannels[DRIVE_MODE_CHANNEL_INDEX];
  json += ",\"rcCh7Us\":";
  json += rcChannels[6];
  json += ",\"rcCh8Us\":";
  json += rcChannels[7];
  json += ",\"driveMode\":\"";
  json += driveModeName();
  json += "\",\"driveModeThrottlePct\":";
  json += driveModeThrottlePercent();
  json += ",\"o4MspLink\":";
  json += o4MspLinkActive() ? "true" : "false";
  json += ",\"o4MspAgeMs\":";
  json += o4MspLinkAgeMs();
  json += ",\"djiArmed\":";
  json += djiShouldReportArmed() ? "true" : "false";
  json += ",\"djiArmedToO4\":";
  json += djiArmedToO4 ? "true" : "false";
  json += ",\"failsafeActive\":";
  json += failsafeActive() ? "true" : "false";
  json += ",\"failsafeRecoveryLocked\":";
  json += escFailsafeRecoveryLocked ? "true" : "false";
  json += ",\"wifiArmingLockout\":";
  json += wifiArmingLockout ? "true" : "false";
  json += ",\"elrsFailsafeCount\":";
  json += elrsFailsafeCount;
  json += ",\"elrsFailsafeActiveMs\":";
  json += elrsFailsafeActiveMs();
  json += ",\"elrsFailsafeLastMs\":";
  json += elrsFailsafeLastDurationMs;
  json += ",\"crsfRcFrameAgeMs\":";
  json += crsfRcFrameAgeMs();
  json += ",\"crsfLinkQuality\":";
  json += crsfLinkQuality;
  json += ",\"crsfSnr\":";
  json += (int)crsfSnr;
  json += ",\"batteryVoltageValid\":";
  json += batteryVoltageValid ? "true" : "false";
  json += ",\"batteryVoltageCv\":";
  json += batteryVoltage_cV;
  json += ",\"batteryAverageCellCv\":";
  json += batteryAverageCellVoltageCv();
  json += ",\"batteryWarningActive\":";
  json += batteryWarningActive ? "true" : "false";
  json += ",\"batteryFuelEmptyActive\":";
  json += batteryFuelEmptyActive ? "true" : "false";
  json += ",\"batteryFuelEmptyCellCv\":";
  json += batteryFuelEmptyCellCv;
  json += ",\"homeSet\":";
  json += homeSet ? "true" : "false";
  json += ",\"homePointEligible\":";
  json += gpsHomePointEligible() ? "true" : "false";
  json += ",\"homeMinSats\":";
  json += homeMinSats;
  json += ",\"homeStabilityMs\":";
  json += homeStabilityMs;
  json += ",\"batteryAdcMv\":";
  json += batteryAdcMilliVolts;
  json += ",\"imuAvailable\":";
  json += imuAvailable ? "true" : "false";
  json += ",\"imuDataValid\":";
  json += imuDataValid ? "true" : "false";
  json += ",\"imuRollDeg\":";
  json += String(displayedImuRollDeg(), 1);
  json += ",\"imuPitchDeg\":";
  json += String(displayedImuPitchDeg(), 1);
  json += ",\"driveTimeMs\":";
  json += currentDriveTimeMs();
  json += ",\"steeringOutUs\":";
  json += steeringPulseUs;
  json += ",\"escOutUs\":";
  json += escPulseUs;
  json += ",\"headPanOutUs\":";
  json += headPanPulseUs;
  json += ",\"headTiltOutUs\":";
  json += headTiltPulseUs;
  json += ",\"headPanLive\":";
  json += headPanOutputLive ? "true" : "false";
  json += ",\"headTiltLive\":";
  json += headTiltOutputLive ? "true" : "false";
  json += ",\"driveStatsActive\":";
  json += driveStatsDisplayActive ? "true" : "false";
  json += ",\"mspRequests\":";
  json += commandCount;
  json += ",\"rawBytes\":";
  json += rawByteCount;
  json += ",\"mspV1Headers\":";
  json += mspV1HeaderCount;
  json += ",\"mspV2Headers\":";
  json += mspV2HeaderCount;
  json += ",\"badChecksums\":";
  json += badChecksumCount;
  json += ",\"dpFrames\":";
  json += displayPortFramesSent;
  json += ",\"dpReacquire\":";
  json += displayPortReacquireCount;
  json += ",\"dpRequests\":";
  json += displayPortRequestCount;
  json += ",\"mspOsdConfigRequests\":";
  json += commandHistogram[MSP_OSD_CONFIG];
  json += ",\"mspSetOsdCanvasRequests\":";
  json += commandHistogram[MSP_SET_OSD_CANVAS];
  json += ",\"mspOsdCanvasRequests\":";
  json += commandHistogram[MSP_OSD_CANVAS];
  json += ",\"wifiStarted\":";
  json += wifiConfigPortalStarted ? "true" : "false";
  json += ",\"wifiAutoOff\":";
  json += wifiAutoOffDone ? "true" : "false";
  json += ",\"wifiClients\":";
  json += wifiConfigPortalStarted ? WiFi.softAPgetStationNum() : 0;
  json += ",\"wifiIdleMs\":";
  json += wifiConfigPortalStarted ? millis() - lastWifiActivityMs : 0;
  json += ",\"wifiStartDelayRemainingMs\":";
  json += wifiConfigPortalStarted ? 0 : (millis() < WIFI_START_DELAY_MS ? WIFI_START_DELAY_MS - millis() : 0);
  json += ",\"cpuLoadPct\":";
  json += cpuLoadPct;
  json += ",\"freeHeapBytes\":";
  json += freeHeapBytes;
  json += ",\"minFreeHeapBytes\":";
  json += minFreeHeapBytes;
  json += ",\"heapSizeBytes\":";
  json += heapSizeBytes;
  json += ",\"uptimeMs\":";
  json += millis();
  json += "}";
  return json;
}

void sendWifiJson(bool includeConfig) {
  lastWifiActivityMs = millis();
  wifiServer.sendHeader("Cache-Control", "no-store");
  wifiServer.send(200, "application/json", buildWifiConfigJson(includeConfig));
}

void applyConfigPair(char *pair) {
  char *equals = strchr(pair, '=');
  if (!equals) return;
  *equals = '\0';
  const char *key = pair;
  const char *value = equals + 1;

  if (strcmp(key, "steeringReversed") == 0) {
    steeringOutputReversed = parseConfigBool(value);
  } else if (strcmp(key, "djiArmedToO4") == 0) {
    djiArmedToO4 = parseConfigBool(value);
  } else if (strcmp(key, "craftName") == 0) {
    setCraftNameFromString(value);
  } else if (strcmp(key, "throttleReversed") == 0) {
    throttleOutputReversed = parseConfigBool(value);
  } else if (strcmp(key, "steeringTrimUs") == 0) {
    steeringTrimUs = (int16_t)atoi(value);
  } else if (strcmp(key, "steeringMinUs") == 0) {
    steeringMinUs = (uint16_t)atoi(value);
  } else if (strcmp(key, "steeringMaxUs") == 0) {
    steeringMaxUs = (uint16_t)atoi(value);
  } else if (strcmp(key, "escMinUs") == 0) {
    escMinUs = (uint16_t)atoi(value);
  } else if (strcmp(key, "escMaxUs") == 0) {
    escMaxUs = (uint16_t)atoi(value);
  } else if (strcmp(key, "escNeutralLowUs") == 0) {
    escNeutralLowUs = (uint16_t)atoi(value);
  } else if (strcmp(key, "escNeutralHighUs") == 0) {
    escNeutralHighUs = (uint16_t)atoi(value);
  } else if (strcmp(key, "escReverseAssist") == 0) {
    escReverseAssistEnabled = parseConfigBool(value);
  } else if (strcmp(key, "escReverseDelayMs") == 0) {
    escReverseDelayMs = (uint16_t)atoi(value);
  } else if (strcmp(key, "headPanEnabled") == 0) {
    headPanEnabled = parseConfigBool(value);
  } else if (strcmp(key, "headPanReversed") == 0) {
    headPanReversed = parseConfigBool(value);
  } else if (strcmp(key, "headPanScalePercent") == 0) {
    headPanScalePercent = (uint8_t)atoi(value);
  } else if (strcmp(key, "headPanTrimUs") == 0) {
    headPanTrimUs = (int16_t)atoi(value);
  } else if (strcmp(key, "headPanMinUs") == 0) {
    headPanMinUs = (uint16_t)atoi(value);
  } else if (strcmp(key, "headPanMaxUs") == 0) {
    headPanMaxUs = (uint16_t)atoi(value);
  } else if (strcmp(key, "headTiltEnabled") == 0) {
    headTiltEnabled = parseConfigBool(value);
  } else if (strcmp(key, "headTiltReversed") == 0) {
    headTiltReversed = parseConfigBool(value);
  } else if (strcmp(key, "headTiltScalePercent") == 0) {
    headTiltScalePercent = (uint8_t)atoi(value);
  } else if (strcmp(key, "headTiltTrimUs") == 0) {
    headTiltTrimUs = (int16_t)atoi(value);
  } else if (strcmp(key, "headTiltMinUs") == 0) {
    headTiltMinUs = (uint16_t)atoi(value);
  } else if (strcmp(key, "headTiltMaxUs") == 0) {
    headTiltMaxUs = (uint16_t)atoi(value);
  } else if (strcmp(key, "imuRotation") == 0) {
    setImuRotation((uint8_t)atoi(value));
  } else if (strcmp(key, "batteryCellCount") == 0) {
    batteryCellCount = (uint8_t)atoi(value);
  } else if (strcmp(key, "batteryWarnCellCv") == 0 || strcmp(key, "batteryWarnCellV") == 0) {
    batteryWarnCellCv = parseCellVoltageCv(value);
    resetBatteryWarningState();
  } else if (strcmp(key, "batteryFuelEmptyCellCv") == 0 || strcmp(key, "batteryFuelEmptyCellV") == 0) {
    batteryFuelEmptyCellCv = parseCellVoltageCv(value);
    batteryFuelEmptyLowSinceMs = 0;
  } else if (strcmp(key, "homeMinSats") == 0) {
    homeMinSats = (uint8_t)atoi(value);
    homeEligibleSinceMs = 0;
  } else if (strcmp(key, "homeStabilityMs") == 0) {
    homeStabilityMs = (uint16_t)atoi(value);
    homeEligibleSinceMs = 0;
  } else if (strcmp(key, "osdShowCraft") == 0) {
    osdShowCraft = parseConfigBool(value);
  } else if (strcmp(key, "osdShowDriveMode") == 0) {
    osdShowDriveMode = parseConfigBool(value);
  } else if (strcmp(key, "osdShowCoords") == 0) {
    osdShowCoords = parseConfigBool(value);
  } else if (strcmp(key, "osdShowGps") == 0) {
    osdShowGps = parseConfigBool(value);
  } else if (strcmp(key, "osdShowBattery") == 0) {
    osdShowBattery = parseConfigBool(value);
  } else if (strcmp(key, "osdShowLinkQuality") == 0) {
    osdShowLinkQuality = parseConfigBool(value);
  } else if (strcmp(key, "osdShowControls") == 0) {
    osdShowControls = parseConfigBool(value);
  } else if (strcmp(key, "osdShowHome") == 0) {
    osdShowHome = parseConfigBool(value);
  } else if (strcmp(key, "osdShowWifi") == 0) {
    osdShowWifi = parseConfigBool(value);
  } else if (strcmp(key, "osdShowImu") == 0) {
    osdShowImu = parseConfigBool(value);
  } else if (strcmp(key, "osdShowDriveTime") == 0) {
    osdShowDriveTime = parseConfigBool(value);
  }
}

void applyWifiConfigArgs() {
  if (wifiServer.hasArg("craftName")) setCraftNameFromString(wifiServer.arg("craftName").c_str());
  if (wifiServer.hasArg("steeringReversed")) steeringOutputReversed = parseConfigBool(wifiServer.arg("steeringReversed").c_str());
  if (wifiServer.hasArg("throttleReversed")) throttleOutputReversed = parseConfigBool(wifiServer.arg("throttleReversed").c_str());
  if (wifiServer.hasArg("djiArmedToO4")) djiArmedToO4 = parseConfigBool(wifiServer.arg("djiArmedToO4").c_str());
  if (wifiServer.hasArg("steeringTrimUs")) steeringTrimUs = (int16_t)wifiServer.arg("steeringTrimUs").toInt();
  if (wifiServer.hasArg("steeringMinUs")) steeringMinUs = (uint16_t)wifiServer.arg("steeringMinUs").toInt();
  if (wifiServer.hasArg("steeringMaxUs")) steeringMaxUs = (uint16_t)wifiServer.arg("steeringMaxUs").toInt();
  if (wifiServer.hasArg("escMinUs")) escMinUs = (uint16_t)wifiServer.arg("escMinUs").toInt();
  if (wifiServer.hasArg("escMaxUs")) escMaxUs = (uint16_t)wifiServer.arg("escMaxUs").toInt();
  if (wifiServer.hasArg("escNeutralLowUs")) escNeutralLowUs = (uint16_t)wifiServer.arg("escNeutralLowUs").toInt();
  if (wifiServer.hasArg("escNeutralHighUs")) escNeutralHighUs = (uint16_t)wifiServer.arg("escNeutralHighUs").toInt();
  if (wifiServer.hasArg("escReverseAssist")) escReverseAssistEnabled = parseConfigBool(wifiServer.arg("escReverseAssist").c_str());
  if (wifiServer.hasArg("escReverseDelayMs")) escReverseDelayMs = (uint16_t)wifiServer.arg("escReverseDelayMs").toInt();
  if (wifiServer.hasArg("headPanEnabled")) headPanEnabled = parseConfigBool(wifiServer.arg("headPanEnabled").c_str());
  if (wifiServer.hasArg("headPanReversed")) headPanReversed = parseConfigBool(wifiServer.arg("headPanReversed").c_str());
  if (wifiServer.hasArg("headPanScalePercent")) headPanScalePercent = (uint8_t)wifiServer.arg("headPanScalePercent").toInt();
  if (wifiServer.hasArg("headPanTrimUs")) headPanTrimUs = (int16_t)wifiServer.arg("headPanTrimUs").toInt();
  if (wifiServer.hasArg("headPanMinUs")) headPanMinUs = (uint16_t)wifiServer.arg("headPanMinUs").toInt();
  if (wifiServer.hasArg("headPanMaxUs")) headPanMaxUs = (uint16_t)wifiServer.arg("headPanMaxUs").toInt();
  if (wifiServer.hasArg("headTiltEnabled")) headTiltEnabled = parseConfigBool(wifiServer.arg("headTiltEnabled").c_str());
  if (wifiServer.hasArg("headTiltReversed")) headTiltReversed = parseConfigBool(wifiServer.arg("headTiltReversed").c_str());
  if (wifiServer.hasArg("headTiltScalePercent")) headTiltScalePercent = (uint8_t)wifiServer.arg("headTiltScalePercent").toInt();
  if (wifiServer.hasArg("headTiltTrimUs")) headTiltTrimUs = (int16_t)wifiServer.arg("headTiltTrimUs").toInt();
  if (wifiServer.hasArg("headTiltMinUs")) headTiltMinUs = (uint16_t)wifiServer.arg("headTiltMinUs").toInt();
  if (wifiServer.hasArg("headTiltMaxUs")) headTiltMaxUs = (uint16_t)wifiServer.arg("headTiltMaxUs").toInt();
  if (wifiServer.hasArg("imuRotation")) setImuRotation((uint8_t)wifiServer.arg("imuRotation").toInt());
  if (wifiServer.hasArg("batteryCellCount")) batteryCellCount = (uint8_t)wifiServer.arg("batteryCellCount").toInt();
  if (wifiServer.hasArg("batteryWarnCellCv")) {
    batteryWarnCellCv = parseCellVoltageCv(wifiServer.arg("batteryWarnCellCv").c_str());
    resetBatteryWarningState();
  }
  if (wifiServer.hasArg("batteryWarnCellV")) {
    batteryWarnCellCv = parseCellVoltageCv(wifiServer.arg("batteryWarnCellV").c_str());
    resetBatteryWarningState();
  }
  if (wifiServer.hasArg("batteryFuelEmptyCellCv")) {
    batteryFuelEmptyCellCv = parseCellVoltageCv(wifiServer.arg("batteryFuelEmptyCellCv").c_str());
    batteryFuelEmptyLowSinceMs = 0;
  }
  if (wifiServer.hasArg("batteryFuelEmptyCellV")) {
    batteryFuelEmptyCellCv = parseCellVoltageCv(wifiServer.arg("batteryFuelEmptyCellV").c_str());
    batteryFuelEmptyLowSinceMs = 0;
  }
  if (wifiServer.hasArg("homeMinSats")) {
    homeMinSats = (uint8_t)wifiServer.arg("homeMinSats").toInt();
    homeEligibleSinceMs = 0;
  }
  if (wifiServer.hasArg("homeStabilityMs")) {
    homeStabilityMs = (uint16_t)wifiServer.arg("homeStabilityMs").toInt();
    homeEligibleSinceMs = 0;
  }
  if (wifiServer.hasArg("osdShowCraft")) osdShowCraft = parseConfigBool(wifiServer.arg("osdShowCraft").c_str());
  if (wifiServer.hasArg("osdShowDriveMode")) osdShowDriveMode = parseConfigBool(wifiServer.arg("osdShowDriveMode").c_str());
  if (wifiServer.hasArg("osdShowCoords")) osdShowCoords = parseConfigBool(wifiServer.arg("osdShowCoords").c_str());
  if (wifiServer.hasArg("osdShowGps")) osdShowGps = parseConfigBool(wifiServer.arg("osdShowGps").c_str());
  if (wifiServer.hasArg("osdShowBattery")) osdShowBattery = parseConfigBool(wifiServer.arg("osdShowBattery").c_str());
  if (wifiServer.hasArg("osdShowLinkQuality")) osdShowLinkQuality = parseConfigBool(wifiServer.arg("osdShowLinkQuality").c_str());
  if (wifiServer.hasArg("osdShowControls")) osdShowControls = parseConfigBool(wifiServer.arg("osdShowControls").c_str());
  if (wifiServer.hasArg("osdShowHome")) osdShowHome = parseConfigBool(wifiServer.arg("osdShowHome").c_str());
  if (wifiServer.hasArg("osdShowWifi")) osdShowWifi = parseConfigBool(wifiServer.arg("osdShowWifi").c_str());
  if (wifiServer.hasArg("osdShowImu")) osdShowImu = parseConfigBool(wifiServer.arg("osdShowImu").c_str());
  if (wifiServer.hasArg("osdShowDriveTime")) osdShowDriveTime = parseConfigBool(wifiServer.arg("osdShowDriveTime").c_str());
  clampSurfaceConfig();
}

void handleUsbConfigLine(char *line) {
  if (strncmp(line, "CFG", 3) != 0) return;
  lastUsbConfigRxMs = millis();

  char *command = line + 3;
  while (*command == ' ') command++;

  if (strcmp(command, "GET") == 0) {
    sendConfigJson("config");
  } else if (strncmp(command, "SET", 3) == 0) {
    char *pairs = command + 3;
    char *token = strtok(pairs, " ");
    while (token) {
      applyConfigPair(token);
      token = strtok(nullptr, " ");
    }
    clampSurfaceConfig();
    sendConfigJson("config");
  } else if (strcmp(command, "SAVE") == 0) {
    saveSurfaceConfig();
    sendConfigJson("saved");
  } else if (strcmp(command, "DEFAULTS") == 0) {
    resetSurfaceConfigToDefaults();
    saveSurfaceConfig();
    sendConfigJson("defaults");
  } else if (strcmp(command, "PING") == 0) {
    sendConfigJson("pong");
  } else if (strcmp(command, "REGRAB") == 0) {
    requestDisplayPortRegrab("usb cfg");
    sendConfigJson("regrab");
  } else if (strcmp(command, "IMU_CAL") == 0) {
    if (engineSwitchOn()) {
      sendConfigJson("imu_cal_disarm");
    } else if (!imuDataValid) {
      sendConfigJson("imu_unavailable");
    } else if (calibrateImuCurrentLevel()) {
      sendConfigJson("imu_calibrated");
    } else {
      sendConfigJson("imu_cal_rejected");
    }
  } else if (strcmp(command, "IMU_RESET") == 0) {
    if (engineSwitchOn()) {
      sendConfigJson("imu_reset_disarm");
    } else {
      resetImuCalibration();
      saveSurfaceConfig();
      sendConfigJson("imu_reset");
    }
  } else if (strncmp(command, "STREAM", 6) == 0) {
    char *value = command + 6;
    while (*value == ' ') value++;
    usbConfigStreamEnabled = parseConfigBool(value);
    sendConfigJson(usbConfigStreamEnabled ? "stream_on" : "stream_off");
  }
}

void parseUsbConfigByte(char c) {
  if (c == '\r') return;

  if (c == '\n') {
    usbConfigLine[usbConfigLineLen] = '\0';
    handleUsbConfigLine(usbConfigLine);
    usbConfigLineLen = 0;
    return;
  }

  if (usbConfigLineLen < sizeof(usbConfigLine) - 1) {
    usbConfigLine[usbConfigLineLen++] = c;
  } else {
    usbConfigLineLen = 0;
  }
}

void shutdownWifiConfigPortal();

void sendWifiPortalPage() {
  lastWifiActivityMs = millis();
  wifiServer.sendHeader("Cache-Control", "no-store");
  wifiServer.send_P(200, "text/html", WIFI_CONFIG_HTML);
}

void redirectWifiPortal() {
  lastWifiActivityMs = millis();
  wifiServer.sendHeader("Location", String("http://") + WIFI_AP_IP.toString() + "/", true);
  wifiServer.send(302, "text/plain", "");
}

void handleWifiNotFound() {
  const String uri = wifiServer.uri();
  if (uri == "/generate_204" || uri == "/gen_204" || uri == "/hotspot-detect.html" || uri == "/ncsi.txt" || uri == "/fwlink") {
    sendWifiPortalPage();
    return;
  }
  redirectWifiPortal();
}

void setupWifiConfigPortal() {
  if (!WIFI_CONFIG_ENABLED || wifiConfigPortalStarted) return;

  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);
  WiFi.softAPConfig(WIFI_AP_IP, WIFI_AP_GATEWAY, WIFI_AP_SUBNET);

  const bool passwordValid = strlen(WIFI_AP_PASSWORD) >= 8;
  wifiConfigPortalStarted = passwordValid
    ? WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASSWORD)
    : WiFi.softAP(WIFI_AP_SSID);

  dnsServer.start(DNS_PORT, "*", WIFI_AP_IP);

  wifiServer.on("/", HTTP_GET, sendWifiPortalPage);
  wifiServer.on("/index.html", HTTP_GET, sendWifiPortalPage);
  wifiServer.on("/api/config", HTTP_GET, []() { sendWifiJson(true); });
  wifiServer.on("/api/status", HTTP_GET, []() { sendWifiJson(false); });
  wifiServer.on("/api/save", HTTP_POST, []() {
    applyWifiConfigArgs();
    saveSurfaceConfig();
    wifiServer.send(200, "text/plain", "Saved to ESP32 flash");
  });
  wifiServer.on("/api/defaults", HTTP_POST, []() {
    resetSurfaceConfigToDefaults();
    saveSurfaceConfig();
    wifiServer.send(200, "text/plain", "Defaults restored");
  });
  wifiServer.on("/api/imu/calibrate", HTTP_POST, []() {
    lastWifiActivityMs = millis();
    if (engineSwitchOn()) {
      wifiServer.send(409, "text/plain", "Disarm CH5 before IMU calibration");
    } else if (!imuDataValid) {
      wifiServer.send(503, "text/plain", "IMU data is not available");
    } else if (calibrateImuCurrentLevel()) {
      wifiServer.send(200, "text/plain", "IMU calibrated: current position is level");
    } else {
      wifiServer.send(409, "text/plain", "IMU calibration rejected");
    }
  });
  wifiServer.on("/api/imu/reset", HTTP_POST, []() {
    lastWifiActivityMs = millis();
    if (engineSwitchOn()) {
      wifiServer.send(409, "text/plain", "Disarm CH5 before resetting IMU calibration");
      return;
    }
    resetImuCalibration();
    saveSurfaceConfig();
    wifiServer.send(200, "text/plain", "IMU calibration reset");
  });
  wifiServer.on("/api/wifi/off", HTTP_POST, []() {
    wifiServer.send(200, "text/plain", "WiFi disconnect requested");
    delay(20);
    shutdownWifiConfigPortal();
  });
  wifiServer.onNotFound(handleWifiNotFound);
  wifiServer.begin();
  wifiConfigPortalStartMs = millis();
  lastWifiActivityMs = wifiConfigPortalStartMs;
}

void shutdownWifiConfigPortal() {
  if (!wifiConfigPortalStarted) return;
  wifiServer.stop();
  dnsServer.stop();
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_OFF);
  wifiConfigPortalStarted = false;
  wifiAutoOffDone = true;
}

void serviceWifiConfigPortal() {
  if (!wifiConfigPortalStarted) return;
  const uint32_t now = millis();
  const uint8_t clientCount = WiFi.softAPgetStationNum();

  if (engineSwitchOn()) {
    if (clientCount > 0) {
      wifiArmingLockout = true;
    } else {
      shutdownWifiConfigPortal();
      return;
    }
  }

  if (clientCount > 0) {
    lastWifiActivityMs = now;
  }
  if (now - lastWifiActivityMs >= WIFI_IDLE_AUTO_OFF_MS) {
    shutdownWifiConfigPortal();
    return;
  }
  if (now - lastWifiServiceMs < WIFI_SERVICE_INTERVAL_MS) return;
  lastWifiServiceMs = now;
  dnsServer.processNextRequest();
  wifiServer.handleClient();
}

void setupWatchdog() {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  esp_task_wdt_config_t watchdogConfig = {};
  watchdogConfig.timeout_ms = WATCHDOG_TIMEOUT_SECONDS * 1000;
  watchdogConfig.idle_core_mask = 0;
  watchdogConfig.trigger_panic = true;
  esp_task_wdt_init(&watchdogConfig);
#else
  esp_task_wdt_init(WATCHDOG_TIMEOUT_SECONDS, true);
#endif
  esp_task_wdt_add(NULL);
}

void setup() {
  pinMode(STEERING_PWM_PIN, INPUT);
  pinMode(ESC_PWM_PIN, INPUT);
  pinMode(HEAD_PAN_PWM_PIN, INPUT);
  pinMode(HEAD_TILT_PWM_PIN, INPUT);
  pinMode(BATTERY_ADC_PIN, INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(BATTERY_ADC_PIN, ADC_11db);
  updateBatteryVoltage(true);
  loadSurfaceConfig();
  setupImu();

  if (USB_CONFIG_MODE || USB_DIAGNOSTIC_MODE) {
    Serial.begin(115200);
    if (USB_DIAGNOSTIC_MODE) delay(300);
  }

  if (usbDebugReady()) {
    Serial.println();
    Serial.println("BOOT");
    Serial.print("DJI O4 MSP Emulator v");
    Serial.println(FIRMWARE_VERSION);
    Serial.println("USB Serial = diagnostic mode");
    Serial.println("Serial1 GPIO16 RX / GPIO15 TX = DJI O4 MSP");
    Serial.println("Serial2 GPIO18 RX / GPIO17 TX = GPS NMEA");
    Serial.println("Serial0 GPIO4 RX / GPIO5 TX = ELRS CRSF");
    Serial.println("GPIO13 = steering PWM output");
    Serial.println("GPIO14 = ESC PWM output, arm gated");
    Serial.println("GPIO11 = camera pan PWM output, optional");
    Serial.println("GPIO12 = camera tilt PWM output, optional");
    Serial.println("GPIO1 = battery ADC via 100k/47k divider");
    Serial.println("GPIO8 SDA / GPIO9 SCL = MPU6050 I2C");
    Serial.print("Steering reversed: ");
    Serial.println(steeringOutputReversed ? "YES" : "NO");
    Serial.print("Throttle reversed: ");
    Serial.println(throttleOutputReversed ? "YES" : "NO");
    Serial.print("Fake armed mode: ");
    Serial.println(FAKE_ARMED_MODE ? "ON" : "OFF");
    Serial.print("Send MSP replies: ");
    Serial.println(sendMspReplies ? "ON" : "OFF");
    Serial.print("DisplayPort test: ");
    Serial.println(DISPLAYPORT_TEST_ENABLED ? "ON" : "OFF");
    Serial.print("DisplayPort both directions: ");
    Serial.println(DISPLAYPORT_SEND_BOTH_DIRECTIONS ? "ON" : "OFF");
    Serial.print("VTX MSP probe: ");
    Serial.println(VTX_MSP_PROBE_ENABLED ? "ON" : "OFF");
    Serial.print("DJI native classic status: ");
    Serial.println(DJI_NATIVE_CLASSIC_STATUS ? "ON" : "OFF");
    Serial.print("DJI native GPS/home: ");
    Serial.println(DJI_NATIVE_GPS_HOME_ENABLED ? "ON" : "OFF");
    Serial.println("USB commands: 0=reply off, 1=reply on, r=regrab OSD, t=TX beacon toggle, d=debug toggle");
  }

  DJISerial.setRxBufferSize(DJI_RX_BUFFER_SIZE);
  DJISerial.setTxBufferSize(DJI_TX_BUFFER_SIZE);
  GPSSerial.setRxBufferSize(GPS_RX_BUFFER_SIZE);
  CRSFSerial.setRxBufferSize(CRSF_RX_BUFFER_SIZE);

  DJISerial.begin(MSP_BAUD, SERIAL_8N1, MSP_RX_PIN, MSP_TX_PIN);
  GPSSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  CRSFSerial.begin(CRSF_BAUD, SERIAL_8N1, CRSF_RX_PIN, CRSF_TX_PIN);
  setupWatchdog();

  displayPortClearRequested = true;

#ifdef LED_BUILTIN
  pinMode(LED_BUILTIN, OUTPUT);
#endif
}

void loop() {
  const uint32_t loopStartUs = micros();

  if (USB_CONFIG_MODE) {
    while (Serial.available()) {
      parseUsbConfigByte((char)Serial.read());
    }
  } else if (USB_DIAGNOSTIC_MODE) {
    while (Serial.available()) {
      handleUsbCommand((char)Serial.read());
    }
  }

  while (DJISerial.available()) {
    rawByteCount++;
    parseMspByte((uint8_t)DJISerial.read());
  }

  while (GPSSerial.available()) {
    gpsByteCount++;
    parseGpsByte((uint8_t)GPSSerial.read());
  }

  while (CRSFSerial.available()) {
    crsfByteCount++;
    parseCrsfByte((uint8_t)CRSFSerial.read());
  }

  updateElrsFailsafeDiagnostics();
  updateWifiArmingLockout();
  updateSteeringPwm();
  updateHeadServoPwm();
  updateEscPwm();
  updateBatteryVoltage();
  updateBatteryWarning();
  updateBatteryFuelEmpty();
  updateImu();
  updateDriveTimer();
  updateDriveStats();
  updateImuCalibrationGesture();
  updateHomePointResetGesture();

  const uint32_t now = millis();

  if (DISPLAYPORT_TEST_ENABLED && (displayPortClearRequested || now - lastDisplayPortMs >= DISPLAYPORT_INTERVAL_MS)) {
    lastDisplayPortMs = now;
    sendDisplayPortTestFrame();
  }

  if (VTX_MSP_PROBE_ENABLED && now - lastVtxMspProbeMs >= 1000) {
    lastVtxMspProbeMs = now;
    sendVtxMspProbeFrame();
  }

  if (txBeaconEnabled && now - lastTxBeaconMs >= 1000) {
    lastTxBeaconMs = now;
    const uint8_t beaconPayload[] = { 'T', 'X', 'O', 'K' };
    sendMspV1PacketWithDirection('>', 250, beaconPayload, sizeof(beaconPayload));
    txBeaconCount++;
  }

  if (now - lastHeartbeatMs >= 2000) {
    lastHeartbeatMs = now;

#ifdef LED_BUILTIN
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
#endif

    if (usbDebugReady()) {
    Serial.print("Alive. MSP requests seen: ");
    Serial.print(commandCount);
    Serial.print(" | raw bytes: ");
    Serial.print(rawByteCount);
    Serial.print(" | '$': ");
    Serial.print(dollarCount);
    Serial.print(" | $M<: ");
    Serial.print(mspV1HeaderCount);
    Serial.print(" | $X<: ");
    Serial.print(mspV2HeaderCount);
    Serial.print(" | bad checksum: ");
    Serial.print(badChecksumCount);
    Serial.print(" | DP frames sent: ");
    Serial.print(displayPortFramesSent);
    Serial.print(" | DP regrab: ");
    Serial.print(displayPortRegrabCount);
    Serial.print(" | DP reacquire: ");
    Serial.print(displayPortReacquireCount);
    Serial.print(" | DP req 182: ");
    Serial.print(displayPortRequestCount);
    Serial.print(" | O4 MSP: ");
    Serial.print(o4MspLinkActive() ? "OK" : "LOST");
    Serial.print(" ");
    Serial.print(o4MspLinkAgeMs());
    Serial.print("ms");
    Serial.print(" | VTX probe: ");
    Serial.print(vtxMspProbeCount);
    Serial.print(" | replies: ");
    Serial.print(sendMspReplies ? "ON" : "OFF");
    Serial.print(" | TX beacon: ");
    Serial.print(txBeaconEnabled ? "ON" : "OFF");
    Serial.print(" ");
    Serial.print(txBeaconCount);
    Serial.print(" | reply packets: ");
    Serial.print(replyPacketCount);
    Serial.print(" | reply bytes: ");
    Serial.println(replyByteCount);

    Serial.print("CMD counts: ");
    for (uint16_t i = 0; i < 256; i++) {
      if (commandHistogram[i] > 0) {
        Serial.print(i);
        Serial.print("=");
        Serial.print(commandHistogram[i]);
        Serial.print(" ");
      }
    }
    Serial.println();

    Serial.print("GPS bytes: ");
    Serial.print(gpsByteCount);
    Serial.print(" | sentences: ");
    Serial.print(gpsSentenceCount);
    Serial.print(" | parsed: ");
    Serial.print(gpsParsedCount);
    Serial.print(" | fix: ");
    Serial.print(gpsFix);
    Serial.print(" | sats: ");
    Serial.print(gpsSats);
    Serial.print(" | lat: ");
    Serial.print(gpsLat);
    Serial.print(" | lon: ");
    Serial.print(gpsLon);
    Serial.print(" | speed cm/s: ");
    Serial.print(gpsSpeedCms);
    Serial.print(" | home m: ");
    Serial.print(homeDistanceMeters);
    Serial.print(" | home set: ");
    Serial.print(homeSet ? "YES" : "NO");
    Serial.print(" | home eligible: ");
    Serial.println(gpsHomePointEligible() ? "YES" : "NO");

    Serial.print("CRSF bytes: ");
    Serial.print(crsfByteCount);
    Serial.print(" | frames: ");
    Serial.print(crsfFrameCount);
    Serial.print(" | rc: ");
    Serial.print(crsfRcFrameCount);
    Serial.print(" | link: ");
    Serial.print(crsfLinkFrameCount);
    Serial.print(" | bad crc: ");
    Serial.print(crsfBadCrcCount);
    Serial.print(" | LQ: ");
    Serial.print(crsfLinkQuality);
    Serial.print(" | FS count: ");
    Serial.print(elrsFailsafeCount);
    Serial.print(" | FS active ms: ");
    Serial.print(elrsFailsafeActiveMs());
    Serial.print(" | FS last ms: ");
    Serial.print(elrsFailsafeLastDurationMs);
    Serial.print(" | RC age ms: ");
    Serial.print(crsfRcFrameAgeMs());
    Serial.print(" | CH1: ");
    Serial.print(rcChannels[0]);
    Serial.print(" | CH2: ");
    Serial.print(rcChannels[1]);
    Serial.print(" | CH5: ");
    Serial.print(rcChannels[4]);
    Serial.print(" | CH6: ");
    Serial.print(rcChannels[DRIVE_MODE_CHANNEL_INDEX]);
    Serial.print(" ");
    Serial.print(driveModeName());
    Serial.print(" | STR PWM: ");
    Serial.print(steeringOutputLive ? "LIVE" : "OFF");
    Serial.print(" | ESC: ");
    Serial.print(escOutputLive ? "LIVE" : "OFF");
    Serial.print(" | ESC attached: ");
    Serial.print(escPwmAttached ? "YES" : "NO");
    Serial.print(" | neutral gate: ");
    Serial.print(escNeutralGateSatisfied ? "OK" : "WAIT");
    Serial.print(" | fuel empty: ");
    Serial.println(batteryFuelEmptyActive ? "YES" : "NO");

    if (rawSampleLen > 0) {
      Serial.print("RAW ");
      for (uint8_t i = 0; i < rawSampleLen; i++) {
        if (rawSample[i] < 16) Serial.print("0");
        Serial.print(rawSample[i], HEX);
        Serial.print(" ");
      }
      Serial.print(" | ");
      for (uint8_t i = 0; i < rawSampleLen; i++) {
        char c = (rawSample[i] >= 32 && rawSample[i] <= 126) ? (char)rawSample[i] : '.';
        Serial.print(c);
      }
      Serial.println();
      rawSampleLen = 0;
    }
    } else {
      rawSampleLen = 0;
    }
  }

  servicePwmOutputs();
  if (WIFI_CONFIG_ENABLED && !wifiAutoOffDone && !wifiConfigPortalStarted && now >= WIFI_START_DELAY_MS && !engineSwitchOn()) {
    setupWifiConfigPortal();
  }
  serviceWifiConfigPortal();
  esp_task_wdt_reset();

  if (USB_CONFIG_MODE && usbConfigStreamEnabled) {
    const uint32_t now = millis();
    if (now - lastUsbConfigRxMs > 3000) {
      usbConfigStreamEnabled = false;
    } else if (now - lastUsbConfigStreamMs >= 50) {
      lastUsbConfigStreamMs = now;
      sendConfigJson("status");
    }
  }

  updatePerformanceStats(loopStartUs, micros());
}
