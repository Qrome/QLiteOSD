/*
 *  QLiteOSD is an simple OSD for DJI FPV System:
 *  This is an Arduino project that handles basic OSD functions
 *  from BMP280 to a Simple Voltage Sensor to feed it 
 *  to DJI FPV System.
 *
 * ------------------------------------------------
 *
 * Copyright (C) 2023 David Payne
 * 
 * This software is based on and uses software published by Paul Kurucz (pkuruz):opentelem_to_bst_bridge
 * as well as software d3ngit : djihdfpv_mavlink_to_msp_V2 and crashsalot : VOT_to_DJIFPV
 * 
 * License info: See the LICENSE file at the repo top level
 *
 * THIS SOFTWARE IS PROVIDED IN AN "AS IS" CONDITION. NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 * TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 * IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 */

/* 
 *  QLiteOSD
 *
 *  Wemos D1 Mini (ESP8266) / Arduino Nano TX1 to DJI Air unit RX(115200)
 *  BMP280 on I2C (A4 and A5 defaults)
 *  Voltage sensor on A0 pin
 *  GPS addition by GravelAxe -- uses D7-RX and D8-TX
 */


/* ----------------------------------------------------- */

#include "config.h"
#include <MSP.h>
#include "MSP_OSD.h"
#include "OSD_positions_config.h"
#include <Adafruit_BMP280.h>  // May need to adjust for I2C address #define BMP280_ADDRESS  (0x76)
#include <Adafruit_NeoPixel.h>

#define VERSION "2.0" // compatible with QLiteOSD v2.X boards
#define BMP_ADDRESS 0x76              // default is 0x77
#define SPEED_IN_KILOMETERS_PER_HOUR  //if commented out defaults to m/s
#define FC_FIRMWARE_NAME "Betaflight"
#define FC_FIRMWARE_IDENTIFIER "BTFL"
#define CONFIG "/conf.txt"

#ifdef ESP8266
#include <FS.h>
#include "web_interface.h"
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <time.h>

static const uint8_t fileServerModePin = D3;  //Pin used to check what mode the program should start in, if high the filesystem server will be started
String ap_ssid = "QLiteOSD";
static const char *ap_psk = "12345678";
static bool fileStarted = false;
static bool fsInit = false;
static int onPinCount = 0;

ESP8266WebServer webserver(80);

static uint32_t gpsLogInterval = 500;
static File gpsLogFile;
static bool gpsLoggingStarted = false;

struct GPS_LOG_FRAME {
  float latitude;
  float longitude;
  float altitude;  //gpx uses meters for altitude
  float heading;
  float speed;
};

GPS_LOG_FRAME lastFrame;
#endif

#ifdef ESP8266
static const int gps_RX_pin = D7, gps_TX_pin = D8;  // swapped in 2.0 because D8 competes with bootloader
#else
static const int gps_RX_pin = 4, gps_TX_pin = 3;
#endif
static const uint32_t GPSBaud = 9600;
static bool activityDetected = false;

TinyGPSPlus gps;
SoftwareSerial gpsSerial(gps_RX_pin, gps_TX_pin);

#ifdef ESP8266
static const int pwm_arm_pin = D5;
static const int led_pin = D6;
#else
static const int pwm_arm_pin = 10; // pro mini arduino
static const int led_pin = 11;
#endif
static int triggerValue = 1700;
static bool fileServerOn = false;

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUM_LEDS, led_pin, NEO_GRB + NEO_KHZ800);

HardwareSerial &mspSerial = Serial;
MSP msp;
Adafruit_BMP280 bme;  // I2C

//Altitude BMP
const float PRESSURE = 1013.25;  // local air pressure
float HomeAlt = 0.0;
int sampleCount = 0;
int lastCount = 0;
float altSamples = 0.0;
static const uint8_t armAltitude = 150;  // Centimeters high at witch arm signal is sent to DJI goggles
int16_t lastAltRead = 0;                 //cm
boolean lightOn = true;
boolean flashOn = true;

//Resisters for Battery Reading
float ValueR1 = 7500.0;   //7.5K Resistor
float ValueR2 = 30000.0;  //30K Resistor
const int alanogPin = A0;
float averageVoltage = 0.0;
int sampleVoltageCount = 0;

//Other
const char fcVariant[5] = "BTFL";
uint32_t previousMillis_MSP = 0;
uint32_t previousMillis_LED = 0;
const uint32_t next_interval_MSP = 100;
uint32_t custom_mode = 0;  //flight mode
uint8_t vbat = 0;
float airspeed = 0;
int16_t groundspeed = 0;
int32_t relative_alt = 0;   // in milimeters
uint32_t altitude_msp = 0;  // EstimatedAltitudeCm
uint16_t rssi = 0;
uint8_t battery_remaining = 0;
uint32_t flightModeFlags = 0x00000002;
int16_t amperage = 0;
uint16_t mAhDrawn = 0;
float f_mAhDrawn = 0.0;
uint8_t numSat = 0;
uint8_t pid_roll[3];
uint8_t pid_pitch[3];
uint8_t pid_yaw[3];
int32_t gps_lon = 0;
int32_t gps_lat = 0;
int32_t gps_alt = 0;
double gps_home_lon = 0;
double gps_home_lat = 0;
int32_t gps_home_alt = 0;
int16_t roll_angle = 0;
int16_t pitch_angle = 0;
uint32_t distanceToHome = 0;  // distance to home in meters
int16_t directionToHome = 0;  // direction to home in degrees
uint16_t batteryCapacity = 2200;
uint8_t legacyBatteryVoltage = 0;
uint8_t batteryState = 0;  // voltage color 0==white, 1==red
uint16_t batteryVoltage = 0;
int16_t heading = 0;
uint8_t set_home = 1;
uint32_t general_counter = next_interval_MSP;
uint16_t blink_sats_orig_pos = osd_gps_sats_pos;
uint16_t blink_sats_blank_pos = 234;
uint32_t previousFlightMode = custom_mode;
int16_t climb_rate = 0;

msp_battery_state_t battery_state = { 0 };
msp_name_t name = { 0 };
msp_fc_version_t fc_version = { 0 };
msp_fc_variant_t fc_variant = { 0 };
//msp_status_BF_t status_BF = {0};
msp_status_DJI_t status_DJI = { 0 };
msp_analog_t analog = { 0 };
msp_raw_gps_t raw_gps = { 0 };
msp_comp_gps_t comp_gps = { 0 };
msp_attitude_t attitude = { 0 };
msp_altitude_t altitude = { 0 };


/* ----------------------------------------------------- */
void setup() {
  Serial.begin(115200);
  msp.begin(mspSerial);
  bme.begin(BMP_ADDRESS);  //Default Address 0x77
  pinMode(LED_BUILTIN, OUTPUT);
  
  logOnDebug("Starting!");

#ifdef ESP8266
  if (SPIFFS.begin()) {
    fsInit = true;
    readConfig();
  }else {
    logOnDebug("FS Init Fail!!");
  }
  pinMode(fileServerModePin, INPUT);
#endif

  pixels.begin();
  handleRGBled();

#ifdef LOG_GPS
  logRemoveOldFiles(10);
#endif

#ifdef USE_GPS
  gpsSerial.begin(GPSBaud);
#endif

  if (USE_PWM_ARM) {
    pinMode(pwm_arm_pin, INPUT_PULLUP);
  }

  if (!USE_DJI_RX_PIN) {
    activityDetected = true;
  }

  delay(1000);

  status_DJI.cycleTime = 0x0080;
  status_DJI.i2cErrorCounter = 0;
  status_DJI.sensor = 0x23;
  status_DJI.configProfileIndex = 0;
  status_DJI.averageSystemLoadPercent = 7;
  status_DJI.accCalibrationAxisFlags = 0;
  status_DJI.DJI_ARMING_DISABLE_FLAGS_COUNT = 20;
  status_DJI.djiPackArmingDisabledFlags = (1 << 24);
  flightModeFlags = 0x00000002;

  // Calibrate and Initialize Home Altitude
  altSamples = bme.readAltitude(PRESSURE);
}

msp_osd_config_t msp_osd_config = { 0 };

void send_osd_config() {

  if (IMPERIAL_UNITS) {
    msp_osd_config.units = 0;
  } else {
    msp_osd_config.units = 1;
  }
  
  msp_osd_config.osd_item_count = 56;
  msp_osd_config.osd_stat_count = 24;
  msp_osd_config.osd_timer_count = 2;
  msp_osd_config.osd_warning_count = 16;  // 16
  msp_osd_config.osd_profile_count = 1;   // 1
  msp_osd_config.osdprofileindex = 1;     // 1
  msp_osd_config.overlay_radio_mode = 0;  //  0

  msp_osd_config.osd_rssi_value_pos = osd_rssi_value_pos;
  msp_osd_config.osd_main_batt_voltage_pos = osd_main_batt_voltage_pos;
  msp_osd_config.osd_crosshairs_pos = osd_crosshairs_pos;
  msp_osd_config.osd_artificial_horizon_pos = osd_artificial_horizon_pos;
  msp_osd_config.osd_horizon_sidebars_pos = osd_horizon_sidebars_pos;
  msp_osd_config.osd_item_timer_1_pos = osd_item_timer_1_pos;
  msp_osd_config.osd_item_timer_2_pos = osd_item_timer_2_pos;
  msp_osd_config.osd_flymode_pos = osd_flymode_pos;
  msp_osd_config.osd_craft_name_pos = osd_craft_name_pos;
  msp_osd_config.osd_throttle_pos_pos = osd_throttle_pos_pos;
  msp_osd_config.osd_vtx_channel_pos = osd_vtx_channel_pos;
  msp_osd_config.osd_current_draw_pos = osd_current_draw_pos;
  msp_osd_config.osd_mah_drawn_pos = osd_mah_drawn_pos;
  msp_osd_config.osd_gps_speed_pos = osd_gps_speed_pos;
  msp_osd_config.osd_gps_sats_pos = osd_gps_sats_pos;
  msp_osd_config.osd_altitude_pos = osd_altitude_pos;
  msp_osd_config.osd_roll_pids_pos = osd_roll_pids_pos;
  msp_osd_config.osd_pitch_pids_pos = osd_pitch_pids_pos;
  msp_osd_config.osd_yaw_pids_pos = osd_yaw_pids_pos;
  msp_osd_config.osd_power_pos = osd_power_pos;
  msp_osd_config.osd_pidrate_profile_pos = osd_pidrate_profile_pos;
  msp_osd_config.osd_warnings_pos = osd_warnings_pos;
  msp_osd_config.osd_avg_cell_voltage_pos = osd_avg_cell_voltage_pos;
  msp_osd_config.osd_gps_lon_pos = osd_gps_lon_pos;
  msp_osd_config.osd_gps_lat_pos = osd_gps_lat_pos;
  msp_osd_config.osd_debug_pos = osd_debug_pos;
  msp_osd_config.osd_pitch_angle_pos = osd_pitch_angle_pos;
  msp_osd_config.osd_roll_angle_pos = osd_roll_angle_pos;
  msp_osd_config.osd_main_batt_usage_pos = osd_main_batt_usage_pos;
  msp_osd_config.osd_disarmed_pos = osd_disarmed_pos;
  msp_osd_config.osd_home_dir_pos = osd_home_dir_pos;
  msp_osd_config.osd_home_dist_pos = osd_home_dist_pos;
  msp_osd_config.osd_numerical_heading_pos = osd_numerical_heading_pos;
  msp_osd_config.osd_numerical_vario_pos = osd_numerical_vario_pos;
  msp_osd_config.osd_compass_bar_pos = osd_compass_bar_pos;
  msp_osd_config.osd_esc_tmp_pos = osd_esc_tmp_pos;
  msp_osd_config.osd_esc_rpm_pos = osd_esc_rpm_pos;
  msp_osd_config.osd_remaining_time_estimate_pos = osd_remaining_time_estimate_pos;
  msp_osd_config.osd_rtc_datetime_pos = osd_rtc_datetime_pos;
  msp_osd_config.osd_adjustment_range_pos = osd_adjustment_range_pos;
  msp_osd_config.osd_core_temperature_pos = osd_core_temperature_pos;
  msp_osd_config.osd_anti_gravity_pos = osd_anti_gravity_pos;
  msp_osd_config.osd_g_force_pos = osd_g_force_pos;
  msp_osd_config.osd_motor_diag_pos = osd_motor_diag_pos;
  msp_osd_config.osd_log_status_pos = osd_log_status_pos;
  msp_osd_config.osd_flip_arrow_pos = osd_flip_arrow_pos;
  msp_osd_config.osd_link_quality_pos = osd_link_quality_pos;
  msp_osd_config.osd_flight_dist_pos = osd_flight_dist_pos;
  msp_osd_config.osd_stick_overlay_left_pos = osd_stick_overlay_left_pos;
  msp_osd_config.osd_stick_overlay_right_pos = osd_stick_overlay_right_pos;
  msp_osd_config.osd_display_name_pos = osd_display_name_pos;
  msp_osd_config.osd_esc_rpm_freq_pos = osd_esc_rpm_freq_pos;
  msp_osd_config.osd_rate_profile_name_pos = osd_rate_profile_name_pos;
  msp_osd_config.osd_pid_profile_name_pos = osd_pid_profile_name_pos;
  msp_osd_config.osd_profile_name_pos = osd_profile_name_pos;
  msp_osd_config.osd_rssi_dbm_value_pos = osd_rssi_dbm_value_pos;
  msp_osd_config.osd_rc_channels_pos = osd_rc_channels_pos;

  msp.send(MSP_OSD_CONFIG, &msp_osd_config, sizeof(msp_osd_config));
}

void invert_pos(uint16_t *pos1, uint16_t *pos2) {
  uint16_t tmp_pos = *pos1;
  *pos1 = *pos2;
  *pos2 = tmp_pos;
}

void set_flight_mode_flags() {
  if (USE_PWM_ARM) {
    //USE PWM signal to ARM
    volatile int pwmValue = readChannel(pwm_arm_pin, 1000, 2000, 0);
    if ((flightModeFlags == 0x00000002) && pwmValue >= triggerValue) {
      flightModeFlags = 0x00000003;  // armed to start recording
    } else if ((flightModeFlags == 0x00000003) && pwmValue < triggerValue && general_counter % 3000 == 0) {
      flightModeFlags = 0x00000002;  // disarm after 3 second delay
    }
  } else {
    //USE Altitude to Arm
    // flightModeFlags = 0x00000003; //Uncomment this to automatically arm on start
    if ((flightModeFlags == 0x00000002) && relative_alt > armAltitude) {  // if altitude is 1 meter or more then arm to record
      flightModeFlags = 0x00000003;                                       // armed to start recording
    }
  }
}

// Read the number of a specified channel and convert to the range provided.
// If the channel is off, return the default value
int readChannel(int channelInput, int minLimit, int maxLimit, int defaultValue) {
  int ch = pulseIn(channelInput, HIGH, 30000);
  if (ch < 100) return defaultValue;
  return map(ch, 1000, 2000, minLimit, maxLimit);
}

void send_msp_to_airunit(uint8_t voltage) {

  uint8_t cellCount = getCellCount(voltage);
  //MSP_FC_VARIANT
  memcpy(fc_variant.flightControlIdentifier, fcVariant, sizeof(fcVariant));
  msp.send(MSP_FC_VARIANT, &fc_variant, sizeof(fc_variant));

  //MSP_FC_VERSION
  fc_version.versionMajor = 4;
  fc_version.versionMinor = 1;
  fc_version.versionPatchLevel = 1;
  msp.send(MSP_FC_VERSION, &fc_version, sizeof(fc_version));

  //MSP_NAME
  memcpy(name.craft_name, craftname, sizeof(craftname));
  msp.send(MSP_NAME, &name, sizeof(name));

  //MSP_STATUS
  status_DJI.flightModeFlags = flightModeFlags;
  status_DJI.armingFlags = 0x0303;
  msp.send(MSP_STATUS_EX, &status_DJI, sizeof(status_DJI));
  status_DJI.armingFlags = 0x0000;
  msp.send(MSP_STATUS, &status_DJI, sizeof(status_DJI));

  //MSP_ANALOG
  analog.vbat = voltage;
  analog.rssi = rssi;
  analog.amperage = amperage;
  analog.mAhDrawn = mAhDrawn;
  msp.send(MSP_ANALOG, &analog, sizeof(analog));

  //MSP_BATTERY_STATE
  battery_state.amperage = amperage;
  battery_state.batteryVoltage = (uint16_t)(voltage * 10);
  battery_state.mAhDrawn = mAhDrawn;
  battery_state.batteryCellCount = cellCount;
  battery_state.batteryCapacity = batteryCapacity;
  battery_state.batteryState = batteryState;
  battery_state.legacyBatteryVoltage = voltage;
  msp.send(MSP_BATTERY_STATE, &battery_state, sizeof(battery_state));

  //MSP_RAW_GPS
  raw_gps.lat = gps_lat;
  raw_gps.lon = gps_lon;
  raw_gps.numSat = numSat;
  raw_gps.alt = gps_alt;
  raw_gps.groundSpeed = groundspeed;  //in cm/s
  msp.send(MSP_RAW_GPS, &raw_gps, sizeof(raw_gps));

  //MSP_COMP_GPS
  comp_gps.distanceToHome = (int16_t)distanceToHome;
  comp_gps.directionToHome = directionToHome - heading;
  msp.send(MSP_COMP_GPS, &comp_gps, sizeof(comp_gps));

  //MSP_ATTITUDE
  attitude.pitch = pitch_angle * 10;
  attitude.roll = roll_angle * 10;
  msp.send(MSP_ATTITUDE, &attitude, sizeof(attitude));

  //MSP_ALTITUDE
  altitude.estimatedActualPosition = relative_alt;
  altitude.estimatedActualVelocity = (int16_t)(climb_rate);  //m/s to cm/s
  msp.send(MSP_ALTITUDE, &altitude, sizeof(altitude));

  //MSP_OSD_CONFIG
  send_osd_config();
}


void blink_sats() {
  if (general_counter % 900 == 0 && set_home == 1 && blink_sats_orig_pos > 2000) {
    invert_pos(&osd_gps_sats_pos, &blink_sats_blank_pos);
  } else if (set_home == 0) {
    osd_gps_sats_pos = blink_sats_orig_pos;
  }
}

uint8_t getCellCount(uint8_t voltage) {
  uint8_t batteryCellCount = 0;
  if (voltage < 43 && voltage > 0) batteryCellCount = 1;
  else if (voltage < 85) batteryCellCount = 2;
  else if (voltage < 127) batteryCellCount = 3;
  else if (voltage < 169) batteryCellCount = 4;
  else if (voltage < 211) batteryCellCount = 5;
  else if (voltage < 255) batteryCellCount = 6;

  return batteryCellCount;
}

void readAltitude() {
  if (HomeAlt == 0.0) {
    calibrateHome();
  }
  altSamples += bme.readAltitude(PRESSURE);
  sampleCount++;
}

void getAltitudeSample() {
  lastAltRead = (int16_t)relative_alt;
  relative_alt = (int)round(((altSamples / sampleCount) - HomeAlt) * 100);
  lastCount = sampleCount;
  sampleCount = 0;
  altSamples = 0.0;
  climb_rate = (lastAltRead - relative_alt) * 5;  // assuming 200ms samples needs to be cm/s
}

void calibrateHome() {
  HomeAlt = 0.0;
  sampleCount = 1;
  while (HomeAlt == 0.0) {
    // Build the first sample of readings for Calibration
    altSamples += bme.readAltitude(PRESSURE);
    sampleCount++;
    if (sampleCount == 20) {
      HomeAlt = altSamples / (sampleCount);
      sampleCount = 0;
      altSamples = 0.0;
    }
    delay(100);
  }
}

void readVoltage() {
  int readValue = analogRead(alanogPin);
  averageVoltage += (readValue * (arduinoVCC / 1024.0)) * (1 + (ValueR2 / ValueR1));
  sampleVoltageCount++;
}

void getVoltageSample() {
  vbat = (uint8_t)((averageVoltage / sampleVoltageCount) * 10);
  sampleVoltageCount = 0;
  averageVoltage = 0.0;
}

#ifdef USE_GPS
void readGPS() {
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }
  if (gps.location.isValid()) {
    gps_lat = (int32_t)(gps.location.lat() * 10000000);
    gps_lon = (int32_t)(gps.location.lng() * 10000000);
    if (gps.satellites.isValid()) {
      numSat = gps.satellites.value();
    }
    gps_alt = gps.altitude.meters();
    groundspeed = (int16_t)(gps.speed.kmph() * 100000 / 3600);  //in cm/s
    heading = gps.course.deg();

    if (set_home == 1 && gps.hdop.isValid() && gps.hdop.hdop() < 2) {
      gps_home_lat = gps.location.lat();
      gps_home_lon = gps.location.lng();
      set_home = 0;
    }
    distanceToHome = (unsigned long)(TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), gps_home_lat, gps_home_lon));
    directionToHome = TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(), gps_home_lat, gps_home_lon);
  }
}
#endif

void loop() {
  uint32_t currentMillis_MSP = millis();
  if ((rgb_mode != "OFF" && rgb_mode != "ON") && (uint32_t)(currentMillis_MSP - previousMillis_LED) >= (next_interval_MSP * 10)) {
    previousMillis_LED = currentMillis_MSP;
    handleRGBled(); // only go here if LED can change during flight
  }

#ifdef ESP8266
  checkTurnOnFileServer();
  if (fileServerOn) {
    digitalWrite(LED_BUILTIN, LOW);
    webserver.handleClient();
    return;
  }
#endif

  readAltitude();
  readVoltage();
#ifdef USE_GPS
  readGPS();
#endif

  if ((uint32_t)(currentMillis_MSP - previousMillis_MSP) >= next_interval_MSP) {
    previousMillis_MSP = currentMillis_MSP;

#ifdef LOG_GPS
    logGPS();
#endif

    if (general_counter % 200 == 0) {  // update the altitude values every 200ms
      getAltitudeSample();
      if (lightOn) {
        digitalWrite(LED_BUILTIN, LOW);
      } else {
        digitalWrite(LED_BUILTIN, HIGH);
      }
      lightOn = !lightOn;
    }
    getVoltageSample();
    set_flight_mode_flags();
    blink_sats();

    // SEND the data out
#ifdef DEBUG
    debugPrint();
#else
  if (activityDetected) {
    send_msp_to_airunit(vbat); // SEND the data to the DJI unit
  } else {
    if (msp.activityDetected()) {
      activityDetected = true;
      logOnDebug("****************** Found DJI MSP Activity ******************");
    } else {
      logOnDebug("*** Waiting for MSP Activity from DJI Unit ***");
    }
  }
    
#endif
    general_counter += next_interval_MSP;
  }
}


void handleRGBled() {
  if (rgb_mode == "OFF") {
    pixels.fill(pixels.Color(0, 0, 0), 0, NUM_LEDS);
    pixels.show();
    return;
  }
  if (fileServerOn) {
    pixels.fill(pixels.Color(0, 0, 155), 0, NUM_LEDS);
    pixels.show();
    return;
  }
  if (rgb_mode == "ON") {
    pixels.fill(pixels.Color(redColor, greenColor, blueColor), 0, NUM_LEDS);
    pixels.show();
    return;
  }
  flashOn = !flashOn;
  if (rgb_mode == "BATTERY") {
    int cellVoltage = (int)((vbat / getCellCount(vbat)));
    int greenValue = map(cellVoltage, 36, 42, 0, 255); // Green decreases as voltage decreases
    int redValue = map(cellVoltage, 36, 42, 255, 0);   // Red increases as voltage decreases

    if (cellVoltage >= 42) {
      greenValue = 255;
      redValue = 0;
    } else if (cellVoltage <= 36) {
      greenValue = 0;
      redValue = 255;
    }

    if (flashOn && cellVoltage <= 36) {
      pixels.fill(pixels.Color(0, 0, 0), 0, NUM_LEDS);
    } else {
      pixels.fill(pixels.Color(redValue, greenValue, 0), 0, NUM_LEDS);
    }
    pixels.show();
    return;
  }
  if (rgb_mode == "ALTITUDE") {
    int greenValue = map(relative_alt, 0, 12192, 255, 0); // Green decreases as voltage decreases
    int redValue = map(relative_alt, 0, 12192, 0, 255);   // Red increases as voltage decreases

    if (relative_alt >= 12192) {
      greenValue = 0;
      redValue = 255;
    } else if (relative_alt <= 0) {
      greenValue = 255;
      redValue = 0;
    }
  
    if (flashOn && relative_alt >= 12192) {
      pixels.fill(pixels.Color(0, 0, 0), 0, NUM_LEDS);
    } else {
      pixels.fill(pixels.Color(redValue, greenValue, 0), 0, NUM_LEDS);
    }
    pixels.show();
    return;
  }
  if (rgb_mode == "STROBE") {
    if (flashOn) {
      pixels.fill(pixels.Color(redColor, greenColor, blueColor), 0, NUM_LEDS);
    } else {
      pixels.fill(pixels.Color(0, 0, 0), 0, NUM_LEDS);
    }
    pixels.show();
    return;
  }
  if (rgb_mode == "AIRCRAFT") {
    pixels.fill(pixels.Color(0, 0, 0), 0, 24);
    pixels.fill(pixels.Color(0, 255, 0), 0, 4);
    pixels.fill(pixels.Color(255, 255, 255), 4, 4);
    pixels.fill(pixels.Color(255, 0, 0), 8, 4);
    if (!flashOn) {
      pixels.fill(pixels.Color(0, 0, 0), 4, 4);
    }
    pixels.show();
    return;
  }
}


#ifdef DEBUG
//*** USED ONLY FOR DEBUG ***
void debugPrint() {
  mspSerial.println("**********************************");
  mspSerial.print("Craft Name: ");
  mspSerial.println(String(craftname));
  mspSerial.print("Imperial: ");
  mspSerial.println(IMPERIAL_UNITS);
  mspSerial.print("Flight Mode: ");
  mspSerial.println(flightModeFlags);
  mspSerial.print("Voltage: ");
  mspSerial.println(((double)vbat / 10), 1);
  mspSerial.print("Cell Count: ");
  mspSerial.println(getCellCount(vbat));
  mspSerial.print("Altitude (cm): ");
  mspSerial.println(relative_alt);
  mspSerial.print("Climb Rate (cm/s): ");
  mspSerial.println(climb_rate);
  mspSerial.print("Sample Count / transmit: ");
  mspSerial.println(lastCount);
  if (USE_PWM_ARM) {
    mspSerial.print("PWM Value: ");
    mspSerial.println(readChannel(pwm_arm_pin, 1000, 2000, 0));
  }
#ifdef USE_GPS
  mspSerial.print("Lat: ");
  mspSerial.println(gps_lat);
  mspSerial.print("Lon: ");
  mspSerial.println(gps_lon);
  mspSerial.print("Num Sat: ");
  mspSerial.println(numSat);
  mspSerial.print("GPS Alt: ");
  mspSerial.println(gps_alt);
  mspSerial.print("Speed: ");
  mspSerial.println(groundspeed);
  mspSerial.print("Heading: ");
  mspSerial.println(heading);
  mspSerial.print("Home Set: ");
  mspSerial.println(set_home);
  mspSerial.print("HOME Lat: ");
  mspSerial.println(gps_home_lat, 6);
  mspSerial.print("HOME Lon: ");
  mspSerial.println(gps_home_lon, 6);
  mspSerial.print("Distance: ");
  mspSerial.println(distanceToHome);
  mspSerial.print("Direction Home: ");
  mspSerial.println(directionToHome);
#endif
}
#endif

void logOnDebug(String inValue) {
#ifdef DEBUG
  mspSerial.println(inValue);
#endif
}

#ifdef ESP8266
//Only used with GPS Option
void logGPSFrame() {
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }
  if (gpsLoggingStarted == false) {
    if (gps.location.isValid()) {
      tm currentTime;
      currentTime.tm_year = gps.date.year() - 1900;
      currentTime.tm_mon = gps.date.month() - 1;
      currentTime.tm_mday = gps.date.day();
      currentTime.tm_hour = gps.time.hour();
      currentTime.tm_min = gps.time.minute();
      currentTime.tm_sec = gps.time.second();

      time_t timeStamp = mktime(&currentTime);
      gpsLogFile.write((uint8_t *)&timeStamp, sizeof(time_t));  //Timestamp is stored at the neginning of the file
      gpsLoggingStarted = true;
    }
  }
  if (!gpsLoggingStarted) {
    return;
  }
  if (!gps.location.isValid()) {
    if (lastFrame.latitude != 0.0f) {
      gpsLogFile.write((uint8_t *)&lastFrame, sizeof(GPS_LOG_FRAME));
    }
    return;
  }
  GPS_LOG_FRAME logFrame;
  logFrame.altitude = (altSamples / sampleCount);  //altitude in meters

  logFrame.latitude = gps.location.lat();
  logFrame.longitude = gps.location.lng();
  logFrame.heading = gps.course.deg();
  logFrame.speed = gps.speed.kmph() / 3.6f;
  lastFrame = logFrame;
  gpsLogFile.write((uint8_t *)&logFrame, sizeof(GPS_LOG_FRAME));
}

uint32_t logGetNextFileNum() {
  uint32_t maxNum = 1;
  Dir dir = SPIFFS.openDir("/");
  File file = dir.openFile("r");
  while (dir.next()) {
    if (dir.fileName() != CONFIG) {
      maxNum++;
    }
  }
  return maxNum;
}

void logRemoveOldFiles(uint32_t fileLimit) {
  uint32_t nextFile = logGetNextFileNum();
  if (fileLimit >= nextFile) {
    return;
  }
  //If next file is 16, we need to remove 16-10=6 files to get the FS back to 9 files
  //The process for this will be removing files 1-6, Then renaming 7 -> 1, 8 -> 2,...,and 15->9, so 10 will be the next file created when the system is armed
  int filesToRemove = nextFile - fileLimit; //16 - 10 = 6
  logOnDebug("Files to Remove: " + String(filesToRemove));
  for (int i = 1; i < filesToRemove + 1; i++) {
    logOnDebug("Delete: " + String("/") + String(i));
    SPIFFS.remove(String("/") + String(i));
  }
  int filesToRename = fileLimit;
  for (int i = 1; i < filesToRename; i++) {
    String newFile = String("/") + String(i);
    String oldFile = String("/") + String(i + filesToRemove);
    SPIFFS.rename(oldFile, newFile);
  }
}

void logFileCreate() {
  uint32_t nextFileNum = logGetNextFileNum();
#ifdef DEBUG
  Serial.println(String("Attempting to create file with name: ") + String(nextFileNum));
#endif
  String fileName = String("/") + String(nextFileNum);
  gpsLogFile = SPIFFS.open(fileName, "w");  //need to test what turning off without closing the file does
  fileStarted = gpsLogFile;
#ifdef DEBUG
  Serial.println("File Created!");
#endif
}

void logGPS() {
  if (flightModeFlags != 3 || fsInit == false) {
    return;
  }
#ifdef DEBUG
  Serial.println("LOG Gps");
#endif
  if (fileStarted == false) {
    logFileCreate();
  }
  if (general_counter % gpsLogInterval == 0) {
    logGPSFrame();
  }
}

void checkTurnOnFileServer() {
  if (digitalRead(fileServerModePin) == LOW && !fileServerOn) {
    uint32_t currentMillis = millis();
    if (onPinCount == 0) {
      onPinCount = currentMillis;
    }
    if ((uint32_t)(currentMillis - onPinCount) < 3000) {  // only turn on if held for 3 seconds
      return;
    }
    gpsSerial.end();
    //Begin fileserver
    fileServerOn = true;
    ap_ssid += "-" + String(ESP.getChipId(), HEX);
    WiFi.softAP(((const char *)ap_ssid.c_str()), ap_psk);
    webserver.on("/", showFiles);                 //Show all logged gps files
    webserver.on("/download", downloadFile);      //Convert and download a gpx file
    webserver.on("/delete", deleteFiles);         //Delete all files
    webserver.on("/wifioff", turnWifiOff);        //Turn off AP Wifi
    webserver.on("/configure", handleConfigure);  //Show the Configure Page
    webserver.on("/saveconfig", handleSaveConfig);//Save the Configure Page
    webserver.on("/osd", handleOSD);  //Show the Configure Page
    webserver.on("/saveosd", handleSaveOSD);//Save the Configure Page
    webserver.on("/reset", handleSystemReset);    //Reset Device
    webserver.begin();
    digitalWrite(LED_BUILTIN, LOW);
    logOnDebug("*** FILE SERVER ON ***");
    delay(2000);
  }
  onPinCount = 0;
}

void turnWifiOff() {
  sendHeader();
  webserver.sendContent("Wifi Turned Off. <a href='/'>Return to main screen</a> when reconnected.");
  webserver.sendContent(getBodyEnd());
  webserver.sendContent("");
  webserver.client().stop();
  fileServerOn = false;
  digitalWrite(LED_BUILTIN, HIGH);
  WiFi.softAPdisconnect(true);
  ESP.reset();
}

void sendHeader() {
  webserver.sendHeader("Cache-Control", "no-cache, no-store");
  webserver.sendHeader("Pragma", "no-cache");
  webserver.sendHeader("Expires", "-1");
  webserver.setContentLength(CONTENT_LENGTH_UNKNOWN);
  webserver.send(200, "text/html", "");
  webserver.sendContent(FPSTR(HEAD_TITLE));
  webserver.sendContent(FPSTR(BODY_MENU));
}

String getBodyEnd() {
  String html = FPSTR(BODY_END);
  html.replace("%VERSION%", String(VERSION));
  return html;
}

void sendFooter() {
  webserver.sendContent(getBodyEnd());
  webserver.sendContent("");
  webserver.client().stop();
}

String getFileName() {
  return "QL-" + String(ESP.getChipId(), HEX) + "-";
}

void handleConfigure() {
  sendHeader();

  String form = FPSTR(CONFIG_FORM);
  form.replace("%CRAFTNAME%", String(craftname));

  String isImperialChecked = "";
  if (IMPERIAL_UNITS) {
    isImperialChecked = "checked='checked'";
  }
  form.replace("%USEIMPERIALCHECKED%", isImperialChecked);

  String isUsePwmChecked = "";
  if (USE_PWM_ARM) {
    isUsePwmChecked = "checked='checked'";
  }
  form.replace("%USEPWMCHECKED%", isUsePwmChecked);
  form.replace("%VCC%", String(arduinoVCC));
  
  String rgbOptions = FPSTR(RGB_OPTIONS);
  rgbOptions.replace(">" + String(rgb_mode) + "<", " selected>" + String(rgb_mode) + "<");
  form.replace("%RGB_OPTIONS%", rgbOptions);

  form.replace("%RED%", String(redColor));
  form.replace("%GREEN%", String(greenColor));
  form.replace("%BLUE%", String(blueColor));
  
  webserver.sendContent(form);
  webserver.sendContent(FPSTR(RGB_JS));
  form = FPSTR(VOLT_JS);
  form.replace("%R1%", String(ValueR1));
  form.replace("%R2%", String(ValueR2));
  form.replace("%READVALUE%", String(analogRead(alanogPin)));
  webserver.sendContent(form);
  sendFooter();
}

void handleSaveConfig() {
  String temp = webserver.arg("craftname_form");
  temp.toCharArray(craftname, sizeof(temp));
  IMPERIAL_UNITS = webserver.hasArg("use_imperial_form");
  USE_PWM_ARM = webserver.hasArg("use_pwm_arm_form");
  rgb_mode = webserver.arg("rgbmode");
  redColor = webserver.arg("red").toInt();
  greenColor = webserver.arg("green").toInt();
  blueColor = webserver.arg("blue").toInt();
  arduinoVCC = webserver.arg("numericInput").toFloat();

  writeConfig();
  redirectHome();
}

void handleOSD() {
  webserver.sendHeader("Cache-Control", "no-cache, no-store");
  webserver.sendHeader("Pragma", "no-cache");
  webserver.sendHeader("Expires", "-1");
  webserver.setContentLength(CONTENT_LENGTH_UNKNOWN);
  webserver.send(200, "text/html", "");
  webserver.sendContent(FPSTR(HEAD_TITLE));
  webserver.sendContent(FPSTR(OSD_JS));
  webserver.sendContent(FPSTR(BODY_MENU));

  String form = FPSTR(OSD_DISPLAY);

  form.replace("%ALT_VAL%", String(osd_altitude_pos));
  form.replace("%CELL_VAL%", String(osd_avg_cell_voltage_pos));
  form.replace("%BAT_VAL%", String(osd_main_batt_voltage_pos));
  form.replace("%CRAFT_VAL%", String(osd_craft_name_pos));
  form.replace("%SATS_VAL%", String(blink_sats_orig_pos));
  form.replace("%HOMEARROW_VAL%", String(osd_home_dir_pos));
  form.replace("%DIST_VAL%", String(osd_home_dist_pos));
  form.replace("%SPEED_VAL%", String(osd_gps_speed_pos));
  form.replace("%LAT_VAL%", String(osd_gps_lat_pos));
  form.replace("%LON_VAL%", String(osd_gps_lon_pos));
  form.replace("%CROSS_VAL%", String(osd_crosshairs_pos));

  webserver.sendContent(form);

  sendFooter();
}

void handleSaveOSD() {

  osd_altitude_pos = strtol(webserver.arg("altitude_pos").c_str(), NULL, 0);
  osd_avg_cell_voltage_pos = strtol(webserver.arg("cell_voltage_pos").c_str(), NULL, 0);
  osd_main_batt_voltage_pos = strtol(webserver.arg("main_batt_voltage_pos").c_str(), NULL, 0);
  osd_craft_name_pos = strtol(webserver.arg("craft_name_pos").c_str(), NULL, 0);
  osd_gps_sats_pos = strtol(webserver.arg("gps_sats_pos").c_str(), NULL, 0);
  osd_home_dir_pos = strtol(webserver.arg("home_dir_pos").c_str(), NULL, 0);
  osd_home_dist_pos = strtol(webserver.arg("home_dist_pos").c_str(), NULL, 0);
  osd_gps_speed_pos = strtol(webserver.arg("gps_speed_pos").c_str(), NULL, 0);
  osd_gps_lat_pos = strtol(webserver.arg("gps_lat_pos").c_str(), NULL, 0);
  osd_gps_lon_pos = strtol(webserver.arg("gps_lon_pos").c_str(), NULL, 0);
  osd_crosshairs_pos = strtol(webserver.arg("crosshairs_pos").c_str(), NULL, 0);

  blink_sats_orig_pos = osd_gps_sats_pos; // update the origin position for sats osd item

  writeConfig();
  redirectHome();
}

void redirectHome() {
  // Send them back to the Root Directory
  webserver.sendHeader("Location", String("/"), true);
  webserver.sendHeader("Cache-Control", "no-cache, no-store");
  webserver.sendHeader("Pragma", "no-cache");
  webserver.sendHeader("Expires", "-1");
  webserver.send(302, "text/plain", "");
  webserver.client().stop();
  delay(1000);
}

void showFiles() {

  if (fsInit == false) {
    webserver.send(200, "text/html", "Error: Filesystem failed to init!");
    return;
  }

  sendHeader();

  String webpage = "<h2>QLiteOSD v" + String(VERSION) + " GPS Log Files</h2><ul>";

  Dir dir = SPIFFS.openDir("/");
  String lastFileNum = String("");
  while (dir.next()) {
    if (dir.fileName() == CONFIG) {
      continue;  // don't look at the config file
    }
    String fileNum = dir.fileName();
    if (lastFileNum == fileNum) {
      break; //In the chance that there is a duplicate at the end, break
    }
    lastFileNum = fileNum;
    String fileName = fileNum;
    fileName.replace("/", "");
    File file = dir.openFile("r");
    time_t time = 0;
    file.readBytes((char *)&time, sizeof(time_t));
    fileName = getFileName() + String(time) + ".gpx";
    webpage += "<li><a href='download?download=" + fileNum + "'>" + fileName + "</a>  &nbsp;&nbsp;" + fileSize(file.size()) + "&nbsp;&nbsp;<span class='time'>" + String(time) + "</span><br/></li>";
  };

  FSInfo fsInfo;
  SPIFFS.info(fsInfo);

  webpage += "</ul>\n";
  webpage += "Space Used: <strong>" + fileSize(fsInfo.usedBytes) + "</strong> | ";
  webpage += "Free Space: <strong>" + fileSize(fsInfo.totalBytes) + "</strong>";
  webpage += "<br/><br/><a href='/delete' onclick='return confirm(\"Do you want to delete all files?\")'><strong>Delete All Files</strong></a>";
  webpage += "<script>var times = document.getElementsByClassName('time'); for (var i = 0; i < times.length; i++) {times.item(i).innerHTML = new Date(times.item(i).innerHTML*1000).toLocaleString();}</script>";

  webserver.sendContent(webpage);

  sendFooter();
}

String fileSize(int bytes) {
  String fsize = "";
  if (bytes < 1024) fsize = String(bytes) + " B";
  else if (bytes < (1024 * 1024)) fsize = roundValue(String(bytes / 1024.0, 3)) + " KB";
  else if (bytes < (1024 * 1024 * 1024)) fsize = roundValue(String(bytes / 1024.0 / 1024.0, 3)) + " MB";
  else fsize = roundValue(String(bytes / 1024.0 / 1024.0 / 1024.0, 3)) + " GB";
  return fsize;
}

String roundValue(String inValue) {
  float x = inValue.toFloat();
  long f = (long)(x * 10L);
  x = (float)f / 10.0;
  return String(x, 1);
}

void downloadFile() {
  static const char *gpxcloser = "</trkseg></trk>\n</gpx>\n";

  if (webserver.args() > 0) {
    if (webserver.hasArg("download")) {
      String filename = webserver.arg(0);
      File rawDataFile = SPIFFS.open(filename, "r");
      time_t startTime = 0;
      rawDataFile.readBytes((char *)&startTime, sizeof(uint64_t));
      String nameAndTimeStamp = getFileName() + String(startTime);
      WiFiClient client = webserver.client();
      size_t fileSize = rawDataFile.size();
      size_t numFrames = (fileSize / sizeof(GPS_LOG_FRAME));
      client.print("HTTP/1.1 200 OK\r\n");
      client.print("Content-Disposition: attachment; filename=" + nameAndTimeStamp + ".gpx\r\n");
      client.print("Content-Type: application/octet-stream\r\n");
      client.print("Connection: close\r\n");
      client.print("Access-Control-Allow-Origin: *\r\n");
      client.print("\r\n");
      String gpxheader = "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<gpx version=\"1.0\">\n\t<name>QLiteOSD v" + String(VERSION) + 
        "</name>\n\t<trk><name>" + nameAndTimeStamp + "</name><number>1</number><trkseg>\n";
      client.print(gpxheader);

      char buf[sizeof("0000-00-00T00:00:00.000Z") + 5];
      char fullbuf[sizeof("\t\t<trkpt lat=\"000.00000000\" lon=\"000.00000000\"><ele>0000</ele><time>0000-00-00T00:00:00.000Z</time><speed>000.000</speed><course>000.000</course></trkpt>") + 30];
      GPS_LOG_FRAME logData;
      for (size_t i = 0; i < numFrames; i++) {
        String gpxFrame;
        time_t currentTime = startTime + (i / 2);
        strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%S", gmtime(&currentTime));
        char *msecVal = ".000";
        if (i % 2 != 0) {  //Currently hardcoded for 500 ms intervals
          msecVal = ".500";
        }
        String fullTime = String(buf) + String(msecVal) + "Z";
        rawDataFile.readBytes((char *)&logData, sizeof(GPS_LOG_FRAME));
        sprintf(fullbuf, "\t\t<trkpt lat=\"%f\" lon=\"%f\"><ele>%f</ele><time>%s</time><speed>%f</speed><course>%f</course></trkpt>\n", logData.latitude, logData.longitude, logData.altitude, fullTime.c_str(), logData.speed, logData.heading);
        client.write(fullbuf);
      }
      client.write(gpxcloser);
      client.write("\n");
      client.stop();
      rawDataFile.close();
    }
  }
}

void deleteFiles() {
  logRemoveOldFiles(0);
  delay(1000);
  webserver.sendHeader("Location", String("/"), true);
  webserver.sendHeader("Cache-Control", "no-cache, no-store");
  webserver.sendHeader("Pragma", "no-cache");
  webserver.sendHeader("Expires", "-1");
  webserver.send(302, "text/plain", "");
  webserver.client().stop();
}
#endif

#ifdef ESP8266

void handleSystemReset() {
  logOnDebug("Reset System Configuration");
  if (SPIFFS.format()) {
    redirectHome();
    ESP.restart();
  }
}


void writeConfig() {
  // Save settings to file for playback on power up.
  File f = SPIFFS.open(CONFIG, "w");
  if (!f) {
    logOnDebug(String(CONFIG) + " file failed to open!");
  } else {
    logOnDebug("Saving settings now...");
    f.println("craftname=" + String(craftname));
    f.println("IMPERIAL_UNITS=" + String(IMPERIAL_UNITS));
    f.println("USE_PWM_ARM=" + String(USE_PWM_ARM));
    f.println("osd_altitude_pos=" + String(osd_altitude_pos));
    f.println("osd_avg_cell_voltage_pos=" + String(osd_avg_cell_voltage_pos));
    f.println("osd_main_batt_voltage_pos=" + String(osd_main_batt_voltage_pos));
    f.println("osd_craft_name_pos=" + String(osd_craft_name_pos));
    f.println("osd_gps_sats_pos=" + String(blink_sats_orig_pos));
    f.println("osd_home_dir_pos=" + String(osd_home_dir_pos));
    f.println("osd_home_dist_pos=" + String(osd_home_dist_pos));
    f.println("osd_gps_speed_pos=" + String(osd_gps_speed_pos));
    f.println("osd_gps_lat_pos=" + String(osd_gps_lat_pos));
    f.println("osd_gps_lon_pos=" + String(osd_gps_lon_pos));
    f.println("osd_crosshairs_pos=" + String(osd_crosshairs_pos));
    f.println("rgb_mode=" + rgb_mode);
    f.println("redColor=" + String(redColor));
    f.println("greenColor=" + String(greenColor));
    f.println("blueColor=" + String(blueColor));
    f.println("arduinoVCC=" + String(arduinoVCC));
  }
  f.close();
  readConfig();
}

void readConfig() {
  if (SPIFFS.exists(CONFIG) == false) {
    logOnDebug("Settings File does not yet exists.");
    writeConfig();
    return;
  }
  File fr = SPIFFS.open(CONFIG, "r");
  String line;
  while (fr.available()) {
    line = fr.readStringUntil('\n');
    if (line.indexOf("craftname=") >= 0) {
      String temp = line.substring(line.lastIndexOf("craftname=") + 10);
      temp.trim();
      temp.toCharArray(craftname, sizeof(temp));
      logOnDebug("craftname: " + String(craftname));
    }
    if (line.indexOf("IMPERIAL_UNITS=") >= 0) {
      IMPERIAL_UNITS = line.substring(line.lastIndexOf("IMPERIAL_UNITS=") + 15).toInt();
      logOnDebug("IMPERIAL_UNITS: " + String(IMPERIAL_UNITS));
    }
    if (line.indexOf("USE_PWM_ARM=") >= 0) {
      USE_PWM_ARM = line.substring(line.lastIndexOf("USE_PWM_ARM=") + 12).toInt();
      logOnDebug("USE_PWM_ARM: " + String(USE_PWM_ARM));
    }
    //OSD Items
    if (line.indexOf("osd_altitude_pos=") >= 0) {
      osd_altitude_pos = line.substring(line.lastIndexOf("osd_altitude_pos=") + 17).toInt();
      logOnDebug("osd_altitude_pos: " + String(osd_altitude_pos));
    }
    if (line.indexOf("osd_avg_cell_voltage_pos=") >= 0) {
      osd_avg_cell_voltage_pos = line.substring(line.lastIndexOf("osd_avg_cell_voltage_pos=") + 25).toInt();
      logOnDebug("osd_avg_cell_voltage_pos: " + String(osd_avg_cell_voltage_pos));
    }
    if (line.indexOf("osd_main_batt_voltage_pos=") >= 0) {
      osd_main_batt_voltage_pos = line.substring(line.lastIndexOf("osd_main_batt_voltage_pos=") + 26).toInt();
      logOnDebug("osd_main_batt_voltage_pos: " + String(osd_main_batt_voltage_pos));
    }
    if (line.indexOf("osd_craft_name_pos=") >= 0) {
      osd_craft_name_pos = line.substring(line.lastIndexOf("osd_craft_name_pos=") + 19).toInt();
      logOnDebug("osd_craft_name_pos: " + String(osd_craft_name_pos));
    }
    if (line.indexOf("osd_gps_sats_pos=") >= 0) {
      osd_gps_sats_pos = line.substring(line.lastIndexOf("osd_gps_sats_pos=") + 17).toInt();
      logOnDebug("osd_gps_sats_pos: " + String(osd_gps_sats_pos));
    }
    if (line.indexOf("osd_home_dir_pos=") >= 0) {
      osd_home_dir_pos = line.substring(line.lastIndexOf("osd_home_dir_pos=") + 17).toInt();
      logOnDebug("osd_home_dir_pos: " + String(osd_home_dir_pos));
    }
    if (line.indexOf("osd_home_dist_pos=") >= 0) {
      osd_home_dist_pos = line.substring(line.lastIndexOf("osd_home_dist_pos=") + 18).toInt();
      logOnDebug("osd_home_dist_pos: " + String(osd_home_dist_pos));
    }
    if (line.indexOf("osd_gps_speed_pos=") >= 0) {
      osd_gps_speed_pos = line.substring(line.lastIndexOf("osd_gps_speed_pos=") + 18).toInt();
      logOnDebug("osd_gps_speed_pos: " + String(osd_gps_speed_pos));
    }
    if (line.indexOf("osd_gps_lat_pos=") >= 0) {
      osd_gps_lat_pos = line.substring(line.lastIndexOf("osd_gps_lat_pos=") + 16).toInt();
      logOnDebug("osd_gps_lat_pos: " + String(osd_gps_lat_pos));
    }
    if (line.indexOf("osd_gps_lon_pos=") >= 0) {
      osd_gps_lon_pos = line.substring(line.lastIndexOf("osd_gps_lon_pos=") + 16).toInt();
      logOnDebug("osd_gps_lon_pos: " + String(osd_gps_lon_pos));
    }
    if (line.indexOf("osd_crosshairs_pos=") >= 0) {
      osd_crosshairs_pos = line.substring(line.lastIndexOf("osd_crosshairs_pos=") + 19).toInt();
      logOnDebug("osd_crosshairs_pos: " + String(osd_crosshairs_pos));
    }
    if (line.indexOf("rgb_mode=") >= 0) {
      rgb_mode = line.substring(line.lastIndexOf("rgb_mode=") + 9);
      rgb_mode.trim();
      logOnDebug("rgb_mode: " + rgb_mode);
    }
    if (line.indexOf("redColor=") >= 0) {
      redColor = line.substring(line.lastIndexOf("redColor=") + 9).toInt();
      logOnDebug("redColor: " + String(redColor));
    }
    if (line.indexOf("greenColor=") >= 0) {
      greenColor = line.substring(line.lastIndexOf("greenColor=") + 11).toInt();
      logOnDebug("greenColor: " + String(greenColor));
    }
    if (line.indexOf("blueColor=") >= 0) {
      blueColor = line.substring(line.lastIndexOf("blueColor=") + 10).toInt();
      logOnDebug("blueColor: " + String(blueColor));
    }
    if (line.indexOf("arduinoVCC=") >= 0) {
      arduinoVCC = line.substring(line.lastIndexOf("arduinoVCC=") + 11).toFloat();
      logOnDebug("arduinoVCC: " + String(arduinoVCC));
    }

    blink_sats_orig_pos = osd_gps_sats_pos; // set original position to the new position

  }
  fr.close();
}
#endif