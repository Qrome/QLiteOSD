/*
 *  QLiteOSD is an simple OSD for DJI FPV System:
 *  This is an Arduino project that handles basic OSD functions
 *  from BMP280 to a Simple Voltage Sensor to feed it 
 *  to DJI FPV System.
 *
 * ------------------------------------------------
 *
 * Copyright (C) 2022 David Payne
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
 *  Arduino Nano TX1 to DJI Air unit RX(115200)
 *  BMP280 on I2C (A4 and A5 defaults)
 *  Voltage sensor on A0 pin
 *  GPS addition by GravelAxe -- uses D7-RX and D8-TX
 */


/* ----------------------------------------------------- */

#include <MSP.h>
#include "MSP_OSD.h"
#include "OSD_positions_config.h"
#include <Adafruit_BMP280.h>  // May need to adjust for I2C address #define BMP280_ADDRESS  (0x76)

#define VERSION "1.0"
#define BMP_ADDRESS 0x76              // default is 0x77
#define MAH_CALIBRATION_FACTOR 1.0f   //used to calibrate mAh reading.
#define SPEED_IN_KILOMETERS_PER_HOUR  //if commented out defaults to m/s
#define IMPERIAL_UNITS                //Altitude in feet, distance to home in miles.
#define FC_FIRMWARE_NAME "Betaflight"
#define FC_FIRMWARE_IDENTIFIER "BTFL"

#ifdef USE_GPS
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

#ifdef ESP8266
static const int gps_RX_pin = D7, gps_TX_pin = D8;
#else
static const int gps_RX_pin = 4, gps_TX_pin = 3;
#endif
static const uint32_t GPSBaud = 9600;

#ifdef LOG_GPS
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <SPI.h>
#include <FS.h>
#include <time.h>
static const uint8_t fileServerModePin = D3;  //Pin used to check what mode the program should start in, if high the filesystem server will be started
static bool fileServerOn = false;
static uint32_t gpsLogInterval = 500;
String ap_ssid = "QLiteOSD";
static const char *ap_psk = "12345678";
static File gpsLogFile;
static bool fsInit = false;
static bool fileStarted = false;
static int onPinCount = 0;
static bool gpsLoggingStarted = false;

struct GPS_LOG_FRAME {
  float latitude;
  float longitude;
  float altitude;  //gpx uses meters for altitude
  float heading;
  float speed;
};

GPS_LOG_FRAME lastFrame;

ESP8266WebServer webserver(80);
#endif

TinyGPSPlus gps;
SoftwareSerial gpsSerial(gps_RX_pin, gps_TX_pin);
//#define STORE_GPS_LOCATION_IN_SUBTITLE_FILE        //comment out to disable. Stores GPS location in the goggles .srt file in place of the "uavBat:" field at a slow rate of ~2-3s per GPS coordinate
#endif

#ifdef USE_PWM_ARM
static const int pwm_arm_pin = D5;
static int triggerValue = 1800;
#endif

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
boolean lightOn = true;

//Voltage and Battery Reading
#ifdef ESP8266
const float arduinoVCC = 3.17;  //Measured ESP8266 3.3 pin voltage
#else
const float arduinoVCC = 5.0;  //Measured Arduino 5V pin voltage
#endif
float ValueR1 = 7500;   //7.5K Resistor
float ValueR2 = 30000;  //30K Resistor
const int alanogPin = A0;
float averageVoltage = 0;
int sampleVoltageCount = 0;

//Other
char fcVariant[5] = "BTFL";
char craftname[15] = "QLiteOSD";
uint32_t previousMillis_MSP = 0;
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
uint8_t numSat = 99;
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
uint8_t fix_type = 0;         // < 0-1: no fix, 2: 2D fix, 3: 3D fix
uint8_t batteryCellCount = 3;
uint16_t batteryCapacity = 2200;
uint8_t legacyBatteryVoltage = 0;
uint8_t batteryState = 0;  // voltage color 0==white, 1==red
uint16_t batteryVoltage = 0;
int16_t heading = 0;
float dt = 0;
#ifdef MAH_CALIBRATION_FACTOR
float mAh_calib_factor = MAH_CALIBRATION_FACTOR;
#else
float mAh_calib_factor = 1;
#endif
uint8_t set_home = 1;
uint32_t general_counter = next_interval_MSP;
uint16_t blink_sats_orig_pos = osd_gps_sats_pos;
uint16_t blink_sats_blank_pos = 234;
uint32_t previousFlightMode = custom_mode;
uint8_t srtCounter = 1;
uint8_t thr_position = 0;
float wind_direction = 0;  // wind direction (degrees)
float wind_speed = 0;      // wind speed in ground plane (m/s)
float relative_wind_direction = 0;
float climb_rate = 0;

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
#ifdef LOG_GPS
  if (SPIFFS.begin()) {
    fsInit = true;
  }
  pinMode(fileServerModePin, INPUT);
#endif

#ifdef USE_GPS
  gpsSerial.begin(GPSBaud);
#endif

#ifdef USE_PWM_ARM
  pinMode(pwm_arm_pin, INPUT_PULLUP);
#endif

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

#ifdef IMPERIAL_UNITS
  msp_osd_config.units = 0;
#else
  msp_osd_config.units = 1;
#endif

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
#ifdef USE_PWM_ARM 
    //USE PWM signal to ARM
    volatile int pwmValue = readChannel(pwm_arm_pin, 1000, 2000, 0);
    if ((flightModeFlags == 0x00000002) && pwmValue >= triggerValue) {
      flightModeFlags = 0x00000003;    // armed to start recording
    } else if ((flightModeFlags == 0x00000003) && pwmValue < triggerValue) {        
      flightModeFlags = 0x00000002;    // disarm
    }
#else  
    //USE Altitude to Arm  
    if ((flightModeFlags == 0x00000002) && relative_alt > armAltitude) {  // if altitude is 1 meter or more then arm to record
      flightModeFlags = 0x00000003;    // armed to start recording
    }
#endif
}

#ifdef USE_PWM_ARM
// Read the number of a specified channel and convert to the range provided.
// If the channel is off, return the default value
int readChannel(int channelInput, int minLimit, int maxLimit, int defaultValue){
  int ch = pulseIn(channelInput, HIGH, 30000);
  if (ch < 100) return defaultValue;
  return map(ch, 1000, 2000, minLimit, maxLimit);
}
#endif


void display_flight_mode() {
  show_text(&craftname);
}

void send_msp_to_airunit() {
  
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
  analog.vbat = vbat;
  analog.rssi = rssi;
  analog.amperage = amperage;
  analog.mAhDrawn = mAhDrawn;
  msp.send(MSP_ANALOG, &analog, sizeof(analog));

  //MSP_BATTERY_STATE
  battery_state.amperage = amperage;
  battery_state.batteryVoltage = vbat * 10;
  battery_state.mAhDrawn = mAhDrawn;
  battery_state.batteryCellCount = batteryCellCount;
  battery_state.batteryCapacity = batteryCapacity;
  battery_state.batteryState = batteryState;
  battery_state.legacyBatteryVoltage = vbat;
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

void show_text(char (*text)[15]) {
  memcpy(craftname, *text, sizeof(craftname));
}

void set_battery_cells_number() {
  if (vbat < 43) batteryCellCount = 1;
  else if (vbat < 85) batteryCellCount = 2;
  else if (vbat < 127) batteryCellCount = 3;
  else if (vbat < 169) batteryCellCount = 4;
  else if (vbat < 211) batteryCellCount = 5;
  else if (vbat < 255) batteryCellCount = 6;
}

void readAltitude() {
  if (HomeAlt == 0.0) {
    calibrateHome();
  }
  altSamples += bme.readAltitude(PRESSURE);
  sampleCount++;
}

void getAltitudeSample() {
  relative_alt = (int)round(((altSamples / sampleCount) - HomeAlt) * 100);
  lastCount = sampleCount;
  sampleCount = 0;
  altSamples = 0.0;
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
  averageVoltage += (readValue * (arduinoVCC / 1024)) * (1 + (ValueR2 / ValueR1));
  ;
  sampleVoltageCount++;
}

void getVoltageSample() {
  vbat = (int)((averageVoltage / sampleVoltageCount) * 10);
  sampleVoltageCount = 0;
  averageVoltage = 0;
}

#ifdef USE_GPS
void readGPS() {
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }
  if (gps.location.isValid()) {
    gps_lat = (int32_t)(gps.location.lat() * 10000000);
    gps_lon = (int32_t)(gps.location.lng() * 10000000);
    numSat = gps.satellites.value();
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
  } else if (gps.satellites.isValid()) {
    numSat = gps.satellites.value();
  }
}
#endif

void loop() {
#ifdef LOG_GPS
  if (fileServerOn) {
    webserver.handleClient();
    return;
  }
#endif

  uint32_t currentMillis_MSP = millis();

  readAltitude();
  readVoltage();
#ifdef USE_GPS
  readGPS();
#endif

  if ((uint32_t)(currentMillis_MSP - previousMillis_MSP) >= next_interval_MSP) {
    previousMillis_MSP = currentMillis_MSP;

#ifdef LOG_GPS
    checkTurnOnFileServer();
    logGPS();
#endif

    if (general_counter % 300 == 0) {  // update the altitude and voltage values every 300ms
      getAltitudeSample();
      getVoltageSample();
      if (lightOn) {
        digitalWrite(LED_BUILTIN, LOW);
      } else {
        digitalWrite(LED_BUILTIN, HIGH);
      }
      lightOn = !lightOn;
    }
    set_flight_mode_flags();
    blink_sats();

#ifdef DEBUG
    debugPrint();
#else
    send_msp_to_airunit();
#endif
    general_counter += next_interval_MSP;
  }
  if (custom_mode != previousFlightMode) {
    previousFlightMode = custom_mode;
    display_flight_mode();
  }

  if (batteryCellCount == 0 && vbat > 0) {
    set_battery_cells_number();
  }

  //display flight mode every 10s
  if (general_counter % 10000 == 0) {
    display_flight_mode();
  }
}

//*** USED ONLY FOR DEBUG ***
void debugPrint() {
  mspSerial.println("**********************************");
  mspSerial.print("Flight Mode: ");
  mspSerial.println(flightModeFlags);
  mspSerial.print("Voltage: ");
  mspSerial.println(((double)vbat / 10), 1);
  mspSerial.print("Altitude (cm): ");
  mspSerial.println(relative_alt);
  mspSerial.print("Sample Count / transmit: ");
  mspSerial.println(lastCount);
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

#ifdef LOG_GPS
void logGPS() {
  if (fsInit == true && fileStarted == false && flightModeFlags == 3) {
    uint32_t maxNum = 1;
    Dir dir = SPIFFS.openDir("/");
    File file = dir.openFile("r");
    while (dir.next()) {
      maxNum++;
    }
    String fileName = "/" + String(maxNum);
    gpsLogFile = SPIFFS.open(fileName, "w");  //need to test what turning off without closing the file does
    if (gpsLogFile) {
      fileStarted = true;
    } else {
      fileStarted = false;
    }
  }
  if (general_counter % gpsLogInterval == 0 && flightModeFlags == 3) {
    while (gpsSerial.available()) {
      gps.encode(gpsSerial.read());
    }
    if (gpsLoggingStarted == false) {
      if (gps.location.isValid()) {
        tm currentTime;
        currentTime.tm_year = (gps.date.year()) - 1900;
        currentTime.tm_mon = (gps.date.month()) - 1;
        currentTime.tm_mday = (gps.date.day());
        currentTime.tm_hour = (gps.time.hour());
        currentTime.tm_min = (gps.time.minute());
        currentTime.tm_sec = (gps.time.second());

        time_t timeStamp = mktime(&currentTime);
        gpsLogFile.write((uint8_t *)&timeStamp, sizeof(time_t));
        gpsLoggingStarted = true;
      }
    }
    if (gpsLoggingStarted == true) {
      if (gps.location.isValid() && gps.location.lat() != 0.0) {
        GPS_LOG_FRAME logFrame;
        logFrame.altitude = (altSamples / sampleCount);  //altitude in meters

        logFrame.latitude = gps.location.lat();
        logFrame.longitude = gps.location.lng();
        logFrame.heading = gps.course.deg();
        logFrame.speed = (gps.speed.kmph() * 1000.0f) / 3600.0f;
        lastFrame = logFrame;
        gpsLogFile.write((uint8_t *)&logFrame, sizeof(GPS_LOG_FRAME));
      } else {
        if (lastFrame.latitude != 0.0f) {
          gpsLogFile.write((uint8_t *)&lastFrame, sizeof(GPS_LOG_FRAME));
        }
      }
    }
  }
}

void checkTurnOnFileServer() {
  if (digitalRead(fileServerModePin) == LOW && !fileServerOn) {
    onPinCount++;
    if (onPinCount < 30) {  // only turn on if held for 3 seconds
      return;
    }
    gpsSerial.end();
    //Begin fileserver
    fileServerOn = true;
    digitalWrite(LED_BUILTIN, LOW);
    ap_ssid += "-" + String(ESP.getChipId(), HEX);
    WiFi.softAP(((const char *)ap_ssid.c_str()), ap_psk);
    webserver.on("/", showFiles);             //Show all logged gps files
    webserver.on("/download", downloadFile);  //Convert and download a gpx file
    webserver.on("/delete", deleteFiles);     //Delete all files
    webserver.on("/wifioff", turnWifiOff);    //Turn off AP Wifi
    webserver.begin();
  }
  onPinCount = 0;
}

void turnWifiOff() {
  webserver.send(200, "text/html", "");
  webserver.client().stop();
  fileServerOn = false;
  digitalWrite(LED_BUILTIN, HIGH);
  WiFi.softAPdisconnect(true);
  gpsSerial.begin(GPSBaud);
}

void sendHeader() {
  webserver.sendHeader("Cache-Control", "no-cache, no-store");
  webserver.sendHeader("Pragma", "no-cache");
  webserver.sendHeader("Expires", "-1");
  webserver.setContentLength(CONTENT_LENGTH_UNKNOWN);
  webserver.send(200, "text/html", "");

  String html = "<!DOCTYPE HTML>";
  html += "<html><head><title>QLiteOSD</title><link rel='icon' href='data:;base64,='>";
  html += "<meta http-equiv='Content-Type' content='text/html; charset=UTF-8' />";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "</head><body>";
  webserver.sendContent(html);
}

void sendFooter() {
  String html = "<br><br><br>";
  html += "</body></html>";
  webserver.sendContent(html);
  webserver.sendContent("");
  webserver.client().stop();
}

void showFiles() {

  if (fsInit == false) {
    webserver.send(200, "text/html", "Error: Filesystem failed to init!");
    return;
  }

  sendHeader();

  String webpage = "<h2>QLiteOSD v" + String(VERSION) + " GPS Log Files</h2><ul>";

  Dir dir = SPIFFS.openDir("/");
  while (dir.next()) {
    String fileNum = dir.fileName();
    String fileName = fileNum;
    fileName.replace("/", "");
    fileName = "QLiteOSD_GPS_" + fileName + ".gpx";
    File file = dir.openFile("r");
    time_t time = 0;
    file.readBytes((char *)&time, sizeof(time_t));
    webpage += "<li><a href='download?download=" + fileNum + "'>" + fileName + "</a>  &nbsp;&nbsp;" + fileSize(file.size()) + "&nbsp;&nbsp;<span class='time'>" + String(time) + "</span><br/></li>";
  };

  FSInfo fsInfo;
  SPIFFS.info(fsInfo);

  webpage += "</ul>\n";
  webpage += "Space Used: <strong>" + fileSize(fsInfo.usedBytes) + "</strong> | ";
  webpage += "Free Space: <strong>" + fileSize(fsInfo.totalBytes) + "</strong>";
  webpage += "<br/><br/><a href='/delete' onclick='return confirm(\"Do you want to delete all files?\")'><strong>Delete All Files</strong></a>";
  webpage += " | <a href='/wifioff' onclick='return confirm(\"Do you want to turn off Wifi Access Point and start OSD again?\")'><strong>Turn OFF Wifi</strong></a>";
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
  static const String gpxheader = "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<gpx version=\"1.0\">\n\t<name>QLiteOSD v" + String(VERSION) + "</name>\n\t<trk><name>QLiteOSDPath</name><number>1</number><trkseg>\n";
  static const char *gpxcloser = "</trkseg></trk>\n</gpx>\n";

  if (webserver.args() > 0) {
    if (webserver.hasArg("download")) {
      String filename = webserver.arg(0);
      File rawDataFile = SPIFFS.open(filename, "r");
      time_t startTime = 0;
      rawDataFile.readBytes((char *)&startTime, sizeof(uint64_t));
      WiFiClient client = webserver.client();
      size_t fileSize = rawDataFile.size();
      size_t numFrames = (fileSize / sizeof(GPS_LOG_FRAME));
      client.print("HTTP/1.1 200 OK\r\n");
      client.print("Content-Disposition: attachment; filename=QLite_GPS_" + filename + ".gpx\r\n");
      client.print("Content-Type: application/octet-stream\r\n");
      //client.print("Content-Length: 0"\r\n");
      client.print("Connection: close\r\n");
      client.print("Access-Control-Allow-Origin: *\r\n");
      client.print("\r\n");
      char sizeWriteBuffer[10];
      //client.write(sprintf(sizeWriteBuffer,"%X\n",sizeof(gpxheader)-2));
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
        //client.write(sprintf(sizeWriteBuffer,"%X\n",strlen(fullbuf)-1));
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
  SPIFFS.format();
  delay(1000);
  webserver.sendHeader("Location", String("/"), true);
  webserver.sendHeader("Cache-Control", "no-cache, no-store");
  webserver.sendHeader("Pragma", "no-cache");
  webserver.sendHeader("Expires", "-1");
  webserver.send(302, "text/plain", "");
  webserver.client().stop();
}
#endif