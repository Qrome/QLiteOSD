
#include "MSP_OSD.h"
#include "craft.h"

MspOSD mspOsd;

void MspOSD::init() {
  msp.begin(mspSerial);
}

void MspOSD::sendCraftMSP() {
  //MSP_FC_VARIANT
  msp_fc_variant_t fc_variant;
  memcpy(fc_variant.flightControlIdentifier, FC_FIRMWARE_IDENTIFIER, 4);
  msp.send(MSP_FC_VARIANT, &fc_variant, sizeof(fc_variant));

  //MSP_FC_VERSION
  msp_fc_version_t fc_version;
  fc_version.versionMajor = 4;
  fc_version.versionMinor = 1;
  fc_version.versionPatchLevel = 1;
  msp.send(MSP_FC_VERSION, &fc_version, sizeof(fc_version));

  //MSP_NAME
  msp_name_t name;
  memcpy(name.craft_name, CRAFT_NAME, sizeof(CRAFT_NAME));
  msp.send(MSP_NAME, &name, sizeof(name));

  //MSP_STATUS
  msp_status_DJI_t status_DJI;
  status_DJI.cycleTime = 0x0080;
  status_DJI.i2cErrorCounter = 0;
  status_DJI.sensor = 0x23;
  status_DJI.configProfileIndex = 0;
  status_DJI.averageSystemLoadPercent = 7;
  status_DJI.accCalibrationAxisFlags = 0;
  status_DJI.DJI_ARMING_DISABLE_FLAGS_COUNT = 20;
  status_DJI.djiPackArmingDisabledFlags = (1 << 24);

  status_DJI.flightModeFlags = craftState.flightModeFlags;
  status_DJI.armingFlags = 0x0303;
  msp.send(MSP_STATUS_EX, &status_DJI, sizeof(status_DJI));
  status_DJI.armingFlags = 0x0000;
  msp.send(MSP_STATUS, &status_DJI, sizeof(status_DJI));

  //MSP_ANALOG
  msp_analog_t analog;
  analog.vbat = (uint8_t)(craftState.voltage * 10.0f);
  analog.rssi = 0;
  analog.amperage = 0;
  analog.mAhDrawn = 0;
  msp.send(MSP_ANALOG, &analog, sizeof(analog));

  //MSP_BATTERY_STATE
  msp_battery_state_t battery_state;
  battery_state.amperage = 0;
  battery_state.batteryVoltage = (uint16_t)(craftState.voltage * 100.0f);
  battery_state.mAhDrawn = 0;
  battery_state.batteryCellCount = craftState.getCellCount();
  battery_state.batteryCapacity = craftState.batteryCapacity;
  battery_state.batteryState = craftState.batteryState;
  battery_state.legacyBatteryVoltage = (uint8_t)(craftState.voltage * 10.0f);
  msp.send(MSP_BATTERY_STATE, &battery_state, sizeof(battery_state));

  //MSP_RAW_GPS
  msp_raw_gps_t raw_gps = {};
#ifdef USE_GPS
  raw_gps.lat = (int32_t)(craftState.gps_lat * 10000000.0f);
  raw_gps.lon = (int32_t)(craftState.gps_lon * 10000000.0f);
  raw_gps.numSat = (uint8_t)craftState.numSats;
  raw_gps.alt = (int32_t)craftState.gps_alt;
  raw_gps.groundSpeed = (int16_t)(craftState.gps_speed * 100.0f);  //in cm/s
#endif

  msp.send(MSP_RAW_GPS, &raw_gps, sizeof(raw_gps));

  //MSP_COMP_GPS
  msp_comp_gps_t comp_gps = {};
#ifdef USE_GPS
  comp_gps.distanceToHome = (int16_t)craftState.distanceToHome;
  comp_gps.directionToHome = (int16_t)(craftState.directionToHome - craftState.gps_heading);
#endif
  msp.send(MSP_COMP_GPS, &comp_gps, sizeof(comp_gps));

  //MSP_ATTITUDE
  msp_attitude_t attitude;
  attitude.pitch = 0;
  attitude.roll = 0;
  msp.send(MSP_ATTITUDE, &attitude, sizeof(attitude));

  //MSP_ALTITUDE
  msp_altitude_t altitude;
  altitude.estimatedActualPosition = (int32_t)roundf(craftState.getRelativeAltitude() * 100.0f);
  altitude.estimatedActualVelocity = (int16_t)(craftState.climbRate*100.0f);  // m/ms to cm/s
  msp.send(MSP_ALTITUDE, &altitude, sizeof(altitude));

  // Unused
  // msp_status_BF_t status_BF;

  //MSP_OSD_CONFIG
  msp_osd_config.units = USE_IMPERIAL_UNITS;
  msp.send(MSP_OSD_CONFIG, &msp_osd_config, sizeof(msp_osd_config));
}

#ifdef DEBUG
//*** USED ONLY FOR DEBUG ***
void MspOSD::debugPrint() {
  mspSerial.print("\033[2J\033[H"); // Clear serial console, move cursor to beginning
  mspSerial.println("**********************************");
  mspSerial.print("Craft Name: ");
  mspSerial.println(String(CRAFT_NAME));
  mspSerial.print("Imperial: ");
  mspSerial.println(USE_IMPERIAL_UNITS);
  mspSerial.print("Flight Mode: ");
  mspSerial.println(craftState.flightModeFlags);
  mspSerial.print("Voltage: ");
  mspSerial.println(craftState.voltage, 1);
  mspSerial.print("Cell Count: ");
  mspSerial.println(craftState.getCellCount());
  mspSerial.print("Altitude (m): ");
  mspSerial.println(craftState.altitude);
  mspSerial.print("Relative Altitude (m): ");
  mspSerial.println(craftState.getRelativeAltitude());
  mspSerial.print("Climb Rate (m/s): ");
  mspSerial.println(craftState.climbRate);
  if (USE_PWM_ARM) {
    mspSerial.print("PWM Value: ");
    mspSerial.println(CraftState::readPWMChannel(PWM_ARM_PIN, 1000, 2000, 0));
  }
#ifdef USE_GPS
  mspSerial.print("Lat: ");
  mspSerial.println(craftState.gps_lat);
  mspSerial.print("Lon: ");
  mspSerial.println(craftState.gps_lon);
  mspSerial.print("Num Sat: ");
  mspSerial.println(craftState.numSats);
  mspSerial.print("GPS Alt: ");
  mspSerial.println(craftState.gps_alt);
  mspSerial.print("Speed: ");
  mspSerial.println(craftState.gps_speed);
  mspSerial.print("Heading: ");
  mspSerial.println(craftState.gps_heading);
  mspSerial.print("Home Set: ");
  mspSerial.println(craftState.gpsHomeSet);
  mspSerial.print("HOME Lat: ");
  mspSerial.println(craftState.gps_home_lat, 6);
  mspSerial.print("HOME Lon: ");
  mspSerial.println(craftState.gps_home_lon, 6);
  mspSerial.print("Distance: ");
  mspSerial.println(craftState.distanceToHome);
  mspSerial.print("Direction Home: ");
  mspSerial.println(craftState.directionToHome);
#endif
}
#endif
