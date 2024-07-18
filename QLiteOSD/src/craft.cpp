#include "craft.h"
#include "MSP_OSD.h"
#include <Adafruit_BMP280.h>  // May need to adjust for I2C address #define BMP280_ADDRESS  (0x76)

#ifdef USE_GPS
#include <TinyGPS++.h>
#endif

CraftState craftState;

Adafruit_BMP280 bme;  // I2C

#ifdef USE_GPS
TinyGPSPlus gps;
SoftwareSerial gpsSerial(gps_RX_pin, gps_TX_pin);
#endif

void CraftState::init() {
  flightModeFlags = FLIGHT_UNARMED;
  batteryCapacity = 2200;

  if (USE_PWM_ARM) {
    pinMode(PWM_ARM_PIN, INPUT_PULLUP);
  }
  
  bme.begin(BMP_ADDRESS);  //Default Address 0x77
  previousAltitudeReadTime = millis();

#ifdef USE_GPS
  gpsSerial.begin(GPSBaud);
  gpsHomeSet = false;
  numSats = 0;
  gps_lat = 0.0f;
  gps_lon = 0.0f;
  gps_home_lat = 0.0f;
  gps_home_lon = 0.0f;
  distanceToHome = 0.0f;
  directionToHome = 0.0f;
#endif

  for (int i = 0; i < 20; i++) {
    sample(); // Get initial values from the sensors
    delay(100);
  }

  read();

  previous_altitude = altitude;
  HomeAlt = altitude;
}

void CraftState::sample() {
  sampleAltitude();
  sampleVoltage();
}

void CraftState::read() {
  readVoltage();
  readAltitude();
#ifdef USE_GPS
  readGPS();
#endif
}

void CraftState::sampleAltitude() {
  altitudeSamples += bme.readAltitude(SEA_LEVEL_PRESSURE);
  sampleAltitudeCount++;
}

void CraftState::readAltitude() {
  if (sampleAltitudeCount == 0) return;
  previous_altitude = altitude;
  altitude = altitudeSamples / sampleAltitudeCount;
  sampleAltitudeCount = 0;
  altitudeSamples = 0.0f;

  uint32_t currentTime = millis();
  climbRate = ((craftState.altitude - craftState.previous_altitude) / (currentTime - previousAltitudeReadTime))/1000.0f;
  previousAltitudeReadTime = currentTime;
}

void CraftState::readVoltage() {
  if (sampleVoltageCount == 0) return;
  voltage = voltageSamples / sampleVoltageCount;
  sampleVoltageCount = 0;
  voltageSamples = 0.0f;
}

void CraftState::sampleVoltage() {
  int readValue = analogRead(ANALOG_PIN);
  voltageSamples += (readValue * (BOARD_VCC / 1024.0f)) * (1.0f + (ValueR2 / ValueR1));
  sampleVoltageCount++;
}

// Read the number of a specified channel and convert to the range provided.
// If the channel is off, return the default value
volatile int CraftState::readPWMChannel(int channelInput, int minLimit, int maxLimit, int defaultValue) {
  int ch = pulseIn(channelInput, HIGH, 30000);
  if (ch < 100) return defaultValue;
  return map(ch, 1000, 2000, minLimit, maxLimit);
}

void CraftState::setFlightModeFlags() {
  if (USE_PWM_ARM) {
    //USE PWM signal to ARM
    volatile int pwmValue = readPWMChannel(PWM_ARM_PIN, 1000, 2000, 0);
    
    if ((flightModeFlags == FLIGHT_UNARMED) && pwmValue >= PWM_TRIGGER_VALUE) {
      pwmDisarmTimerStart = 0;
      flightModeFlags = FLIGHT_ARMED;
    } else if ((flightModeFlags == FLIGHT_ARMED) && pwmValue < PWM_TRIGGER_VALUE) {
      
      if (pwmDisarmTimerStart == 0) {
        pwmDisarmTimerStart = millis();
      }else if (millis() - pwmDisarmTimerStart >= PWM_DISARM_TIME) {
        flightModeFlags = FLIGHT_UNARMED;  // disarm after a delay
        pwmDisarmTimerStart = 0;
      }

    }

  } else {

    //USE Altitude to Arm
    // flightModeFlags = FLIGHT_ARMED; //Uncomment this to automatically arm on start
    
    if (flightModeFlags == FLIGHT_UNARMED && getRelativeAltitude() > ARM_ALTITUDE) {  // if altitude is 1 meter or more then arm to record
      flightModeFlags = FLIGHT_ARMED;                                       // armed to start recording
    }
  }
}

#ifdef USE_GPS
void CraftState::readGPS() {
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }
  if (!gps.location.isValid()) {
    return;
  }

  // Make sure that current latitude and longitude don't get reset
  float lat = gps.location.lat();
  if (lat != 0.0f) {
    gps_lat = lat;
  }
  float lon = gps.location.lng();
  if (lon != 0.0f) {
    gps_lon = lon;
  }

  gps_alt = gps.altitude.meters();
  gps_speed = gps.speed.mps();
  gps_heading = gps.course.deg();

  tm currentTime;
  currentTime.tm_year = gps.date.year() - 1900;
  currentTime.tm_mon = gps.date.month() - 1;
  currentTime.tm_mday = gps.date.day();
  currentTime.tm_hour = gps.time.hour();
  currentTime.tm_min = gps.time.minute();
  currentTime.tm_sec = gps.time.second();
  gps_time = mktime(&currentTime); // Convert the gps time to a unix timestamp

  if (gps.satellites.isValid()) {
    numSats = gps.satellites.value();
  }

  // Ensure the position is accurate enough before setting home
  if (gpsHomeSet == false && gps.hdop.isValid() && gps.hdop.hdop() < 2) {
    gps_home_lat = gps_lat;
    gps_home_lon = gps_lon;
    gpsHomeSet = true;
  }

  if (gpsHomeSet) {
    distanceToHome = TinyGPSPlus::distanceBetween(gps_lat, gps_lon, gps_home_lat, gps_home_lon);
    directionToHome = TinyGPSPlus::courseTo(gps_lat, gps_lon, gps_home_lat, gps_home_lon);
  }
}
#endif

uint8_t CraftState::getCellCount() {
  // Todo map this to a function
  if (voltage < 4.3f) return 1;
  else if (voltage < 8.5f) return 2;
  else if (voltage < 12.7f) return 3;
  else if (voltage < 16.9f) return 4;
  else if (voltage < 21.1f) return 5;
  else return 6;
}
