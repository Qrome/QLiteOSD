#pragma once

#include "config.h"
#include <time.h>

#ifdef USE_GPS
#include <SoftwareSerial.h>
#endif

class CraftState {
private:
    float voltageSamples;
    uint32_t sampleVoltageCount;

    float altitudeSamples;
    uint32_t sampleAltitudeCount;

    uint32_t pwmDisarmTimerStart;

    float previous_altitude; // used to calculate climb rate
    uint32_t previousAltitudeReadTime; // in ms

public:
    void init();
    void read();
    void sample();
    void setFlightModeFlags();
    void sampleAltitude();
    void readAltitude();
    void sampleVoltage();
    void readVoltage();
    void readGPS();

    uint8_t getCellCount();

    static volatile int readPWMChannel(int channelInput, int minLimit, int maxLimit, int defaultValue);

    inline float getRelativeAltitude() { return altitude - HomeAlt; }

    uint32_t flightModeFlags; // 0x00000002 by default

    float voltage;
    uint16_t batteryCapacity; // 2200 by default
    uint8_t batteryState;  // voltage color 0==white, 1==red

    float climbRate; // in m/s
    float altitude; // in meters from sea level

    float HomeAlt;

#ifdef USE_GPS
    bool gpsHomeSet;

    uint32_t numSats;

    float gps_speed; // in m/s
    float gps_heading;

    float gps_lon;
    float gps_lat;
    float gps_alt;

    float gps_home_lon;
    float gps_home_lat;

    float distanceToHome; // in meters
    float directionToHome; // in degrees

    time_t gps_time;
#endif
};

extern CraftState craftState;

#ifdef USE_GPS
extern SoftwareSerial gpsSerial;
#endif
