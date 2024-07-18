#pragma once
#include "config.h"

#ifdef LOG_GPS

#include <time.h>

struct gps_logfile_header_t {
    uint32_t version;
    time_t fileStartTime; // unix timestamp
};

struct gps_logfile_frame_t {
  uint32_t timeOffset; // ms since flight started
  float latitude;
  float longitude;
  float altitude;  //in meters
  float heading; // in degrees
  float speed; // in meters/second
};

void gpsLogInit();
void gpsLogLoop();
void gpsLogFrame();
void gpsLogSave();
void gpsLogEnd();
void gpsLogDeleteOldFiles();

extern bool gpsLogging;

#endif
