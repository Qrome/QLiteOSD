#include "gpsLog.h"

#ifdef LOG_GPS

#include "craft.h"
#include "filesystem.h"

#include "MSP_OSD.h" //Used for the debugLog def

bool gpsLogging = false;

static uint32_t lastLogTime = 0;
static uint32_t startLogTime = 0;
static File logFile;
static time_t logFileId;

void gpsLogInit() {
  // Create the Log file
  gpsLogging = true;

  gps_logfile_header_t header;
  header.version = 1;
  header.fileStartTime = craftState.gps_time;

  // Make the file name the current gps time, so files can be uniquely identified
  logFileId = header.fileStartTime;

  debugLog(String("gpsLog start time: ")+String(logFileId)+String("\nFile start millis: ")+String(millis()));

  logFile = filesystem.open(String(LOG_DIRECTORY_PREFIX)+String(logFileId),"w");
  logFile.write((uint8_t*)&header,sizeof(header));

  debugLog(String("gpsLog start end time: ")+String(millis()));

  startLogTime = millis();
  lastLogTime = startLogTime;

  gpsLogFrame(); // Log the current flight values
}

void gpsLogLoop() {
  uint32_t currentTime = millis();
  if (currentTime - lastLogTime >= GPS_LOG_INTERVAL) {
    lastLogTime = currentTime;
    gpsLogFrame();
  }

  static uint32_t lastSaveTime;
  if (currentTime - lastSaveTime >= GPS_LOG_SAVE_INTERVAL) {
    lastSaveTime = currentTime;
    gpsLogSave();
  }
}

void gpsLogFrame() {
  gps_logfile_frame_t frame;

  frame.timeOffset = millis() - startLogTime;
  frame.altitude = craftState.altitude;
  frame.heading = craftState.gps_heading;
  frame.latitude = craftState.gps_lat;
  frame.longitude = craftState.gps_lon;
  frame.speed = craftState.gps_speed;

  debugLog(String("gpsLog start time: ")+String(millis()));
  logFile.write((uint8_t*)&frame,sizeof(frame));
  debugLog(String("gpsLog end time: ")+String(millis()));
}

// Flush the file
void gpsLogSave() {
  debugLog(String("gpsLog flush start time: ")+String(millis()));
  logFile.flush();
  debugLog(String("gpsLog flush end time: ")+String(millis()));
}

void gpsLogEnd() {
  logFile.close();
  gpsLogging = false;
}

// Assumes that all log files are their unix timestamp number
void gpsLogDeleteOldFiles() {
  int currentNumFiles = 0;

  {
    Dir logDir = filesystem.openDir(LOG_DIRECTORY_PREFIX);
    while (logDir.next()) {
      if (logDir.isDirectory()) {
        continue;
      }

      currentNumFiles++;
    }
  } // Make sure the directory destructor is called before continuing

  if (currentNumFiles <= GPS_LOG_FILE_LIMIT) {
    return;
  }

  for (int i = 0; i<currentNumFiles-GPS_LOG_FILE_LIMIT; i++) {
    Dir logDir = filesystem.openDir(LOG_DIRECTORY_PREFIX);
    time_t oldest = INT64_MAX;
    while (logDir.next()) {
      time_t fileTime = atoll(logDir.fileName().c_str());
      if (fileTime < oldest) {
        oldest = fileTime;
      }
    }

    filesystem.remove(LOG_DIRECTORY_PREFIX+String(oldest));
  }
}

#endif
