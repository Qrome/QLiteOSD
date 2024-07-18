#include "filesystem.h"

#ifdef WEB_INTERFACE

#include "MSP_OSD.h"

FS& filesystem = LittleFS;

void filesystemInit() {
  // The standard will format the filesystem if it can't mount, so this should rarely fail
  if (!filesystem.begin()) {
    debugLog("FS Init Fail!!");
  }
}

void filesystemFormat() {
    if (!filesystem.format()) {
        debugLog("FS Format Failed!");
    }
}

#endif

