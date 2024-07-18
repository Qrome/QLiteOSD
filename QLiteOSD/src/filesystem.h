#pragma once
#include "config.h"

#ifdef WEB_INTERFACE

#include <FS.h>
#include <LittleFS.h>

void filesystemInit();
void filesystemFormat();

extern FS& filesystem;

#endif
