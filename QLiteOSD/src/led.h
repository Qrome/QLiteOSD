#pragma once
#include "config.h"

#ifdef USE_LEDS

void ledInit();
void ledUpdate();

#ifdef WEB_INTERFACE
void ledWebserverColors();
#endif

#endif
