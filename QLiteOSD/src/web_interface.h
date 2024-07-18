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
#pragma once
#include "config.h"

#ifdef WEB_INTERFACE

#include <Arduino.h>

void webInterfaceInit();
void webInterfaceLoop();
void webInterfaceSendHeader();
void webInterfaceSendFooter();
void webInterfaceRedirect(const char* url);
void webInterfaceHome();
void webInterfaceConfigure();
void webInterfaceConfigSave();
void webInterfaceWifiOff();
void webInterfaceSystemFormat();

#ifdef LOG_GPS
void webInterfaceDownloadLog();
void webInterfaceDeleteLog();
#endif

extern bool webInterfaceOn;

extern const char* HEAD_TITLE PROGMEM;
extern const char* BODY_MENU PROGMEM;
extern const char* BODY_END PROGMEM;
extern const char* CONFIG_FORM PROGMEM;
extern const char* CONFIG_FORM_SAVE PROGMEM;
extern const char* CONFIG_FORM_OSD PROGMEM;
extern const char* CONFIG_FORM_OSD_2 PROGMEM;
extern const char* CONFIG_JAVASCRIPT PROGMEM;
extern const char* CONFIG_OSD PROGMEM;
extern const char* LOGFILES_JAVASCRIPT PROGMEM;

#ifdef USE_LEDS
extern const char* CONFIG_FORM_LED PROGMEM;
extern const char* CONFIG_RGB_OPTIONS PROGMEM;
extern const char* CONFIG_RGB_JS PROGMEM;
#endif

#endif
