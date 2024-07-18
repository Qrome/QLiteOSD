/*
 *  QLiteOSD is an simple OSD for DJI FPV System:
 *  This is an Arduino project that handles basic OSD functions
 *  from BMP280 to a Simple Voltage Sensor to feed it 
 *  to DJI FPV System.
 *
 * ------------------------------------------------
 *
 * Copyright (C) 2024 David Payne
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

#include <stdint.h>

#define ESP8266_TARGET  // Uncommment this line if using the ESP8266 (Wemos D1 Mini)
#define USE_GPS  //comment out to disable.  Reads and displays GPS data - requires Nano 328 due to file size
#define USE_LEDS // comment out to disable. Adds controls for an LED strip

#define BOARD_VERSION 11
// #define BOARD_VERSION 20

#if defined(ESP8266_TARGET)
#define WEB_INTERFACE // uncomment to disable the web interface
#endif

#if defined(WEB_INTERFACE) && defined(USE_GPS) // do not remove or comment this line out
#define LOG_GPS //uncomment to log position and altitude data to the internal filesystem (requires a pushbutton to be added to switch to wifi mode)
#endif
// #define DEBUG    // uncomment this line to debug in Serial Monitor 

// Hardcoded config
#define VERSION "2.0"
#define BMP_ADDRESS 0x76              // default is 0x77
#define FC_FIRMWARE_IDENTIFIER "BTFL"
#define CONFIG "/conf.txt"

#define TICK_INTERVAL 100 // in ms
#define ALTITUDE_UPDATE_INTERVAL 2 // in ticks. TICK_INTERVAL * ALTITUDE_UPDATE_INTERVAL ms
#define LIGHT_UPDATE_INTERVAL 2 // in ticks, TICK_INTERVAL * LIGHT_UPDATE_INTERVAL ms
#define PWM_DISARM_TIME 3000 // in ms

#ifdef USE_GPS
#define GPS_SATELITE_BLINK_UPDATE_INTERVAL 9 // / in ticks, TICK_INTERVAL * GPS_SATELITE_BLINK_UPDATE_INTERVAL ms
#endif

#define SEA_LEVEL_PRESSURE 1013.25f

#define ARM_ALTITUDE 1.5f // 1.5 meters up

#define ValueR1 7500.0f //7.5K Resistor
#define ValueR2 30000.0f //30K Resistor

#define GPSBaud 9600

#define ANALOG_PIN A0

#define PWM_TRIGGER_VALUE 1700

#if BOARD_VERSION >= 20
#define USE_DJI_RX_PIN true
#endif

#ifdef ESP8266_TARGET
#define BOARD_VCC_DEFAULT 3.25f  //Measured ESP8266 3.3 pin voltage
#define PWM_ARM_PIN D5

#if BOARD_VERSION < 20
#define gps_RX_pin D8
#define gps_TX_pin D7
#elif BOARD_VERSION >= 20
#define gps_RX_pin D7
#define gps_TX_pin D8
#endif

#ifdef USE_LEDS
#define LED_PIN D6
#endif

#else
#define BOARD_VCC_DEFAULT 4.95f  //Measured Arduino 5V pin voltage
#define PWM_ARM_PIN 10
#define gps_RX_pin 4
#define gps_TX_pin 3

#ifdef USE_LEDS
#define LED_PIN 11
#endif

#endif

#ifdef WEB_INTERFACE
#define AP_SSID_PREFIX "QLiteOSD"
#define AP_PSK "12345678"

#define WEB_INTERFACE_BUTTON_PIN D3
#define WEB_INTERFACE_ENABLE_TICKS 30 // 30 ticks of the button held down to enable the web interface

#define CONFIG_FILE_PATH "/config.txt"
#endif

#ifdef LOG_GPS
#define GPS_LOG_INTERVAL 500 // in ms
#define GPS_LOG_SAVE_INTERVAL 10000 // in ms
#define LOG_DIRECTORY_PREFIX "/logs/"
#define GPS_LOG_FILE_LIMIT 10 // Maximum amount of log files
#endif

//  ********************************************************************
//  Config defaults
//  
//  These settings are the defaults and are only loaded the first run.
//  When the values are updated in the web interface they are retained
//  until a Reset from the web interface.

#define CRAFT_NAME_DEFAULT "QLiteOSD"
#define USE_IMPERIAL_UNITS_DEFAULT true
#define USE_PWM_ARM_DEFAULT false

// Helper macro to map out positions correctly
// X values are 0-26 from left to right
// Y values are 0-15 from top to bottom
#define OSD_POS(x,y) (2048 + x + (y*32))
#define OSD_HIDDEN 234

// General OSD Positions

#define MAIN_BATT_VOLT_POS_DEFAULT OSD_POS(1,2)
#define CROSSHAIRS_POS_DEFAULT     OSD_POS(14,6)
#define CRAFTNAME_POS_DEFAULT      OSD_POS(0,0)
#define ALTITUDE_POS_DEFAULT       OSD_POS(0,6)
#define AVG_CELL_VOLT_POS_DEFAULT  OSD_POS(1,3)
#define NUM_VARIO_POS_DEFAULT      OSD_POS(1,7)


// GPS OSD Positions

#ifdef USE_GPS
#define GPS_SATS_POS_DEFAULT      OSD_POS(26,0)
#define GPS_HOME_DIR_POS_DEFAULT  OSD_POS(15,1)
#define GPS_HOME_DIST_POS_DEFAULT OSD_POS(1,9)
#define GPS_SPEED_POS_DEFAULT     OSD_POS(1,8)
#define GPS_LAT_POS_DEFAULT       OSD_POS(0,12)
#define GPS_LON_POS_DEFAULT       OSD_POS(0,13)
#else
#define GPS_SATS_POS_DEFAULT      OSD_HIDDEN
#define GPS_HOME_DIR_POS_DEFAULT  OSD_HIDDEN
#define GPS_HOME_DIST_POS_DEFAULT OSD_HIDDEN
#define GPS_SPEED_POS_DEFAULT     OSD_HIDDEN
#define GPS_LAT_POS_DEFAULT       OSD_HIDDEN
#define GPS_LON_POS_DEFAULT       OSD_HIDDEN
#endif

extern char CRAFT_NAME[15]; // Do not make larger than 14 characters
extern bool USE_IMPERIAL_UNITS;  // Set to false to see units in Metric
extern bool USE_PWM_ARM;  // pin D5 ESP8266 -- If commented out arming will occure with altitude change
extern float BOARD_VCC;
extern uint16_t OSD_GPS_SATS_POS;

extern struct msp_osd_config_t msp_osd_config;

#ifdef USE_LEDS

#define NUM_LEDS 12
#define LED_UPDATE_INTERVAL 10 // in ticks

#define RGB_MODE_DEFAULT "ON"

#define RED_VALUE_DEFAULT 0
#define GREEN_VALUE_DEFAULT 255
#define BLUE_VALUE_DEFAULT 0

extern char RGB_MODE[16];
extern uint8_t RED_VALUE;
extern uint8_t GREEN_VALUE;
extern uint8_t BLUE_VALUE;

#define LED_CONFIG_NUM 4

#else
#define LED_CONFIG_NUM 0
#endif

enum configType_e {
    CONFIG_VALUE_UINT16,
    CONFIG_VALUE_UINT8,
    CONFIG_VALUE_BOOL,
    CONFIG_VALUE_STRING,
    CONFIG_VALUE_FLOAT
};

struct configValue_t {
    const char* key;
    void* globalValue;
    configType_e type;
    uint32_t optionalData;
};

#define CONFIG_VALUE_COUNT (15 + LED_CONFIG_NUM)

extern const configValue_t configValues[CONFIG_VALUE_COUNT];

// config.cpp functions
void configInit();
void configRead();
void configWrite();
