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

/*
 *  QLiteOSD
 *
 *  Wemos D1 Mini (ESP8266) / Arduino Nano TX1 to DJI Air unit RX(115200)
 *  BMP280 on I2C (A4 and A5 defaults)
 *  Voltage sensor on A0 pin
 *  GPS addition by GravelAxe -- uses D7-RX and D8-TX
 */

/* ----------------------------------------------------- */

#include "src/libraries/MSP.h"

#include "src/config.h"
#include "src/MSP_OSD.h"
#include "src/craft.h"

#include "src/filesystem.h"
#include "src/web_interface.h"
#include "src/gpsLog.h"
#include "src/led.h"

/* ----------------------------------------------------- */
void setup()
{
  Serial.begin(115200);

#ifdef WEB_INTERFACE
  filesystemInit();
  configInit();

#ifdef LOG_GPS
  gpsLogDeleteOldFiles();
#endif

  pinMode(WEB_INTERFACE_BUTTON_PIN, INPUT);
#endif

  craftState.init();
  mspOsd.init();

#ifdef USE_LEDS
  ledInit(); // todo: see if this needs to go before GPS init
#endif

  pinMode(LED_BUILTIN, OUTPUT);

  debugLog("Starting!");
}

// Ran once every TICK_INTERVAL msecs
void tick()
{
  static uint32_t flight_ticks = 0;
  static bool lightOn = false;

  flight_ticks++;

#ifdef WEB_INTERFACE
  // 30 tick wait for enabling the web interface
  static uint32_t webInterfaceEnableTicks = 0;

  if (digitalRead(WEB_INTERFACE_BUTTON_PIN) == LOW)
  {
    webInterfaceEnableTicks++;
    if (webInterfaceEnableTicks >= WEB_INTERFACE_ENABLE_TICKS)
    {
      webInterfaceInit();
      return;
    }
  }
  else
  {
    webInterfaceEnableTicks = 0;
  }
#endif

  craftState.readVoltage();

  if (flight_ticks % ALTITUDE_UPDATE_INTERVAL == 0)
  {
    craftState.readAltitude();
  }

  if (flight_ticks % LIGHT_UPDATE_INTERVAL == 0)
  {
    digitalWrite(LED_BUILTIN, lightOn ? LOW : HIGH);
    lightOn = !lightOn;
  }

#ifdef USE_GPS
  craftState.readGPS();
  if (flight_ticks % GPS_SATELITE_BLINK_UPDATE_INTERVAL == 0)
  {
    if (!craftState.gpsHomeSet)
    {
      static bool satsOn = false;
      msp_osd_config.osd_gps_sats_pos = satsOn ? OSD_GPS_SATS_POS : OSD_HIDDEN;
      satsOn = !satsOn;
    }
    else
    {
      msp_osd_config.osd_gps_sats_pos = OSD_GPS_SATS_POS;
    }
  }
#endif

  craftState.setFlightModeFlags();

#ifdef LOG_GPS
  // Once the flight is armed and home is set, GPS is accurate enough to start logging
  if (!gpsLogging && craftState.flightModeFlags == FLIGHT_ARMED && craftState.gpsHomeSet)
  {
    gpsLogInit();
  }

  // End the gps log if the flight gets unarmed
  if (gpsLogging && craftState.flightModeFlags == FLIGHT_UNARMED)
  {
    gpsLogEnd();
  }
#endif

#ifdef USE_LEDS
  String ledMode = RGB_MODE;
  if ((ledMode != "OFF" && ledMode != "ON") && flight_ticks % LED_UPDATE_INTERVAL == 0) {
    ledUpdate();
  }
#endif


  // SEND the data out
#ifdef DEBUG
  if (!gpsLogging)
    mspOsd.debugPrint();
#else

#ifdef USE_DJI_RX_PIN
  static bool mspStarted = false;

  if (mspStarted)
  {
    mspOsd.sendCraftMSP();
  }
  else
  {
    if (mspOsd.msp.activityDetected())
    {
      mspStarted = true;
      debugLog("****************** Found DJI MSP Activity ******************");
    }
    else
    {
      debugLog("*** Waiting for MSP Activity from DJI Unit ***");
    }
  }

#else
  mspOsd.sendCraftMSP();
#endif

#endif
}

void loop()
{
#ifdef WEB_INTERFACE
  if (webInterfaceOn)
  {
    webInterfaceLoop();
    return;
  }
#endif

  static uint32_t lastLoopTime = 0;

  craftState.sample();

  uint32_t currentLoopTime = millis();
  if (currentLoopTime - lastLoopTime >= TICK_INTERVAL)
  {
    lastLoopTime = currentLoopTime /*  + (currentLoopTime-lastLoopTime-TICK_INTERVAL) */;
    tick();
  }

#ifdef LOG_GPS
  if (gpsLogging)
  {
    gpsLogLoop();
  }
#endif
}
