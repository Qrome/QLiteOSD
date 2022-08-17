# QLiteOSD - Simple Arduino OSD for DJI FPV System

QLiteOSD is a simple Arduino Nano/ESP8266 based OSD for DJI FPV System for those times you want OSD but without the flight controller.  Main features provide flight battery voltage, altitude, and GPS support.  Perfect for Sport FPV flying.

Kits Available:  https://www.etsy.com/listing/1272353602/qliteosd-for-dji-fpv-on-screen-display

## Components
* Wemos D1 Mini (ESP8266) board https://amzn.to/3wW6TIj
* OR Arduino Nano 168 or 328 5V board (note GPS requires 328) https://amzn.to/3wvhZ6Q
* BMP280 - barameter sensor https://amzn.to/3PmJAQ1
* BN-220 GPS (9600 baud) https://amzn.to/3GtVovX (optional)
* Mini-360 DC-DC Buck Converter Step Down Module https://amzn.to/3FMzDqQ
* 30K Resistor -- for voltage reading https://amzn.to/3NgcgbH
* 7.5K Resistor -- for voltage reading https://amzn.to/39k5Vxe

Copyright (C) 2022 David Payne  
 
This software is based on and uses software published by Paul Kurucz (pkuruz):opentelem_to_bst_bridge
as well as software from d3ngit : djihdfpv_mavlink_to_msp_V2
and crashsalot : VOT_to_DJIFPV
  
THIS SOFTWARE IS PROVIDED IN AN "AS IS" CONDITION. NO WARRANTIES,
WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 
## Basic Configuration
* Select the Micro Controller board you are using in the OSD_positions_config.h file.  Uncomment ESP8266 to use the Wemos D1 Mini board.  If it is commented out it will default to the ATMega chip and pins.
* Arduino Nano or Wemos D1 Mini TX1 to DJI Air unit RX (baud: 115200)
* BMP280 Sensor on I2C default pins (ATMega A4 and A5/Wemos ESP8266 pins D1 and D2)
* Optional GPS (ATMega RX:4 and TX:3/Wemos D1 Mini RX:D8 and TX:D7) -- GPS is optional and requires that you uncomment the USE_GPS in the OSD_positions_config.h file.
* Voltage Sensor on A0 analog pin (use a 30K/7.5K Resistor divider)
* Set Mini-360 DC-DC Buck Converter Step Down Module to 5V
* Place the MSP folder under libraries in your local Arduino/libraries path  
* GPS Logging to Wemos D1 Mini file system
* Download GPS logs through WiFi Access Point
* ESP866 D3 pin grounded for 3 seconds will turn on WiFi mode (turns off OSD mode)
* You can move the OSD items around the screen by using the chart below and setting the value in the OSD_positions_config.h file.  If you wish to hide an option use the value 234.  Note: not all OSD options are supported by DJI FPV goggles.
 
## Operation
* Run Pack Voltage to BAT pins (7.4 - 17V) -- This voltage will also power the DJI FPV unit.
* Board -> DJI using the TX --> DJI Rx pin and battery/ground.
* To send the arm command to the DJI unit to start recording raise the craft 1.5 meters higher than the home altitude.  There is no disarm needed.
* GPS Logging to OSD system and download from WiFi Access Point
* Put in WiFi mode by grounding pin D3 for 3 seconds
* When in WiFi mode connect to the Access Point using password 12345678 
* When connected to the access point pull up the web interface on http://192.168.4.1
* 3D print your own case for the QLiteOSD:  https://www.printables.com/model/259387-qliteosd-cover-for-dji-fpv-osd
 
## OSD Placement
![QLiteOSD initial Testing](/images/OSD_positions.png)  

## OSD with Pack Voltage, Altitude, GPS, and Cross Hairs
![QLiteOSD Preview](/images/PXL_20220612_040647213.jpg)  

## QLiteOSD Board
![QLiteOSD v1.0 Board Diagram](/images/PXL_20220613_010941035.jpg)  
![QLiteOSD Basic Kit](/images/PXL_20220612_231228968.jpg)  
![QLiteOSD Basic Kit](/images/PXL_20220613_004957916.jpg)  
![QLiteOSD Basic Kit](/images/PXL_20220613_010424662.jpg)  

