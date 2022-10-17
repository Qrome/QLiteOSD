# QLiteOSD - Simple Arduino OSD for DJI FPV System

QLiteOSD is a simple Arduino Nano/ESP8266 based OSD for DJI FPV System for those times you want OSD but without the flight controller.  Main features provide flight battery voltage, altitude, and GPS support.  Perfect for Sport FPV flying.

Kits Available:  https://www.etsy.com/listing/1272353602/qliteosd-for-dji-fpv-on-screen-display  
Assembly Guide:  https://github.com/Qrome/QLiteOSD/blob/master/QLiteOSD%20Board%20Assembly.pdf  
FAQ:  https://github.com/Qrome/QLiteOSD/blob/master/README.md#faq

Painless360 Build: https://www.youtube.com/watch?v=KWHZCemLLxw  
Painless360 Compiling and Customization: https://www.youtube.com/watch?v=f42vbrB-Z7Y  

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
* Wemos D1 Mini TX1 to DJI Air unit RX (baud: 115200)
* BMP280 Sensor on I2C default pins (ATMega A4 and A5/Wemos ESP8266 pins D1 and D2)
* Optional GPS (ATMega RX:4 and TX:3/Wemos D1 Mini RX:D8 and TX:D7) -- GPS is optional and requires that you uncomment the USE_GPS in the OSD_positions_config.h file.
* Voltage Sensor on A0 analog pin (use a 30K/7.5K Resistor divider)
* Set Mini-360 DC-DC Buck Converter Step Down Module to 5V
* Place the MSP folder under libraries in your local Arduino/libraries path  
* GPS Logging to Wemos D1 Mini file system
* Download GPS logs through WiFi Access Point
* ESP866 D3 pin grounded for 3 seconds will turn on WiFi mode (turns off OSD mode)
* You can move the OSD items around the screen by using the chart below and setting the value in the OSD_positions_config.h file.  If you wish to hide an option use the value 234.  Note: not all OSD options are supported by DJI FPV goggles.

## Compiled Binary Files
* QLiteOSD.GPS_IMP_1.X.bin -- with GPS support and Imperial measure (feet, Mph)  
* QLiteOSD.GPS_MET_1.X.bin -- with GPS support and Metric measure (Meters, Kph)  
* QLiteOSD.NOGPS_IMP_1.X.bin -- NO GPS support and Imperial measure (feet altitude)  
* QLiteOSD.NOGPS_MET_1.X.bin -- NO GPS support and Metric measure (Meters altitude)  
 
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

## FAQ
**Will the QLiteOSD send the armed command to the DJI Video Transmitter?**  
* Yes, the arm signal is sent when the OSD detects a change of +1.5 meters (about 5 feet) above the altitude the OSD was powered on with.  This is nice as it will tell the DJI system to start recording and bump the transmit power up if you are using auto power on arming.  This does not require GPS.  

**Is GPS required to use the QLiteOSD?**  
* No, the GPS is not required.  If you choose not to use the GPS, then you may want to consider using the NOGPS binary or compile the source with the defined USE_GPS commented out.   This will remove the items on the OSD that are GPS related.  

**Can I move the locations of the OSD elements around in the goggles?**  
* Yes, it would require editing the OSD_positions_config.h and referencing the mapping image to define the locations.  You can enable or disable elements by the number assigned to it.  See the notes in that file.  After making changes to the config file you would then compile and load the binary to your OSD.  

**Can I have RSSI or LQ in the QLiteOSD?**  
* This is currently not supported. Often this can be setup with audible alerts in your own radio.  The QLiteOSD does not currently require any connection from the aircraft receiver.  

**Can I adjust the reading of the pack voltage?**  
* Yes.  The best way to get an accurate reading of the pack voltage is to adjust the output voltage of the Mini360 step down to as close to 6V as you can.  When the Wemos D1 mini board is powered with 6V, the internal system 5V is very close to 5V and this is used in the calculation.  

**How do I enable the QLiteOSD GPS logging feature?**  
* When using the GPS and the default compiled binary, it is enabled by default and every flight should log to the internal memory on the Wemos D1 Mini.  To disable logging, compile the source code with #define LOG_GPS commented out in the OSD_positions_config.h file.  Logging starts when the altitude changes + 1.5 meter above the powered on altitude and there is a GPS lock.  

**How do I download the GPX (GPS Log files) from the QLiteOSD?**  
* Put the QLiteOSD device into WiFi Mode, join the Access Point and download the files.
    - When the OSD is on, jumper the Wemos D1 Mini D3 pin with GND for 3 seconds.  The light will turn solid indicating it is now in WiFi Mode and is visible as an Access Point.
    - QLiteOSD_xxxxx – where the xxxxxx is the unique serial number on the chip will show.  Join your phone or computer to the Access Point using the default password 12345678.
    - After connected to the divide over wifi, pull open a web browser to the following address:   http://192.168.4.1  – this will pull up the logged files.
* There are several free GPX Log file viewers including GeoTracker for Android.  Google Earth will also view them.  


