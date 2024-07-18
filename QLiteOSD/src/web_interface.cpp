#include "web_interface.h"

#ifdef WEB_INTERFACE

#include "craft.h"
#include "filesystem.h"
#include "gpsLog.h"
#include "led.h"

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

bool webInterfaceOn = false;

// todo check if we should allocate this on the heap instead
ESP8266WebServer webserver(80);

void webInterfaceInit() {
#ifdef LOG_GPS
    // the gps communication must be stopped to start wifi communication
    if (gpsLogging) {
        gpsLogEnd();
    }
    gpsSerial.end();
#endif

    //Begin fileserver
    webInterfaceOn = true;

    WiFi.softAP(String(AP_SSID_PREFIX)+"-"+String(ESP.getChipId(), HEX), String(AP_PSK));

    webserver.on("/", webInterfaceHome);                 //Show all logged gps files
#ifdef LOG_GPS
    webserver.on("/download", webInterfaceDownloadLog);      //Convert and download a gpx file
    webserver.on("/delete", webInterfaceDeleteLog);         //Delete the log given by the parameter
#endif
    webserver.on("/wifioff", webInterfaceWifiOff);        //Turn off AP Wifi
    webserver.on("/configure", webInterfaceConfigure);  //Show the Configure Page
    webserver.on("/configsave", webInterfaceConfigSave);//Show the Configure Page
    webserver.on("/reset", webInterfaceSystemFormat);    //Reset Device
    webserver.begin();
    
    digitalWrite(LED_BUILTIN, LOW);
#ifdef USE_LEDS
    ledWebserverColors();
#endif
}

void webInterfaceLoop() {
    webserver.handleClient();
}

void webInterfaceSendHeader() {
  webserver.sendHeader("Cache-Control", "no-cache, no-store");
  webserver.sendHeader("Pragma", "no-cache");
  webserver.sendHeader("Expires", "-1");
//   webserver.chunkedResponseModeStart(200,"text/html");
  webserver.setContentLength(CONTENT_LENGTH_UNKNOWN);
  webserver.send(200, "text/html", "");
  webserver.sendContent_P(HEAD_TITLE);
  webserver.sendContent_P(BODY_MENU);
}

void webInterfaceSendFooter() {
  webserver.sendContent_P(BODY_END);
  webserver.sendContent("");
//   webserver.chunkedResponseFinalize();
  webserver.client().stop();
}

void webInterfaceRedirect(const char* url) {
    webserver.sendHeader("Location", url, true);
    webserver.sendHeader("Cache-Control", "no-cache, no-store");
    webserver.sendHeader("Pragma", "no-cache");
    webserver.sendHeader("Expires", "-1");
    webserver.send(302, "text/plain", "");
    webserver.client().stop();
    delay(1000);
}

static String getFileSizeString(size_t sizeInBytes) {
  if (sizeInBytes < 1024) return String(sizeInBytes) + " B";
  else if (sizeInBytes < (1024 * 1024)) return String(sizeInBytes / 1024.0f, 1) + " KB";
  else if (sizeInBytes < (1024 * 1024 * 1024)) return String(sizeInBytes / 1024.0f / 1024.0f, 1) + " MB";
  else return String(sizeInBytes / 1024.0f / 1024.0f / 1024.0f, 3) + " GB";
}

void webInterfaceHome() {
    webInterfaceSendHeader();

#ifdef LOG_GPS

    webserver.sendContent("<h1>QLiteOSD GPS Log Files</h1><ul>");

    Dir logDir = filesystem.openDir(LOG_DIRECTORY_PREFIX);
    while (logDir.next()) {
        if (logDir.isDirectory()) {
            continue;
        }

        // Reads the file header, if needed for the webpage
        // gps_logfile_header_t header;
        // File logFile = logDir.openFile("r");
        // logFile.readBytes((char*)&header,sizeof(header));
        // logFile.close();

        String name = logDir.fileName();
        webserver.sendContent("<li><a href='/download?file="+name+"' class='timeToConvert'>"+name+"</a> "
        +getFileSizeString(logDir.fileSize())+" | <a href='/delete?file="+name+"' onclick='return confirm(\"Do you want to delete this file?\")'>Delete</a></li>");
    }

    webserver.sendContent("</ul>");

    webserver.sendContent_P(LOGFILES_JAVASCRIPT);

#endif

    FSInfo fsInfo;
    filesystem.info(fsInfo);
    
    String spaceString = String("Space Used: <strong>")+getFileSizeString(fsInfo.usedBytes)+"</strong> | Free Space: <strong>";
    spaceString += getFileSizeString(fsInfo.totalBytes - fsInfo.usedBytes)+"</strong>";
    
    webserver.sendContent(spaceString);

    webInterfaceSendFooter();
}

static void replaceFormConfig(String& form) {
    for (int i = 0; i < CONFIG_VALUE_COUNT; i++) {
        String replacementKey = String("%")+configValues[i].key+"%";

        // if (form.indexOf(replacementKey) == -1) {
        //     continue;
        // }

        switch (configValues[i].type) {
        case CONFIG_VALUE_STRING:
            form.replace(replacementKey,String((char*)configValues[i].globalValue));
            break;
        case CONFIG_VALUE_BOOL:
            form.replace(replacementKey,(*(bool*)configValues[i].globalValue) ? "checked='checked'" : "");
            break;
        case CONFIG_VALUE_UINT16:
            form.replace(replacementKey,String(*(uint16_t*)configValues[i].globalValue));
            break;
        case CONFIG_VALUE_UINT8:
            form.replace(replacementKey,String(*(uint8_t*)configValues[i].globalValue));
            break;
        case CONFIG_VALUE_FLOAT:
            form.replace(replacementKey,String(*(float*)configValues[i].globalValue));
            break;
        }
    }
}

void webInterfaceConfigure() {
    webInterfaceSendHeader();

    {
        String form = FPSTR(CONFIG_FORM);
        replaceFormConfig(form);
        webserver.sendContent(form);
    }

#ifdef USE_LEDS
    {
        String options = FPSTR(CONFIG_RGB_OPTIONS);
        options.replace(">" + String(RGB_MODE) + "<", " selected>" + String(RGB_MODE) + "<");
        webserver.sendContent(options);
    }

    {
        String form = FPSTR(CONFIG_FORM_LED);
        replaceFormConfig(form);
        webserver.sendContent(form);
    }

    webserver.sendContent_P(CONFIG_RGB_JS);
#endif

    {
        String form = FPSTR(CONFIG_FORM_OSD);
        replaceFormConfig(form);
        webserver.sendContent(form);
    }

    {
        String form = FPSTR(CONFIG_FORM_OSD_2);
        replaceFormConfig(form);
        webserver.sendContent(form);
    }

    webserver.sendContent_P(CONFIG_FORM_SAVE);

    {
        String formJs = FPSTR(CONFIG_JAVASCRIPT);
        formJs.replace("%READVALUE%",String(analogRead(ANALOG_PIN)));
        formJs.replace("%VALUER1%",String(ValueR1));
        formJs.replace("%VALUER2%",String(ValueR2));
        webserver.sendContent(formJs);
    }

    webserver.sendContent_P(CONFIG_OSD);

    webInterfaceSendFooter();
}

void webInterfaceConfigSave() {
    for (int i = 0; i < CONFIG_VALUE_COUNT; i++) {
        String argumentName = String(configValues[i].key)+"_FORM";

        if (configValues[i].type == CONFIG_VALUE_BOOL) {
            *(bool*)configValues[i].globalValue = webserver.hasArg(argumentName);
            continue;
        }

        if (!webserver.hasArg(argumentName)) {
            continue;
        }

        String argValue = webserver.arg(argumentName);
        switch(configValues[i].type) {
        case CONFIG_VALUE_STRING:
            argValue.toCharArray((char*)configValues[i].globalValue,configValues[i].optionalData);
            break;
        case CONFIG_VALUE_UINT16:
            *(uint16_t*)configValues[i].globalValue = (uint16_t)argValue.toInt();
            break;
        case CONFIG_VALUE_UINT8:
            *(uint8_t*)configValues[i].globalValue = (uint8_t)argValue.toInt();
            break;
        case CONFIG_VALUE_FLOAT:
            *(float*)configValues[i].globalValue = argValue.toFloat();
            break;
        }
    }

    configWrite();
    webInterfaceRedirect("/");
}

static void resetSystem() {
    webserver.stop();
    digitalWrite(LED_BUILTIN, HIGH);
    WiFi.softAPdisconnect(true);
    ESP.reset();
}

void webInterfaceWifiOff() {
    webInterfaceSendHeader();
    webserver.sendContent("<p>WiFi Turned Off and OSD rebooted. <a href='/'>Return to main screen</a> when reconnected.</p>");
    webInterfaceSendFooter();
    resetSystem();
}

void webInterfaceSystemFormat() {
    webInterfaceSendHeader();

    if (filesystem.format()) {
        webserver.sendContent("<p>Filesystem and config successfully reset. OSD has been rebooted. <a href='/'>Return to main screen</a> when reconnected.</p>");

        webInterfaceSendFooter();
        resetSystem();
    }else{
        webserver.sendContent("<p>Filesystem failed to format! Please format it manually!</p>");
        webInterfaceSendFooter();
    }
}

#ifdef LOG_GPS
static void sendFileError(const String& error) {
    webInterfaceSendHeader();
    webserver.sendContent(error);
    webInterfaceSendFooter();
    return;
}

void webInterfaceDownloadLog() {
    if (!webserver.hasArg("file")) {
        sendFileError("No file given!");
        return;
    }

    String fileName = webserver.arg("file");
    File logFile = filesystem.open(String(LOG_DIRECTORY_PREFIX)+fileName,"r");
    if (!logFile) {
        sendFileError("File failed to open!");
        return;
    }
    if (logFile.size() <= sizeof(gps_logfile_header_t)) {
        sendFileError("File has no data!");
        return;
    }

    // debug: send the raw file
    // webserver.streamFile(logFile,"application/octet-stream");
    // webserver.client().stop();
    // return;
    
    gps_logfile_header_t header;
    logFile.readBytes((char*)&header,sizeof(header));
    size_t numFrames = ((logFile.size() - sizeof(header)) / sizeof(gps_logfile_frame_t));

    WiFiClient& client = webserver.client();
    client.print("HTTP/1.1 200 OK\r\n");
    client.print("Content-Disposition: attachment; filename=QLiteOSD_Path_" + fileName + ".gpx\r\n");
    client.print("Content-Type: application/octet-stream\r\n");
    client.print("Connection: close\r\n");
    client.print("Access-Control-Allow-Origin: *\r\n");
    client.print("\r\n");

    client.print(String("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<gpx version=\"1.0\">\n\t<name>QLiteOSD v" VERSION) + 
        "</name>\n\t<trk><name>QLiteOSD_Path_" + fileName + "</name><number>1</number><trkseg>\n");

    for (size_t i = 0; i < numFrames || logFile.available(); i++) {
        gps_logfile_frame_t frame;
        logFile.readBytes((char*)&frame,sizeof(frame));

        client.printf("\t\t<trkpt lat=\"%f\" lon=\"%f\"><ele>%f</ele>",frame.latitude,frame.longitude,frame.altitude);

        time_t currentTime = header.fileStartTime + (time_t)floorf(frame.timeOffset/1000.0f);

        char timeBuffer[sizeof("0000-00-00T00:00:00.000Z") + 5];
        strftime(timeBuffer,sizeof(timeBuffer),"%Y-%m-%dT%H:%M:%S", gmtime(&currentTime));
        
        client.printf("<time>%s.%03dZ</time><speed>%f</speed><course>%f</course></trkpt>\n",timeBuffer,frame.timeOffset%1000,frame.speed,frame.heading);
    }

    client.print("</trkseg></trk>\n</gpx>\n\n");
    client.stop();
    logFile.close();
}

void webInterfaceDeleteLog() {
    if (!webserver.hasArg("file")) {
        sendFileError("No file given!");
        return;
    }

    String fileName = webserver.arg("file");
    if (!filesystem.remove(String(LOG_DIRECTORY_PREFIX)+fileName)) {
        sendFileError("LogFile "+fileName+" failed to delete!");
    }

    webInterfaceRedirect("/");
}
#endif

#endif
