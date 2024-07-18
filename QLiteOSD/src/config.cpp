#include "config.h"

#ifdef WEB_INTERFACE

#include "filesystem.h"

void configInit() {
  if (!filesystem.exists(CONFIG_FILE_PATH)) {
    configWrite(); // Write defaults to config file
  }else {
    configRead();
  }
}

// Read values from the config file and set them globally
void configRead() {
    File configFile = filesystem.open(CONFIG_FILE_PATH,"r");

    while(configFile.available()) {
        String line = configFile.readStringUntil('\n');
        // Bad O(n^2) search
        for (int i = 0; i < CONFIG_VALUE_COUNT; i++) {
            if (line.indexOf(String(configValues[i].key)+"=") != 0) {
                continue;
            }

            int valueStartIndex = strlen(configValues[i].key)+1;
            String valueString = line.substring(valueStartIndex);

            switch(configValues[i].type) {
            case CONFIG_VALUE_STRING:
                if (valueString.length() <= configValues[i].optionalData) {
                    valueString.trim();
                    valueString.toCharArray((char*)configValues[i].globalValue,configValues[i].optionalData);
                }
                break;
            case CONFIG_VALUE_BOOL:
                *(bool*)configValues[i].globalValue = (bool)valueString.toInt();
                break;
            case CONFIG_VALUE_UINT16:
                *(uint16_t*)configValues[i].globalValue = (uint16_t)valueString.toInt();
                break;
            case CONFIG_VALUE_UINT8:
                *(uint8_t*)configValues[i].globalValue = (uint8_t)valueString.toInt();
                break;
            case CONFIG_VALUE_FLOAT:
                *(float*)configValues[i].globalValue = valueString.toFloat();
              break;
            }

            break;
        }
    }
    
}

// Save configuration values based on what is currently set in memory
void configWrite() {
  File configFile = filesystem.open(CONFIG_FILE_PATH,"w");
  
  for (int i = 0; i < CONFIG_VALUE_COUNT; i++) {
    configFile.printf("%s=",configValues[i].key);
    switch(configValues[i].type) {
      case CONFIG_VALUE_STRING:
        configFile.println(String((char*)configValues[i].globalValue));
        break;
      case CONFIG_VALUE_BOOL:
        configFile.println(String((int)*(bool*)configValues[i].globalValue));
        break;
      case CONFIG_VALUE_UINT16:
        configFile.println(String(*(uint16_t*)configValues[i].globalValue));
        break;
      case CONFIG_VALUE_UINT8:
        configFile.println(String(*(uint8_t*)configValues[i].globalValue));
        break;
      case CONFIG_VALUE_FLOAT:
        configFile.println(String(*(float*)configValues[i].globalValue));
        break;
      }
  }

  configFile.close();
}

#endif
