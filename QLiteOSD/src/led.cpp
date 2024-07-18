#include "led.h"

#ifdef USE_LEDS

#include <Adafruit_NeoPixel.h>
#include "craft.h"

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

#ifdef WEB_INTERFACE
void ledWebserverColors() {
    pixels.fill(pixels.Color(0, 0, 155), 0, NUM_LEDS);
    pixels.show();
}
#endif

void ledInit() {
    pixels.begin();
    ledUpdate(); // Set the LEDs to their initial values
}

void ledUpdate() {
    static bool flashOn = true;

    String ledMode = RGB_MODE;
    flashOn = !flashOn;

    if (ledMode == "OFF") {
        pixels.fill(pixels.Color(0, 0, 0), 0, NUM_LEDS);
    } else if (ledMode == "ON") {
        pixels.fill(pixels.Color(RED_VALUE,GREEN_VALUE,BLUE_VALUE), 0, NUM_LEDS);
    } else if (ledMode == "BATTERY") {
        float cellVoltage = craftState.voltage / craftState.getCellCount();
        uint8_t rVal;
        uint8_t gVal;

        if (cellVoltage >= 4.2f) {
            gVal = 255;
            rVal = 0;
        }else if(cellVoltage <= 3.6f) {
            gVal = 0;
            rVal = 255;
        }else {
            float percent = (cellVoltage-3.6f)/(4.2f-3.6f);
            gVal = percent*255;
            rVal = (1.0f-percent)*255;
        }

        if (flashOn && cellVoltage <= 36) {
            pixels.fill(pixels.Color(0, 0, 0), 0, NUM_LEDS);
        } else {
            pixels.fill(pixels.Color(rVal, gVal, 0), 0, NUM_LEDS);
        }
    } else if (ledMode == "ALTITUDE") {
        float alt = craftState.getRelativeAltitude();
        uint8_t rVal;
        uint8_t gVal;

        if (alt >= 121.92f) { // 400 ft
            gVal = 0;
            rVal = 255;
        }else if(alt <= 0.0f) {
            gVal = 255;
            rVal = 0;
        }else {
            float percent = (alt)/(121.92f);
            rVal = percent*255;
            gVal = (1.0f-percent)*255;
        }

        if (flashOn && alt >= 121.92f) {
            pixels.fill(pixels.Color(0, 0, 0), 0, NUM_LEDS);
        } else {
            pixels.fill(pixels.Color(rVal, gVal, 0), 0, NUM_LEDS);
        }
    } else if (ledMode == "STROBE") {
        if (flashOn) {
            pixels.fill(pixels.Color(RED_VALUE, GREEN_VALUE, BLUE_VALUE), 0, NUM_LEDS);
        } else {
            pixels.fill(pixels.Color(0, 0, 0), 0, NUM_LEDS);
        }
    } else if (ledMode == "AIRCRAFT") {
        pixels.fill(pixels.Color(0, 0, 0), 0, 24);
        pixels.fill(pixels.Color(0, 255, 0), 0, 4);
        pixels.fill(pixels.Color(255, 255, 255), 4, 4);
        pixels.fill(pixels.Color(255, 0, 0), 8, 4);
        if (!flashOn) {
          pixels.fill(pixels.Color(0, 0, 0), 4, 4);
        }
    }

    pixels.show();
}

#endif
