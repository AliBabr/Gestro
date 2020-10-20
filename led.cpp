#include <Adafruit_NeoPixel.h>
#include "led.h"
#include "GastroMarket.h"
#ifdef __AVR__
#include <avr/power.h>
#endif

// How many NeoPixels are attached to the Arduino?
#define NUMPIXELS      4

// When we setup the NeoPixel library, we tell it how many pixels, and which pin to use to send signals.
// Note that for older NeoPixel strips you might need to change the third parameter--see the strandtest
// example for more information on possible values.
//Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, RED_LED_HAND, NEO_GRB + NEO_KHZ800);

void LEDsSetup() {
  pixels.begin(); // This initializes the NeoPixel library.
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if (WheelPos < 85) {
    return pixels.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if (WheelPos < 170) {
    WheelPos -= 85;
    return pixels.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return pixels.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}

void SetLEDs() {
  static uint16_t i, j = 0;
  j = !j;
  if (j)
  {
    pixels.setPixelColor(i, pixels.Color(0, 150, 0));
  }
  else
  {
     pixels.setPixelColor(i, pixels.Color(0, 0, 0));
      i++;
  }
  pixels.show();
 
  if (i == 4) i = 0;
}
