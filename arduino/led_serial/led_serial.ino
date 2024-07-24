#include <FastLED.h>

#define LED_PIN 6
#define NUM_LEDS 200
#define LED_TYPE WS2811
#define COLOR_ORDER RGB

/*
Serial Commands:
- "OFF": turns off all leds (same as "COLOR 0,0,0")
- "COLOR r,g,b": change color of all leds
- "BRIGHTNESS n": change brightness of all leds
*/

CRGB leds[NUM_LEDS];

void setup() {
  // Initialize the FastLED library
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(255);  // Set initial brightness to maximum

  // Start serial communication
  Serial.begin(9600);

  // Initially set all LEDs to off
  FastLED.clear();
  FastLED.show();
}

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    handleCommand(command);
  }
}

void handleCommand(String command) {
  command.trim();  // Remove any leading/trailing whitespace characters

  if (command.equalsIgnoreCase("OFF")) {
    FastLED.clear();
    FastLED.show();
  } else if (command.startsWith("COLOR")) {
    int firstComma = command.indexOf(',');
    int secondComma = command.indexOf(',', firstComma + 1);

    if (firstComma > 0 && secondComma > firstComma) {
      int r = command.substring(6, firstComma).toInt();
      int g = command.substring(firstComma + 1, secondComma).toInt();
      int b = command.substring(secondComma + 1).toInt();
      fill_solid(leds, NUM_LEDS, CRGB(r, g, b));
      FastLED.show();
    } else {
      Serial.println("ERROR: Invalid COLOR command. Use COLOR r,g,b format.");
    }
  } else if (command.startsWith("BRIGHTNESS")) {
    int brightness = command.substring(11).toInt();
    if (brightness >= 0 && brightness <= 255) {
      FastLED.setBrightness(brightness);
      FastLED.show();
    } else {
      Serial.println("ERROR: Brightness must be between 0 and 255.");
    }
  } else {
    Serial.println("ERROR: Unknown command.");
  }
}
