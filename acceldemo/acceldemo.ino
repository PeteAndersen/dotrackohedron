// Basic demo for accelerometer readings from Adafruit LIS3DH

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_NeoPixel.h>

#define LIS3DH_CS 5
Adafruit_LIS3DH lis = Adafruit_LIS3DH(LIS3DH_CS);

#define MAX_BRIGHTNESS 255

Adafruit_NeoPixel pixels(1, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

void setup(void) {
  while (!Serial);

  Serial.begin(9600);
  Serial.println("LIS3DH test!");
  
  if (! lis.begin(0x18)) {   // change this to 0x19 for alternative i2c address
    Serial.println("Couldnt start");
    while (1);
  }
  Serial.println("LIS3DH found!");
  
  lis.setRange(LIS3DH_RANGE_2_G);   // 2, 4, 8 or 16 G!
  
  Serial.print("Range = "); Serial.print(2 << lis.getRange());  
  Serial.println("G");

  pixels.begin();
}

void loop() {
  uint8_t r, g, b;
  lis.read();      // get X Y and Z data at once

  /* Or....get a new sensor event, normalized */ 
  sensors_event_t event; 
  lis.getEvent(&event);

  r = scaleAccelToColor(event.acceleration.x);
  g = scaleAccelToColor(event.acceleration.y);
  b = scaleAccelToColor(event.acceleration.z);

  pixels.setPixelColor(0, pixels.Color(r, g, b));
  pixels.show();
  
  /* Display the results (acceleration is measured in m/s^2) */
  Serial.print("rgb: "); Serial.print(r);
  Serial.print(", "); Serial.print(g);
  Serial.print(", "); Serial.print(b);
  Serial.print("\t\tX: "); Serial.print(event.acceleration.x);
  Serial.print(" \tY: "); Serial.print(event.acceleration.y); 
  Serial.print(" \tZ: "); Serial.print(event.acceleration.z); 
  Serial.println();
  
  delay(50);
}

uint8_t scaleAccelToColor(float accel) {
  float clamped_accel = max(min(accel, SENSORS_GRAVITY_EARTH), -SENSORS_GRAVITY_EARTH);
  float m = (0.0 - MAX_BRIGHTNESS) / (-SENSORS_GRAVITY_EARTH - SENSORS_GRAVITY_EARTH);
  float b = m * SENSORS_GRAVITY_EARTH;
  return m * clamped_accel + b;
}
