#include <Wire.h>
#include <SPI.h>
#include "lis3dh.h"
#include <Adafruit_NeoPixel.h>

#define LIS3DH_CS 5
lis3dh lis = lis3dh(LIS3DH_CS);

#define MAX_BRIGHTNESS 255
Adafruit_NeoPixel pixels(1, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

void setup(void)
{
  if (!lis.begin())
  {
    Serial.println("Failed to initialize LIS3DH");
    while (1)
      ;
  }
  Serial.println("LIS3DH found!");

  lis.setRange(LIS3DH_RANGE_2_G);
  lis.setDataRate(LIS3DH_DATARATE_25_HZ);

  Serial.print("Range = ");
  Serial.print(2 << lis.getRange());
  Serial.println("G");
  Serial.println();

  delay(1000);

  Serial.print("Calibrating...");
  if (!lis.calibrate())
  {
    Serial.println("Failed to calibrate LIS3DH");
    while (1)
      ;
  }

  pixels.begin();
}

void loop()
{
  uint8_t r, g, b;
  lis.read();

  r = scaleAccelToColor(lis.x_g);
  g = scaleAccelToColor(lis.y_g);
  b = scaleAccelToColor(lis.z_g);

  pixels.setPixelColor(0, pixels.Color(r, g, b));
  pixels.show();

  /* Display the results */
  Serial.print("rgb: ");
  Serial.print(r);
  Serial.print(", ");
  Serial.print(g);
  Serial.print(", ");
  Serial.print(b);
  Serial.print("\t\tX: ");
  Serial.print(lis.x);
  Serial.print(" \tY: ");
  Serial.print(lis.y);
  Serial.print(" \tZ: ");
  Serial.print(lis.z);
  Serial.println();

  delay(50);
}

uint8_t scaleAccelToColor(float accel)
{
  float clamped_accel = max(min(accel, 1.0), -1.0);
  return (MAX_BRIGHTNESS) / 2 * clamped_accel + (MAX_BRIGHTNESS / 2);
}
