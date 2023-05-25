/*

The MIT License (MIT)

Copyright (c) 2015 thewknd

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

*/
#if defined(ARDUINO_ARCH_ESP32)
#include "Wire.h"
#ifndef __MATH_H
#include <math.h>
#endif
#include "veml6040.h"
#include <Arduino.h>

VEML6040::VEML6040(void)
{
}

void VEML6040::nairdaBegin(void)
{
  if (!working)
  {

    begin();
    setConfiguration(VEML6040_IT_40MS + VEML6040_TRIG_ENABLE + VEML6040_AF_AUTO + VEML6040_SD_ENABLE);
    ledcSetup(0, 50, 8);
    ledcAttachPin(16, 0);
    ledcWrite(0, 255);
    /* for (int i = 0; i < 1000; i++)
     {
       readFixedColors();
     }*/

    working = true;
  }
}

void VEML6040::nairdaEnd(void)
{
  working = false;
}

bool VEML6040::begin(void)
{
  bool sensorExists = false;
  Wire.begin(23, 22);
  Wire.beginTransmission(VEML6040_I2C_ADDRESS);
  if (Wire.endTransmission() == 0)
  {
    sensorExists = true;
  }
  return sensorExists;
}

void VEML6040::setConfiguration(uint8_t configuration)
{
  Wire.beginTransmission(VEML6040_I2C_ADDRESS);
  Wire.write(COMMAND_CODE_CONF);
  Wire.write(configuration);
  Wire.write(0);
  Wire.endTransmission();
  lastConfiguration = configuration;
}

uint16_t VEML6040::read(uint8_t commandCode)
{
  uint16_t data = 0;

  Wire.beginTransmission(VEML6040_I2C_ADDRESS);
  Wire.write(commandCode);
  Wire.endTransmission(false);
  Wire.requestFrom(VEML6040_I2C_ADDRESS, 2);
  while (Wire.available())
  {
    data = Wire.read();
    data |= Wire.read() << 8;
  }

  return data;
}

uint16_t VEML6040::getRed(void)
{
  return (read(COMMAND_CODE_RED));
}

uint16_t VEML6040::getGreen(void)
{
  return (read(COMMAND_CODE_GREEN));
}

uint16_t VEML6040::getBlue(void)
{
  return (read(COMMAND_CODE_BLUE));
}

uint16_t VEML6040::getWhite(void)
{
  return (read(COMMAND_CODE_WHITE));
}

void VEML6040::readFixedColors(void)
{

  fixedRed = map(getRed(), MIN_RED, MAX_RED, 0, 255);
  fixedGreen = map(getGreen(), MIN_GREEN, MAX_GREEN, 0, 255);
  fixedBlue = map(getBlue(), MIN_BLUE, MAX_BLUE, 0, 255);
  fixedWhite = map(getWhite(), MIN_WHITE, MAX_WHITE, 0, 100);

  double colorsTogether = ((double)fixedRed + fixedGreen + fixedBlue);
  int min;
  float a;

  if (colorsTogether > 10)
  {
    double factor = 100.0 / colorsTogether;
    fixedRed = fixedRed * factor;
    fixedGreen = fixedGreen * factor;
    fixedBlue = fixedBlue * factor;
  }

  min = (fixedRed < fixedGreen) ? (fixedRed < fixedBlue ? fixedRed : fixedBlue) : (fixedGreen < fixedBlue ? fixedGreen : fixedBlue);
  a = (255 - min) / 255.0;

  fixedRed = (int)((fixedRed - min) / a);
  fixedGreen = (int)((fixedGreen - min) / a);
  fixedBlue = (int)((fixedBlue - min) / a);

  colorsTogether = ((double)fixedRed + fixedGreen + fixedBlue);
  if (colorsTogether > 10)
  {
    double factor = 100.0 / colorsTogether;
    fixedRed = fixedRed * factor;
    fixedGreen = fixedGreen * factor;
    fixedBlue = fixedBlue * factor;
  }

  if (fixedWhite > 90 && fixedRed < 10 && fixedGreen < 10 && fixedBlue < 10)
  {
    fixedRed = 100;
    fixedGreen = 100;
    fixedBlue = 100;
  }
  else if (fixedWhite < 5)
  {
    fixedRed = 0;
    fixedGreen = 0;
    fixedBlue = 0;
  }

  /* Serial.print(" ");
   Serial.print((int)fixedRed);
   Serial.print(" ");
   Serial.print((int)fixedGreen);
   Serial.print(" ");
   Serial.print((int)fixedBlue);
   Serial.print(" ");
   Serial.print((int)fixedWhite);
   Serial.println("");*/
}

uint8_t VEML6040::getFixedRed(void)
{

  return fixedRed;
}

uint8_t VEML6040::getFixedGreen(void)
{
  return fixedGreen;
}

uint8_t VEML6040::getFixedBlue(void)
{
  return fixedBlue;
}

float VEML6040::getAmbientLight(void)
{
  uint16_t sensorValue;
  float ambientLightInLux;

  sensorValue = read(COMMAND_CODE_GREEN);

  switch (lastConfiguration & 0x70)
  {

  case VEML6040_IT_40MS:
    ambientLightInLux = sensorValue * VEML6040_GSENS_40MS;
    break;
  case VEML6040_IT_80MS:
    ambientLightInLux = sensorValue * VEML6040_GSENS_80MS;
    break;
  case VEML6040_IT_160MS:
    ambientLightInLux = sensorValue * VEML6040_GSENS_160MS;
    break;
  case VEML6040_IT_320MS:
    ambientLightInLux = sensorValue * VEML6040_GSENS_320MS;
    break;
  case VEML6040_IT_640MS:
    ambientLightInLux = sensorValue * VEML6040_GSENS_640MS;
    break;
  case VEML6040_IT_1280MS:
    ambientLightInLux = sensorValue * VEML6040_GSENS_1280MS;
    break;
  default:
    ambientLightInLux = -1;
    break;
  }
  return ambientLightInLux;
}

uint16_t VEML6040::getCCT(float offset)
{
  uint16_t red, blue, green;
  float cct, ccti;

  red = read(COMMAND_CODE_RED);
  green = read(COMMAND_CODE_GREEN);
  blue = read(COMMAND_CODE_BLUE);

  ccti = ((float)red - (float)blue) / (float)green;
  ccti = ccti + offset;
  cct = 4278.6 * pow(ccti, -1.2455);

  return ((uint16_t)cct);
}
#endif
