#if defined(ARDUINO_ARCH_ESP32)

#include "kits/kidsy.h"
#include "virtual_machine/virtual_machine.h"
#include <Arduino.h>

void calibrateKidsyColorSensor(VEML6040 RGBWSensor)
{
    pinMode(BUTTON_A, INPUT);
    pinMode(BUTTON_C, INPUT);

    if (digitalRead(BUTTON_A) == HIGH || digitalRead(BUTTON_C) == HIGH)
    {
        bool noCalibrate = digitalRead(BUTTON_C) == HIGH;

        Adafruit_NeoPixel neoPixel = Adafruit_NeoPixel(1, 19, NEO_GRB + NEO_KHZ800);
        RGBWSensor.nairdaBegin();
        if (!noCalibrate)
        {

            pinMode(BUTTON_B, INPUT);
            uint16_t minValues[4];
            uint16_t maxValues[4];

            while (digitalRead(BUTTON_B) == LOW)
            {
                for (uint8_t i = 0; i < 2; i++)
                {
                    neoPixel.setPixelColor(0, i ? 80 : 0, i ? 80 : 0, i ? 80 : 0);
                    neoPixel.show();
                    minValues[0] = RGBWSensor.getRed();
                    minValues[1] = RGBWSensor.getGreen();
                    minValues[2] = RGBWSensor.getBlue();
                    minValues[3] = RGBWSensor.getWhite();
                    delay(100);
                }
            }
            neoPixel.setPixelColor(0, 80, 80, 80);
            neoPixel.show();
            delay(1000);
            while (digitalRead(BUTTON_B) == LOW)
            {

                maxValues[0] = RGBWSensor.getRed();
                maxValues[1] = RGBWSensor.getGreen();
                maxValues[2] = RGBWSensor.getBlue();
                maxValues[3] = RGBWSensor.getWhite();
                delay(100);
            }
            neoPixel.setPixelColor(0, 0, 0, 0);
            neoPixel.show();
            spi_flash_erase_range(0x200000 + (4096 * 127), 4096);
            delay(150);
            uint8_t buffer[16];
            for (uint8_t i = 0; i < 4; i++)
            {
                buffer[i * 2] = minValues[i] & 0xFF;
                buffer[(i * 2) + 1] = (minValues[i] >> 8) & 0xFF;
            }
            for (uint8_t i = 0; i < 4; i++)
            {
                buffer[8 + (i * 2)] = maxValues[i] & 0xFF;
                buffer[8 + ((i * 2) + 1)] = (maxValues[i] >> 8) & 0xFF;
            }
            spi_flash_write(0x200000 + (4096 * 127), buffer, 16);
            delay(150);
        }
        RGBWSensor.readCalibration();
        while (true)
        {
            RGBWSensor.readFixedColors();
            delay(100);
            neoPixel.setPixelColor(0, RGBWSensor.getFixedRed(), RGBWSensor.getFixedGreen(), RGBWSensor.getFixedBlue());
            neoPixel.show();
        }
    }
}

#endif