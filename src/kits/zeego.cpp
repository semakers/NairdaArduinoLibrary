#if defined(ARDUINO_ARCH_ESP32)

#include "kits/zeego.h"
#include "virtual_machine/virtual_machine.h"
#include <Arduino.h>

uint16_t offsetValues[5] = {4095, 4095, 4095, 4095, 4095};

void readFloorCalibration()
{
    uint8_t buffer[10];
    spi_flash_read(0x200000 + (4096 * 127), buffer, 10);
    delay(150);
    for (int i = 0; i < 5; i++)
    {
        offsetValues[i] = (uint16_t)buffer[i * 2] | ((uint16_t)buffer[i * 2 + 1] << 8);
    }
}

uint8_t readFloorValue()
{
    uint8_t floorPins[5] = {FLOOR_5, FLOOR_4, FLOOR_3, FLOOR_2, FLOOR_1};
    uint8_t result = 0;
    for (int i = 0; i < 5; i++)
    {
        int rawValue = analogRead(floorPins[i]);
        result |= (rawValue <= offsetValues[i] ? 1 : 0) << i;
    }
    return result;
}

void calibrateZeegoFloorSensor(Adafruit_SSD1306 display)
{
    pinMode(JOYSTICK_UP, INPUT);
    pinMode(JOYSTICK_DOWN, INPUT);
    if (digitalRead(JOYSTICK_UP) == LOW)
    {

        if (display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
        {
            display.clearDisplay();
            display.setTextSize(2);
            display.setTextColor(1);
            display.setCursor(1, 1);
            display.print("Calibrando");
            display.display();
        }

        uint8_t florrPins[5] = {FLOOR_1, FLOOR_2, FLOOR_3, FLOOR_4, FLOOR_5};
        while (digitalRead(JOYSTICK_DOWN) == HIGH)
        {
            for (uint8_t i = 0; i < 5; i++)
            {
                uint16_t rawValue = analogRead(florrPins[i]);
                if (rawValue < offsetValues[i])
                {
                    offsetValues[i] = rawValue;
                }
            }
        }
        for (uint8_t i = 0; i < 5; i++)
        {

            offsetValues[i] = offsetValues[i] + 20;
        }
        spi_flash_erase_range(0x200000 + (4096 * 127), 4096);
        delay(150);
        uint8_t buffer[10];
        for (uint8_t i = 0; i < 5; i++)
        {
            buffer[i * 2] = offsetValues[i] & 0xFF;
            buffer[(i * 2) + 1] = (offsetValues[i] >> 8) & 0xFF;
        }
        spi_flash_write(0x200000 + (4096 * 127), buffer, 16);
        delay(150);

        display.clearDisplay();
        display.setTextSize(2);
        display.setTextColor(1);
        display.setCursor(1, 1);
        display.print("Listo");
        display.display();

        while (true)
        {
        }
    }
}

#endif