#if defined(ARDUINO_ARCH_ESP32)

#include "kits/zeego.h"
#include "virtual_machine/virtual_machine.h"
#include <Arduino.h>

uint16_t offsetValues[5] = {4095, 4095, 4095, 4095, 4095};
uint8_t lastPosition;

void readFloorCalibration()
{
    uint8_t buffer[10];
    esp_flash_read(esp_flash_default_chip, buffer, 0x200000 + (4096 * 127), 10);
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

uint8_t readFloorNumValue()
{
    uint8_t floorPins[5] = {FLOOR_5, FLOOR_4, FLOOR_3, FLOOR_2, FLOOR_1};
    bool readedValues[5];
    int counter = 0;
    for (int i = 0; i < 5; i++)
    {
        int rawValue = analogRead(floorPins[i]);
        readedValues[i] = rawValue <= offsetValues[i];
        if (readedValues[i])
        {
            counter++;
        }
    }
    switch (counter)
    {
    case 1:

        if (readedValues[0])
        {
            lastPosition = 1;
        }
        if (readedValues[1])
        {
            lastPosition = 3;
        }
        if (readedValues[2])
        {
            lastPosition = 5;
        }
        if (readedValues[3])
        {
            lastPosition = 7;
        }
        if (readedValues[4])
        {
            lastPosition = 9;
        }
        break;
    case 2:
        if (readedValues[0] && readedValues[1])
        {
            lastPosition = 2;
        }
        if (readedValues[1] && readedValues[2])
        {
            lastPosition = 4;
        }

        if (readedValues[2] && readedValues[3])
        {
            lastPosition = 6;
        }
        if (readedValues[3] && readedValues[4])
        {
            lastPosition = 8;
        }
        break;
    case 3:
        if (readedValues[0] && readedValues[1] && readedValues[2])
        {
            lastPosition = 3;
        }
        if (readedValues[1] && readedValues[2] && readedValues[3])
        {
            lastPosition = 5;
        }
        if (readedValues[2] && readedValues[3] && readedValues[4])
        {
            lastPosition = 7;
        }
        break;
    case 4:
        if (readedValues[0] && readedValues[1] && readedValues[2] && readedValues[3])
        {
            lastPosition = 1;
        }
        if (readedValues[1] && readedValues[2] && readedValues[3] && readedValues[4])
        {
            lastPosition = 9;
        }
        break;
    }

    return lastPosition;
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

            offsetValues[i] = offsetValues[i] + 150;
        }
        esp_flash_erase_region(esp_flash_default_chip, 0x200000 + (4096 * 127), 4096);
        delay(150);
        uint8_t buffer[10];
        for (uint8_t i = 0; i < 5; i++)
        {
            buffer[i * 2] = offsetValues[i] & 0xFF;
            buffer[(i * 2) + 1] = (offsetValues[i] >> 8) & 0xFF;
        }
        esp_flash_write(esp_flash_default_chip, buffer, 0x200000 + (4096 * 127), 10);
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