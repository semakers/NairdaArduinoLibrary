#if defined(ARDUINO_ARCH_ESP32)

#include <Arduino.h>
#include "virtual_machine/virtual_machine.h"
#include "nairda.h"

#include "extern_libraries/veml6040/VEML6040.h"
#include "extern_libraries/dht11/DHT.h"
#include "kits/zeego.h"

extern VEML6040 RGBWSensor;
extern DHT dht;
extern int temp;
extern int hum;
extern uint8_t currentKit;

static unsigned long previousMillis = 0;

// ── Analogic ───────────────────────────────────────────────────────

void analogicCreate(uint16_t *args, component_t *component)
{
    component->pins[0] = args[1];
    if (currentKit == ROBBUS_KIDSY_KIT && (args[1] == 37 || args[1] == 38 || args[1] == 39))
    {
        RGBWSensor.nairdaBegin();
    }
}

void analogicSense(uint8_t *pins, uint8_t *tempRead)
{
    if (currentKit == LK32_KIT)
    {
        if (pins[0] == 16 || pins[0] == 17)
        {
            unsigned long currentMillis = millis();
            if (currentMillis - previousMillis >= 2000)
            {
                delay(10);
                previousMillis = currentMillis;
                temp = (round(dht.readTemperature()));
                hum = (round(dht.readHumidity()));
            }
        }
        switch (pins[0])
        {
        case 16:
            tempRead[0] = temp < 0 ? 0 : temp > 100 ? 100 : temp;
            break;
        case 17:
            tempRead[0] = hum < 0 ? 0 : hum > 100 ? 100 : hum;
            break;
        default:
            tempRead[0] = map(analogRead(pins[0]), 0, 4095, 0, 100);
        }
    }
    else if (currentKit == ROBBUS_KIDSY_KIT)
    {
        if (pins[0] == 37 || pins[0] == 38 || pins[0] == 39)
        {
            RGBWSensor.readFixedColors();
        }
        switch (pins[0])
        {
        case 37:
            tempRead[0] = RGBWSensor.getFixedRed();
            break;
        case 38:
            tempRead[0] = RGBWSensor.getFixedGreen();
            break;
        case 39:
            tempRead[0] = RGBWSensor.getFixedBlue();
            break;
        default:
            tempRead[0] = map(analogRead(pins[0]), 0, 4095, 0, 100);
        }
    }
    else if (currentKit == ROBBUS_ZEEGO_KIT)
    {
        if (pins[0] == 37 || pins[0] == 38)
        {
            switch (pins[0])
            {
            case 37:
                tempRead[0] = readFloorValue();
                break;
            case 38:
                tempRead[0] = readFloorNumValue();
                break;
            }
        }
        else
        {
            tempRead[0] = map(analogRead(pins[0]), 0, 4095, 0, 100);
        }
    }
    else
    {
        tempRead[0] = map(analogRead(pins[0]), 0, 4095, 0, 100);
    }
}

// ── Digital Input ──────────────────────────────────────────────────

static uint8_t isKidsyArrowPin(uint8_t pin)
{
    uint8_t kidsyArrowsPins[4] = {12, 13, 15, 14};
    for (uint8_t i = 0; i < 4; i++)
    {
        if (pin == kidsyArrowsPins[i])
            return 1;
    }
    return 0;
}

void digitalInSense(uint8_t *pins, uint8_t *tempRead)
{
    if (currentKit == ROBBUS_KIDSY_KIT)
    {
#if !defined(CONFIG_IDF_TARGET_ESP32C3)
        if (isKidsyArrowPin(pins[0]) == 1) {
            tempRead[0] = touchRead(pins[0]) > 15 ? 0 : 1;
        } else {
            tempRead[0] = digitalRead(pins[0]);
        }
#else
        tempRead[0] = digitalRead(pins[0]);
#endif
    }
    else
    {
        tempRead[0] = digitalRead(pins[0]);
    }
}

#endif
