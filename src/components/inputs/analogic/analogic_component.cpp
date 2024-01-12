#include "virtual_machine/virtual_machine.h"
#include "analogic_component.h"
#include "components/inputs/digital_in/digital_in_component.h"
#include "extern_libraries/linked_list/linked_list.h"

#include <Arduino.h>

extern bool loadedAnalogics;
extern int temp;
extern int hum;

#if defined(ARDUINO_ARCH_ESP32)
#include "extern_libraries/veml6040/VEML6040.h"
#include "extern_libraries/dht11/DHT.h"
#include "kits/zeego.h"
extern VEML6040 RGBWSensor;
extern DHT dht;
unsigned long previousMillis = 0;

#endif

extern uint8_t currentKit;

void analogicCreate(uint16_t *args, component_t *component)
{

    component->pins[0] = args[1];
#if defined(ARDUINO_ARCH_ESP32)
    if (currentKit == ROBBUS_KIDSY_KIT && (args[1] == 37 || args[1] == 38 || args[1] == 39))
    {
        RGBWSensor.nairdaBegin();
    }
#endif
}

void analogicSense(uint8_t *pins, uint8_t *tempRead)
{
#if defined(ARDUINO_ARCH_STM32)
    tempRead[0] = map(analogRead(pins[0]), 0, 1023, 0, 100);
#elif defined(ARDUINO_ARCH_ESP32)

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
            tempRead[0] = temp < 0 ? 0 : temp > 100 ? 100
                                                    : temp;
            break;
        case 17:

            tempRead[0] = hum < 0 ? 0 : hum > 100 ? 100
                                                  : hum;
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
        if (pins[0] == 37)
        {
            tempRead[0] = readFloorValue();
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
#else
    tempRead[0] = map(analogRead(pins[0]), 0, 1023, 0, 100);
#endif
}

void analogicOff()
{
}

void analogicDebugLoad(VolatileMemory *volatileMemory)
{
    volatileMemory->descArgsBuffer[0] = ANALOGIC;
    volatileMemory->descArgsBuffer[1] = getMapedPin(volatileMemory->declarationBuffer[0]);
    component_t *component = (component_t *)malloc(sizeof(component_t));
    analogicCreate(volatileMemory->descArgsBuffer, component);
    volatileMemory->components[ANALOGIC].add(component);
}

void analogicEepromLoad(VolatileMemory *volatileMemory)
{
#ifndef __AVR_ATmega168__
    uint8_t currentByte;
    while (!loadedAnalogics)
    {
        currentByte = nextByte();
        if (currentByte == endAnalogics)
        {
            loadedAnalogics = true;
        }
        else
        {
            volatileMemory->descArgsBuffer[0] = ANALOGIC;
            volatileMemory->descArgsBuffer[1] = getMapedPin(currentByte);
            component_t *component = (component_t *)malloc(sizeof(component_t));
            analogicCreate(volatileMemory->descArgsBuffer, component);
            volatileMemory->components[ANALOGIC].add(component);
        }
    }
    digitalInEepromLoad(volatileMemory);
#endif
}

int32_t analogicEepromRead(VolatileMemory *volatileMemory)
{
    return getSensVal(ANALOGIC, volatileMemory->components[ANALOGIC].get(nextByte()));
}
