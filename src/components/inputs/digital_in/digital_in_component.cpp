#include "virtual_machine/virtual_machine.h"
#include "digital_in_component.h"
#include "extern_libraries/linked_list/linked_list.h"

#include <Arduino.h>

extern bool loadedDigitalIns;
extern VolatileMemory volatileMemory;
extern uint8_t currentKit;

#if defined(ARDUINO_ARCH_ESP32)

uint8_t isKidsyArrowPin(uint8_t pin)
{
    uint8_t kidsyArrowsPins[4] = {12, 13, 15, 14};

    for (uint8_t i = 0; i < 4; i++)
    {
        if (pin == kidsyArrowsPins[i])
        {
            return 1;
        }
    }
    return 0;
}

#endif

void digitalInCreate(uint16_t *args, component_t *component)
{
    component->pins[0] = args[1];
    pinMode(component->pins[0], INPUT);
}

void digitalInSense(uint8_t *pins, uint8_t *tempRead)
{
#if defined(ARDUINO_ARCH_ESP32)
    if (currentKit == ROBUS_KIDSY_KIT)
    {
        if (isKidsyArrowPin(pins[0]) == 1)
        {
            tempRead[0] = touchRead(pins[0]) > 15 ? 0 : 1;
        }
        else
        {
            tempRead[0] = digitalRead(pins[0]);
        }
    }
    else
    {
        tempRead[0] = digitalRead(pins[0]);
    }
#else
    tempRead[0] = digitalRead(pins[0]);
#endif
}

void digitalInOff()
{
}

void digitalInDebugLoad(VolatileMemory *volatileMemory)
{
    volatileMemory->descArgsBuffer[0] = DIGITAL_IN;
    volatileMemory->descArgsBuffer[1] = getMapedPin(volatileMemory->declarationBuffer[0]);
    component_t *component = (component_t *)malloc(sizeof(component_t));
    digitalInCreate(volatileMemory->descArgsBuffer, component);
    volatileMemory->components[DIGITAL_IN].add(component);
}

void digitalInEepromLoad(VolatileMemory *volatileMemory)
{
    uint8_t currentByte;
    while (!loadedDigitalIns)
    {
        currentByte = nextByte();
        if (currentByte == endDigitals)
        {
            loadedDigitalIns = true;
        }
        else
        {
            volatileMemory->descArgsBuffer[0] = DIGITAL_IN;
            volatileMemory->descArgsBuffer[1] = getMapedPin(currentByte);
            component_t *component = (component_t *)malloc(sizeof(component_t));
            digitalInCreate(volatileMemory->descArgsBuffer, component);
            volatileMemory->components[DIGITAL_IN].add(component);
        }
    }
    ultrasonicEepromLoad(volatileMemory);
}

int32_t digitalInEepromRead(VolatileMemory *volatileMemory)
{
    return getSensVal(DIGITAL_IN, volatileMemory->components[DIGITAL_IN].get(nextByte()));
}
