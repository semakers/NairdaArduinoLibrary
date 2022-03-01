#include "virtual_machine/virtual_machine.h"
#include "digital_in_component.h"
#include "extern_libraries/linked_list/linked_list.h"

#include <Arduino.h>

extern bool loadedDigitalIns;

void digitalInCreate(uint16_t *args, component_t *component)
{
    component->pins[0] = args[1];
    pinMode(component->pins[0], INPUT);
}

void digitalInSense(uint8_t *pins, uint8_t *tempRead)
{
    tempRead[0] = digitalRead(pins[0]);
}

void digitalInOff()
{
}

void digitalInDebugLoad(VolatileMemory *volatileMemory)
{
    volatileMemory->descArgsBuffer[0] = DIGITAL_IN;
    volatileMemory->descArgsBuffer[1] = getMapedPin(volatileMemory->declarationBuffer[0]);
    component_t component;
    digitalInCreate(volatileMemory->descArgsBuffer, &component);
    volatileMemory->components[DIGITAL_IN].add(&component);
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
            component_t component;
            digitalInCreate(volatileMemory->descArgsBuffer, &component);
            volatileMemory->components[DIGITAL_IN].add(&component);
        }
    }
    ultrasonicEepromLoad(volatileMemory);
}

int32_t digitalInEepromRead(VolatileMemory *volatileMemory)
{
    return getSensVal(DIGITAL_IN, volatileMemory->components[DIGITAL_IN].get(nextByte()));
}
