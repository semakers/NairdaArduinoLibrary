#include "virtual_machine/virtual_machine.h"
#include "ultrasonic_component.h"
#include "extern_libraries/linked_list/linked_list.h"

#include <Arduino.h>

extern bool loadedUltrasonics;

void ultrasonicCreate(uint16_t *args, component_t *component)
{
}

void ultrasonicSense(uint8_t *pins, uint8_t *tempRead)
{
}

void ultrasonicOff()
{
}

void ultrasonicDebugLoad(VolatileMemory *volatileMemory)
{
    volatileMemory->descArgsBuffer[0] = ULTRASONIC;
    volatileMemory->descArgsBuffer[1] = getMapedPin(volatileMemory->declarationBuffer[0]);
    volatileMemory->descArgsBuffer[2] = getMapedPin(volatileMemory->declarationBuffer[1]);
    component_t *component = (component_t *)malloc(sizeof(component_t));
    ultrasonicCreate(volatileMemory->descArgsBuffer, component);
    volatileMemory->components[ULTRASONIC].add(component);
}

void ultrasonicEepromLoad(VolatileMemory *volatileMemory)
{
#ifndef __AVR_ATmega168__
    uint8_t currentByte;
    while (!loadedUltrasonics)
    {
        currentByte = nextByte();
        if (currentByte == endUltrasonics)
        {
            loadedUltrasonics = true;
        }
        else
        {
            uint8_t ultraBytes[2];
            ultraBytes[0] = currentByte;
            for (uint8_t i = 1; i < 2; i++)
            {
                ultraBytes[i] = nextByte();
            }
            volatileMemory->descArgsBuffer[0] = ULTRASONIC;
            volatileMemory->descArgsBuffer[1] = getMapedPin(ultraBytes[0]);
            volatileMemory->descArgsBuffer[2] = getMapedPin(ultraBytes[1]);
            component_t *component = (component_t *)malloc(sizeof(component_t));
            ultrasonicCreate(volatileMemory->descArgsBuffer, component);
            volatileMemory->components[ULTRASONIC].add(component);
        }
    }

    variableEepromLoad();
#endif
}

int32_t ultrasonicEepromRead(VolatileMemory *volatileMemory)
{
    return getSensVal(ULTRASONIC, volatileMemory->components[ULTRASONIC].get(nextByte()));
}