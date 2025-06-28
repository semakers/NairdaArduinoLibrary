#include "virtual_machine/virtual_machine.h"
#include "analogic_component.h"
#include "components/inputs/digital_in/digital_in_component.h"
#include "extern_libraries/linked_list/linked_list.h"

#include <Arduino.h>

extern bool loadedAnalogics;
extern int temp;
extern int hum;

extern uint8_t currentKit;

void analogicCreate(uint16_t *args, component_t *component)
{

    component->pins[0] = args[1];
}

void analogicSense(uint8_t *pins, uint8_t *tempRead)
{
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
