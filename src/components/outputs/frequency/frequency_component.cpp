
#include "virtual_machine/virtual_machine.h"
#include "frequency_component.h"
#include "extern_libraries/linked_list/linked_list.h"
#include "extern_libraries/free_tone/timer_free_tone.h"
#include "value_conversion/value_conversion.h"

#include <Arduino.h>

extern bool loadedFrequencies;

void frequencyCreate(uint16_t *args, component_t * component)
{
    component->pins[0] = args[1];
}

void frequencyExec(uint32_t *execArgs, uint8_t *pins)
{
    TimerFreeTone(pins[0], (execArgs[0] * 100) + execArgs[1], (execArgs[2] * 10000) + (execArgs[3] * 100) + execArgs[4], execArgs[5]);
}

void frequencyOff()
{
}

void frequencyDebugLoad(VolatileMemory *volatileMemory)
{

    volatileMemory->descArgsBuffer[0] = FREQUENCY;
    volatileMemory->descArgsBuffer[1] = getMapedPin(volatileMemory->declarationBuffer[0]);
    component_t component;
    frequencyCreate(volatileMemory->descArgsBuffer,&component);
    volatileMemory->components[FREQUENCY].add(&component);
}

void frequencyEepromLoad(VolatileMemory *volatileMemory)
{
    uint8_t currentByte;
    while (!loadedFrequencies)
    {
        currentByte = nextByte();
        if (currentByte == endFrequencies)
        {
            loadedFrequencies = true;
        }
        else
        {
            volatileMemory->descArgsBuffer[0] = FREQUENCY;
            volatileMemory->descArgsBuffer[1] = getMapedPin(currentByte);
            component_t component;
            frequencyCreate(volatileMemory->descArgsBuffer,&component);
            volatileMemory->components[FREQUENCY].add(&component);
        }
    }
    
    neoPixelEepromLoad(volatileMemory);
}

void frequencyEepromRun(uint8_t id,VolatileMemory *volatileMemory)
{
    volatileMemory->execBuffer[0] = getInputValue(nextByte());
    volatileMemory->execBuffer[1] = getInputValue(nextByte());
    volatileMemory->execBuffer[2] = getInputValue(nextByte());
    uint32_t frequencyBuffer[6];

    frequencyBuffer[0] = (uint32_t)secondValue(volatileMemory->execBuffer[0]);
    frequencyBuffer[1] = (uint32_t)thirdValue(volatileMemory->execBuffer[0]);
    frequencyBuffer[2] = (uint32_t)firstValue(volatileMemory->execBuffer[2]);
    frequencyBuffer[3] = (uint32_t)secondValue(volatileMemory->execBuffer[2]);
    frequencyBuffer[4] = (uint32_t)thirdValue(volatileMemory->execBuffer[2]);
    frequencyBuffer[5] = volatileMemory->execBuffer[1];

    execAct(frequencyBuffer, FREQUENCY, volatileMemory->components[FREQUENCY].get(id));
}