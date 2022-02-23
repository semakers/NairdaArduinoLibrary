#include <Arduino.h>
#include "load_from_eeprom.h"
#include "frequency_component.h"
#include "components/component.h"
#include "components/outputs/neo_pixel/neo_pixel_component.h"
#include "extern_libraries/linked_list/linked_list.h"
#include "extern_libraries/free_tone/timer_free_tone.h"
#include "value_conversion/value_conversion.h"

extern LinkedList<component_t *> listFrequencies;
extern bool loadedFrequencies;
extern uint16_t descArgsBuffer[5];
extern uint32_t execBuffer[6];

void frequencyCreate(uint16_t *args,uint8_t *pins)
{
    pins[0] = args[1];
}

void frequencyExec(uint32_t *execArgs, uint8_t *pins)
{
    TimerFreeTone(pins[0], (execArgs[0] * 100) + execArgs[1], (execArgs[2] * 10000) + (execArgs[3] * 100) + execArgs[4], execArgs[5]);
}

void frequencyOff()
{
}

void frequencyEepromLoad()
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
            descArgsBuffer[0] = FREQUENCY;
            descArgsBuffer[1] = getMapedPin(currentByte);
            component_t *tempFrequency = newComponent(descArgsBuffer);
            listFrequencies.add(tempFrequency);
        }
    }
    neoPixelEepromLoad();
}

void frequencyEepromRun(uint8_t id)
{
    execBuffer[0] = getInputValue(nextByte());
    execBuffer[1] = getInputValue(nextByte());
    execBuffer[2] = getInputValue(nextByte());
    uint32_t frequencyBuffer[6];

    frequencyBuffer[0] = (uint32_t)secondValue(execBuffer[0]);
    frequencyBuffer[1] = (uint32_t)thirdValue(execBuffer[0]);
    frequencyBuffer[2] = (uint32_t)firstValue(execBuffer[2]);
    frequencyBuffer[3] = (uint32_t)secondValue(execBuffer[2]);
    frequencyBuffer[4] = (uint32_t)thirdValue(execBuffer[2]);
    frequencyBuffer[5] = execBuffer[1];

    execAct(frequencyBuffer, FREQUENCY,listFrequencies.get(id));
}