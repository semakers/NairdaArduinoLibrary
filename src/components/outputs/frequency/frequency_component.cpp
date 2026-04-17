
#include "virtual_machine/virtual_machine.h"
#include "frequency_component.h"
#include "extern_libraries/linked_list/linked_list.h"
#include "extern_libraries/free_tone/timer_free_tone.h"
#include "value_conversion/value_conversion.h"

#include <Arduino.h>

void frequencyCreate(uint16_t *args, component_t *component);

void setupFrequency(component_t* component, int pin) {
  uint16_t descArgsBuffer[2];
  descArgsBuffer[0] = FREQUENCY;
  descArgsBuffer[1] = pin;
  frequencyCreate(descArgsBuffer, component);
}

void runFrequency(component_t *component, int frequency, int duration, int volume) {
  uint32_t execArgs[6];
  execArgs[0] = frequency / 100;
  execArgs[1] = frequency % 100;
  execArgs[2] = duration / 10000;
  execArgs[3] = (duration % 10000) / 100;
  execArgs[4] = duration % 100;
  execArgs[5] = volume;
  frequencyExec(execArgs, component->pins);
  nairdaLoop();
}

void frequencyCreate(uint16_t *args, component_t *component)
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
    component_t *component = (component_t *)malloc(sizeof(component_t));
    frequencyCreate(volatileMemory->descArgsBuffer, component);
    volatileMemory->components[FREQUENCY].add(component);
}

