#include "virtual_machine/virtual_machine.h"
#include "digital_in_component.h"
#include "extern_libraries/linked_list/linked_list.h"

#include <Arduino.h>

extern VolatileMemory volatileMemory;

void digitalInCreate(uint16_t *args, component_t *component);

void setupDigitalIn(component_t *component, int pin) {
  uint16_t descArgsBuffer[2];
  descArgsBuffer[0] = DIGITAL_IN;
  descArgsBuffer[1] = pin;
  digitalInCreate(descArgsBuffer, component);
}

uint8_t readDigitalIn(component_t *component) {
  uint8_t pins[1];
  pins[0] = component->pins[0];
  uint8_t values[1];
  digitalInSense(pins, values);
  nairdaLoop();
  return values[0];
}

void digitalInCreate(uint16_t *args, component_t *component)
{
    component->pins[0] = args[1];
    pinMode(component->pins[0], INPUT);
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
