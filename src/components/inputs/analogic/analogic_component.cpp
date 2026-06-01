#include "virtual_machine/virtual_machine.h"
#include "analogic_component.h"
#include "extern_libraries/linked_list/linked_list.h"

#include <Arduino.h>

void setupAnalogic(component_t* component, int pin) {
  uint16_t descArgsBuffer[2];
  descArgsBuffer[0] = ANALOGIC;
  descArgsBuffer[1] = pin;
  analogicCreate(descArgsBuffer, component);
}

uint8_t readAnalogic(component_t *component) {
  uint8_t pins[1];
  pins[0] = component->pins[0];
  uint8_t values[1];
  analogicSense(pins, values);
  nairdaLoop();
  // Wire protocol: real (0..100) + 1, range 1..101. User-app macro undoes it.
  uint8_t v = values[0] > 100 ? 100 : values[0];
  return v + 1;
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
