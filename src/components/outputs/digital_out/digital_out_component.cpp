#include "virtual_machine/virtual_machine.h"
#include "digital_out_component.h"
#include "extern_libraries/linked_list/linked_list.h"

#include <Arduino.h>

void setupDigitalOut(component_t* component, int pin) {
  uint16_t descArgsBuffer[2];
  descArgsBuffer[0] = DIGITAL_OUT;
  descArgsBuffer[1] = pin;
  digitalOutCreate(descArgsBuffer, component);
}

void runDigitalOut(component_t *component, int value) {
  uint32_t execArgs[1];
  execArgs[0] = value;
  uint8_t pins[1];
  pins[0] = component->pins[0];
  uint8_t values[1];
  digitalOutExec(execArgs, pins, values);
  nairdaLoop();
}

void digitalOutDebugLoad(VolatileMemory *volatileMemory)
{
    volatileMemory->descArgsBuffer[0] = DIGITAL_OUT;
    volatileMemory->descArgsBuffer[1] = getMapedPin(volatileMemory->declarationBuffer[0]);

    component_t *component = (component_t *)malloc(sizeof(component_t));
    digitalOutCreate(volatileMemory->descArgsBuffer, component);
    volatileMemory->components[DIGITAL_OUT].add(component);
}
