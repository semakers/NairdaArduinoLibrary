#include "virtual_machine/virtual_machine.h"
#include "ultrasonic_component.h"
#include "extern_libraries/linked_list/linked_list.h"

#include <Arduino.h>

void setupUltrasonic(component_t* component, int triggerPin, int echoPin) {
  uint16_t descArgsBuffer[3];
  descArgsBuffer[0] = ULTRASONIC;
  descArgsBuffer[1] = triggerPin;
  descArgsBuffer[2] = echoPin;
  ultrasonicCreate(descArgsBuffer, component);
}

uint8_t readUltrasonic(component_t *component) {
  uint8_t values[1];
  ultrasonicSenseImpl(component, values);
  nairdaLoop();
  return values[0];
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
