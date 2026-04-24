#include "virtual_machine/virtual_machine.h"
#include "motor_component.h"
#include "extern_libraries/linked_list/linked_list.h"

#include <Arduino.h>

void setupMotor(component_t* component, int pin1, int pin2, int pinSpeed) {
  uint16_t descArgsBuffer[4];
  descArgsBuffer[0] = MOTOR;
  descArgsBuffer[1] = pin1;
  descArgsBuffer[2] = pin2;
  descArgsBuffer[3] = pinSpeed;
  motorCreate(descArgsBuffer, component);
}

void runMotor(component_t *component, int speed, int direction) {
  uint32_t execArgs[2];
  execArgs[0] = speed;
  execArgs[1] = direction;
  motorExec(execArgs, component->pins, component->values);
  nairdaLoop();
}

void motorDebugLoad(VolatileMemory *volatileMemory)
{
    volatileMemory->descArgsBuffer[0] = MOTOR;
    volatileMemory->descArgsBuffer[1] = getMapedPin(volatileMemory->declarationBuffer[0]);
    volatileMemory->descArgsBuffer[2] = getMapedPin(volatileMemory->declarationBuffer[1]);
    volatileMemory->descArgsBuffer[3] = getMapedPin(volatileMemory->declarationBuffer[2]);

    component_t *component = (component_t *)malloc(sizeof(component_t));
    motorCreate(volatileMemory->descArgsBuffer, component);
    volatileMemory->components[MOTOR].add(component);
}
