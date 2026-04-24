

#include "virtual_machine/virtual_machine.h"
#include "servo_component.h"
#include "extern_libraries/linked_list/linked_list.h"

#include <Arduino.h>

void servoCreate(uint16_t *args, component_t *component);

void setupServo(component_t* component, int pin, int minPulse, int maxPulse, int initialAngle) {
  uint16_t descArgsBuffer[5];
  descArgsBuffer[0] = SERVO;
  descArgsBuffer[1] = pin;
  descArgsBuffer[2] = minPulse;
  descArgsBuffer[3] = maxPulse;
  descArgsBuffer[4] = initialAngle;
  servoCreate(descArgsBuffer, component);
}

void runServo(component_t *component, int angle) {
  uint32_t execArgs[1];
  execArgs[0] = angle;
  servoExec(execArgs, component->servo);
  nairdaLoop();
}

void servoCreate(uint16_t *args, component_t *component)
{
    component->servo = new Servo();
    component->pins[0] = args[1];
    component->servo->attach(args[1], args[2], args[3]);
    component->servo->write(args[4]);
}

void servoExec(uint32_t *execArgs, Servo *servo)
{
    servo->write((execArgs[0] < 0) ? 0 : (execArgs[0] > 180) ? 180
                                                             : execArgs[0]);
}

void servoOff(Servo *servo)
{
    servo->detach();
}

void servoDebugLoad(VolatileMemory *volatileMemory)
{
    volatileMemory->descArgsBuffer[0] = SERVO;
    volatileMemory->descArgsBuffer[1] = getMapedPin(volatileMemory->declarationBuffer[0]);
    volatileMemory->descArgsBuffer[2] = (volatileMemory->declarationBuffer[1] * 100) + volatileMemory->declarationBuffer[2];
    volatileMemory->descArgsBuffer[3] = (volatileMemory->declarationBuffer[3] * 100) + volatileMemory->declarationBuffer[4];
    volatileMemory->descArgsBuffer[4] = (volatileMemory->declarationBuffer[5] * 100) + volatileMemory->declarationBuffer[6];
     component_t *component = (component_t *)malloc(sizeof(component_t));
    servoCreate(volatileMemory->descArgsBuffer, component);
    volatileMemory->components[SERVO].add(component);
}
