#include "virtual_machine/virtual_machine.h"
#include "motor_component.h"
#include "extern_libraries/linked_list/linked_list.h"

#include <Arduino.h>

extern bool loadedMotors;

void motorCreate(uint16_t *args, component_t *component)
{

    component->pins[0] = args[1];
    component->pins[1] = args[2];
    component->pins[2] = args[3];
}
void motorExec(uint32_t *execArgs, uint8_t *pins, uint8_t *values, int8_t *ledcChannel)
{
}
void motorOff(uint8_t *pins, int8_t *ledcChannel)
{
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

void motorEepromLoad(VolatileMemory *volatileMemory)
{
#ifndef __AVR_ATmega168__
    uint8_t currentByte;
    while (!loadedMotors)
    {
        currentByte = nextByte();
        if (currentByte == endDC)
        {
            loadedMotors = true;
        }
        else
        {
            uint8_t dcBytes[3];
            dcBytes[0] = currentByte;
            for (uint8_t i = 1; i < 3; i++)
            {
                dcBytes[i] = nextByte();
            }

            volatileMemory->descArgsBuffer[0] = MOTOR;
            volatileMemory->descArgsBuffer[1] = getMapedPin(dcBytes[0]);
            volatileMemory->descArgsBuffer[2] = getMapedPin(dcBytes[1]);
            volatileMemory->descArgsBuffer[3] = getMapedPin(dcBytes[2]);

            component_t *component = (component_t *)malloc(sizeof(component_t));
            motorCreate(volatileMemory->descArgsBuffer, component);

            volatileMemory->components[MOTOR].add(component);
        }
    }
    digitalOutEepromLoad(volatileMemory);
#endif
}

void motorEepromRun(uint8_t id, VolatileMemory *volatileMemory)
{
    int32_t vel = getInputValue(nextByte());
    vel = (vel < 0) ? 0 : (vel > 100) ? 100
                                      : vel;

    volatileMemory->execBuffer[0] = vel;
    volatileMemory->execBuffer[1] = nextByte();

    execAct(volatileMemory->execBuffer, MOTOR, volatileMemory->components[MOTOR].get(id));
}