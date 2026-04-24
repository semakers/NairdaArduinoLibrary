#include "virtual_machine/virtual_machine.h"
#include "platform/platform_hal.h"

void execAct(uint32_t *execArgs, uint8_t type, component_t *component)
{
    switch (type)
    {
    case SERVO:
        servoExec(execArgs, component->servo);
        break;
    case MOTOR:
        motorExec(execArgs, component->pins, component->values);
        break;
    case DIGITAL_OUT:
        digitalOutExec(execArgs, component->pins, component->values);
        break;
    case FREQUENCY:
        frequencyExec(execArgs, component->pins);
        break;
    case NEOPIXEL:
        neoPixelExec(execArgs, component->neopixel);
        break;
    }
}

uint8_t getSensVal(uint8_t type, component_t *component)
{
    uint8_t tempRead;
    switch (type)
    {
    case DIGITAL_IN:
        digitalInSense(component->pins, &tempRead);
        break;
    case ANALOGIC:
        analogicSense(component->pins, &tempRead);
        break;
    case ULTRASONIC:
        ultrasonicSenseImpl(component, &tempRead);
        break;
    }
    return (tempRead < 0) ? 0 : (tempRead > 100) ? 100
                                                 : tempRead;
}

void sendSensVal(uint8_t type, component_t *component)
{
    uint8_t val = getSensVal(type, component);
    hal_sendByte(val);
}

void off(uint8_t type, component_t *component)
{
    switch (type)
    {
    case NEOPIXEL:
        neoPixelOff(component->neopixel);
        break;
    case SERVO:
        servoOff(component->servo);
        break;
    case ULTRASONIC:
        ultrasonicOffImpl(component);
        break;
    case MOTOR:
        motorOff(component->pins);
        break;
    case DIGITAL_OUT:
        digitalOutOff(component->pins);
        break;
    }
}
