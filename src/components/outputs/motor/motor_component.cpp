#include "virtual_machine/virtual_machine.h"
#include "motor_component.h"
#include "extern_libraries/linked_list/linked_list.h"

#include <Arduino.h>

#if !defined(ARDUINO_ARCH_ESP32)
#include "extern_libraries/soft_pwm/soft_pwm.h"
#endif

void motorCreate(uint16_t *args, component_t *component);

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

void motorCreate(uint16_t *args, component_t *component)
{

    component->pins[0] = args[1];
    component->pins[1] = args[2];
    component->pins[2] = args[3];
#if defined(ARDUINO_ARCH_ESP32)

    if (args[3] == 0)
    {
        ledcAttach(args[1], 50, 16);
        ledcAttach(args[2], 50, 16);
    }
    else
    {
        pinMode(args[1], OUTPUT);
        pinMode(args[2], OUTPUT);
        ledcAttach(args[3], 50, 16);
    }

#else
    if (args[3] == 0)
    {
        SoftPWMSet(args[1], 0);
        SoftPWMSet(args[2], 0);
    }
    else
    {
        pinMode(args[1], OUTPUT);
        pinMode(args[2], OUTPUT);
        SoftPWMSet(args[3], 0);
    }

#endif
}
void motorExec(uint32_t *execArgs, uint8_t *pins, uint8_t *values)
{
    values[0] = (execArgs[0] < 0) ? 0 : (execArgs[0] > 100) ? 100
                                                            : execArgs[0];
#if defined(ARDUINO_ARCH_ESP32)
    if (pins[2] == 0)
    {
        switch (execArgs[1])
        {
        case 0:
            ledcWrite(pins[0], map(values[0], 0, 100, 0, 65535));
            ledcWrite(pins[1], 0);
            break;
        case 1:
            ledcWrite(pins[0], 0);
            ledcWrite(pins[1], 0);
            break;
        case 2:
            ledcWrite(pins[0], 0);
            ledcWrite(pins[1], map(values[0], 0, 100, 0, 65535));
            break;
        }
    }
    else
    {
        switch (execArgs[1])
        {
        case 0:
            digitalWrite(pins[0], HIGH);
            digitalWrite(pins[1], LOW);
            ledcWrite(pins[2], map(values[0], 0, 100, 0, 65535));
            break;
        case 1:
            digitalWrite(pins[0], LOW);
            digitalWrite(pins[1], LOW);
            ledcWrite(pins[2], 0);
            break;
        case 2:
            digitalWrite(pins[0], LOW);
            digitalWrite(pins[1], HIGH);
            ledcWrite(pins[2], map(values[0], 0, 100, 0, 65535));
            break;
        }
    }

#else
    if (pins[2] == 0)
    {
        switch (execArgs[1])
        {
        case 0:
            SoftPWMSetPercent(pins[0], values[0]);
            SoftPWMSetPercent(pins[1], 0);

            break;
        case 1:
            SoftPWMSetPercent(pins[0], 0);
            SoftPWMSetPercent(pins[1], 0);
            break;
        case 2:
            SoftPWMSetPercent(pins[0], 0);
            SoftPWMSetPercent(pins[1], values[0]);
            break;
        }
    }
    else
    {
        switch (execArgs[1])
        {
        case 0:
            digitalWrite(pins[0], HIGH);
            digitalWrite(pins[1], LOW);
            SoftPWMSetPercent(pins[2], values[0]);

            break;
        case 1:
            digitalWrite(pins[0], LOW);
            digitalWrite(pins[1], LOW);
            SoftPWMSetPercent(pins[2], 0);
            break;
        case 2:
            digitalWrite(pins[0], LOW);
            digitalWrite(pins[1], HIGH);
            SoftPWMSetPercent(pins[2], values[0]);
            break;
        }
    }

#endif
}
void motorOff(uint8_t *pins)
{
    if (pins[2] == 0)
    {
#if defined(ARDUINO_ARCH_ESP32)
        ledcWrite(pins[0], 0);
        ledcDetach(pins[0]);
        ledcWrite(pins[1], 0);
        ledcDetach(pins[1]);
#else
        SoftPWMSet(pins[0], 0);
        SoftPWMEnd(pins[0]);
        SoftPWMSet(pins[1], 0);
        SoftPWMEnd(pins[1]);
#endif
    }
    else
    {
        digitalWrite(pins[0], LOW);
        digitalWrite(pins[1], LOW);
#if defined(ARDUINO_ARCH_ESP32)
        ledcWrite(pins[2], 0);
        ledcDetach(pins[2]);
#else
        SoftPWMSet(pins[2], 0);
        SoftPWMEnd(pins[2]);
#endif
    }
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
