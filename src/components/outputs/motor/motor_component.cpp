#include "virtual_machine/virtual_machine.h"
#include "motor_component.h"
#include "extern_libraries/linked_list/linked_list.h"

#include <Arduino.h>

#if !defined(ARDUINO_ARCH_STM32) && !defined(ARDUINO_ARCH_ESP32)
#include "extern_libraries/soft_pwm/soft_pwm.h"
#endif

extern bool loadedMotors;

void motorCreate(uint16_t *args, component_t *component)
{

    component->pins[0] = args[1];
    component->pins[1] = args[2];
    component->pins[2] = args[3];
#if defined(ARDUINO_ARCH_STM32)
    pinMode(args[1], OUTPUT);
    pinMode(args[2], OUTPUT);
    softPwmSTM32Attach(args[3], 0);
#else
#if defined(ARDUINO_ARCH_ESP32)
    pinMode(args[1], OUTPUT);
    pinMode(args[2], OUTPUT);
    if (getCurrentChannel() < 16)
    {
        ledcSetup(getCurrentChannel(), 50, 16);
        ledcAttachPin(args[3], getCurrentChannel());
        component->ledcChannel[0] = getCurrentChannel();
        nextCurrentChannel();
    }
    else
    {
        pinMode(args[3], OUTPUT);
    }

#else
    pinMode(args[1], OUTPUT);
    pinMode(args[2], OUTPUT);
    SoftPWMSet(args[3], 0);

#endif
#endif
}
void motorExec(uint32_t *execArgs, uint8_t *pins, uint8_t *values, int8_t *ledcChannel)
{
    values[0] = (execArgs[0] < 0) ? 0 : (execArgs[0] > 100) ? 100
                                                            : execArgs[0];
#if defined(ARDUINO_ARCH_STM32)
    switch (execArgs[1])
    {
    case 0:
        digitalWrite(pins[0], HIGH);
        digitalWrite(pins[1], LOW);
        softPwmSTM32Set(pins[2], values[0]);

        break;
    case 1:
        digitalWrite(pins[0], LOW);
        digitalWrite(pins[1], LOW);
        softPwmSTM32Set(pins[2], 0);
        break;
    case 2:
        digitalWrite(pins[0], LOW);
        digitalWrite(pins[1], HIGH);
        softPwmSTM32Set(pins[2], values[0]);
        break;
    }
#else
#if defined(ARDUINO_ARCH_ESP32)
    switch (execArgs[1])
    {
    case 0:
        digitalWrite(pins[0], HIGH);
        digitalWrite(pins[1], LOW);
        if (ledcChannel[0] != -1)
        {
            ledcWrite(ledcChannel[0], map(values[0], 0, 100, 0, 65535));
        }
        else
        {
            if (values[0] >= 0 && values[0] <= 50)
            {
                digitalWrite(pins[2], LOW);
            }
            else if (values[0] > 50 && values[0] <= 100)
            {
                digitalWrite(pins[2], HIGH);
            }
        }

        break;
    case 1:
        digitalWrite(pins[0], LOW);
        digitalWrite(pins[1], LOW);
        ledcWrite(ledcChannel[0], 0);
        break;
    case 2:
        digitalWrite(pins[0], LOW);
        digitalWrite(pins[1], HIGH);
        if (ledcChannel[0] != -1)
        {
            ledcWrite(ledcChannel[0], map(values[0], 0, 100, 0, 65535));
        }
        else
        {
            if (values[0] >= 0 && values[0] <= 50)
            {
                digitalWrite(pins[2], LOW);
            }
            else if (values[0] > 50 && values[0] <= 100)
            {
                digitalWrite(pins[2], HIGH);
            }
        }
        break;
    }

#else
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
#endif
#endif
}
void motorOff(uint8_t *pins, int8_t *ledcChannel)
{
    digitalWrite(pins[0], LOW);
    digitalWrite(pins[1], LOW);
#if defined(ARDUINO_ARCH_STM32)

    softPwmSTM32Set(pins[2], 0);
    softPwmSTM32Dettach(pins[2]);
#elif defined(ARDUINO_ARCH_ESP32)

    ledcWrite(ledcChannel[0], 0);
    ledcDetachPin(pins[2]);
#else
    SoftPWMSet(pins[2], 0);
    SoftPWMEnd(pins[2]);
#endif
}

void motorDebugLoad(VolatileMemory *volatileMemory)
{
    volatileMemory->descArgsBuffer[0] = MOTOR;
    volatileMemory->descArgsBuffer[1] = getMapedPin(volatileMemory->declarationBuffer[0]);
    volatileMemory->descArgsBuffer[2] = getMapedPin(volatileMemory->declarationBuffer[1]);
    volatileMemory->descArgsBuffer[3] = getMapedPin(volatileMemory->declarationBuffer[2]);

    component_t component;
    motorCreate(volatileMemory->descArgsBuffer, &component);
    volatileMemory->components[MOTOR].add(&component);
}

void motorEepromLoad(VolatileMemory *volatileMemory)
{
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


                component_t component;
            motorCreate(volatileMemory->descArgsBuffer, &component);

            volatileMemory->components[MOTOR].add(&component);
        }
    }
    digitalOutEepromLoad(volatileMemory);
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