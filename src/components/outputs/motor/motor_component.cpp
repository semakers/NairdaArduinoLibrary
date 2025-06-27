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
    if (args[3] == 0)
    {
        softPwmSTM32Attach(args[1], 0);
        softPwmSTM32Attach(args[2], 0);
    }
    else
    {
        pinMode(args[1], OUTPUT);
        pinMode(args[2], OUTPUT);
        softPwmSTM32Attach(args[3], 0);
    }

#else
#if defined(ARDUINO_ARCH_ESP32)

    if (args[3] == 0)
    {
        if (getCurrentChannel() < 16)
        {
            uint8_t channel = getCurrentChannel();
            ledcAttachChannel(args[1], 50, 16, channel);
            component->ledcChannel[0] = channel;
            nextCurrentChannel();
        }
        else
        {
            pinMode(args[1], OUTPUT);
        }

        if (getCurrentChannel() < 16)
        {
            uint8_t channel = getCurrentChannel();
            ledcAttachChannel(args[2], 50, 16, channel);
            component->ledcChannel[0] = channel;
            nextCurrentChannel();
        }
        else
        {
            pinMode(args[2], OUTPUT);
        }
    }
    else
    {
        pinMode(args[1], OUTPUT);
        pinMode(args[2], OUTPUT);
        if (getCurrentChannel() < 16)
        {
            uint8_t channel = getCurrentChannel();
            ledcAttachChannel(args[3], 50, 16, channel);
            component->ledcChannel[0] = channel;
            nextCurrentChannel();
        }
        else
        {
            pinMode(args[3], OUTPUT);
        }
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
#endif
}
void motorExec(uint32_t *execArgs, uint8_t *pins, uint8_t *values, int8_t *ledcChannel)
{
    values[0] = (execArgs[0] < 0) ? 0 : (execArgs[0] > 100) ? 100
                                                            : execArgs[0];
#if defined(ARDUINO_ARCH_STM32)
    if (pins[2] == 0)
    {
        switch (execArgs[1])
        {
        case 0:
            softPwmSTM32Set(pins[0], values[0]);
            softPwmSTM32Set(pins[1], 0);

            break;
        case 1:
            softPwmSTM32Set(pins[0], 0);
            softPwmSTM32Set(pins[1], 0);
            break;
        case 2:
            softPwmSTM32Set(pins[0], 0);
            softPwmSTM32Set(pins[1], values[0]);
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
    }

#else
#if defined(ARDUINO_ARCH_ESP32)
    if (pins[2] == 0)
    {
        switch (execArgs[1])
        {
        case 0:
            if (ledcChannel[0] != -1)
            {
                ledcWrite(pins[0], map(values[0], 0, 100, 0, 65535));
            }
            else
            {
                if (values[0] >= 0 && values[0] <= 50)
                {
                    digitalWrite(pins[0], LOW);
                }
                else if (values[0] > 50 && values[0] <= 100)
                {
                    digitalWrite(pins[0], HIGH);
                }
            }
            if (ledcChannel[1] != -1)
            {
                ledcWrite(pins[1], 0);
            }
            else
            {
                digitalWrite(pins[1], LOW);
            }

            break;
        case 1:
            digitalWrite(pins[0], LOW);
            digitalWrite(pins[1], LOW);
            ledcWrite(pins[0], 0);
            break;
        case 2:
            if (ledcChannel[1] != -1)
            {
                ledcWrite(pins[1], map(values[0], 0, 100, 0, 65535));
            }
            else
            {
                if (values[0] >= 0 && values[0] <= 50)
                {
                    digitalWrite(pins[1], LOW);
                }
                else if (values[0] > 50 && values[0] <= 100)
                {
                    digitalWrite(pins[1], HIGH);
                }
            }
            if (ledcChannel[0] != -1)
            {
                ledcWrite(pins[0], 0);
            }
            else
            {
                digitalWrite(pins[0], LOW);
            }
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
            if (ledcChannel[0] != -1)
            {
                ledcWrite(pins[0], map(values[0], 0, 100, 0, 65535));
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
            ledcWrite(pins[0], 0);
            break;
        case 2:
            digitalWrite(pins[0], LOW);
            digitalWrite(pins[1], HIGH);
            if (ledcChannel[0] != -1)
            {
                ledcWrite(pins[0], map(values[0], 0, 100, 0, 65535));
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
#endif
}
void motorOff(uint8_t *pins, int8_t *ledcChannel)
{
    if (pins[2] == 0)
    {
#if defined(ARDUINO_ARCH_STM32)

        softPwmSTM32Set(pins[0], 0);
        softPwmSTM32Set(pins[1], 0);
        softPwmSTM32Dettach(pins[0]);
        softPwmSTM32Dettach(pins[1]);
#elif defined(ARDUINO_ARCH_ESP32)

        ledcWrite(ledcChannel[0], 0);
        ledcDetach(pins[0]);
        ledcWrite(ledcChannel[1], 0);
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
#if defined(ARDUINO_ARCH_STM32)

        softPwmSTM32Set(pins[2], 0);
        softPwmSTM32Dettach(pins[2]);
#elif defined(ARDUINO_ARCH_ESP32)

        ledcWrite(ledcChannel[0], 0);
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