#include "virtual_machine/virtual_machine.h"
#include "digital_out_component.h"
#include "extern_libraries/linked_list/linked_list.h"

#include <Arduino.h>

#if defined(ARDUINO_ARCH_STM32)
#include "extern_libraries/soft_pwm_stm32/softPwmStm32.h"
#elif defined(ARDUINO_ARCH_ESP32)
#else
#include "extern_libraries/soft_pwm/soft_pwm.h"
#endif

extern bool loadedDigitalOuts;
extern uint8_t currentKit;

void digitalOutCreate(uint16_t *args, component_t *component)
{
    component->pins[0] = args[1];
#if defined(ARDUINO_ARCH_STM32)
    softPwmSTM32Attach(args[1], 0);

#elif defined(ARDUINO_ARCH_ESP32)
    if (getCurrentChannel() < 16)
    {
        if (currentKit == LK32_KIT && args[1] == 25)
        {
            pinMode(args[1], OUTPUT);
        }
        else
        {
            uint8_t channel = getCurrentChannel();
            ledcAttachChannel(args[1], 50, 16, channel);
            component->ledcChannel[0] = channel;
            nextCurrentChannel();
        }
    }
    else
    {
        pinMode(args[1], OUTPUT);
    }
#else
    pinMode(args[1], OUTPUT);
    SoftPWMSet(args[1], 0);

#endif
}

void digitalOutExec(uint32_t *execArgs, uint8_t *pins, uint8_t *values, int8_t *ledcChannel)
{
#if defined(ARDUINO_ARCH_STM32)
    softPwmSTM32Set(pins[0], (execArgs[0] < 0) ? 0 : (execArgs[0] > 100) ? 100
                                                                         : execArgs[0]);
#elif defined(ARDUINO_ARCH_ESP32)

    values[0] = (execArgs[0] < 0) ? 0 : (execArgs[0] > 100) ? 100
                                                            : execArgs[0];

    if (currentKit == LK32_KIT && pins[0] == 25)
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
    else
    {
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
    }

#else
    SoftPWMSetPercent(pins[0], (execArgs[0] < 0) ? 0 : (execArgs[0] > 100) ? 100
                                                                           : execArgs[0]);
#endif
}

void digitalOutOff(uint8_t *pins, int8_t *ledcChannel)
{
#if defined(ARDUINO_ARCH_STM32)
    softPwmSTM32Set(pins[0], 0);
    softPwmSTM32Dettach(pins[0]);
#elif defined(ARDUINO_ARCH_ESP32)

    ledcWrite(ledcChannel[0], 0);
    ledcDetach(pins[0]);

#else
    SoftPWMSet(pins[0], 0);
    SoftPWMEnd(pins[0]);
#endif
}

void digitalOutDebugLoad(VolatileMemory *volatileMemory)
{
    volatileMemory->descArgsBuffer[0] = DIGITAL_OUT;
    volatileMemory->descArgsBuffer[1] = getMapedPin(volatileMemory->declarationBuffer[0]);

    component_t *component = (component_t *)malloc(sizeof(component_t));
    digitalOutCreate(volatileMemory->descArgsBuffer, component);
    volatileMemory->components[DIGITAL_OUT].add(component);
}

void digitalOutEepromLoad(VolatileMemory *volatileMemory)
{
#ifndef __AVR_ATmega168__
    uint8_t currentByte;
    while (!loadedDigitalOuts)
    {
        currentByte = nextByte();
        if (currentByte == endLeds)
        {
            loadedDigitalOuts = true;
        }
        else
        {

            volatileMemory->descArgsBuffer[0] = DIGITAL_OUT;
            volatileMemory->descArgsBuffer[1] = getMapedPin(currentByte);

            component_t *component = (component_t *)malloc(sizeof(component_t));
            digitalOutCreate(volatileMemory->descArgsBuffer, component);
            volatileMemory->components[DIGITAL_OUT].add(component);
        }
    }
    frequencyEepromLoad(volatileMemory);
#endif
}

void digitalOutEepromRun(uint8_t id, VolatileMemory *volatileMemory)
{
    int32_t intensity = getInputValue(nextByte());
    intensity = (intensity < 0) ? 0 : (intensity > 100) ? 100
                                                        : intensity;
    volatileMemory->execBuffer[0] = intensity;
    execAct(volatileMemory->execBuffer, DIGITAL_OUT, volatileMemory->components[DIGITAL_OUT].get(id));
}