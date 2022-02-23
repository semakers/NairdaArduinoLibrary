#include "digital_out_component.h"
#include "load_from_eeprom.h"
#include "components/component.h"
#include "extern_libraries/linked_list/linked_list.h"
#include "components/outputs/frequency/frequency_component.h"

#include <Arduino.h>

#if defined(ARDUINO_ARCH_STM32)
#include "extern_libraries/soft_pwm_stm32/softPwmStm32.h"
#elif defined(ARDUINO_ARCH_ESP32)
#else
#include "extern_libraries/soft_pwm/soft_pwm.h"
#endif

extern LinkedList<component_t *> listDigitalOuts;
extern bool loadedDigitalOuts;
extern uint16_t descArgsBuffer[5];
extern uint32_t execBuffer[6];

void digitalOutCreate(uint16_t *args, uint8_t *pins, int8_t *ledcChannel)
{
    pins[0] = args[1];
#if defined(ARDUINO_ARCH_STM32)
    softPwmSTM32Attach(args[1], 0);

#elif defined(ARDUINO_ARCH_ESP32)
    if (getCurrentChannel() < 16)
    {
        ledcSetup(getCurrentChannel(), 50, 16);
        ledcAttachPin(args[1], getCurrentChannel());
        ledcChannel[0] = getCurrentChannel();
        nextCurrentChannel();
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
    if (ledcChannel[0] != -1)
    {
        ledcWrite(ledcChannel[0], map(values[0], 0, 100, 0, 65535));
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

#else
    SoftPWMSetPercent(pins[0], (execArgs[0] < 0) ? 0 : (execArgs[0] > 100) ? 100
                                                                           : execArgs[0]);
#endif
}

void digitalOutOff(uint8_t *pins,int8_t *ledcChannel)
{
#if defined(ARDUINO_ARCH_STM32)
    softPwmSTM32Set(pins[0], 0);
    softPwmSTM32Dettach(pins[0]);
#elif defined(ARDUINO_ARCH_ESP32)

    ledcWrite(ledcChannel[0], 0);
    ledcDetachPin(pins[0]);

#else
    SoftPWMSet(pins[0], 0);
    SoftPWMEnd(pins[0]);
#endif
}

void digitalOutEepromLoad()
{
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
            descArgsBuffer[0] = DIGITAL_OUT;
            descArgsBuffer[1] = getMapedPin(currentByte);
            component_t *digitalOut = newComponent(descArgsBuffer);
            listDigitalOuts.add(digitalOut);
        }
    }
    frequencyEepromLoad();
}

void digitalOutEepromRun(uint8_t id)
{
    int32_t intensity = getInputValue(nextByte());
    intensity = (intensity < 0) ? 0 : (intensity > 100) ? 100
                                                        : intensity;
    execBuffer[0] = intensity;
    execAct(execBuffer, DIGITAL_OUT,listDigitalOuts.get(id));
}