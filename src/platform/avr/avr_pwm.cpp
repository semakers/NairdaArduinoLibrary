#if !defined(ARDUINO_ARCH_ESP32)

#include <Arduino.h>
#include "virtual_machine/virtual_machine.h"
#include "extern_libraries/soft_pwm/soft_pwm.h"
#include "nairda.h"

extern uint8_t currentKit;

// ── Motor ──────────────────────────────────────────────────────────

void motorCreate(uint16_t *args, component_t *component)
{
    component->pins[0] = args[1];
    component->pins[1] = args[2];
    component->pins[2] = args[3];

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
}

void motorExec(uint32_t *execArgs, uint8_t *pins, uint8_t *values)
{
    values[0] = (execArgs[0] < 0) ? 0 : (execArgs[0] > 100) ? 100
                                                              : execArgs[0];
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
}

void motorOff(uint8_t *pins)
{
    if (pins[2] == 0)
    {
        SoftPWMSet(pins[0], 0);
        SoftPWMEnd(pins[0]);
        SoftPWMSet(pins[1], 0);
        SoftPWMEnd(pins[1]);
    }
    else
    {
        digitalWrite(pins[0], LOW);
        digitalWrite(pins[1], LOW);
        SoftPWMSet(pins[2], 0);
        SoftPWMEnd(pins[2]);
    }
}

// ── DigitalOut ─────────────────────────────────────────────────────

void digitalOutCreate(uint16_t *args, component_t *component)
{
    component->pins[0] = args[1];
    pinMode(args[1], OUTPUT);
    SoftPWMSet(args[1], 0);
}

void digitalOutExec(uint32_t *execArgs, uint8_t *pins, uint8_t *values)
{
    SoftPWMSetPercent(pins[0], (execArgs[0] < 0) ? 0 : (execArgs[0] > 100) ? 100
                                                                           : execArgs[0]);
}

void digitalOutOff(uint8_t *pins)
{
    SoftPWMSet(pins[0], 0);
    SoftPWMEnd(pins[0]);
}

#endif
