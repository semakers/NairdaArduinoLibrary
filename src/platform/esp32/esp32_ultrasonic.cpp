#if defined(ARDUINO_ARCH_ESP32)

#include <Arduino.h>
#include "virtual_machine/virtual_machine.h"

void ultrasonicCreate(uint16_t *args, component_t *component)
{
    component->pins[0] = args[1];
    component->pins[1] = args[2];
    pinMode(args[1], OUTPUT);
    pinMode(args[2], INPUT);
}

void ultrasonicSenseImpl(component_t *component, uint8_t *tempRead)
{
    uint8_t *pins = component->pins;
    digitalWrite(pins[0], LOW);
    delayMicroseconds(2);
    digitalWrite(pins[0], HIGH);
    delayMicroseconds(10);
    digitalWrite(pins[0], LOW);
    tempRead[0] = pulseIn(pins[1], HIGH, 10000) / 27.6233 / 2;

    static int lastValue;
    static int zeroCounter = 0;

    if (zeroCounter == 3)
    {
        if (tempRead[0] == 0)
            tempRead[0] = 99;
        else
            zeroCounter = 0;
    }
    else
    {
        if (tempRead[0] == 0)
            zeroCounter++;
        lastValue = (tempRead[0] == 0) ? lastValue : tempRead[0];
        tempRead[0] = (tempRead[0] == 0) ? lastValue : tempRead[0];
    }
}

void ultrasonicOffImpl(component_t *component)
{
    // No cleanup needed on ESP32
}

#endif
