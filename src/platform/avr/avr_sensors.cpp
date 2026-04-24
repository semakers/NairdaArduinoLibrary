#if !defined(ARDUINO_ARCH_ESP32)

#include <Arduino.h>
#include "virtual_machine/virtual_machine.h"

void analogicCreate(uint16_t *args, component_t *component)
{
    component->pins[0] = args[1];
}

void analogicSense(uint8_t *pins, uint8_t *tempRead)
{
    tempRead[0] = map(analogRead(pins[0]), 0, 1023, 0, 100);
}

void digitalInSense(uint8_t *pins, uint8_t *tempRead)
{
    tempRead[0] = digitalRead(pins[0]);
}

#endif
