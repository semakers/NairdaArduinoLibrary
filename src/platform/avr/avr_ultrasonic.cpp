#if !defined(ARDUINO_ARCH_ESP32)

#include <Arduino.h>
#include "virtual_machine/virtual_machine.h"
#include "extern_libraries/new_ping/new_ping.h"

void ultrasonicCreate(uint16_t *args, component_t *component)
{
    component->sonar = new NewPing(args[1], args[2], 100);
}

void ultrasonicSenseImpl(component_t *component, uint8_t *tempRead)
{
    NewPing *sonar = (NewPing *)component->sonar;
    tempRead[0] = sonar->ping_cm();
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
    NewPing *sonar = (NewPing *)component->sonar;
    free(sonar);
}

#endif
