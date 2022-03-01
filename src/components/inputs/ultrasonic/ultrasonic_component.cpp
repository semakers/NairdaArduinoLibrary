#include "virtual_machine/virtual_machine.h"
#include "ultrasonic_component.h"
#include "extern_libraries/linked_list/linked_list.h"

#include <Arduino.h>

#if !defined(ARDUINO_ARCH_STM32) && !defined(ARDUINO_ARCH_ESP32)
#include "extern_libraries/new_ping/new_ping.h"
#endif

extern bool loadedUltrasonics;

#if !defined(ARDUINO_ARCH_STM32) && !defined(ARDUINO_ARCH_ESP32)

void ultrasonicCreate(uint16_t *args, component_t *component)
{
    component->sonar = new NewPing(args[1], args[2], 100);
}

void ultrasonicSense(uint8_t *pins, uint8_t *tempRead, NewPing *sonar)
{

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

void ultrasonicOff(NewPing *sonar)
{
    free(sonar);
}

#else

void ultrasonicCreate(uint16_t *args, component_t *component)
{
    component->pins[0] = args[1];
    component->pins[1] = args[2];
#if defined(ARDUINO_ARCH_STM32)

    pinMode(args[1], OUTPUT);
    pinMode(args[2], INPUT);
#elif defined(ARDUINO_ARCH_ESP32)
    pinMode(args[1], OUTPUT);
    pinMode(args[2], INPUT);
#endif
}

void ultrasonicSense(uint8_t *pins, uint8_t *tempRead)
{
#if defined(ARDUINO_ARCH_STM32)
    digitalWrite(pins[0], LOW);
    delayMicroseconds(2);
    digitalWrite(pins[0], HIGH);
    delayMicroseconds(10);
    digitalWrite(pins[0], LOW);
    tempRead[0] = pulseIn(pins[1], HIGH) / 27.6233 / 2;
#elif defined(ARDUINO_ARCH_ESP32)
    digitalWrite(pins[0], LOW);
    delayMicroseconds(2);
    digitalWrite(pins[0], HIGH);
    delayMicroseconds(10);
    digitalWrite(pins[0], LOW);
    tempRead[0] = pulseIn(pins[1], HIGH) / 27.6233 / 2;
#endif
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

void ultrasonicOff()
{
}

#endif

void ultrasonicDebugLoad(VolatileMemory *volatileMemory)
{
    volatileMemory->descArgsBuffer[0] = ULTRASONIC;
    volatileMemory->descArgsBuffer[1] = getMapedPin(volatileMemory->declarationBuffer[0]);
    volatileMemory->descArgsBuffer[2] = getMapedPin(volatileMemory->declarationBuffer[2]);
    component_t component;
    ultrasonicCreate(volatileMemory->descArgsBuffer, &component);
    volatileMemory->components[ULTRASONIC].add(&component);
}

void ultrasonicEepromLoad(VolatileMemory *volatileMemory)
{
    uint8_t currentByte;
    while (!loadedUltrasonics)
    {
        currentByte = nextByte();
        if (currentByte == endUltrasonics)
        {
            loadedUltrasonics = true;
        }
        else
        {
            uint8_t ultraBytes[2];
            ultraBytes[0] = currentByte;
            for (uint8_t i = 1; i < 2; i++)
            {
                ultraBytes[i] = nextByte();
            }
            volatileMemory->descArgsBuffer[0] = ULTRASONIC;
            volatileMemory->descArgsBuffer[1] = getMapedPin(ultraBytes[0]);
            volatileMemory->descArgsBuffer[2] = getMapedPin(ultraBytes[1]);
            component_t component;
            ultrasonicCreate(volatileMemory->descArgsBuffer, &component);
            volatileMemory->components[ULTRASONIC].add(&component);
        }
    }
    
    variableEepromLoad();
}

int32_t ultrasonicEepromRead(VolatileMemory *volatileMemory)
{
    return getSensVal(ULTRASONIC, volatileMemory->components[ULTRASONIC].get(nextByte()));
}