#include "ultrasonic_component.h"
#include "load_from_eeprom.h"
#include "components/component.h"
#include "extern_libraries/linked_list/linked_list.h"
#include "blocks_instructions/variables/variables_instructions.h"

#include <Arduino.h>

#include "volatile_memory/volatile_memory.h"

#if !defined(ARDUINO_ARCH_STM32) && !defined(ARDUINO_ARCH_ESP32)
#include "extern_libraries/new_ping/new_ping.h"
#endif

extern LinkedList<component_t *> listUltrasonics;
extern bool loadedUltrasonics;
extern uint16_t descArgsBuffer[5];
extern uint32_t execBuffer[6];

#if !defined(ARDUINO_ARCH_STM32) && !defined(ARDUINO_ARCH_ESP32)

void ultrasonicCreate(uint16_t *args, uint8_t *pins, NewPing *sonar)
{
    sonar = new NewPing(args[1], args[2], 100);
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

void ultrasonicCreate(uint16_t *args, uint8_t *pins)
{
    pins[0] = args[1];
    pins[1] = args[2];
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
    component_t *tempUltrasonic = newComponent(volatileMemory->descArgsBuffer);
    volatileMemory->components[ULTRASONIC].add(tempUltrasonic);
}

void ultrasonicEepromLoad()
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
            descArgsBuffer[0] = ULTRASONIC;
            descArgsBuffer[1] = getMapedPin(ultraBytes[0]);
            descArgsBuffer[2] = getMapedPin(ultraBytes[1]);
            component_t *tempUltrasonic = newComponent(descArgsBuffer);
            listUltrasonics.add(tempUltrasonic);
        }
    }
    variableEepromLoad();
}

int32_t ultrasonicEepromRead()
{
    return getSensVal(ULTRASONIC, listUltrasonics.get(nextByte()));
}