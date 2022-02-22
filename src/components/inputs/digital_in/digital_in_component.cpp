#include "digital_in_component.h"
#include "load_from_eeprom.h"
#include "components/inputs/ultrasonic/ultrasonic_component.h"

#include <Arduino.h>

extern LinkedList<component *> listDigitalIns;
extern bool loadedDigitalIns;
extern uint16_t descArgsBuffer[5];
extern uint32_t execBuffer[6];

void digitalInCreate(uint16_t *args, uint8_t *pins)
{
    pins[0] = args[1];
    pinMode(pins[0], INPUT);
}

void digitalInSense(uint8_t *pins, uint8_t *tempRead)
{
     tempRead[0] = digitalRead(pins[0]);
}

void digitalInOff()
{
}

void digitalInEepromLoad()
{
    uint8_t currentByte;
    while (!loadedDigitalIns)
    {
        currentByte = nextByte();
        if (currentByte == endDigitals)
        {
            loadedDigitalIns = true;
        }
        else
        {
            descArgsBuffer[0] = DIGITAL_IN;
            descArgsBuffer[1] = getMapedPin(currentByte);
            component *tempDigital = new component(descArgsBuffer);
            listDigitalIns.add(tempDigital);
        }
    }
    ultrasonicEepromLoad();
}

int32_t digitalInEepromRead()
{
    return listDigitalIns.get(nextByte())->getSensVal(DIGITAL_IN);
}
