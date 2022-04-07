#include "kits/v1.h"
#include <Arduino.h>

const byte MAP[] = {0xC0, 0xF9, 0xA4, 0xB0, 0x99, 0x92, 0x82, 0xF8, 0X80, 0X90};
const byte POS[] = {0xF1, 0xF2, 0xF4, 0xF8};

void initKitDisplay()
{
    pinMode(17, OUTPUT);
    pinMode(14, OUTPUT);
    pinMode(12, OUTPUT);
}

void writeDigit(byte s, byte v)
{
    digitalWrite(17, LOW);
    shiftOut(12, 14, MSBFIRST, MAP[v]);
    shiftOut(12, 14, MSBFIRST, POS[s]);
    digitalWrite(17, HIGH);
}

void writeKitDisplay()
{
    writeDigit(0, DI0);
    writeDigit(1, DI1);
    writeDigit(2, DI2);
    writeDigit(3, DI3);
}