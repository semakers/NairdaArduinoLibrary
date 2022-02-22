#include <Arduino.h>
#include "load_from_eeprom.h"
#include "neo_pixel_component.h"
#include "extern_libraries/neo_pixel/Adafruit_NeoPixel.h"
#include "components/inputs/analogic/analogic_component.h"

extern LinkedList<component *> listNeopixels;
extern bool loadedNeoPixels;
extern uint16_t descArgsBuffer[5];
extern uint32_t execBuffer[6];



void neoPixelCreate(uint16_t *args, uint8_t *pins,Adafruit_NeoPixel *neopixel)
{
    pins[0] = args[1];
    neopixel = new Adafruit_NeoPixel(args[2], pins[0], NEO_GRB + NEO_KHZ800);
    neopixel->begin();
}

void neoPixelExec(uint32_t *execArgs,Adafruit_NeoPixel *neopixel)
{
    neopixel->setPixelColor(execArgs[3], execArgs[0], execArgs[1], execArgs[2]);
    neopixel->show();
}

void neoPixelOff(Adafruit_NeoPixel *neopixel)
{
    free(neopixel);
}

void neoPixelEepromLoad()
{
     uint8_t currentByte;

    while (!loadedNeoPixels)
    {
        currentByte = nextByte();

        if (currentByte == endNeopixels)
        {
            loadedNeoPixels = true;
        }
        else
        {
            descArgsBuffer[0] = NEOPIXEL;
            descArgsBuffer[1] = getMapedPin(currentByte);
            descArgsBuffer[2] = nextByte();

            component *tempNeopixel = new component(descArgsBuffer);
            listNeopixels.add(tempNeopixel);
        }
    }
    analogicEepromLoad();
}

void neoPixelEepromRun(uint8_t id)
{
     execBuffer[0] = getInputValue(nextByte());
    execBuffer[1] = getInputValue(nextByte());
    execBuffer[2] = getInputValue(nextByte());
    execBuffer[3] = getInputValue(nextByte());

    listNeopixels.get(id)->execAct(execBuffer, NEOPIXEL);
}
