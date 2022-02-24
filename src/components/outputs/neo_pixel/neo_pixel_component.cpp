#include <Arduino.h>
#include "load_from_eeprom.h"
#include "neo_pixel_component.h"
#include "components/component.h"
#include "extern_libraries/neo_pixel/Adafruit_NeoPixel.h"
#include "components/inputs/analogic/analogic_component.h"
#include "extern_libraries/linked_list/linked_list.h"
#include "volatile_memory/volatile_memory.h"

#if defined(ARDUINO_ARCH_ESP32)

extern "C" void espShow(
    uint16_t pin, uint8_t *pixels, uint32_t numBytes, uint8_t type);

#endif

extern LinkedList<component_t *> listNeopixels;
extern bool loadedNeoPixels;
extern uint16_t descArgsBuffer[5];
extern uint32_t execBuffer[6];

void neoPixelCreate(uint16_t *args, uint8_t *pins, Adafruit_NeoPixel *neopixel)
{
    pins[0] = args[1];
    neopixel = new Adafruit_NeoPixel(args[2], pins[0], NEO_GRB + NEO_KHZ800);
    neopixel->begin();
}

void neoPixelExec(uint32_t *execArgs, Adafruit_NeoPixel *neopixel)
{
    neopixel->setPixelColor(execArgs[3], execArgs[0], execArgs[1], execArgs[2]);
    neopixel->show();
}

void neoPixelOff(Adafruit_NeoPixel *neopixel)
{
    free(neopixel);
}

void neoPixelDebugLoad(VolatileMemory *volatileMemory)
{
    volatileMemory->descArgsBuffer[0] = NEOPIXEL;
    volatileMemory->descArgsBuffer[1] = getMapedPin(volatileMemory->declarationBuffer[0]);
    volatileMemory->descArgsBuffer[2] = volatileMemory->declarationBuffer[1];
    component_t *tempNeopixel = newComponent(volatileMemory->descArgsBuffer);
    volatileMemory->components[NEOPIXEL].add(tempNeopixel);
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

            component_t *tempNeopixel = newComponent(descArgsBuffer);
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

    execAct(execBuffer, NEOPIXEL, listNeopixels.get(id));
}
