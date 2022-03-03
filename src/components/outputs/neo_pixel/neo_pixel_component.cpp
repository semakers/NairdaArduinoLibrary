#include "virtual_machine/virtual_machine.h"
#include "neo_pixel_component.h"
#include "extern_libraries/linked_list/linked_list.h"
#include <Arduino.h>

#if defined(ARDUINO_ARCH_ESP32)

extern "C" void espShow(
    uint16_t pin, uint8_t *pixels, uint32_t numBytes, uint8_t type);

#endif

extern bool loadedNeoPixels;

void neoPixelCreate(uint16_t *args, component_t *component)
{
    component->pins[0] = args[1];
    component->neopixel = new Adafruit_NeoPixel(args[2], component->pins[0], NEO_GRB + NEO_KHZ800);
    component->neopixel->begin();
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
    component_t component;
    neoPixelCreate(volatileMemory->descArgsBuffer, &component);
    volatileMemory->components[NEOPIXEL]
        .add(&component);
}

void neoPixelEepromLoad(VolatileMemory *volatileMemory)
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
            volatileMemory->descArgsBuffer[0] = NEOPIXEL;
            volatileMemory->descArgsBuffer[1] = getMapedPin(currentByte);
            volatileMemory->descArgsBuffer[2] = nextByte();

             component_t *component = (component_t *)malloc(sizeof(component_t));
            neoPixelCreate(volatileMemory->descArgsBuffer, component);
            volatileMemory->components[NEOPIXEL]
                .add(component);
        }
    }
    analogicEepromLoad(volatileMemory);
}

void neoPixelEepromRun(uint8_t id, VolatileMemory *volatileMemory)
{
    volatileMemory->execBuffer[0] = getInputValue(nextByte());
    volatileMemory->execBuffer[1] = getInputValue(nextByte());
    volatileMemory->execBuffer[2] = getInputValue(nextByte());
    volatileMemory->execBuffer[3] = getInputValue(nextByte());

    execAct(volatileMemory->execBuffer, NEOPIXEL, volatileMemory->components[NEOPIXEL].get(id));
}
