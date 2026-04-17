#include "virtual_machine/virtual_machine.h"
#include "neo_pixel_component.h"
#include "extern_libraries/linked_list/linked_list.h"
#include <Arduino.h>

#if defined(ARDUINO_ARCH_ESP32)

extern "C" void espShow(
    uint16_t pin, uint8_t *pixels, uint32_t numBytes, uint8_t type);

#endif

void neoPixelCreate(uint16_t *args, component_t *component);

void setupNeoPixel(component_t* component, int pin, int numPixels) {
  uint16_t descArgsBuffer[3];
  descArgsBuffer[0] = NEOPIXEL;
  descArgsBuffer[1] = pin;
  descArgsBuffer[2] = numPixels;
  neoPixelCreate(descArgsBuffer, component);
}

void runNeoPixel(component_t *component, int r, int g, int b, int index) {
  uint32_t execArgs[4];
  execArgs[0] = r;
  execArgs[1] = g;
  execArgs[2] = b;
  execArgs[3] = index;
  neoPixelExec(execArgs, component->neopixel);
  nairdaLoop();
}

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
    component_t *component = (component_t *)malloc(sizeof(component_t));
    neoPixelCreate(volatileMemory->descArgsBuffer, component);
    volatileMemory->components[NEOPIXEL]
        .add(component);
}

