#include <stdint.h>
#include "extern_libraries/neo_pixel/Adafruit_NeoPixel.h"

void setupNeoPixel(component_t *component, int pin, int numPixels);
void runNeoPixel(component_t *component, int r, int g, int b, int index);
void neoPixelExec(uint32_t *execArgs,Adafruit_NeoPixel *neopixel);
void neoPixelOff(Adafruit_NeoPixel *neopixel);
void neoPixelDebugLoad(VolatileMemory *volatileMemory);