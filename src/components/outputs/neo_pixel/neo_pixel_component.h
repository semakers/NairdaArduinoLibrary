#include <stdint.h>
#include "extern_libraries/neo_pixel/Adafruit_NeoPixel.h"



void neoPixelExec(uint32_t *execArgs,Adafruit_NeoPixel *neopixel);
void neoPixelOff(Adafruit_NeoPixel *neopixel);
void neoPixelEepromLoad(VolatileMemory* volatileMemory);
void neoPixelEepromRun(uint8_t id,VolatileMemory* volatileMemory);
void neoPixelDebugLoad(VolatileMemory *volatileMemory);