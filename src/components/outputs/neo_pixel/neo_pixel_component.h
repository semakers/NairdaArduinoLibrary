#include <stdint.h>
#include "extern_libraries/neo_pixel/Adafruit_NeoPixel.h"

void neoPixelCreate(uint16_t *args,uint8_t * pins,Adafruit_NeoPixel *neopixel);
void neoPixelExec(uint32_t *execArgs,Adafruit_NeoPixel *neopixel);
void neoPixelOff(Adafruit_NeoPixel *neopixel);
void neoPixelEepromLoad();
void neoPixelEepromRun(uint8_t id);