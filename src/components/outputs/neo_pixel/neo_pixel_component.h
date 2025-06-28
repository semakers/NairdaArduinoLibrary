#include <stdint.h>

void neoPixelExec(uint32_t *execArgs);
void neoPixelOff();
void neoPixelEepromLoad(VolatileMemory *volatileMemory);
void neoPixelEepromRun(uint8_t id, VolatileMemory *volatileMemory);
void neoPixelDebugLoad(VolatileMemory *volatileMemory);