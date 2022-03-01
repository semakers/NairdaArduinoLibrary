#include <stdint.h>

void analogicSense(uint8_t * pins,uint8_t* tempRead);
void analogicOff();
void analogicEepromLoad(VolatileMemory *volatileMemory);
int32_t analogicEepromRead(VolatileMemory *volatileMemory);
void analogicDebugLoad(VolatileMemory *volatileMemory);