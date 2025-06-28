#include <stdint.h>

void ultrasonicSense(uint8_t *pins, uint8_t *tempRead);
void ultrasonicOff();

void ultrasonicEepromLoad(VolatileMemory *volatileMemory);
int32_t ultrasonicEepromRead(VolatileMemory *volatileMemory);
void ultrasonicDebugLoad(VolatileMemory *volatileMemory);