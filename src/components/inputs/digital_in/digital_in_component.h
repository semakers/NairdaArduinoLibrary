#include <stdint.h>

void digitalInCreate(uint16_t *args,uint8_t * pins);
void digitalInSense(uint8_t * pins,uint8_t* tempRead);
void digitalInOff();
void digitalInEepromLoad(VolatileMemory *volatileMemory);
int32_t digitalInEepromRead(VolatileMemory *volatileMemory);
void digitalInDebugLoad(VolatileMemory *volatileMemory);