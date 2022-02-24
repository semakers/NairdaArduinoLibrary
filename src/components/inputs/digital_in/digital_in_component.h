#include <stdint.h>
#include "volatile_memory/volatile_memory.h"

void digitalInCreate(uint16_t *args,uint8_t * pins);
void digitalInSense(uint8_t * pins,uint8_t* tempRead);
void digitalInOff();
void digitalInEepromLoad();
int32_t digitalInEepromRead();
void digitalInDebugLoad(VolatileMemory *volatileMemory);