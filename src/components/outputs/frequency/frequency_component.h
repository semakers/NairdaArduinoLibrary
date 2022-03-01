#include <stdint.h>


void frequencyExec(uint32_t *execArgs,uint8_t * pins);
void frequencyOff();
void frequencyEepromLoad(VolatileMemory* volatileMemory);
void frequencyEepromRun(uint8_t id,VolatileMemory* volatileMemory);
void frequencyDebugLoad(VolatileMemory *volatileMemory);