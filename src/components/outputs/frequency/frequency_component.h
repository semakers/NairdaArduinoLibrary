#include <stdint.h>

void frequencyCreate(uint16_t *args,uint8_t * pins);
void frequencyExec(uint32_t *execArgs,uint8_t * pins);
void frequencyOff();
void frequencyEepromLoad(VolatileMemory* volatileMemory);
void frequencyEepromRun(uint8_t id,VolatileMemory* volatileMemory);
void frequencyDebugLoad(VolatileMemory *volatileMemory);