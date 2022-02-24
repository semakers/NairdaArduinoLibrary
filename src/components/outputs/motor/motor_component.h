#include <stdint.h>
#include "volatile_memory/volatile_memory.h"

void motorCreate(uint16_t *args,uint8_t * pins,int8_t* ledcChannel);
void motorExec(uint32_t *execArgs,uint8_t * pins,uint8_t* values,int8_t* ledcChannel);
void motorOff(uint8_t * pins,int8_t *ledcChannel);
void motorEepromLoad();
void motorEepromRun(uint8_t id);
void motorDebugLoad(VolatileMemory *volatileMemory);