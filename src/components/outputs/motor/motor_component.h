#include <stdint.h>

void motorCreate(uint16_t *args,uint8_t * pins,int8_t* ledcChannel);
void motorExec(uint32_t *execArgs,uint8_t * pins,uint8_t* values,int8_t* ledcChannel);
void motorOff(uint8_t * pins,int8_t *ledcChannel);
void motorEepromLoad(VolatileMemory* volatileMemory);
void motorEepromRun(uint8_t id,VolatileMemory* volatileMemory);
void motorDebugLoad(VolatileMemory *volatileMemory);