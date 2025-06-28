#include <stdint.h>

void servoExec(uint32_t *execArgs, uint8_t *pins);
void servoOff();
void servoEepromLoad(VolatileMemory *volatileMemory);
void servoEepromRun(uint8_t id, VolatileMemory *volatileMemory);
void servoDebugLoad(VolatileMemory *volatileMemory);