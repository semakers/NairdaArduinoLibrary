#if !defined(ARDUINO_ARCH_ESP32)

#include <string.h>
#include "volatile_memory/volatile_memory.h"
#include "nairda.h"

extern uint8_t currentKit;

void clearVolatileMemory(VolatileMemory *volatileMemory, bool offComonents)
{
    freeCompList(&(volatileMemory->components[MOTOR]), MOTOR);
    freeCompList(&(volatileMemory->components[DIGITAL_OUT]), DIGITAL_OUT);
}

#endif
