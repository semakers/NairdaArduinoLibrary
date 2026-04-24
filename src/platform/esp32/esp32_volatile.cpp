#if defined(ARDUINO_ARCH_ESP32)

#include <string.h>
#include "volatile_memory/volatile_memory.h"
#include "nairda.h"

#include "extern_libraries/veml6040/VEML6040.h"
extern VEML6040 RGBWSensor;
extern uint8_t currentKit;

void clearVolatileMemory(VolatileMemory *volatileMemory, bool offComonents)
{
    if (offComonents == true)
    {
        for (int i = 0; i < COMPONENTS_SIZE; i++)
        {
            freeCompList(&(volatileMemory->components[i]), i);
        }
    }
    volatileMemory->declaratedDescriptor = false;
    volatileMemory->executedComponent = NON_COMPONENT;
    memset(volatileMemory->declaratedComponents, false, 8);
    memset(volatileMemory->executeActuator, false, 5);
    memset(volatileMemory->executionBoolean, false, 7);
    memset(volatileMemory->executionBuffer, 0, 7);
    memset(volatileMemory->declarationboolean, false, 7);
    memset(volatileMemory->declarationBuffer, 0, 7);
    memset(volatileMemory->descArgsBuffer, 0, 5);
    memset(volatileMemory->execBuffer, 0, 6);

    if (currentKit == ROBBUS_KIDSY_KIT)
    {
        RGBWSensor.nairdaEnd();
    }
}

#endif
