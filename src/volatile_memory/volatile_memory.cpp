#include "volatile_memory.h"
#include <string.h>

#include "nairda.h"

#if defined(ARDUINO_ARCH_ESP32)
#include "extern_libraries/veml6040/VEML6040.h"
extern int runProgrammTimeOut;
extern VEML6040 RGBWSensor;
#endif

extern uint8_t currentKit;

void freeCompList(LinkedList<component_t *> *list, uint8_t type)
{
    for (int i = 0; i < list->size(); i++)
    {
        off(type, list->get(i));
        free(list->get(i));
    }
    list->clear();
}

void clearVolatileMemory(VolatileMemory *volatileMemory, bool offComonents)
{

    for (int i = 0; i < COMPONENTS_SIZE; i++)
    {
        freeCompList(&(volatileMemory->components[i]), i);
    }
    volatileMemory->declaratedDescriptor = false;
    volatileMemory->currentChannel = 1;
    volatileMemory->executedComponent = NON_COMPONENT;
    memset(volatileMemory->declaratedComponents, false, 8);
    memset(volatileMemory->executeActuator, false, 5);
    memset(volatileMemory->executionBoolean, false, 7);
    memset(volatileMemory->executionBuffer, 0, 7);
    memset(volatileMemory->declarationboolean, false, 7);
    memset(volatileMemory->declarationBuffer, 0, 7);
    memset(volatileMemory->descArgsBuffer, 0, 5);
    memset(volatileMemory->execBuffer, 0, 6);
    freeCompList(&(volatileMemory->components[MOTOR]), MOTOR);
    freeCompList(&(volatileMemory->components[DIGITAL_OUT]), DIGITAL_OUT);
}

void initVolatileMemory(VolatileMemory *volatileMemory)
{
    for (int i = 0; i < COMPONENTS_SIZE; i++)
    {
        volatileMemory->components[i] = LinkedList<component_t *>();
    }
    clearVolatileMemory(volatileMemory, false);
}