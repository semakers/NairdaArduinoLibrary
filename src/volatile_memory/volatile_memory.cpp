#include "volatile_memory.h"
#include "nairda.h"

void freeCompList(LinkedList<component_t *> *list, uint8_t type)
{
    for (int i = 0; i < list->size(); i++)
    {
        off(type, list->get(i));
        free(list->get(i));
    }
    list->clear();
}

void initVolatileMemory(VolatileMemory *volatileMemory)
{
    for (int i = 0; i < COMPONENTS_SIZE; i++)
    {
        volatileMemory->components[i] = LinkedList<component_t *>();
    }
    clearVolatileMemory(volatileMemory, false);
}
