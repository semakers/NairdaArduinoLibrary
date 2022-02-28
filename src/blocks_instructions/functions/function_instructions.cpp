#include <stdint.h>
#include "virtual_machine/virtual_machine.h"
#include "extern_libraries/linked_list/linked_list.h"
#include "function_instructions.h"

extern uint32_t currentOffset;
LinkedList<int> directionsStack = LinkedList<int>();

void runGoToFunction()
{
    uint8_t jumBytes[3];
    for (uint8_t i = 0; i < 3; i++)
    {
        jumBytes[i] = nextByte();
    }
    uint16_t jump = (jumBytes[0] * 10000) + (jumBytes[1] * 100) + jumBytes[2];
    bool contains = false;
    for (uint16_t i = 0; i < directionsStack.size(); i++)
    {
        if (directionsStack.get(i) == currentOffset)
        {
            contains = true;
        }
    }
    if (!contains)
    {
        directionsStack.add(currentOffset);
    }
    currentOffset = jump;
}

void runEndOfFunction()
{
    uint16_t jump = directionsStack.get(directionsStack.size() - 1);
    directionsStack.remove(directionsStack.size() - 1);
    currentOffset = jump;
}
