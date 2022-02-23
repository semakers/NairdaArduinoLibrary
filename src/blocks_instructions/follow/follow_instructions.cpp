#include "load_from_eeprom.h"
#include "extern_libraries/linked_list/linked_list.h"
#include "follow_instructions.h"
#include <Arduino.h>

class repeatBegin
{
public:
    uint32_t offsetStart;
    uint32_t offsetEnd;
    uint8_t loop;
    uint32_t times;
    repeatBegin(uint32_t cOffsetStart, uint32_t cOffsetEnd, uint8_t cLoop, uint32_t cTimes)
    {
        offsetStart = cOffsetStart;
        offsetEnd = cOffsetEnd;
        loop = cLoop;
        times = cTimes;
    }
};

LinkedList<repeatBegin *> listRepeatBegins = LinkedList<repeatBegin *>();
extern uint32_t currentOffset;


void freeRepeatBegins()
{
    for (int i = 0; i < listRepeatBegins.size(); i++)
    {
        free(listRepeatBegins.get(i));
    }
    listRepeatBegins.clear();
}

void runDelay()
{
    uint32_t delayTime = getInputValue(nextByte());
#if defined(ARDUINO_ARCH_ESP32)
    for (uint64_t i = 0; i < delayTime * 2500; i++)
    {
        if (i % 2500 == 0)
        {
            if (callInterrupt() == 1)
            {
                Serial.println("Interrupt called");
                break;
            }
        }
    }
#else
    uint32_t currentTime = millis();
    while ((millis() - currentTime) < delayTime && callInterrupt() == 0)
    {
    }
#endif
}

repeatBegin *findRepeatBegin(uint32_t repeatStartOffset)
{
    for (uint8_t i = 0; i < listRepeatBegins.size(); i++)
    {
        if (listRepeatBegins.get(i)->offsetStart == repeatStartOffset)
        {
            return listRepeatBegins.get(i);
        }
    }
    return NULL;
}

uint8_t findRepeatBeginIndex(uint32_t repeatStartOffset)
{
    for (uint8_t i = 0; i < listRepeatBegins.size(); i++)
    {
        if (listRepeatBegins.get(i)->offsetStart == repeatStartOffset)
        {
            return i;
        }
    }
    return 0;
}

void runRepeat()
{
    uint32_t sos = currentOffset - 1;
    repeatBegin *currentBegin = findRepeatBegin(sos);
    uint8_t loop = nextByte();
    int32_t times = getInputValue(nextByte());
    uint8_t eosBytes[3];
    for (uint8_t i = 0; i < 3; i++)
    {
        eosBytes[i] = nextByte();
    }
    if (currentBegin == NULL)
    {
        uint32_t eos = (eosBytes[0] * 10000) + (eosBytes[1] * 100) + eosBytes[2];
        currentBegin = new repeatBegin(sos, eos, loop, times);
        listRepeatBegins.add(currentBegin);
    }

    if (currentBegin->loop == 0)
    {

        if (currentBegin->times == 0)
        {
            currentOffset = currentBegin->offsetEnd;
            listRepeatBegins.remove(findRepeatBeginIndex(currentBegin->offsetStart));
            free(currentBegin);
        }
        else
        {
            currentBegin->times = currentBegin->times - 1;
        }
    }
}

void runEndRepeat()
{
    currentOffset = listRepeatBegins.get(listRepeatBegins.size() - 1)->offsetStart;
}

void runBreak()
{
    repeatBegin *breakBegin = listRepeatBegins.get(listRepeatBegins.size() - 1);
    currentOffset = breakBegin->offsetEnd;
    listRepeatBegins.remove(listRepeatBegins.size() - 1);
    free(breakBegin);
}
