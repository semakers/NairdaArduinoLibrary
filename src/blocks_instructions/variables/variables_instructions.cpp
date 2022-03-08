#include <stdint.h>
#include "virtual_machine/virtual_machine.h"
#include "extern_libraries/linked_list/linked_list.h"
#include "variables_instructions.h"
#include <Arduino.h>

extern uint32_t currentOffset;
LinkedList<int32_t *> listVariables = LinkedList<int32_t *>();
extern bool loadedVariables;


void variableEepromLoad()
{
    uint8_t currentByte;
    while (!loadedVariables)
    {
        currentByte = nextByte();
        if (currentByte == endVariables)
        {
            loadedVariables = true;
        }
        else
        {
            uint8_t varBytes[4];
            varBytes[0] = currentByte;
            for (uint8_t i = 1; i < 4; i++)
            {
                varBytes[i] = nextByte();
            }
            int32_t positiveValue = (varBytes[1] * 10000) + (varBytes[2] * 100) + varBytes[3];
            int32_t* tempVariable= (int32_t*)malloc(sizeof(int32_t));
            tempVariable[0] = varBytes[0] == 0 ? positiveValue : (positiveValue * -1);
            listVariables.add(tempVariable);
        }
    }
    nairdaRunMachineState();
}


int32_t getVariableValue()
{
    return listVariables.get(nextByte())[0];
}

void setVarValue(int32_t newValue, int32_t *value)
{
    value[0] = (newValue > 999999) ? 999999 : (newValue < -999999) ? -999999
                                                                   : newValue;
}

void runSetVarValue(uint8_t id)
{
    setVarValue(getInputValue(nextByte()), listVariables.get(id));
}


void freeVariables()
{
    for (int i = 0; i < listVariables.size(); i++)
    {
        free(listVariables.get(i));
    }
    listVariables.clear();
}
