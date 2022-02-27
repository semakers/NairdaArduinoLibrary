#include <stdint.h>
#include "load_from_eeprom.h"
#include "blue_methods/blue_methods.h"

uint8_t declaratedCommands[] = {endServos, endDC, endLeds, endFrequencies, endNeopixels, endAnalogics, endDigitals,
                                endUltrasonics};

void (*debugLoad)(VolatileMemory *)[] = {servoDebugLoad, motorDebugLoad, digitalOutDebugLoad, frequencyDebugLoad,
                                         neoPixelDebugLoad, analogicDebugLoad, digitalInDebugLoad, ultrasonicDebugLoad};

uint8_t argsSizeByComponent[] = {7, 3, 1, 1, 2, 1, 1, 2};

uint8_t execArgsSizeByComponent[] = {1, 2, 1, 6, 4};

void declarateComponents(uint8_t *currentValue, VolatileMemory *volatileMemory)
{
    volatileMemory->declaratedDescriptor == true;
    for (int i = 0; i < COMPONENTS_SIZE; i++)
    {
        if (currentValue[0] == declaratedCommands[i])
        {
            volatileMemory->declaratedComponents[i] = true;
        }
        else if (!volatileMemory->declaratedComponents[i] && currentValue[0] < 100)
        {
            volatileMemory->declaratedDescriptor = false;
            for (int j = 0; j < argsSizeByComponent[i]; j++)
            {
                if (!volatileMemory->declarationboolean[j])
                {
                    volatileMemory->declarationBuffer[j] = currentValue[0];
                    volatileMemory->declarationboolean[j] = true;
                    if (j == argsSizeByComponent[i] - 1)
                    {
                        debugLoad[i](volatileMemory);
                    }
                    while (!nextBlueByte(currentValue))
                    {
                    }
                }
            }
        }
    }
}

void executeComponent(uint8_t *currentValue, VolatileMemory *volatileMemory)
{

    if (volatileMemory->executedComponent != NON_COMPONENT)
    {
        uint8_t indexArray[COMPONENTS_SIZE];
        uint8_t componentId = 0;
        for (int8_t i = 0; i < COMPONENTS_SIZE; i++)
        {
            if (i == 0)
            {
                indexArray[i] = volatileMemory->components[i].size();
            }
            else
            {
                indexArray[i] = volatileMemory->components[i].size() + indexArray[i - 1];
            }
        }
        for (int8_t i = 0; i < COMPONENTS_SIZE; i++)
        {
            if (currentValue[0] >= i == 0 ? 0 : indexArray[i - 1] && currentValue[0] < indexArray[i] && volatileMemory->components[i].size() > 0)
            {
                componentId = currentValue[0] - i == 0 ? 0 : indexArray[i - 1];
                if (i < ACTUATORS_SIZE)
                {
                    volatileMemory->executedComponent = i;
                }
                else
                {
                    sendSensVal(i, volatileMemory->components[i].get(volatileMemory->executionBoolean[0]));
                }
            }
        }

        for (uint8_t i = 0; i < ACTUATORS_SIZE; i++)
        {
            if (volatileMemory->executedComponent == i)
            {
                for (uint8_t j = 0; j < execArgsSizeByComponent[i]; j++)
                {
                    while (!nextBlueByte(currentValue))
                    {
                    }
                    volatileMemory->executionBuffer[j] = currentValue[0];
                }
                execAct(volatileMemory->executionBuffer, i,
                        volatileMemory->components[i].get(componentId));
            }
        }
        volatileMemory->executedComponent = NON_COMPONENT;
    }
}
