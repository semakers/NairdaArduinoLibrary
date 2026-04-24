#include <stdint.h>
#include "virtual_machine/virtual_machine.h"
#include "blue_methods/blue_methods.h"
#include "platform/platform_hal.h"

uint8_t declaratedCommands[] = {endServos, endDC, endLeds, endFrequencies, endNeopixels, endAnalogics, endDigitals,
                                endUltrasonics};

typedef void (*DebugLoad)(VolatileMemory *volatileMemory);
DebugLoad debugLoad[8] = {servoDebugLoad, motorDebugLoad, digitalOutDebugLoad, frequencyDebugLoad,
                          neoPixelDebugLoad, analogicDebugLoad, digitalInDebugLoad, ultrasonicDebugLoad};

uint8_t argsSizeByComponent[] = {7, 3, 1, 1, 2, 1, 1, 2};

uint8_t execArgsSizeByComponent[] = {1, 2, 1, 6, 4};

uint8_t indexArray[COMPONENTS_SIZE];
uint8_t componentId = 0;

void fillIndexArray(VolatileMemory *volatileMemory)
{
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
}

int declarateComponents(uint8_t *currentValue, VolatileMemory *volatileMemory)
{
    for (int i = 0; i < COMPONENTS_SIZE; i++)
    {
        if (currentValue[0] == declaratedCommands[i])
        {
            volatileMemory->declaratedComponents[i] = true;
            if (i == COMPONENTS_SIZE - 1)
            {
                volatileMemory->declaratedDescriptor = true;
                fillIndexArray(volatileMemory);
            }
            return 0;
        }
        else if (!volatileMemory->declaratedComponents[i] && currentValue[0] < 100)
        {
            for (int j = 0; j < argsSizeByComponent[i]; j++)
            {
                if (!volatileMemory->declarationboolean[j])
                {
                    volatileMemory->declarationBuffer[j] = currentValue[0];
                    volatileMemory->declarationboolean[j] = true;
                    if (j == argsSizeByComponent[i] - 1)
                    {
                        debugLoad[i](volatileMemory);

                        memset(volatileMemory->declarationboolean, 0, 7);
                    }
                    return 0;
                }
            }
        }
    }
}

int executeComponent(uint8_t *currentValue, VolatileMemory *volatileMemory)
{
    if (volatileMemory->executedComponent == NON_COMPONENT)
    {

        for (int8_t i = 0; i < COMPONENTS_SIZE; i++)
        {
            if (currentValue[0] >= (i == 0 ? 0 : indexArray[i - 1]) && currentValue[0] < indexArray[i] && volatileMemory->components[i].size() > 0)
            {
                componentId = currentValue[0] - ((i == 0) ? 0 : indexArray[i - 1]);
                if (i < ACTUATORS_SIZE)
                {
                    volatileMemory->executedComponent = i;
                }
                else
                {
                    sendSensVal(i, volatileMemory->components[i].get(componentId));
                    return 0;
                }
            }
        }
        return 0;
    }

    for (uint8_t i = 0; i < ACTUATORS_SIZE; i++)
    {
        if (volatileMemory->executedComponent == i)
        {
            for (uint8_t j = 0; j < execArgsSizeByComponent[i]; j++)
            {
                if (!volatileMemory->executionBoolean[j])
                {
                    volatileMemory->executionBuffer[j] = currentValue[0];
                    volatileMemory->executionBoolean[j] = true;
                    if (j == execArgsSizeByComponent[i] - 1)
                    {
                        if (i == SERVO)
                        {
                            volatileMemory->executionBuffer[0] = map(volatileMemory->executionBuffer[0], 0, 99, 0, 180);
                        }
                        execAct(volatileMemory->executionBuffer, i, volatileMemory->components[i].get(componentId));

                        memset(volatileMemory->executionBoolean, false, 7);
                        volatileMemory->executedComponent = NON_COMPONENT;
                    }
                    return 0;
                }
            }

            return 0;
        }
    }
    volatileMemory->executedComponent = NON_COMPONENT;
}

void nairdaDebug(uint8_t currentValue, VolatileMemory *volatileMemory)
{
    if (currentValue == projectInit)
    {
        hal_handleProjectInit(volatileMemory);
    }
    else if (currentValue == userBootloader)
    {
        hal_handleBootloader(volatileMemory);
    }
    else if (currentValue == versionCommand)
    {
        hal_handleVersionCommand();
    }
    else if (volatileMemory->declaratedDescriptor == false)
    {
        declarateComponents(&currentValue, volatileMemory);
    }
    else
    {
        executeComponent(&currentValue, volatileMemory);
    }
}
