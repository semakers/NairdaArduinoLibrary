#include <stdint.h>
#include "virtual_machine/virtual_machine.h"
#include "blue_methods/blue_methods.h"

enum
{
    noMemory,
    memory1k,
    memory4k,
    memory256k,
    memory512k
};

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
            /* Serial.print("current size ");
             Serial.print(volatileMemory->components[i].size());
             Serial.print(" before size ");
             Serial.println(indexArray[i - 1]);*/
            indexArray[i] = volatileMemory->components[i].size() + indexArray[i - 1];
        }
        /*Serial.print("index array: ");
         Serial.print(i);
         Serial.print(" ");
         Serial.println(indexArray[i]);*/
    }
}

#ifndef __AVR_ATmega168__
void cleanSavingBoolean(bool *savingBoolean)
{
    for (int j = 0; j < 4; j++)
    {
        savingBoolean[j] = false;
    }
}
#endif

int declarateComponents(uint8_t *currentValue, VolatileMemory *volatileMemory)
{
    // Serial.println(currentValue[0]);
    for (int i = 0; i < COMPONENTS_SIZE; i++)
    {
        if (currentValue[0] == declaratedCommands[i])
        {
            volatileMemory->declaratedComponents[i] = true;
            if (i == COMPONENTS_SIZE - 1)
            {
                //  Serial.println("loaded descriptor");
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
    /* Serial.print("byte: ");
     Serial.println(currentValue[0]);*/

    if (volatileMemory->executedComponent == NON_COMPONENT)
    {

        for (int8_t i = 0; i < COMPONENTS_SIZE; i++)
        {
            /* Serial.print(currentValue[0]);
             Serial.print(">=");
             Serial.print(( i == 0 ? 0 : indexArray[i - 1]));
             Serial.print("&&");
             Serial.print(currentValue[0]);
             Serial.print("<");
             Serial.print(indexArray[i]);
             Serial.print("&&");
             Serial.print(volatileMemory->components[i].size());
             Serial.println(">0");*/
            if (currentValue[0] >= (i == 0 ? 0 : indexArray[i - 1]) && currentValue[0] < indexArray[i] && volatileMemory->components[i].size() > 0)
            {
                componentId = currentValue[0] - ((i == 0) ? 0 : indexArray[i - 1]);
                if (i < ACTUATORS_SIZE)
                {
                    volatileMemory->executedComponent = i;
                }
                else
                {
                    /*Serial.print("Send sens val ");
                    Serial.println(i);*/
                    sendSensVal(i, volatileMemory->components[i].get(componentId));
                    return 0;
                }
            }
        }
        /* Serial.print("executed component: ");
         Serial.println(volatileMemory->executedComponent);*/
        return 0;
    }
    /*Serial.print("component: ");
    Serial.println(componentId);*/

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
                        /*Serial.print("execute component: ");
                        Serial.print(i);
                        Serial.print("on index ");
                        Serial.print(componentId);
                        Serial.print(" pin ");
                        Serial.print(volatileMemory->components[i].get(componentId)->pins[0]);
                        Serial.print(" ledc ");
                        Serial.println(volatileMemory->components[i].get(componentId)->ledcChannel[0]);*/
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
    static uint32_t currentProgramOffset = 0;
    static bool savingBoolean[4];
    static uint8_t savingBuffer[4];
    static int32_t programmSize = 0;
    static bool startSaving = false;

    if (currentValue == projectInit)
    {
#if defined(__AVR_ATmega32U4__) || (ARDUINO_ARCH_ESP32) || (ARDUINO_ARCH_STM32)
        clearVolatileMemory(volatileMemory, true);
#else
        clearVolatileMemory(volatileMemory, true);

        asm volatile("jmp 0");
#endif
        // Serial.println("Se limpriaron las listas");
    }
    if (currentValue == saveCommand)
    {
        uint32_t memorySize;

#if !defined(_24LC_256) && !defined(_24LC_512)
#if defined(__AVR_ATmega168__)
        memorySize = 0;
#endif

#if defined(ARDUINO_ARCH_ESP32)
        startSaving = true;
        memorySize = 512 * 1024;
#endif

#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega328P__)
        startSaving = true;
        memorySize = 1024;
#endif

#if defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)
        startSaving = true;
        memorySize = 4 * 1024;
#endif

#if defined(ARDUINO_ARCH_STM32)
        startSaving = true;
        memorySize = EEPROM.length();
#endif

#else
        startSaving = true;

#if defined(_24LC_256)
        memorySize = 256 * 1024;
#endif

#if defined(_24LC_512)
        memorySize = 512 * 1024;
#endif

#endif
        sendMemorySize(memorySize);
    }
    else if (startSaving)
    {
#ifndef __AVR_ATmega168__
        if (!savingBoolean[0])
        {
            savingBoolean[0] = true;
            savingBuffer[0] = currentValue;
        }
        else if (!savingBoolean[1])
        {
            savingBoolean[1] = true;
            savingBuffer[1] = currentValue;
        }
        else if (!savingBoolean[2])
        {
            savingBoolean[2] = true;
            savingBuffer[2] = currentValue;
        }
        else if (!savingBoolean[3])
        {
            currentProgramOffset = 4;
            savingBoolean[3] = true;
            savingBuffer[3] = currentValue;
            writeByte(0, savingBuffer[0]);
            writeByte(1, savingBuffer[1]);
            writeByte(2, savingBuffer[2]);
            writeByte(3, savingBuffer[3]);
            programmSize = (savingBuffer[1] * 10000) + (savingBuffer[2] * 100) + savingBuffer[3] - currentProgramOffset;
        }
        else
        {
            if (programmSize > 1)
            {
                writeByte(currentProgramOffset, currentValue);
                currentProgramOffset++;
                programmSize--;
            }
            else
            {
                writeByte(currentProgramOffset, currentValue);
                startSaving = false;
                cleanSavingBoolean(savingBoolean);
            }
        }
#endif
    }
    else if (currentValue == versionCommand)
    {

#if defined(ARDUINO_ARCH_ESP32)
        bleWrite(CURRENT_VERSION);
#else

#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)
        Serial1.write(((char)CURRENT_VERSION));
#endif
        Serial.write(((char)CURRENT_VERSION));
#endif
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
