#include "virtual_machine/virtual_machine.h"
#include "analogic_component.h"
#include "components/inputs/digital_in/digital_in_component.h"
#include "extern_libraries/linked_list/linked_list.h"

#include <Arduino.h>

extern bool loadedAnalogics;

#if defined(ARDUINO_ARCH_ESP32)
#include "extern_libraries/veml6040/VEML6040.h"
extern VEML6040 RGBWSensor;
#endif

extern uint8_t currentKit;

void analogicCreate(uint16_t *args, component_t *component)
{

    component->pins[0] = args[1];
#if defined(ARDUINO_ARCH_ESP32)
    if (currentKit == ROBUS_KIDSY_KIT && (args[1] == 37 || args[1] == 38 || args[1] == 39))
    {
        RGBWSensor.nairdaBegin();
    }
#endif
}

void analogicSense(uint8_t *pins, uint8_t *tempRead)
{
#if defined(ARDUINO_ARCH_STM32)
    tempRead[0] = map(analogRead(pins[0]), 0, 1023, 0, 100);
#elif defined(ARDUINO_ARCH_ESP32)
    if (currentKit == ROBUS_KIDSY_KIT)
    {
        if (pins[0] == 37 || pins[0] == 38 || pins[0] == 39)
        {
            RGBWSensor.readFixedColors();
        }
        switch (pins[0])
        {
        case 37:
            tempRead[0] = RGBWSensor.getFixedRed();
            break;
        case 38:
            tempRead[0] = RGBWSensor.getFixedGreen();
            break;
        case 39:
            tempRead[0] = RGBWSensor.getFixedBlue();
            break;
        default:
            tempRead[0] = map(analogRead(pins[0]), 0, 4095, 0, 100);
        }
    }
    else
    {
        tempRead[0] = map(analogRead(pins[0]), 0, 4095, 0, 100);
    }
#else
    tempRead[0] = map(analogRead(pins[0]), 0, 1023, 0, 100);
#endif
}

void analogicOff()
{
}

void analogicDebugLoad(VolatileMemory *volatileMemory)
{
    volatileMemory->descArgsBuffer[0] = ANALOGIC;
    volatileMemory->descArgsBuffer[1] = getMapedPin(volatileMemory->declarationBuffer[0]);
    component_t *component = (component_t *)malloc(sizeof(component_t));
    analogicCreate(volatileMemory->descArgsBuffer, component);
    volatileMemory->components[ANALOGIC].add(component);
}

void analogicEepromLoad(VolatileMemory *volatileMemory)
{
    uint8_t currentByte;
    while (!loadedAnalogics)
    {
        currentByte = nextByte();
        if (currentByte == endAnalogics)
        {
            loadedAnalogics = true;
        }
        else
        {
            volatileMemory->descArgsBuffer[0] = ANALOGIC;
            volatileMemory->descArgsBuffer[1] = getMapedPin(currentByte);
            component_t *component = (component_t *)malloc(sizeof(component_t));
            analogicCreate(volatileMemory->descArgsBuffer, component);
            volatileMemory->components[ANALOGIC].add(component);
        }
    }
    digitalInEepromLoad(volatileMemory);
}

int32_t analogicEepromRead(VolatileMemory *volatileMemory)
{
    return getSensVal(ANALOGIC, volatileMemory->components[ANALOGIC].get(nextByte()));
}
