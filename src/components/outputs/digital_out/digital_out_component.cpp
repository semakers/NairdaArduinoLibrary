#include "virtual_machine/virtual_machine.h"
#include "digital_out_component.h"
#include "extern_libraries/linked_list/linked_list.h"

#include <Arduino.h>

#if !defined(ARDUINO_ARCH_ESP32)
#include "extern_libraries/soft_pwm/soft_pwm.h"
#endif

extern uint8_t currentKit;

void digitalOutCreate(uint16_t *args, component_t *component);

void setupDigitalOut(component_t* component, int pin) {
  uint16_t descArgsBuffer[2];
  descArgsBuffer[0] = DIGITAL_OUT;
  descArgsBuffer[1] = pin;
  digitalOutCreate(descArgsBuffer, component);
}

void runDigitalOut(component_t *component, int value) {
  uint32_t execArgs[1];
  execArgs[0] = value;
  uint8_t pins[1];
  pins[0] = component->pins[0];
  uint8_t values[1];
  digitalOutExec(execArgs, pins, values);
  nairdaLoop();
}

void digitalOutCreate(uint16_t *args, component_t *component)
{
    component->pins[0] = args[1];
#if defined(ARDUINO_ARCH_ESP32)
    if (currentKit == LK32_KIT && args[1] == 25)
    {
        pinMode(args[1], OUTPUT);
    }
    else
    {
        ledcAttach(args[1], 50, 16);
    }
#else
    pinMode(args[1], OUTPUT);
    SoftPWMSet(args[1], 0);
#endif
}

void digitalOutExec(uint32_t *execArgs, uint8_t *pins, uint8_t *values)
{
#if defined(ARDUINO_ARCH_ESP32)

    values[0] = (execArgs[0] < 0) ? 0 : (execArgs[0] > 100) ? 100
                                                            : execArgs[0];

    if (currentKit == LK32_KIT && pins[0] == 25)
    {
        if (values[0] >= 0 && values[0] <= 50)
        {
            digitalWrite(pins[0], LOW);
        }
        else if (values[0] > 50 && values[0] <= 100)
        {
            digitalWrite(pins[0], HIGH);
        }
    }
    else
    {
        ledcWrite(pins[0], map(values[0], 0, 100, 0, 65535));
    }

#else
    SoftPWMSetPercent(pins[0], (execArgs[0] < 0) ? 0 : (execArgs[0] > 100) ? 100
                                                                           : execArgs[0]);
#endif
}

void digitalOutOff(uint8_t *pins)
{
#if defined(ARDUINO_ARCH_ESP32)
    ledcWrite(pins[0], 0);
    ledcDetach(pins[0]);
#else
    SoftPWMSet(pins[0], 0);
    SoftPWMEnd(pins[0]);
#endif
}

void digitalOutDebugLoad(VolatileMemory *volatileMemory)
{
    volatileMemory->descArgsBuffer[0] = DIGITAL_OUT;
    volatileMemory->descArgsBuffer[1] = getMapedPin(volatileMemory->declarationBuffer[0]);

    component_t *component = (component_t *)malloc(sizeof(component_t));
    digitalOutCreate(volatileMemory->descArgsBuffer, component);
    volatileMemory->components[DIGITAL_OUT].add(component);
}
