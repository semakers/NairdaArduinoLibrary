#include "virtual_machine/virtual_machine.h"
#include "digital_out_component.h"
#include "extern_libraries/linked_list/linked_list.h"
#include <DynamixelSDK.h>

#include <Arduino.h>

extern bool loadedDigitalOuts;
extern uint8_t currentKit;
extern dynamixel::PortHandler *portHandler;
extern dynamixel::PacketHandler *packetHandler;

void digitalOutCreate(uint16_t *args, component_t *component)
{
    component->pins[0] = args[1];
    Serial.print("Torque servo on:  ");
    Serial.println(args[1]);
    /*pinMode(args[1], OUTPUT);
     Serial.print("Digital out created in pin:  ");
     Serial.println(args[1]);*/
}

void digitalOutExec(uint32_t *execArgs, uint8_t *pins, uint8_t *values, int8_t *ledcChannel)
{

    if (pins[0] >= 1 && pins[0] <= 16)
    {
        uint8_t dxl_error = 0;
        if (execArgs[0] >= 0 && execArgs[0] <= 50)
        {
            packetHandler->write1ByteTxRx(portHandler, pins[0], 24, 0, &dxl_error);
            Serial.print("torque off:  ");
            Serial.println(pins[0]);
        }
        else if (execArgs[0] > 50 && execArgs[0] <= 100)
        {
            packetHandler->write1ByteTxRx(portHandler, pins[0], 24, 1, &dxl_error);
            Serial.print("torque on:  ");
            Serial.println(pins[0]);
        }
    }
    /*Serial.print("args:  ");
    Serial.println(execArgs[0]);

    */
}

void digitalOutOff(uint8_t *pins, int8_t *ledcChannel)
{
}

void digitalOutDebugLoad(VolatileMemory *volatileMemory)
{
    volatileMemory->descArgsBuffer[0] = DIGITAL_OUT;
    volatileMemory->descArgsBuffer[1] = getMapedPin(volatileMemory->declarationBuffer[0]);

    component_t *component = (component_t *)malloc(sizeof(component_t));
    digitalOutCreate(volatileMemory->descArgsBuffer, component);
    volatileMemory->components[DIGITAL_OUT].add(component);
}

void digitalOutEepromLoad(VolatileMemory *volatileMemory)
{
#ifndef __AVR_ATmega168__
    uint8_t currentByte;
    while (!loadedDigitalOuts)
    {
        currentByte = nextByte();
        if (currentByte == endLeds)
        {
            loadedDigitalOuts = true;
        }
        else
        {

            volatileMemory->descArgsBuffer[0] = DIGITAL_OUT;
            volatileMemory->descArgsBuffer[1] = getMapedPin(currentByte);

            component_t *component = (component_t *)malloc(sizeof(component_t));
            digitalOutCreate(volatileMemory->descArgsBuffer, component);
            volatileMemory->components[DIGITAL_OUT].add(component);
        }
    }
    frequencyEepromLoad(volatileMemory);
#endif
}

void digitalOutEepromRun(uint8_t id, VolatileMemory *volatileMemory)
{
    int32_t intensity = getInputValue(nextByte());
    intensity = (intensity < 0) ? 0 : (intensity > 100) ? 100
                                                        : intensity;
    volatileMemory->execBuffer[0] = intensity;
    execAct(volatileMemory->execBuffer, DIGITAL_OUT, volatileMemory->components[DIGITAL_OUT].get(id));
}