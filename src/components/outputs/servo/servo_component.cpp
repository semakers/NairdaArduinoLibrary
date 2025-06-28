

#include "virtual_machine/virtual_machine.h"
#include "servo_component.h"
#include "extern_libraries/linked_list/linked_list.h"

#include <Arduino.h>
#include <DynamixelSDK.h>

extern bool loadedServos;
extern dynamixel::PortHandler *portHandler;
extern dynamixel::PacketHandler *packetHandler;

void servoCreate(uint16_t *args, component_t *component)
{

    component->pins[0] = args[1];
    Serial.print("Servo created in :  ");
    Serial.println(args[1] - 16);
}

void servoExec(uint32_t *execArgs, uint8_t *pins)
{
    if (pins[0] >= 17 && pins[0] <= 32)
    {
        uint8_t dxl_error = 0;
        uint8_t pin = pins[0] - 16;
        uint16_t pos = 0;
        pos = map((execArgs[0] < 0) ? 0 : (execArgs[0] > 180) ? 180
                                                              : execArgs[0],
                  0, 180, 0, 1024);
        packetHandler->write2ByteTxRx(portHandler, pin, 30, pos, &dxl_error);
        Serial.print("Servo:  ");
        Serial.print(pin);
        Serial.print("  Position:  ");
        Serial.println(pos);
    }
}

void servoOff()
{
}

void servoDebugLoad(VolatileMemory *volatileMemory)
{
    volatileMemory->descArgsBuffer[0] = SERVO;
    volatileMemory->descArgsBuffer[1] = getMapedPin(volatileMemory->declarationBuffer[0]);
    volatileMemory->descArgsBuffer[2] = (volatileMemory->declarationBuffer[1] * 100) + volatileMemory->declarationBuffer[2];
    volatileMemory->descArgsBuffer[3] = (volatileMemory->declarationBuffer[3] * 100) + volatileMemory->declarationBuffer[4];
    volatileMemory->descArgsBuffer[4] = (volatileMemory->declarationBuffer[5] * 100) + volatileMemory->declarationBuffer[6];
    component_t *component = (component_t *)malloc(sizeof(component_t));
    servoCreate(volatileMemory->descArgsBuffer, component);
    volatileMemory->components[SERVO].add(component);
}

void servoEepromLoad(VolatileMemory *volatileMemory)
{
    uint8_t currentByte;
    while (!loadedServos)
    {
        currentByte = nextByte();
        if (currentByte == endServos)
        {
            loadedServos = true;
        }
        else
        {
            uint8_t servoBytes[7];
            servoBytes[0] = currentByte;
            for (uint8_t i = 1; i < 7; i++)
            {
                servoBytes[i] = nextByte();
            }
            volatileMemory->descArgsBuffer[0] = SERVO;
            volatileMemory->descArgsBuffer[1] = getMapedPin(servoBytes[0]);
            volatileMemory->descArgsBuffer[2] = (servoBytes[1] * 100) + servoBytes[2];
            volatileMemory->descArgsBuffer[3] = (servoBytes[3] * 100) + servoBytes[4];
            volatileMemory->descArgsBuffer[4] = (servoBytes[5] * 100) + servoBytes[6];
            component_t *component = (component_t *)malloc(sizeof(component_t));
            servoCreate(volatileMemory->descArgsBuffer, component);
            volatileMemory->components[SERVO].add(component);
        }
    }

    motorEepromLoad(volatileMemory);
}

void servoEepromRun(uint8_t id, VolatileMemory *volatileMemory)
{
    volatileMemory->execBuffer[0] = getInputValue(nextByte());
    execAct(volatileMemory->execBuffer, SERVO, volatileMemory->components[SERVO].get(id));
}
