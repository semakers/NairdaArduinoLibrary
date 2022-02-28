

#include "virtual_machine/virtual_machine.h"
#include "servo_component.h"
#include "extern_libraries/linked_list/linked_list.h"

#if defined(ARDUINO_ARCH_STM32)
#include <Servo.h>
#elif defined(ARDUINO_ARCH_ESP32)
#include "extern_libraries/esp32_servo/esp32_servo.h"
#else
#include <Servo.h>
#endif

#include <Arduino.h>

extern bool loadedServos;

void servoCreate(uint16_t *args, uint8_t *pins, int8_t *ledcChannel,Servo* servo)
{
    servo= new Servo();

    pins[0] = args[1];
#if defined(ARDUINO_ARCH_STM32)
    servo->attach(args[1], args[2], args[3]);
    servo->write(args[4]);
#else
#if defined(ARDUINO_ARCH_ESP32)

    if (getCurrentChannel() < 16)
    {
        servo->attach(args[1], getCurrentChannel(), 0, 180, args[2], args[3]);
        servo->write(args[4]);
        ledcChannel[0] = getCurrentChannel();
        nextCurrentChannel();
    }

#else

    servo->attach(args[1], args[2], args[3]);
    servo->write(args[4]);

#endif
#endif
}

void servoExec(uint32_t *execArgs,Servo* servo)
{
    servo->write((execArgs[0] < 0) ? 0 : (execArgs[0] > 180) ? 180
                                                            : execArgs[0]);
}

void servoOff(Servo* servo)
{
    servo->detach();
}

void servoDebugLoad(VolatileMemory* volatileMemory){
     volatileMemory->descArgsBuffer[0] = SERVO;
        volatileMemory->descArgsBuffer[1] = getMapedPin(volatileMemory->declarationBuffer[0]);
        volatileMemory->descArgsBuffer[2] = (volatileMemory->declarationBuffer[1] * 100) + volatileMemory->declarationBuffer[2];
        volatileMemory->descArgsBuffer[3] = (volatileMemory->declarationBuffer[3] * 100) + volatileMemory->declarationBuffer[4];
        volatileMemory->descArgsBuffer[4] = (volatileMemory->declarationBuffer[5] * 100) + volatileMemory->declarationBuffer[6];
        component_t *tempServo = newComponent(volatileMemory->descArgsBuffer);
        volatileMemory->components[SERVO].add(tempServo);
}

void servoEepromLoad(VolatileMemory* volatileMemory){
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
            component_t *tempServo = newComponent(volatileMemory->descArgsBuffer);
            volatileMemory->components[SERVO].add(tempServo);
        }
    }
  motorEepromLoad(volatileMemory);
}

void servoEepromRun(uint8_t id, VolatileMemory* volatileMemory){
    volatileMemory->execBuffer[0] = getInputValue(nextByte());
    execAct(volatileMemory->execBuffer, SERVO,volatileMemory->components[SERVO].get(id));
}


