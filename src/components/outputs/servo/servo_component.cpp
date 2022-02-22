#if defined(ARDUINO_ARCH_STM32)
#include <Servo.h>
#elif defined(ARDUINO_ARCH_ESP32)
#include "extern_libraries/esp32_servo/esp32_servo.h"
#else
#include <Servo.h>
#endif

#include "load_from_eeprom.h"
#include "servo_component.h"
#include "components/outputs/motor/motor_component.h"
#include <Arduino.h>

extern LinkedList<component *> listServos;
extern bool loadedServos;
extern uint16_t descArgsBuffer[5];
extern uint32_t execBuffer[6];



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


void servoEepromLoad(){
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
            descArgsBuffer[0] = SERVO;
            descArgsBuffer[1] = getMapedPin(servoBytes[0]);
            descArgsBuffer[2] = (servoBytes[1] * 100) + servoBytes[2];
            descArgsBuffer[3] = (servoBytes[3] * 100) + servoBytes[4];
            descArgsBuffer[4] = (servoBytes[5] * 100) + servoBytes[6];
            component *tempServo = new component(descArgsBuffer);
            listServos.add(tempServo);
        }
    }
  motorEepromLoad();
}

void servoEepromRun(uint8_t id){
    execBuffer[0] = getInputValue(nextByte());
    listServos.get(id)->execAct(execBuffer, SERVO);
}


