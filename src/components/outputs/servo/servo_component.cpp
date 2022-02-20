#include "servo_component.h"

#include <Arduino.h>

#if defined(ARDUINO_ARCH_STM32)
#include <Servo.h>
#elif defined(ARDUINO_ARCH_ESP32)
#include "load_from_eeprom.h"
#include "extern_libraries/esp32_servo/esp32_servo.h"
#else
#include <Servo.h>
#endif

Servo servo;

void servo_create(uint16_t *args, uint8_t *pins, int8_t *ledcChannel)
{
    pins[0] = args[1];
#if defined(ARDUINO_ARCH_STM32)
    servo.attach(args[1], args[2], args[3]);
    servo.write(args[4]);
#else
#if defined(ARDUINO_ARCH_ESP32)

    if (getCurrentChannel() < 16)
    {
        servo.attach(args[1], getCurrentChannel(), 0, 180, args[2], args[3]);
        servo.write(args[4]);
        ledcChannel[0] = getCurrentChannel();
        nextCurrentChannel();
    }

#else

    servo.attach(args[1], args[2], args[3]);
    servo.write(args[4]);

#endif
#endif
}

void servo_exec(uint32_t *execArgs)
{
    servo.write((execArgs[0] < 0) ? 0 : (execArgs[0] > 180) ? 180
                                                            : execArgs[0]);
}

void servo_off()
{
    servo.detach();
}
