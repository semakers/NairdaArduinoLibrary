#include <stdint.h>

#if defined(ARDUINO_ARCH_STM32)
#include <Servo.h>
#elif defined(ARDUINO_ARCH_ESP32)
#include "extern_libraries/esp32_servo/esp32_servo.h"
#else
#include <Servo.h>
#endif

void servoCreate(uint16_t *args,uint8_t * pins,int8_t* ledcChannel,Servo* servo);
void servoExec(uint32_t *execArgs,Servo* servo);
void servoOff(Servo* servo);
void servoEepromLoad();
void servoEepromRun(uint8_t id);