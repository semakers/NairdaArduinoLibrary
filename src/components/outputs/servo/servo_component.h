#include <stdint.h>

#if defined(ARDUINO_ARCH_STM32)
#include <Servo.h>
#elif defined(ARDUINO_ARCH_ESP32)
#include "extern_libraries/esp32_servo/ESP32Servo.h"
#else
#include <Servo.h>
#endif

void setupServo(component_t *component, int pin, int minPulse, int maxPulse, int initialAngle);
void runServo(component_t *component, int angle);
void servoExec(uint32_t *execArgs,Servo* servo);
void servoOff(Servo* servo);
void servoEepromLoad(VolatileMemory* volatileMemory);
void servoEepromRun(uint8_t id,VolatileMemory* volatileMemory);
void servoDebugLoad(VolatileMemory* volatileMemory);