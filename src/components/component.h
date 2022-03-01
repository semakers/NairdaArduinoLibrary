#include <stdint.h>
#include <Arduino.h>

#if defined(ARDUINO_ARCH_STM32)
#include <Servo.h>
#elif defined(ARDUINO_ARCH_ESP32)
#include "extern_libraries/esp32_servo/esp32_servo.h"
#else
#include "extern_libraries/new_ping/new_ping.h"
#include <Servo.h>
#endif

#include "extern_libraries/neo_pixel/Adafruit_NeoPixel.h"

struct component_t{
      int8_t* ledcChannel= (int8_t*)malloc(sizeof(int8_t));
      uint8_t pins[5] = {0, 0, 0, 0, 0};
      uint8_t values[5] = {0, 0, 0, 0, 0};

      Adafruit_NeoPixel *neopixel;
      Servo *servo;
      #if !defined(ARDUINO_ARCH_STM32) && !defined(ARDUINO_ARCH_ESP32)
      NewPing *sonar;
#endif
};

void execAct(uint32_t *execArgs, uint8_t type, component_t *component);
uint8_t getSensVal(uint8_t type, component_t *component);
void sendSensVal(uint8_t type,component_t * component);
void off(uint8_t type, component_t *component);