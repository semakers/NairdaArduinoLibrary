#ifndef component_h
#define component_h
#include <stdint.h>
#include <Arduino.h>

#if defined(ARDUINO_ARCH_ESP32)
#include "extern_libraries/esp32_servo/ESP32Servo.h"
#else
#include <Servo.h>
#endif

#include "extern_libraries/neo_pixel/Adafruit_NeoPixel.h"

struct component_t{
      uint8_t pins[5] = {0, 0, 0, 0, 0};
      uint8_t values[5] = {0, 0, 0, 0, 0};

      Adafruit_NeoPixel *neopixel;
      Servo *servo;
      void *sonar;
};

void execAct(uint32_t *execArgs, uint8_t type, component_t *component);
uint8_t getSensVal(uint8_t type, component_t *component);
void sendSensVal(uint8_t type,component_t * component);
void off(uint8_t type, component_t *component);
#endif
