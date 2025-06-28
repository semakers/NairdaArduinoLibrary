#include <stdint.h>
#include <Arduino.h>

struct component_t
{
      int8_t ledcChannel[2] = {0, 0};
      uint8_t pins[5] = {0, 0, 0, 0, 0};
      uint8_t values[5] = {0, 0, 0, 0, 0};
};

void execAct(uint32_t *execArgs, uint8_t type, component_t *component);
uint8_t getSensVal(uint8_t type, component_t *component);
void sendSensVal(uint8_t type, component_t *component);
void off(uint8_t type, component_t *component);