#include <stdint.h>

void setupAnalogic(component_t *component, int pin);
uint8_t readAnalogic(component_t *component);
void analogicCreate(uint16_t *args, component_t *component);
void analogicSense(uint8_t *pins, uint8_t *tempRead);
void analogicOff();
void analogicDebugLoad(VolatileMemory *volatileMemory);
