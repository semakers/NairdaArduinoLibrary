#include <stdint.h>

void setupDigitalOut(component_t *component, int pin);
void runDigitalOut(component_t *component, int value);
void digitalOutCreate(uint16_t *args, component_t *component);
void digitalOutExec(uint32_t *execArgs, uint8_t *pins, uint8_t *values);
void digitalOutOff(uint8_t *pins);
void digitalOutDebugLoad(VolatileMemory *volatileMemory);
