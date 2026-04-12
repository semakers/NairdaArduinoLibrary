#include <stdint.h>

void setupDigitalOut(component_t *component, int pin);
void runDigitalOut(component_t *component, int value);
void digitalOutExec(uint32_t *execArgs, uint8_t *pins, uint8_t *values, int8_t *ledcChannel);
void digitalOutOff(uint8_t *pins, int8_t *ledcChannel);
void digitalOutEepromLoad(VolatileMemory *volatileMemory);
void digitalOutEepromRun(uint8_t id, VolatileMemory *volatileMemory);
void digitalOutDebugLoad(VolatileMemory *volatileMemory);