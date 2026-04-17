#include <stdint.h>

void setupFrequency(component_t *component, int pin);
void runFrequency(component_t *component, int frequency, int duration, int volume);
void frequencyExec(uint32_t *execArgs,uint8_t * pins);
void frequencyOff();
void frequencyDebugLoad(VolatileMemory *volatileMemory);