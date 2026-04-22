
#include <stdint.h>

int declarateComponents(uint8_t *currentValue, VolatileMemory *volatileMemory);
int executeComponent(uint8_t *currentValue, VolatileMemory *volatileMemory);
void nairdaDebug(uint8_t tempValue,VolatileMemory* volatileMemory);

#if defined(ARDUINO_ARCH_ESP32)
extern volatile bool esp32RebootRequested;
#endif