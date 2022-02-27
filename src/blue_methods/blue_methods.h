#include <stdint.h>

#if defined(ARDUINO_ARCH_ESP32)
void bleWrite(uint8_t byte);
bool bleAvailable();
uint8_t bleRead();
void bleInit(const char *deviceName);
#endif


void sendMemorySize(uint32_t memorySize);
bool nextBlueByte(uint8_t *blueByte);