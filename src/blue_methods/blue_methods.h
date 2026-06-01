#include <stdint.h>

#if defined(ARDUINO_ARCH_ESP32)
void bleWrite(uint8_t byte);
bool bleAvailable();
uint8_t bleRead();
void bleInit(const char *deviceName);
// Consume any deferred BLE actions (e.g. restart-advertising after disconnect).
// Must be called from the main task, not from a BLE callback.
void blePollActions();
#endif

bool nextBlueByte(uint8_t *blueByte);
