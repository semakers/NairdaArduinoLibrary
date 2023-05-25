#ifndef nairda_h
#define nairda_h
#include <stdint.h>

#define NO_KIT 0
#define ROBUS_KIDSY_KIT 1

#if defined(ARDUINO_ARCH_ESP32)
void nairdaBegin(const char *deviceName);
#else
void nairdaBegin(long int bauds);
#endif
void setKit(uint8_t kitCode);
void nairdaLoop();
#endif
