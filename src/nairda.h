#ifndef nairda_h
#define nairda_h
#include <stdint.h>

#define NO_KIT 0
#define ROBBUS_KIDSY_KIT 1
#define LK32_KIT 2
#define ROBBUS_ZEEGO_KIT 3

#if defined(ARDUINO_ARCH_ESP32)
void nairdaBegin(const char *deviceName);
#else
void nairdaBegin(long int bauds);
#endif
void setKit(uint8_t kitCode);
void nairdaLoop();
#endif
