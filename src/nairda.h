#ifndef nairda_h
#define nairda_h
#include <stdint.h>

#if defined(ARDUINO_ARCH_ESP32)
void nairdaBegin(const char*deviceName);
#else
void nairdaBegin(long int bauds);
#endif
void nairdaLoop();

#endif
