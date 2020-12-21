#ifndef nairda_h
#define nairda_h
#include <stdint.h>

#if defined(__AVR_ATmega32U4__) || defined(ARDUINO_ARCH_ESP32)

void resetLeonardoMemory();
void resetMemory();
void bleWrite(uint8_t byte);
bool bleAvailable();
uint8_t bleRead();

#endif


#if defined(ARDUINO_ARCH_ESP32)
void nairdaBegin(const char*deviceName);
#else
void nairdaBegin(long int bauds);
#endif
void nairdaLoop();

#endif
