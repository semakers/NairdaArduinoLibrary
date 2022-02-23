#ifndef nairda_h
#define nairda_h
#include <stdint.h>


#if defined(__AVR_ATmega32U4__) || defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_STM32)
void resetLeonardoMemory();
void resetMemory();
#endif

#if defined(ARDUINO_ARCH_ESP32) 

//void idleAnimation(bool red,bool green,bool blue,bool execute);
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
