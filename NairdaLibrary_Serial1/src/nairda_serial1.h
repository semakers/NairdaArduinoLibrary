#ifndef nairda_serial1_h
#define nairda_serial1_h
#include <Arduino.h>
#if defined(ARDUINO_ARCH_AVR)
#include "avr/Servo.h"
#elif defined(ARDUINO_ARCH_SAM)
#include "sam/Servo.h"
#else
#error "This library only supports boards with an AVR or SAM processor."
#endif


void nairdaBegin(long int baudRate);
void nairdaLoop();

#endif
