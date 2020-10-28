#ifndef nairda_h
#define nairda_h
#include <stdint.h>

#ifdef __AVR_ATmega32U4__

void resetLeonardoMemory();
void resetMemory();

#endif


void nairdaBegin(long int bauds);
void nairdaLoop();

#endif
