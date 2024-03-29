#include <stdint.h>

#if !defined(ARDUINO_ARCH_STM32) && !defined(ARDUINO_ARCH_ESP32)
#include "extern_libraries/new_ping/new_ping.h"

void ultrasonicSense(uint8_t * pins,uint8_t* tempRead, NewPing *sonar);
void ultrasonicOff( NewPing *sonar);
#else

void ultrasonicSense(uint8_t * pins,uint8_t* tempRead);
void ultrasonicOff();
#endif


void ultrasonicEepromLoad(VolatileMemory *volatileMemory);
int32_t ultrasonicEepromRead(VolatileMemory *volatileMemory);
void ultrasonicDebugLoad(VolatileMemory *volatileMemory);