#include <stdint.h>
#include "volatile_memory/volatile_memory.h"

#if !defined(ARDUINO_ARCH_STM32) && !defined(ARDUINO_ARCH_ESP32)
#include "extern_libraries/new_ping/new_ping.h"
void ultrasonicCreate(uint16_t *args,uint8_t * pins, NewPing *sonar);
void ultrasonicSense(uint8_t * pins,uint8_t* tempRead, NewPing *sonar);
void ultrasonicOff( NewPing *sonar);
#else
void ultrasonicCreate(uint16_t *args,uint8_t * pins);
void ultrasonicSense(uint8_t * pins,uint8_t* tempRead);
void ultrasonicOff();
#endif


void ultrasonicEepromLoad();
int32_t ultrasonicEepromRead();
void ultrasonicDebugLoad(VolatileMemory *volatileMemory);