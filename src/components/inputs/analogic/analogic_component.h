#include <stdint.h>

void analogicCreate(uint16_t *args,uint8_t * pins);
void analogicSense(uint8_t * pins,uint8_t* tempRead);
void analogicOff();
void analogicEepromLoad();
int32_t analogicEepromRead();