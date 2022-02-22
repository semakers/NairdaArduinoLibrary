#include <stdint.h>

void digitalOutCreate(uint16_t *args,uint8_t * pins,int8_t* ledcChannel);
void digitalOutExec(uint32_t *execArgs,uint8_t * pins,uint8_t* values,int8_t* ledcChannel);
void digitalOutOff(uint8_t * pins,int8_t *ledcChannel);
void digitalOutEepromLoad();
void digitalOutEepromRun(uint8_t id);