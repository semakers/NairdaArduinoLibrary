#include <stdint.h>

void servo_create(uint16_t *args,uint8_t * pins,int8_t* ledcChannel);
void servo_exec(uint32_t *execArgs);
void servo_off();