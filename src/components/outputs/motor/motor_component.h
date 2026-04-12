#include <stdint.h>

void setupMotor(component_t *component, int pin1, int pin2, int pinSpeed);
void runMotor(component_t *component, int speed, int direction);
void motorExec(uint32_t *execArgs,uint8_t * pins,uint8_t* values,int8_t* ledcChannel);
void motorOff(uint8_t * pins,int8_t *ledcChannel);
void motorEepromLoad(VolatileMemory* volatileMemory);
void motorEepromRun(uint8_t id,VolatileMemory* volatileMemory);
void motorDebugLoad(VolatileMemory *volatileMemory);