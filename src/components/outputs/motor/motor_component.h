#include <stdint.h>

void setupMotor(component_t *component, int pin1, int pin2, int pinSpeed);
void runMotor(component_t *component, int speed, int direction);
void motorExec(uint32_t *execArgs, uint8_t *pins, uint8_t *values);
void motorOff(uint8_t *pins);
void motorDebugLoad(VolatileMemory *volatileMemory);
