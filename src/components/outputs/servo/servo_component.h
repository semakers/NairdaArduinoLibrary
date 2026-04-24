#include <stdint.h>

void setupServo(component_t *component, int pin, int minPulse, int maxPulse, int initialAngle);
void runServo(component_t *component, int angle);
void servoExec(uint32_t *execArgs, Servo *servo);
void servoOff(Servo *servo);
void servoDebugLoad(VolatileMemory *volatileMemory);
