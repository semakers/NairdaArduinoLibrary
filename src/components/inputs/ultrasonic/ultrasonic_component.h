#include <stdint.h>

void ultrasonicCreate(uint16_t *args, component_t *component);
void ultrasonicSenseImpl(component_t *component, uint8_t *tempRead);
void ultrasonicOffImpl(component_t *component);

void setupUltrasonic(component_t *component, int triggerPin, int echoPin);
uint8_t readUltrasonic(component_t *component);
void ultrasonicDebugLoad(VolatileMemory *volatileMemory);
