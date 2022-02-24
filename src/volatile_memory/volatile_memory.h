#include <stdint.h>
#include "extern_libraries/linked_list/linked_list.h"
#include "components/component.h"

#define COMPONENTS_SIZE 8

enum
{
      SERVO = 0,
      MOTOR,
      DIGITAL_OUT,
      FREQUENCY,
      NEOPIXEL,
      DIGITAL_IN,
      ANALOGIC,
      ULTRASONIC
};

struct VolatileMemory
{
    LinkedList<component_t *> components[COMPONENTS_SIZE];
    

    bool declaratedDescriptor = false;

    bool declaratedComponents[8];

    bool executeActuator[5];

    bool executionBoolean[7];
    uint8_t executionBuffer[7];

    bool declarationboolean[7];
    uint8_t declarationBuffer[7];

    uint16_t descArgsBuffer[5];
    uint32_t execBuffer[6];

    uint8_t currentChannel = 0;
}

void clearVolatileMemory(VolatileMemory *volatileMemory, bool offComonents);

void initVolatileMemory(VolatileMemory* volatileMemory);