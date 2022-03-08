#include <stdint.h>
#include "extern_libraries/linked_list/linked_list.h"
#include "components/component.h"

#define COMPONENTS_SIZE 8
#define ACTUATORS_SIZE 5
#define NON_COMPONENT -1

enum
{
      SERVO = 0,
      MOTOR,
      DIGITAL_OUT,
      FREQUENCY,
      NEOPIXEL,
        ANALOGIC,
      DIGITAL_IN,
     
      ULTRASONIC
};

struct VolatileMemory
{
    LinkedList<component_t *> components[COMPONENTS_SIZE];
    

    bool declaratedDescriptor = false;

    bool declaratedComponents[8];
    int8_t executedComponent=NON_COMPONENT;

    bool executeActuator[5];

    bool executionBoolean[7];
    uint32_t executionBuffer[7];

    bool declarationboolean[7];
    uint8_t declarationBuffer[7];

    uint16_t descArgsBuffer[5]={0,0,0,0,0};
    uint32_t execBuffer[6];

    uint8_t currentChannel = 0;
};

void clearVolatileMemory(VolatileMemory *volatileMemory, bool offComonents);

void initVolatileMemory(VolatileMemory* volatileMemory);