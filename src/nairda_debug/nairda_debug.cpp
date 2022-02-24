#include <stdint.h>
#include "load_from_eeprom.h"
#include "volatile_memory/volatile_memory.h"

#include "components/outputs/servo/servo_component.h"
#include "components/outputs/motor/motor_component.h"
#include "components/outputs/digital_out/digital_out_component.h"
#include "components/outputs/frequency/frequency_component.h"
#include "components/outputs/neo_pixel/neo_pixel_component.h"

#include "components/inputs/analogic/analogic_component.h"
#include "components/inputs/digital_in/digital_in_component.h"
#include "components/inputs/ultrasonic/ultrasonic_component.h"

uint8_t declaratedCommands[] = {endServos, endDC, endLeds, endFrequencies, endNeopixels, endAnalogics, endDigitals,
                                endUltrasonics};

void (*debugLoad)(VolatileMemory *)[] = {servoDebugLoad, motorDebugLoad, digitalOutDebugLoad, frequencyDebugLoad,
                                         neoPixelDebugLoad, analogicDebugLoad, digitalInDebugLoad, ultrasonicDebugLoad};

uint8_t argsSizeByComponent[] = {7, 3, 1, 1, 2, 1, 1, 2};

void declaratedComponents(uint8_t currentValue, VolatileMemory *volatileMemory)
{
    volatileMemory->declaratedDescriptor == true;
    for (int i = 0; i < COMPONENTS_SIZE; i++)
    {
        if (currentValue == declaratedCommands[i])
        {
            volatileMemory->declaratedComponents[i] = true;
        }
        else if (!volatileMemory->declaratedComponents[i])
        {
            volatileMemory->declaratedDescriptor = false for (int j = 0; j < argsSizeByComponent[i]; j++)
            {
                if (!volatileMemory->declarationboolean[j])
                {
                    volatileMemory->declarationBuffer[j] = currentValue;
                    volatileMemory->declarationboolean[j] = true;
                    if (j == argsSizeByComponent[i] - 1)
                    {
                        debugLoad[i](volatileMemory);
                    }
                }
            }
        }
    }
}
