#include "nairda.h"
#include "virtual_machine/virtual_machine.h"
#include "blue_methods/blue_methods.h"
#include "nairda_debug/nairda_debug.h"
#include "platform/platform_hal.h"
#include "kits/v1.h"

uint8_t currentValue;
VolatileMemory volatileMemory;
uint8_t currentKit = NO_KIT;

void setKit(uint8_t kitCode)
{
    currentKit = kitCode;
}

void nairdaDelay(unsigned long ms)
{
    unsigned long start = millis();
    while (millis() - start < ms) {
        nairdaLoop();
    }
}

void nairdaLoop()
{
#if defined(KIT_V1_ENABLED)
    writeKitDisplay();
#endif

    if (hal_checkRebootRequest()) return;

    if (nextBlueByte(&currentValue) == true)
    {
        nairdaDebug(currentValue, &volatileMemory);
    }
}
