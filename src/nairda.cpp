#include "nairda.h"
#include "virtual_machine/virtual_machine.h"
#include "blue_methods/blue_methods.h"
#include "nairda_debug/nairda_debug.h"
#include "platform/platform_hal.h"
#include "kits/v1.h"
#include "nairda_log.h"

uint8_t currentValue;
VolatileMemory volatileMemory;
uint8_t currentKit = NO_KIT;

void setKit(uint8_t kitCode)
{
    currentKit = kitCode;
}

void nairdaDelay(unsigned long ms)
{
    NRD_LOG_THROTTLED(500, "[JT] nairdaDelay ms=%lu\n", ms);
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

#if defined(ARDUINO_ARCH_ESP32)
    // Consume any deferred BLE actions queued by the BLE task callbacks
    // (e.g. re-start advertising after a disconnect). Must run on the main
    // task — see blePollActions docs.
    blePollActions();
#endif

    if (nextBlueByte(&currentValue) == true)
    {
        nairdaDebug(currentValue, &volatileMemory);
    }
}
