#if !defined(ARDUINO_ARCH_ESP32)

#include <Arduino.h>
#include "volatile_memory/volatile_memory.h"
#include "virtual_machine/virtual_machine.h"
#include "blue_methods/blue_methods.h"
#include "nairda_debug/nairda_debug.h"
#include "flash_writer/flash_writer.h"
#include "nairda.h"
#include "extern_libraries/soft_pwm/soft_pwm.h"

#if defined(_24LC_256) || defined(_24LC_512)
#include <Wire.h>
#endif

#include <avr/eeprom.h>
#include <avr/pgmspace.h>

extern uint8_t currentValue;
extern VolatileMemory volatileMemory;

void nairdaBegin(long int bauds)
{
#if defined(_24LC_256) || defined(_24LC_512)
    Wire.begin();
#endif
    Serial.begin(bauds);
    SoftPWMBegin();

    initVolatileMemory(&volatileMemory);

    {
        uint8_t bjMode = eeprom_read_byte(BJ_EE_MODE);
        if (bjMode == 0xBB) {
            nairdaDebug(userBootloader, &volatileMemory);
            return;
        }
    }

    unsigned long bootStart = millis();
    while (millis() - bootStart < 2000) {
        if (nextBlueByte(&currentValue)) {
            nairdaDebug(currentValue, &volatileMemory);
            return;
        }
    }

    uint8_t flag = pgm_read_byte(USER_SPACE_ADDR);
    if (flag == USER_FLAG_VALID) {
        Serial.flush();
        ((void (*)(void))USER_PROGRAM_WORD_ADDR)();
    }
}

#endif
