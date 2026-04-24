#if !defined(ARDUINO_ARCH_ESP32)

#include <Arduino.h>
#include "platform/platform_hal.h"

bool nextBlueByte(uint8_t *blueByte)
{
    if (Serial.available())
    {
        blueByte[0] = Serial.read();
        return true;
    }
    return false;
}

void hal_sendByte(uint8_t byte)
{
    Serial.write(byte);
}

#endif
