#if defined(ARDUINO_ARCH_ESP32)

#include <Arduino.h>
#include "virtual_machine/virtual_machine.h"
#include "blue_methods/blue_methods.h"
#include "flash_writer/flash_writer.h"
#include "platform/platform_hal.h"
#include "nairda.h"

extern VolatileMemory volatileMemory;

static volatile bool esp32RebootRequested = false;

// ── ESP32 Bootloader Mode ──────────────────────────────────────────

static bool esp32NextByte(uint8_t* out) {
    unsigned long timeout = millis() + 5000;
    while (!nextBlueByte(out)) {
        if (millis() > timeout) return false;
        delay(10);
    }
    return true;
}

static void esp32EnterBootloaderMode() {
    static uint8_t dataBuffer[USER_SPACE_SIZE];
    uint16_t totalBytes = 0;
    uint8_t chunkData[CHUNK_MAX];
    uint8_t byte_val;

    while (true) {
        if (!esp32NextByte(&byte_val)) return;
        uint8_t len = byte_val;

        if (!esp32NextByte(&byte_val)) return;
        uint8_t expected_checksum = byte_val;

        if (len == 0 && expected_checksum == 0) break;
        if (len > CHUNK_MAX) continue;

        uint8_t checksum_calc = 0;
        for (uint8_t i = 0; i < len; i++) {
            if (!esp32NextByte(&byte_val)) return;
            chunkData[i] = byte_val;
            checksum_calc += byte_val;
        }
        checksum_calc = checksum_calc % 64;

        if (checksum_calc != expected_checksum) continue;

        if (totalBytes + len <= USER_SPACE_SIZE) {
            memcpy(dataBuffer + totalBytes, chunkData, len);
            totalBytes += len;
        } else {
            break;
        }
    }

    if (totalBytes > 0) {
        esp32FlashWriteUserProgram(dataBuffer, totalBytes);
        esp32ExecuteUserCode();
    }
}

// ── HAL implementations ────────────────────────────────────────────

void hal_handleProjectInit(VolatileMemory *volatileMemory)
{
    clearVolatileMemory(volatileMemory, true);
    esp32RebootRequested = true;
    esp32AbortUserCode();
}

void hal_handleBootloader(VolatileMemory *volatileMemory)
{
    clearVolatileMemory(volatileMemory, true);
    esp32EnterBootloaderMode();
}

void hal_handleVersionCommand()
{
    hal_sendByte(CURRENT_VERSION);
}

bool hal_checkRebootRequest()
{
    if (esp32RebootRequested) {
        esp32RebootRequested = false;
        esp32BootWindow();
        return true;
    }
    return false;
}

#endif
