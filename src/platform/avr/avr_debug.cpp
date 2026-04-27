#if !defined(ARDUINO_ARCH_ESP32)

#include <Arduino.h>
#include "virtual_machine/virtual_machine.h"
#include "blue_methods/blue_methods.h"
#include "flash_writer/flash_writer.h"
#include "platform/platform_hal.h"
#include "nairda.h"

// ── AVR BootJacker protocol (ATmega328P only) ──────────────────────
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328__)

extern "C" {
    void jt_setupDigitalOut(void*, int);
    void jt_runDigitalOut(void*, int);
    void jt_setupServo(void*, int, int, int, int);
    void jt_runServo(void*, int);
    void jt_setupMotor(void*, int, int, int);
    void jt_runMotor(void*, int, int);
    void jt_setupNeoPixel(void*, int, int);
    void jt_runNeoPixel(void*, int, int, int, int);
    void jt_setupFrequency(void*, int);
    void jt_runFrequency(void*, int, int, int);
    void jt_setupDigitalIn(void*, int);
    uint8_t jt_readDigitalIn(void*);
    void jt_setupAnalogic(void*, int);
    uint8_t jt_readAnalogic(void*);
    void jt_setupUltrasonic(void*, int, int);
    uint8_t jt_readUltrasonic(void*);
    void jt_nairdaDelay(unsigned long);
    long jt_nairdaRandom(long, long);
    long jt_nairdaMap(long, long, long, long, long);
    int jt_stringCompare(const char*, const char*);
    void jt_stringToLower(const char*, char*, int);
    void jt_stringToUpper(const char*, char*, int);
    void jt_tableCreate(void*, int, int, int32_t);
    void jt_tableSet(void*, int, int, int32_t);
    int32_t jt_tableGet(void*, int, int);
    int jt_tableHeight(void*);
    int jt_tableWidth(void*);
    void jt_tableAddRow(void*);
    void jt_tableAddColumn(void*);
    void jt_tableRemoveRow(void*);
    void jt_tableRemoveColumn(void*);
}

static void fillJumpTable(void) {
    for (uint8_t i = 0; i < SPM_PAGESIZE; i++) bj_page_buf[i] = 0xFF;
    uint16_t ptrs[31] = {
        (uint16_t)(void *)jt_setupDigitalOut,
        (uint16_t)(void *)jt_runDigitalOut,
        (uint16_t)(void *)jt_setupServo,
        (uint16_t)(void *)jt_runServo,
        (uint16_t)(void *)jt_setupMotor,
        (uint16_t)(void *)jt_runMotor,
        (uint16_t)(void *)jt_setupNeoPixel,
        (uint16_t)(void *)jt_runNeoPixel,
        (uint16_t)(void *)jt_setupFrequency,
        (uint16_t)(void *)jt_runFrequency,
        (uint16_t)(void *)jt_setupDigitalIn,
        (uint16_t)(void *)jt_readDigitalIn,
        (uint16_t)(void *)jt_setupAnalogic,
        (uint16_t)(void *)jt_readAnalogic,
        (uint16_t)(void *)jt_setupUltrasonic,
        (uint16_t)(void *)jt_readUltrasonic,
        (uint16_t)(void *)jt_nairdaDelay,
        (uint16_t)(void *)jt_nairdaRandom,
        (uint16_t)(void *)jt_nairdaMap,
        (uint16_t)(void *)jt_stringCompare,
        (uint16_t)(void *)jt_stringToLower,
        (uint16_t)(void *)jt_stringToUpper,
        (uint16_t)(void *)jt_tableCreate,
        (uint16_t)(void *)jt_tableSet,
        (uint16_t)(void *)jt_tableGet,
        (uint16_t)(void *)jt_tableHeight,
        (uint16_t)(void *)jt_tableWidth,
        (uint16_t)(void *)jt_tableAddRow,
        (uint16_t)(void *)jt_tableAddColumn,
        (uint16_t)(void *)jt_tableRemoveRow,
        (uint16_t)(void *)jt_tableRemoveColumn,
    };
    for (uint8_t i = 0; i < 31; i++) {
        bj_page_buf[i * 2]     = ptrs[i] & 0xFF;
        bj_page_buf[i * 2 + 1] = ptrs[i] >> 8;
    }
}

static void enterBootloaderMode() {
    uint8_t byte_val;

    bj_mode_magic = BJ_MAGIC_ACTIVE;

    while (true) {
        while (!nextBlueByte(&byte_val));

        switch (byte_val) {
            case 'F':
                bjPageClear();
                break;

            case 'L': {
                uint8_t off, len;
                while (!nextBlueByte(&off));
                while (!nextBlueByte(&len));
                if (len > CHUNK_MAX) break;
                uint8_t tmp[CHUNK_MAX];
                for (uint8_t i = 0; i < len; i++) {
                    while (!nextBlueByte(&byte_val));
                    tmp[i] = byte_val;
                }
                bjPageLoad(off, tmp, len);
                break;
            }

            case 'E': {
                uint8_t lo, hi;
                while (!nextBlueByte(&lo));
                while (!nextBlueByte(&hi));
                uint16_t addr = lo | ((uint16_t)hi << 8);
                bjErase(addr);
                break;
            }

            case 'W': {
                uint8_t lo, hi;
                while (!nextBlueByte(&lo));
                while (!nextBlueByte(&hi));
                uint16_t addr = lo | ((uint16_t)hi << 8);
                bjFillWrite(addr);
                break;
            }

            case 'J':
                fillJumpTable();
                break;

            case 'R':
                bj_mode_magic = 0;
                if (flashUserProgramValid()) {
                    ((void (*)(void))USER_PROGRAM_WORD_ADDR)();
                }
                return;
        }
    }
}

#endif // __AVR_ATmega328P__

// ── HAL implementations ────────────────────────────────────────────

void hal_handleProjectInit(VolatileMemory *volatileMemory)
{
    clearVolatileMemory(volatileMemory, true);
    asm volatile("jmp 0");
}

void hal_handleBootloader(VolatileMemory *volatileMemory)
{
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328__)
    clearVolatileMemory(volatileMemory, true);
    enterBootloaderMode();
#endif
}

void hal_handleVersionCommand()
{
    hal_sendByte(CURRENT_VERSION);
}

bool hal_checkRebootRequest()
{
    return false;
}

#endif
