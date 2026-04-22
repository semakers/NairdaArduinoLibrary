#include <stdint.h>
#include "virtual_machine/virtual_machine.h"
#include "blue_methods/blue_methods.h"
#include "flash_writer/flash_writer.h"

uint8_t declaratedCommands[] = {endServos, endDC, endLeds, endFrequencies, endNeopixels, endAnalogics, endDigitals,
                                endUltrasonics};

typedef void (*DebugLoad)(VolatileMemory *volatileMemory);
DebugLoad debugLoad[8] = {servoDebugLoad, motorDebugLoad, digitalOutDebugLoad, frequencyDebugLoad,
                          neoPixelDebugLoad, analogicDebugLoad, digitalInDebugLoad, ultrasonicDebugLoad};

uint8_t argsSizeByComponent[] = {7, 3, 1, 1, 2, 1, 1, 2};

uint8_t execArgsSizeByComponent[] = {1, 2, 1, 6, 4};

uint8_t indexArray[COMPONENTS_SIZE];
uint8_t componentId = 0;

void fillIndexArray(VolatileMemory *volatileMemory)
{
    for (int8_t i = 0; i < COMPONENTS_SIZE; i++)
    {
        if (i == 0)
        {
            indexArray[i] = volatileMemory->components[i].size();
        }
        else
        {
            indexArray[i] = volatileMemory->components[i].size() + indexArray[i - 1];
        }
    }
}

int declarateComponents(uint8_t *currentValue, VolatileMemory *volatileMemory)
{
    for (int i = 0; i < COMPONENTS_SIZE; i++)
    {
        if (currentValue[0] == declaratedCommands[i])
        {
            volatileMemory->declaratedComponents[i] = true;
            if (i == COMPONENTS_SIZE - 1)
            {
                volatileMemory->declaratedDescriptor = true;
                fillIndexArray(volatileMemory);
            }
            return 0;
        }
        else if (!volatileMemory->declaratedComponents[i] && currentValue[0] < 100)
        {
            for (int j = 0; j < argsSizeByComponent[i]; j++)
            {
                if (!volatileMemory->declarationboolean[j])
                {
                    volatileMemory->declarationBuffer[j] = currentValue[0];
                    volatileMemory->declarationboolean[j] = true;
                    if (j == argsSizeByComponent[i] - 1)
                    {
                        debugLoad[i](volatileMemory);

                        memset(volatileMemory->declarationboolean, 0, 7);
                    }
                    return 0;
                }
            }
        }
    }
}

int executeComponent(uint8_t *currentValue, VolatileMemory *volatileMemory)
{
    if (volatileMemory->executedComponent == NON_COMPONENT)
    {

        for (int8_t i = 0; i < COMPONENTS_SIZE; i++)
        {
            if (currentValue[0] >= (i == 0 ? 0 : indexArray[i - 1]) && currentValue[0] < indexArray[i] && volatileMemory->components[i].size() > 0)
            {
                componentId = currentValue[0] - ((i == 0) ? 0 : indexArray[i - 1]);
                if (i < ACTUATORS_SIZE)
                {
                    volatileMemory->executedComponent = i;
                }
                else
                {
                    sendSensVal(i, volatileMemory->components[i].get(componentId));
                    return 0;
                }
            }
        }
        return 0;
    }

    for (uint8_t i = 0; i < ACTUATORS_SIZE; i++)
    {
        if (volatileMemory->executedComponent == i)
        {
            for (uint8_t j = 0; j < execArgsSizeByComponent[i]; j++)
            {
                if (!volatileMemory->executionBoolean[j])
                {
                    volatileMemory->executionBuffer[j] = currentValue[0];
                    volatileMemory->executionBoolean[j] = true;
                    if (j == execArgsSizeByComponent[i] - 1)
                    {
                        if (i == SERVO)
                        {
                            volatileMemory->executionBuffer[0] = map(volatileMemory->executionBuffer[0], 0, 99, 0, 180);
                        }
                        execAct(volatileMemory->executionBuffer, i, volatileMemory->components[i].get(componentId));

                        memset(volatileMemory->executionBoolean, false, 7);
                        volatileMemory->executedComponent = NON_COMPONENT;
                    }
                    return 0;
                }
            }

            return 0;
        }
    }
    volatileMemory->executedComponent = NON_COMPONENT;
}

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328__)

#include <avr/eeprom.h>

// Forward declarations de los wrappers de la Jump Table (en NairdaKernel_Proxy.cpp)
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

// Llena bj_page_buf con las 31 direcciones de la Jump Table (word addresses)
static void fillJumpTable(void) {
    for (uint8_t i = 0; i < SPM_PAGESIZE; i++) bj_page_buf[i] = 0xFF;
    uint16_t ptrs[31] = {
        (uint16_t)(void *)jt_setupDigitalOut,    // 0
        (uint16_t)(void *)jt_runDigitalOut,      // 1
        (uint16_t)(void *)jt_setupServo,         // 2
        (uint16_t)(void *)jt_runServo,           // 3
        (uint16_t)(void *)jt_setupMotor,         // 4
        (uint16_t)(void *)jt_runMotor,           // 5
        (uint16_t)(void *)jt_setupNeoPixel,      // 6
        (uint16_t)(void *)jt_runNeoPixel,        // 7
        (uint16_t)(void *)jt_setupFrequency,     // 8
        (uint16_t)(void *)jt_runFrequency,       // 9
        (uint16_t)(void *)jt_setupDigitalIn,     // 10
        (uint16_t)(void *)jt_readDigitalIn,      // 11
        (uint16_t)(void *)jt_setupAnalogic,      // 12
        (uint16_t)(void *)jt_readAnalogic,       // 13
        (uint16_t)(void *)jt_setupUltrasonic,    // 14
        (uint16_t)(void *)jt_readUltrasonic,     // 15
        (uint16_t)(void *)jt_nairdaDelay,        // 16
        (uint16_t)(void *)jt_nairdaRandom,       // 17
        (uint16_t)(void *)jt_nairdaMap,          // 18
        (uint16_t)(void *)jt_stringCompare,      // 19
        (uint16_t)(void *)jt_stringToLower,      // 20
        (uint16_t)(void *)jt_stringToUpper,      // 21
        (uint16_t)(void *)jt_tableCreate,        // 22
        (uint16_t)(void *)jt_tableSet,           // 23
        (uint16_t)(void *)jt_tableGet,           // 24
        (uint16_t)(void *)jt_tableHeight,        // 25
        (uint16_t)(void *)jt_tableWidth,         // 26
        (uint16_t)(void *)jt_tableAddRow,        // 27
        (uint16_t)(void *)jt_tableAddColumn,     // 28
        (uint16_t)(void *)jt_tableRemoveRow,     // 29
        (uint16_t)(void *)jt_tableRemoveColumn,  // 30
    };
    for (uint8_t i = 0; i < 31; i++) {
        bj_page_buf[i * 2]     = ptrs[i] & 0xFF;
        bj_page_buf[i * 2 + 1] = ptrs[i] >> 8;
    }
}

// BootJacker bootloader mode — command-driven, each E/W causes WDT reset.
// Flutter re-enters this mode after each WDT via the boot window.
// Protocol: F, L, E, W, J, R (single-byte commands via BLE/Serial)
static void enterBootloaderMode() {
    uint8_t byte_val;

    // Mark bootloader active in EEPROM (survives WDT resets)
    eeprom_update_byte(BJ_EE_MODE, 0xBB);

    while (true) {
        while (!nextBlueByte(&byte_val));

        switch (byte_val) {
            case 'F':
                // Fill page buffer with 0xFF
                bjPageClear();
                break;

            case 'L': {
                // Load chunk: 'L' offset len [data...]
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
                // Erase page: 'E' addr_lo addr_hi → WDT reset
                uint8_t lo, hi;
                while (!nextBlueByte(&lo));
                while (!nextBlueByte(&hi));
                uint16_t addr = lo | ((uint16_t)hi << 8);
                bjErase(addr);  // noreturn — WDT resets chip
                break;
            }

            case 'W': {
                // Write page_buf to Flash: 'W' addr_lo addr_hi → WDT reset
                uint8_t lo, hi;
                while (!nextBlueByte(&lo));
                while (!nextBlueByte(&hi));
                uint16_t addr = lo | ((uint16_t)hi << 8);
                bjFillWrite(addr);  // noreturn — WDT resets chip
                break;
            }

            case 'J':
                // Fill page_buf with jump table entries
                fillJumpTable();
                break;

            case 'R':
                // Exit bootloader, execute user program
                eeprom_update_byte(BJ_EE_MODE, 0x00);
                if (flashUserProgramValid()) {
                    ((void (*)(void))USER_PROGRAM_WORD_ADDR)();
                }
                return;  // no valid program — fall back to kernel loop
        }
    }
}
#endif

// ── ESP32 Bootloader ────────────────────────────────────────────────────────
#if defined(ARDUINO_ARCH_ESP32)

// Flag global: cuando se pone en true, nairdaLoop debe re-entrar
// a la ventana de boot en vez de seguir en modo intérprete.
volatile bool esp32RebootRequested = false;

static bool esp32NextByte(uint8_t* out) {
    unsigned long timeout = millis() + 5000;
    while (!nextBlueByte(out)) {
        if (millis() > timeout) return false;
        delay(10); // yield al RTOS para que BLE/watchdog no muera
    }
    return true;
}

static void esp32EnterBootloaderMode() {
    static uint8_t dataBuffer[USER_SPACE_SIZE]; // static: BSS, no stack
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

#endif // ARDUINO_ARCH_ESP32

void nairdaDebug(uint8_t currentValue, VolatileMemory *volatileMemory)
{
    if (currentValue == projectInit)
    {
#if defined(ARDUINO_ARCH_ESP32)
        clearVolatileMemory(volatileMemory, true);
        esp32RebootRequested = true;
        esp32AbortUserCode(); // longjmp: sale del user code si está corriendo
#elif defined(__AVR_ATmega32U4__)
        clearVolatileMemory(volatileMemory, true);
#else
        clearVolatileMemory(volatileMemory, true);
        asm volatile("jmp 0");
#endif
    }
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328__)
    else if (currentValue == userBootloader)
    {
        clearVolatileMemory(volatileMemory, true);
        enterBootloaderMode();
    }
#endif
#if defined(ARDUINO_ARCH_ESP32)
    else if (currentValue == userBootloader)
    {
        clearVolatileMemory(volatileMemory, true);
        esp32EnterBootloaderMode();
    }
#endif
    else if (currentValue == versionCommand)
    {

#if defined(ARDUINO_ARCH_ESP32)
        bleWrite(CURRENT_VERSION);
#else

#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)
        Serial1.write(((char)CURRENT_VERSION));
#endif
        Serial.write(((char)CURRENT_VERSION));
#endif
    }
    else if (volatileMemory->declaratedDescriptor == false)
    {
        declarateComponents(&currentValue, volatileMemory);
    }
    else
    {
        executeComponent(&currentValue, volatileMemory);
    }
}
