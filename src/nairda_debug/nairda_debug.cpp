#include <stdint.h>
#include "virtual_machine/virtual_machine.h"
#include "blue_methods/blue_methods.h"

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328__)
#include "flash_writer/flash_writer.h"
#endif

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
// Modo Bootloader: recibe chunks por BLE/Serial y los escribe en Flash.
// Protocolo: [len][checksum][datos...] — terminador [0][0]
// Comunicación unidireccional (no envía respuestas).
static void enterBootloaderMode() {
    uint8_t page_buffer[SPM_PAGESIZE];
    uint8_t first_page_copy[SPM_PAGESIZE];
    uint16_t current_addr = USER_SPACE_ADDR;
    uint8_t chunk_half = 0;  // 0 = primera mitad, 1 = segunda mitad
    uint8_t byte_val;

    while (true) {
        // Esperar len
        while (!nextBlueByte(&byte_val));
        uint8_t len = byte_val;

        // Esperar checksum
        while (!nextBlueByte(&byte_val));
        uint8_t expected_checksum = byte_val;

        // Paquete terminador [0][0]
        if (len == 0 && expected_checksum == 0) {
            if (chunk_half == 1) {
                // Página incompleta: rellenar segunda mitad con 0xFF
                memset(page_buffer + 64, 0xFF, 64);
                if (current_addr == USER_SPACE_ADDR) {
                    page_buffer[0] = 0x00;  // Flag inválido
                    memcpy(first_page_copy + 64, page_buffer + 64, 64);
                }
                flashWritePage(current_addr, page_buffer);
            }
            break;
        }

        if (len > 64) continue;  // Chunk inválido, ignorar

        // Recibir datos del chunk
        uint8_t offset = chunk_half * 64;
        uint8_t checksum_calc = 0;
        for (uint8_t i = 0; i < len; i++) {
            while (!nextBlueByte(&byte_val));
            page_buffer[offset + i] = byte_val;
            checksum_calc += byte_val;
        }
        // Rellenar resto con 0xFF si len < 64
        if (len < 64) {
            memset(page_buffer + offset + len, 0xFF, 64 - len);
        }

        // Validar checksum
        checksum_calc = checksum_calc % 64;
        if (checksum_calc != expected_checksum) {
            continue;  // Chunk corrupto, descartar
        }

        // Primera página: forzar flag inválido y guardar copia
        if (current_addr == USER_SPACE_ADDR) {
            if (chunk_half == 0) {
                page_buffer[0] = 0x00;
                memcpy(first_page_copy, page_buffer, 64);
            } else {
                memcpy(first_page_copy + 64, page_buffer + 64, 64);
            }
        }

        chunk_half++;
        if (chunk_half == 2) {
            // Página completa: escribir a Flash
            flashWritePage(current_addr, page_buffer);
            current_addr += SPM_PAGESIZE;
            chunk_half = 0;
        }
    }

    // Reescribir primera página con flag válido
    first_page_copy[0] = USER_FLAG_VALID;
    flashWritePage(USER_SPACE_ADDR, first_page_copy);

    // Reset por software
    asm volatile("jmp 0");
}
#endif

void nairdaDebug(uint8_t currentValue, VolatileMemory *volatileMemory)
{
    if (currentValue == projectInit)
    {
#if defined(__AVR_ATmega32U4__) || (ARDUINO_ARCH_ESP32)
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
        // No retorna — hace jmp 0 al final
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
