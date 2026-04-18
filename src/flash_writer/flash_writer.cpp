#include "flash_writer.h"

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328__)

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <string.h>

uint8_t flashWritePage(uint16_t target_addr, const uint8_t *data) {
    uint8_t sreg = SREG;
    cli();

    // 1. Page Erase
    DO_SPM(target_addr, SPM_PAGE_ERASE, 0);

    // 2. Page Fill (64 words = 128 bytes)
    for (uint16_t i = 0; i < SPM_PAGESIZE; i += 2) {
        uint16_t word = data[i] | ((uint16_t)data[i + 1] << 8);
        DO_SPM(target_addr + i, SPM_PAGE_FILL, word);
    }

    // 3. Page Write
    DO_SPM(target_addr, SPM_PAGE_WRITE, 0);

    // 4. RWW Enable
    DO_SPM(0, SPM_RWW_ENABLE, 0);

    SREG = sreg;

    // 5. Verificar
    for (uint16_t i = 0; i < SPM_PAGESIZE; i++) {
        if (pgm_read_byte(target_addr + i) != data[i]) {
            return 0;
        }
    }
    return 1;
}

bool flashUserProgramValid(void) {
    return pgm_read_byte(USER_SPACE_ADDR) == USER_FLAG_VALID;
}

#endif
