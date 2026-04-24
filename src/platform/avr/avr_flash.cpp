#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328__)

#include "flash_writer/flash_writer.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include <string.h>

#define GADGET_ERASE_W       0x3F99
#define GADGET_FILL_WRITE_W  0x3F80

volatile uint8_t bj_page_buf[SPM_PAGESIZE] __attribute__((section(".noinit")));

void bjPageClear(void) {
    for (uint8_t i = 0; i < SPM_PAGESIZE; i++) bj_page_buf[i] = 0xFF;
}

void bjPageLoad(uint8_t offset, const uint8_t *data, uint8_t len) {
    for (uint8_t i = 0; i < len && (offset + i) < SPM_PAGESIZE; i++)
        bj_page_buf[offset + i] = data[i];
}

void bjErase(uint16_t addr) {
    cli();
    wdt_enable(WDTO_15MS);
    asm volatile(
        "mov r12, %A[a] \n\t" "mov r13, %B[a] \n\t"
        "ldi r30, 0x03  \n\t" "mov r10, r30   \n\t"
        "ldi r30, 0x11  \n\t" "mov r9,  r30   \n\t"
        "ldi r30, lo8(%[g]) \n\t" "ldi r31, hi8(%[g]) \n\t"
        "ijmp \n\t"
        : : [a] "r" (addr), [g] "n" (GADGET_ERASE_W)
    );
    __builtin_unreachable();
}

void bjFillWrite(uint16_t addr) {
    cli();
    for (uint8_t i = 0; i < SPM_PAGESIZE; i++)
        ((volatile uint8_t *)OPTIBOOT_BUF)[i] = bj_page_buf[i];
    wdt_enable(WDTO_15MS);
    asm volatile(
        "mov r12, %A[a] \n\t" "mov r13, %B[a] \n\t"
        "ldi r30, 0x01  \n\t" "mov r8,  r30   \n\t"
        "ldi r30, 0x05  \n\t" "mov r10, r30   \n\t"
        "ldi r30, 0x11  \n\t" "mov r9,  r30   \n\t"
        "ldi r30, lo8(%[g]) \n\t" "ldi r31, hi8(%[g]) \n\t"
        "ijmp \n\t"
        : : [a] "r" (addr), [g] "n" (GADGET_FILL_WRITE_W)
    );
    __builtin_unreachable();
}

bool flashUserProgramValid(void) {
    return pgm_read_byte(USER_SPACE_ADDR) == USER_FLAG_VALID;
}

#endif
