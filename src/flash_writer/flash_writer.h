#ifndef FLASH_WRITER_H
#define FLASH_WRITER_H

#include <stdint.h>

// ── Constantes compartidas AVR / ESP32 ──────────────────────────────
#define USER_FLAG_VALID   0x01
#define USER_SPACE_SIZE   8192
#define CHUNK_MAX         64

bool flashUserProgramValid(void);

// ── AVR (ATmega328P) — BootJacker ROP on Optiboot 4.4 ──────────────
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328__)

#include <avr/pgmspace.h>

// SRAM address where Optiboot's fill+write gadget reads page data
#define OPTIBOOT_BUF      0x0100

// Flash memory layout
#define JUMP_TABLE_ADDR   0x5F80
#define USER_SPACE_ADDR   0x6000
#define USER_PROGRAM_BYTE_ADDR  0x6002
#define USER_PROGRAM_WORD_ADDR  0x3001

// .noinit magic — survives jmp 0 / WDT reset, lost on power cycle / reflash
#define BJ_MAGIC_ACTIVE  0xBEEF
extern volatile uint16_t bj_mode_magic;

// Page buffer (volatile .noinit — survives WDT reset)
extern volatile uint8_t bj_page_buf[SPM_PAGESIZE];

// BootJacker API
void bjPageClear(void);                              // fill page_buf with 0xFF
void bjPageLoad(uint8_t offset, const uint8_t *data, uint8_t len);  // load chunk
void bjErase(uint16_t addr);                         // erase page → WDT reset (noreturn)
void bjFillWrite(uint16_t addr);                     // write page_buf to Flash → WDT reset (noreturn)

#endif // __AVR_ATmega328P__

// ── ESP32 ───────────────────────────────────────────────────────────
#if defined(ARDUINO_ARCH_ESP32)

#include "esp_partition.h"
#include "esp_heap_caps.h"

#define ESP32_JUMP_TABLE_ADDR ((volatile void**)0x50000000)
#define USER_PARTITION_LABEL "userapp"
#define USER_HEADER_SIZE 4
#define ESP32_BOOT_WINDOW_MS 2000

void esp32AbortUserCode(void);
void esp32FlashInit(void);
bool esp32FlashWriteUserProgram(uint8_t *dataBuffer, uint16_t totalBytes);
void esp32ExecuteUserCode(void);
void esp32SetupJumpTable(void);

#endif // ARDUINO_ARCH_ESP32

#endif // FLASH_WRITER_H
