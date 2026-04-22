#ifndef FLASH_WRITER_H
#define FLASH_WRITER_H

#include <stdint.h>

// ── Constantes compartidas AVR / ESP32 ──────────────────────────────
#define USER_FLAG_VALID   0x01
#define USER_SPACE_SIZE   8192
#define CHUNK_MAX         64

bool flashUserProgramValid(void);

// ── AVR (ATmega328P) ────────────────────────────────────────────────
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328__)

#include <avr/pgmspace.h>

typedef void (*do_spm_t)(uint16_t address, uint8_t command, uint16_t data);
#define DO_SPM ((do_spm_t)(0x3F01))

#define SPM_PAGE_ERASE  0x03
#define SPM_PAGE_FILL   0x01
#define SPM_PAGE_WRITE  0x05
#define SPM_RWW_ENABLE  0x11

#define JUMP_TABLE_ADDR   0x5F80
#define USER_SPACE_ADDR   0x6000
#define USER_PROGRAM_BYTE_ADDR  0x6002
#define USER_PROGRAM_WORD_ADDR  0x3001

uint8_t flashWritePage(uint16_t target_addr, const uint8_t *data);

#endif // __AVR_ATmega328P__

// ── ESP32 ───────────────────────────────────────────────────────────
#if defined(ARDUINO_ARCH_ESP32)

#include "esp_partition.h"
#include "esp_heap_caps.h"

// Jump Table en RTC slow memory (dirección fija, equivalente a 0x5F80 en AVR)
#define ESP32_JUMP_TABLE_ADDR ((volatile void**)0x50000000)

// Partición "userapp" — últimos 8KB del flash virtual 2MB
#define USER_PARTITION_LABEL "userapp"

// Header en flash: [flag][size_lo][size_hi][entry_offset][code...]
#define USER_HEADER_SIZE 4

// Ventana de boot en segundos
#define ESP32_BOOT_WINDOW_MS 2000

// Salta fuera del user code de vuelta al kernel (llamar al recibir cmd 100)
void esp32AbortUserCode(void);

// Inicializa la partición y la jump table
void esp32FlashInit(void);

// Escribe el programa de usuario en la partición
// dataBuffer ya contiene [0x00, entry_offset, ...code] de Flutter
// Retorna true si la escritura fue exitosa
bool esp32FlashWriteUserProgram(uint8_t *dataBuffer, uint16_t totalBytes);

// Ejecuta el programa de usuario desde flash
void esp32ExecuteUserCode(void);

// Llena la jump table con los punteros a funciones del kernel
void esp32SetupJumpTable(void);

#endif // ARDUINO_ARCH_ESP32

#endif // FLASH_WRITER_H
