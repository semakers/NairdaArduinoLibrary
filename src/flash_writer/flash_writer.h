#ifndef FLASH_WRITER_H
#define FLASH_WRITER_H

#include <stdint.h>

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328__)

#include <avr/pgmspace.h>

// do_spm expuesto por Optiboot v8+ en (FLASHEND - 511 + 2) >> 1
// Para ATmega328P: (0x7FFF - 511 + 2) >> 1 = 0x3F01
typedef void (*do_spm_t)(uint16_t address, uint8_t command, uint16_t data);
#define DO_SPM ((do_spm_t)(0x3F01))

// Comandos SPM
#define SPM_PAGE_ERASE  0x03
#define SPM_PAGE_FILL   0x01
#define SPM_PAGE_WRITE  0x05
#define SPM_RWW_ENABLE  0x11

// Jump Table: 17 slots × 2 bytes = 34 bytes
// En su propia página Flash (0x5F80-0x5FFF) para que escribir el espacio
// de usuario no borre la tabla.
#define JUMP_TABLE_ADDR   0x5F80

// Dirección base del espacio de usuario
#define USER_SPACE_ADDR   0x6000
#define USER_SPACE_SIZE   8192
#define USER_FLAG_VALID   0x01
// El programa empieza en 0x6002 (byte 2 del espacio de usuario).
// Byte 0 = flag, byte 1 = padding (AVR requiere alineación par).
// En AVR, function pointers son word addresses: 0x6002 / 2 = 0x3001
#define USER_PROGRAM_BYTE_ADDR  0x6002
#define USER_PROGRAM_WORD_ADDR  0x3001

// Escribe una página completa (128 bytes) en Flash.
// target_addr debe estar alineada a 128 bytes.
// Retorna 1 si la verificación pasa, 0 si falla.
uint8_t flashWritePage(uint16_t target_addr, const uint8_t *data);

// Lee el flag de integridad en USER_SPACE_ADDR.
// Retorna true si == USER_FLAG_VALID.
bool flashUserProgramValid(void);

#endif // __AVR_ATmega328P__
#endif // FLASH_WRITER_H
