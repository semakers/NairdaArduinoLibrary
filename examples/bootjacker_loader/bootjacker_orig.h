// ============================================================================
// bootjacker.h
// Flash self-programming for ATmega328P with stock Optiboot 4.4 bootloader.
//
// Uses ROP gadgets found in the bootloader binary to execute SPM instructions
// from application code. Timer2 Compare Match A ISR rescues execution after
// the gadget falls into Optiboot's blocking main loop.
//
// Gadget addresses verified via avr-objdump of optiboot_atmega328.hex
// (the bootloader shipped with every Arduino Uno).
// ============================================================================

#ifndef BOOTJACKER_ORIG_H
#define BOOTJACKER_ORIG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Initialize BootJacker (placeholder — config happens per-write)
void bjo_init(void);

// Write a 128-byte page to Flash.
// addr must be page-aligned (multiple of SPM_PAGESIZE = 128).
// Returns 0 on success, 1 on verification failure.
uint8_t bjo_write_page(uint16_t addr, const uint8_t *data);

// Self-test: icall into bootloader's getchar (blocks forever),
// verify Timer2 rescues. Returns 0 if rescue worked, 1 if it didn't.
uint8_t bjo_test_rescue(void);

#ifdef __cplusplus
}
#endif

#endif // BOOTJACKER_ORIG_H
