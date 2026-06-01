#ifndef ESP32_BLE_NAME_H
#define ESP32_BLE_NAME_H

#if defined(ARDUINO_ARCH_ESP32)

#include <stdint.h>
#include <stdbool.h>

// Flash sector used to persist the BLE advertising name across reboots.
// Lives in the same 2MB-vs-4MB free region as the color sensor calibration,
// one sector BEFORE it so the two cannot overlap:
//   0x27E000 - 0x27EFFF : BLE name sector (this)
//   0x27F000 - 0x27FFFF : VEML6040 calibration sector
//
// Layout in the sector (first 31 bytes, rest left erased):
//   byte 0    : magic 0xBE       (any other value = no stored name)
//   byte 1    : length (1..29)
//   bytes 2.. : ASCII name (not NUL-terminated on flash)
#define BLE_NAME_FLASH_ADDR  0x27E000
#define BLE_NAME_MAGIC       0xBE
#define BLE_NAME_MAX_LEN     29   // BLE adv payload practical max

// Writes the BLE name to flash. Called from user binaries via jump table.
// Skips the write if the stored value already matches (avoids flash wear when
// the same permanent program runs every boot).
void setBleName(const char *name, uint8_t len);

// Reads the stored BLE name into `out_buf` (must be >= BLE_NAME_MAX_LEN + 1).
// Returns true if a valid stored name was found, false otherwise.
bool readStoredBleName(char *out_buf);

#endif // ARDUINO_ARCH_ESP32
#endif // ESP32_BLE_NAME_H
