#if defined(ARDUINO_ARCH_ESP32)

#include "esp32_ble_name.h"
#include "esp_flash.h"
#include "nairda_log.h"
#include <string.h>

void setBleName(const char *name, uint8_t len)
{
    NRD_LOG("[NRD/BLE] setBleName(len=%u) request\n", len);
    if (len > BLE_NAME_MAX_LEN) {
        NRD_LOG("[NRD/BLE]   len > %d, truncating\n", BLE_NAME_MAX_LEN);
        len = BLE_NAME_MAX_LEN;
    }

    // Wear-protection: if the sector already holds this exact value, do nothing.
    // A permanent program that calls setBleName on every boot would otherwise
    // erase + write the same bytes on every power-up.
    uint8_t current[BLE_NAME_MAX_LEN + 2];
    esp_flash_read(esp_flash_default_chip, current, BLE_NAME_FLASH_ADDR, sizeof(current));
    NRD_LOG("[NRD/BLE]   current sector head: magic=0x%02X len=%u\n",
            current[0], current[1]);
    if (current[0] == BLE_NAME_MAGIC && current[1] == len &&
        memcmp(current + 2, name, len) == 0) {
        NRD_LOGLN("[NRD/BLE]   SKIP — sector already holds this value (no flash write)");
        return;
    }

    NRD_LOG("[NRD/BLE]   erasing sector 0x%08X (4096 bytes)\n", BLE_NAME_FLASH_ADDR);
    esp_err_t err = esp_flash_erase_region(esp_flash_default_chip, BLE_NAME_FLASH_ADDR, 4096);
    NRD_LOG("[NRD/BLE]   erase err=%d\n", err);

    if (len == 0) {
        NRD_LOGLN("[NRD/BLE]   len=0, leaving sector erased (clears stored name)");
        return;
    }

    uint8_t buf[BLE_NAME_MAX_LEN + 2];
    buf[0] = BLE_NAME_MAGIC;
    buf[1] = len;
    memcpy(buf + 2, name, len);
    err = esp_flash_write(esp_flash_default_chip, buf, BLE_NAME_FLASH_ADDR, len + 2);
    NRD_LOG("[NRD/BLE]   write err=%d, %u bytes written to 0x%08X\n",
            err, len + 2, BLE_NAME_FLASH_ADDR);
}

bool readStoredBleName(char *out_buf)
{
    uint8_t buf[BLE_NAME_MAX_LEN + 2];
    esp_err_t err = esp_flash_read(esp_flash_default_chip, buf, BLE_NAME_FLASH_ADDR, sizeof(buf));
    NRD_LOG("[NRD/BLE] readStoredBleName: flash_read err=%d magic=0x%02X len=%u\n",
            err, buf[0], buf[1]);

    if (buf[0] != BLE_NAME_MAGIC) {
        NRD_LOGLN("[NRD/BLE]   magic mismatch → no stored name");
        return false;
    }
    uint8_t len = buf[1];
    if (len == 0 || len > BLE_NAME_MAX_LEN) {
        NRD_LOG("[NRD/BLE]   invalid len=%u → no stored name\n", len);
        return false;
    }

    memcpy(out_buf, buf + 2, len);
    out_buf[len] = '\0';
    NRD_LOG("[NRD/BLE]   stored name = \"%s\" (len=%u)\n", out_buf, len);
    return true;
}

#endif
