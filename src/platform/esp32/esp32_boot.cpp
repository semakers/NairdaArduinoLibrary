#if defined(ARDUINO_ARCH_ESP32)

#include <Arduino.h>
#include <esp32-hal.h>
#include <SPI.h>
#include <Wire.h>

#include "volatile_memory/volatile_memory.h"
#include "virtual_machine/virtual_machine.h"
#include "blue_methods/blue_methods.h"
#include "nairda_debug/nairda_debug.h"
#include "flash_writer/flash_writer.h"
#include "nairda.h"
#include "nairda_log.h"

#include "kits/kidsy.h"
#include "kits/zeego.h"

#include "extern_libraries/veml6040/VEML6040.h"
#include "extern_libraries/ssd1306/Adafruit_SSD1306.h"
#include "extern_libraries/dht11/DHT.h"

#include "platform/esp32/esp32_ble_name.h"

extern uint8_t currentValue;
extern VolatileMemory volatileMemory;
extern uint8_t currentKit;

VEML6040 RGBWSensor;
DHT dht(16, DHT11);
int hum;
int temp;

void nairdaBegin(const char *deviceName, long int bauds)
{
    Serial.begin(bauds);
    delay(100);
    NRD_LOG("\n[NRD] === nairdaBegin start, kit=%d, bauds=%ld ===\n", currentKit, bauds);

    if (currentKit == ROBBUS_KIDSY_KIT)
    {
        NRD_LOGLN("[NRD] entering calibrateKidsyColorSensor (no-op unless BTN A/C held)");
        calibrateKidsyColorSensor(RGBWSensor);
        NRD_LOGLN("[NRD] calibrateKidsyColorSensor returned");

        NRD_LOGLN("[NRD] entering calibrateKidsyTouchSensors (no-op unless BTN B held)");
        calibrateKidsyTouchSensors();
        NRD_LOGLN("[NRD] calibrateKidsyTouchSensors returned");
        readKidsyTouchCalibration();
    }

    if (currentKit == LK32_KIT)
    {
        dht.begin();
        temp = int(round(dht.readTemperature()));
        hum = int(round(dht.readHumidity()));
        Adafruit_SSD1306 display(128, 64, false, &Wire, -1);
        if (display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
        {
            display.clearDisplay();
            display.setTextSize(2);
            display.setTextColor(1);
            display.setCursor(1, 1);
            display.print(deviceName);
            display.display();
        }
    }

    if (currentKit == ROBBUS_ZEEGO_KIT)
    {
        Adafruit_SSD1306 display(128, 64, true, &Wire, -1);
        calibrateZeegoFloorSensor(display);
        readFloorCalibration();
        if (display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
        {
            display.clearDisplay();
            display.setTextSize(2);
            display.setTextColor(1);
            display.setCursor(1, 1);
            display.print(deviceName);
            display.display();
        }
    }

#if defined(KIT_V1_ENABLED)
    initKitDisplay();
#endif

    NRD_LOGLN("[NRD] calling bleInit");
    // If the user has stored a custom BLE name via setBleName from a previous
    // program run, use that instead of the default.
    char storedBleName[BLE_NAME_MAX_LEN + 1];
    const bool hasStored = readStoredBleName(storedBleName);
    const char *effectiveBleName = hasStored ? storedBleName : deviceName;
    NRD_LOG("[NRD]   ble name source=%s value=\"%s\"\n",
            hasStored ? "FLASH" : "default(deviceName arg)", effectiveBleName);
    bleInit(effectiveBleName);

    NRD_LOGLN("[NRD] calling esp32FlashInit");
    esp32FlashInit();
    NRD_LOGLN("[NRD] calling esp32SetupJumpTable");
    esp32SetupJumpTable();

    NRD_LOGLN("[NRD] calling initVolatileMemory");
    initVolatileMemory(&volatileMemory);

    NRD_LOGLN("[NRD] entering esp32BootWindow");
    esp32BootWindow();
    NRD_LOGLN("[NRD] esp32BootWindow returned");
}

void esp32BootWindow()
{
    NRD_LOG("[NRD] BootWindow waiting %lu ms for BLE/serial\n", (unsigned long)ESP32_BOOT_WINDOW_MS);
    unsigned long bootStart = millis();
    while (millis() - bootStart < ESP32_BOOT_WINDOW_MS) {
        if (nextBlueByte(&currentValue)) {
            NRD_LOG("[NRD] BootWindow → live programming path (byte=0x%02X)\n", currentValue);
            nairdaDebug(currentValue, &volatileMemory);
            return;
        }
        delay(1);
    }

    NRD_LOGLN("[NRD] BootWindow timeout — no live programming");
    if (flashUserProgramValid()) {
        NRD_LOGLN("[NRD] → executing PERMANENT program");
        Serial.flush();
        esp32ExecuteUserCode();
    } else {
        NRD_LOGLN("[NRD] No valid permanent program in userapp");
    }
}

#endif
