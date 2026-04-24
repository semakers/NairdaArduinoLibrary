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

#include "kits/kidsy.h"
#include "kits/zeego.h"

#include "extern_libraries/veml6040/VEML6040.h"
#include "extern_libraries/ssd1306/Adafruit_SSD1306.h"
#include "extern_libraries/dht11/DHT.h"

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

    if (currentKit == ROBBUS_KIDSY_KIT)
    {
        calibrateKidsyColorSensor(RGBWSensor);
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

    bleInit(deviceName);

    esp32FlashInit();
    esp32SetupJumpTable();

    initVolatileMemory(&volatileMemory);

    esp32BootWindow();
}

void esp32BootWindow()
{
    unsigned long bootStart = millis();
    while (millis() - bootStart < ESP32_BOOT_WINDOW_MS) {
        if (nextBlueByte(&currentValue)) {
            nairdaDebug(currentValue, &volatileMemory);
            return;
        }
        delay(1);
    }

    if (flashUserProgramValid()) {
        Serial.flush();
        esp32ExecuteUserCode();
    }
}

#endif
