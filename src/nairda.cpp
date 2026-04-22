#include "nairda.h"
#include "virtual_machine/virtual_machine.h"
#include "extern_libraries/linked_list/linked_list.h"

#include "value_conversion/value_conversion.h"
#include "blue_methods/blue_methods.h"
#include "nairda_debug/nairda_debug.h"
#include "kits/v1.h"

#include "flash_writer/flash_writer.h"
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328__)
#include <avr/eeprom.h>
#endif

#if defined(ARDUINO_ARCH_ESP32)
#include <esp32-hal.h>
#include "kits/kidsy.h"
#include "kits/zeego.h"
#include "nairda_debug/nairda_debug.h"
#endif
#if !defined(ARDUINO_ARCH_ESP32)
#include "extern_libraries/soft_pwm/soft_pwm.h"
#endif

uint8_t currentValue;
VolatileMemory volatileMemory;
uint8_t currentKit = NO_KIT;


void setKit(uint8_t kitCode)
{
  currentKit = kitCode;
}

#if defined(ARDUINO_ARCH_ESP32)
#include "extern_libraries/veml6040/VEML6040.h"
#include "extern_libraries/ssd1306/Adafruit_SSD1306.h"
#include "extern_libraries/dht11/DHT.h"

#include <SPI.h>
#include <Wire.h>

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

  // Inicializar flash y jump table para bootloader ESP32
  esp32FlashInit();
  esp32SetupJumpTable();

#else
void nairdaBegin(long int bauds)
{

#if defined(_24LC_256) || defined(_24LC_512)
  Wire.begin();
#endif
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)
  Serial1.begin(bauds);
#endif
  Serial.begin(bauds);
  Serial.println(F("NK:1"));  // Después de Serial.begin
  SoftPWMBegin();
  Serial.println(F("NK:2"));  // Después de SoftPWMBegin
#endif

  initVolatileMemory(&volatileMemory);
  Serial.println(F("NK:3"));  // Después de initVolatileMemory

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328__)
  Serial.println(F("NK:BOOT"));

  // Si estamos en modo bootloader (EEPROM flag de un WDT reset previo),
  // re-entrar directamente sin esperar la ventana de 2 segundos.
  {
      uint8_t bjMode = eeprom_read_byte(BJ_EE_MODE);
      if (bjMode == 0xBB) {
          Serial.println(F("NK:BJ"));
          // Re-entrar al bootloader — Flutter enviará el siguiente comando
          nairdaDebug(userBootloader, &volatileMemory);
          return;
      }
  }

  // Ventana de decisión: esperar 2 segundos por comando 100 o 150.
  unsigned long bootStart = millis();
  while (millis() - bootStart < 2000) {
      if (nextBlueByte(&currentValue)) {
          Serial.print(F("NK:WIN ")); Serial.println(currentValue);
          nairdaDebug(currentValue, &volatileMemory);
          return;
      }
  }

  // Timeout: verificar flag
  uint8_t flag = pgm_read_byte(USER_SPACE_ADDR);
  Serial.print(F("NK:FLAG ")); Serial.println(flag, HEX);

  if (flag == USER_FLAG_VALID) {
      Serial.println(F("NK:EXEC"));
      Serial.flush();
      ((void (*)(void))USER_PROGRAM_WORD_ADDR)();
  }

  Serial.println(F("NK:LOOP"));
#endif

#if defined(ARDUINO_ARCH_ESP32)
  esp32BootWindow();
#endif
}

#if defined(ARDUINO_ARCH_ESP32)
void esp32BootWindow() {
  Serial.println(F("NK:BOOT"));

  unsigned long bootStart = millis();
  while (millis() - bootStart < ESP32_BOOT_WINDOW_MS) {
    if (nextBlueByte(&currentValue)) {
      Serial.print(F("NK:WIN ")); Serial.println(currentValue);
      nairdaDebug(currentValue, &volatileMemory);
      return;
    }
    delay(1);
  }

  if (flashUserProgramValid()) {
    Serial.println(F("NK:EXEC"));
    Serial.flush();
    esp32ExecuteUserCode();
  }

  Serial.println(F("NK:LOOP"));
}
#endif

void nairdaDelay(unsigned long ms)
{
  unsigned long start = millis();
  while (millis() - start < ms) {
    nairdaLoop();
  }
}

void nairdaLoop()
{
#if defined(KIT_V1_ENABLED)
  writeKitDisplay();
#endif

#if defined(ARDUINO_ARCH_ESP32)
  if (esp32RebootRequested) {
    esp32RebootRequested = false;
    esp32BootWindow();
    return;
  }
#endif

  if (nextBlueByte(&currentValue) == true)
  {
    nairdaDebug(currentValue, &volatileMemory);
  }
}
