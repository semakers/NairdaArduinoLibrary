#include "nairda.h"
#include "virtual_machine/virtual_machine.h"
#include "extern_libraries/linked_list/linked_list.h"

#include "value_conversion/value_conversion.h"
#include "blue_methods/blue_methods.h"
#include "nairda_debug/nairda_debug.h"
#include "kits/v1.h"
#include <DynamixelSDK.h>

#if defined(ARDUINO_ARCH_ESP32)
// #include "esp_//spi_flash.h"
#include <esp32-hal.h>
#include "kits/kidsy.h"
#include "kits/zeego.h"
#endif

uint8_t currentValue;
int runProgrammTimeOut = 0;
VolatileMemory volatileMemory;
uint8_t currentKit = NO_KIT;
bool running = false;

extern dynamixel::PortHandler *portHandler;
extern dynamixel::PacketHandler *packetHandler;

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

  Serial3.begin(bauds);

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

#else
void nairdaBegin(long int bauds)
{

#if defined(_24LC_256) || defined(_24LC_512)
  Wire.begin();
#endif
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)
  Serial3.begin(bauds);
#endif
  Serial.begin(9600);
  Serial3.begin(bauds);
  portHandler = dynamixel::PortHandler::getPortHandler("1");
  packetHandler = dynamixel::PacketHandler::getPacketHandler(2.0);

  portHandler->openPort();
  portHandler->setBaudRate(1000000);

#if defined(ARDUINO_ARCH_STM32)
  softPwmSTM32Init();
#else
#endif
#endif
  runProgrammTimeOut = millis();

  initVolatileMemory(&volatileMemory);
}

void nairdaLoop()
{
#if defined(KIT_V1_ENABLED)
  writeKitDisplay();
#endif
  /*
  #ifndef __AVR_ATmega168__
  #ifdef __AVR_ATmega32U4__

    if (asmOperations > 250000 && volatileMemory.declaratedComponents == false)
    {
      loadEepromDescriptor();
    }
    else
    {
      if (asmOperations <= 200000)
      {
        asmOperations++;
      }
    }

  #else
  #if defined(ARDUINO_ARCH_ESP32)
    if ((millis() - runProgrammTimeOut) > 2500 && volatileMemory.declaratedComponents[0] == false)
    {
  #else
    if ((millis() - runProgrammTimeOut) > 2500 && volatileMemory.declaratedComponents[0] == false)
    {
  #endif
      loadEepromDescriptor();
      runProgrammTimeOut = millis();
    }


  #endif
  #endif
  */

  if (nextBlueByte(&currentValue) == true)
  {
    nairdaDebug(currentValue, &volatileMemory);
  }
}
