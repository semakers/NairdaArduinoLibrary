#include "nairda.h"
#include "virtual_machine/virtual_machine.h"
#include "extern_libraries/linked_list/linked_list.h"
#include "value_conversion/value_conversion.h"
#include "blue_methods/blue_methods.h"
#include "nairda_debug/nairda_debug.h"
#include "kits/v1.h"

#if defined(ARDUINO_ARCH_ESP32)
#include "esp_spi_flash.h"
#endif
#if !defined(ARDUINO_ARCH_STM32) && !defined(ARDUINO_ARCH_ESP32)
#include "extern_libraries/soft_pwm/soft_pwm.h"
#endif

uint8_t currentValue;
int runProgrammTimeOut = 0;
VolatileMemory volatileMemory;
#if defined(ARDUINO_ARCH_ESP32)
void nairdaBegin(const char *deviceName)
{
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
  Serial1.begin(bauds);
#endif
  Serial.begin(bauds);
#if defined(ARDUINO_ARCH_STM32)
  softPwmSTM32Init();
#else
  SoftPWMBegin();
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
  /**/
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

  if (nextBlueByte(&currentValue) == true)
  {
    nairdaDebug(currentValue, &volatileMemory);
  }
}
