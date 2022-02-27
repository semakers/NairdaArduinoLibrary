#include "nairda.h"
#include "load_from_eeprom.h"
#include "extern_libraries/linked_list/linked_list.h"
#include "value_conversion/value_conversion.h"
#include "blue_methods/blue_methods.h"
#include "nairda_debug/nairda_debug.h"

#if defined(ARDUINO_ARCH_ESP32)
#include "esp_spi_flash.h"
#endif

#if !defined(ARDUINO_ARCH_STM32) && !defined(ARDUINO_ARCH_ESP32)
#include "extern_libraries/soft_pwm/soft_pwm.h"
#endif

void nairdaDebug(uint8_t tempValue);
bool running = false;
bool preInit = false;
int32_t programmSize = 0;
bool startSaving = false;
#ifndef __AVR_ATmega168__

uint32_t currentProgramOffset = 0;
bool savingBoolean[4];
uint8_t savingBuffer[4];
#endif

void cleanSavingBoolean();

enum
{
  noMemory,
  memory1k,
  memory4k,
  memory256k,
  memory512k
};

uint8_t i;
uint8_t tempValue;
int runProgrammTimeOut = 0;

VolatileMemory volatileMemory;

#ifndef __AVR_ATmega168__
void cleanSavingBoolean()
{
  for (int j = 0; j < 4; j++)
  {
    savingBoolean[j] = false;
  }
}
#endif

#if defined(ARDUINO_ARCH_ESP32)
void nairdaBegin(const char *deviceName)
{
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

  /**/
#ifndef __AVR_ATmega168__
#ifdef __AVR_ATmega32U4__

  if (asmOperations > 250000 && volatileMemory.declaratedComponents == false) 
  {
    loadEepromDescriptor(&volatileMemory);
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
  if ((millis() - runProgrammTimeOut) > 2500 && preInit == false)
  {
#else
  if ((millis() - runProgrammTimeOut) > 2500 && volatileMemory.declaratedComponents == false)
  {
#endif

    loadEepromDescriptor(&volatileMemory);
    runProgrammTimeOut = millis();
  }

#endif
#endif

  if (nextBlueByte(&tempValue) == true)
  {
    nairdaDebug(tempValue);
  }
}

void nairdaDebug(uint8_t tempValue)
{
  if (tempValue == projectInit)
  {
#if defined(__AVR_ATmega32U4__) || (ARDUINO_ARCH_ESP32) || (ARDUINO_ARCH_STM32)
    clearVolatileMemory();
#else
    clearVolatileMemory(&volatileMemory,true);
    asm volatile("jmp 0");
#endif
    // Serial.println("Se limpriaron las listas");
  }
  if (tempValue == saveCommand)
  {
    uint32_t memorySize;

#if !defined(_24LC_256) && !defined(_24LC_512)
#if defined(__AVR_ATmega168__)
    memorySize = 0;
#endif

#if defined(ARDUINO_ARCH_ESP32)
    startSaving = true;
    memorySize = 512 * 1024;
#endif

#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega328P__)
    startSaving = true;
    memorySize = 1024;
#endif

#if defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)
    startSaving = true;
    memorySize = 4 * 1024;
#endif

#if defined(ARDUINO_ARCH_STM32)
    startSaving = true;
    memorySize = EEPROM.length();
#endif

#else
    startSaving = true;

#if defined(_24LC_256)
    memorySize = 256 * 1024;
#endif

#if defined(_24LC_512)
    memorySize = 512 * 1024;
#endif

#endif
    sendMemorySize(memorySize);
  }
  else if (startSaving)
  {
#ifndef __AVR_ATmega168__
    if (!savingBoolean[0])
    {
      savingBoolean[0] = true;
      savingBuffer[0] = tempValue;
    }
    else if (!savingBoolean[1])
    {
      savingBoolean[1] = true;
      savingBuffer[1] = tempValue;
    }
    else if (!savingBoolean[2])
    {
      savingBoolean[2] = true;
      savingBuffer[2] = tempValue;
    }
    else if (!savingBoolean[3])
    {
      currentProgramOffset = 4;
      savingBoolean[3] = true;
      savingBuffer[3] = tempValue;
      writeByte(0, savingBuffer[0]);
      writeByte(1, savingBuffer[1]);
      writeByte(2, savingBuffer[2]);
      writeByte(3, savingBuffer[3]);
      programmSize = (savingBuffer[1] * 10000) + (savingBuffer[2] * 100) + savingBuffer[3] - currentProgramOffset;
    }
    else
    {
      if (programmSize > 1)
      {
        writeByte(currentProgramOffset, tempValue);
        currentProgramOffset++;
        programmSize--;
      }
      else
      {
        writeByte(currentProgramOffset, tempValue);
        startSaving = false;
        cleanSavingBoolean();
      }
    }
#endif
  }
  else if (tempValue == versionCommand)
  {

#if defined(ARDUINO_ARCH_ESP32)
    bleWrite(CURRENT_VERSION);
#else

#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)
    Serial1.write(((char)CURRENT_VERSION));
#endif
    Serial.write(((char)CURRENT_VERSION));
#endif
  }
  else if (volatileMemory.declaratedDescriptor == false)
  {
    declarateComponents(&tempValue, &volatileMemory);
  }
  else
  {
    executeComponent(&tempValue, &volatileMemory);
  }
}
