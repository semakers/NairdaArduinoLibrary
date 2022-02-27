#include "load_from_eeprom.h"
#include "blue_methods/blue_methods.h"

#if defined(ARDUINO_ARCH_ESP32)

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "esp_spi_flash.h"
#endif

#ifndef __AVR_ATmega168__

VolatileMemory* volatileMemory=0;

uint8_t readByte(uint32_t address);
extern bool running;
uint32_t currentOffset = 7;
uint32_t ProgrammSize = 0;
uint32_t initdirection = 0;
bool loadedServos = false;
bool loadedMotors = false;
bool loadedDigitalOuts = false;
bool loadedFrequencies = false;
bool loadedNeoPixels = false;
bool loadedDigitalIns = false;
bool loadedAnalogics = false;
bool loadedUltrasonics = false;
bool loadedVariables = false;

uint8_t currentChannel = 0;

uint8_t getCurrentChannel()
{
    return currentChannel;
}

void nextCurrentChannel()
{
    currentChannel++;
}

void clearCurrentChannel()
{
    currentChannel = 0;
}

void writeByte(uint32_t address, uint8_t byte)
{
#if defined(ARDUINO_ARCH_ESP32)
    static uint8_t mbuffer[1];
    mbuffer[0] = byte;
    spi_flash_write(0x200000 + address, mbuffer, 1);
#else
#if defined(_24LC_256) || defined(_24LC_512)
    if (readByte(address) != byte)
    {
        Wire.beginTransmission(0x50);
        Wire.write((uint8_t)(address >> 8));   // MSB
        Wire.write((uint8_t)(address & 0xFF)); // LSB
        Wire.write(byte);
        Wire.endTransmission();
        delay(5);
    }
#else
    // eeprom_write_byte(address, byte);
    EEPROM.update(address, byte);
    // HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_BYTE, (address + DATA_EEPROM_BASE), (uint32_t)byte);
    
    // delay(30);
#endif
#endif
}

uint8_t readByte(uint32_t address)
{
#if defined(ARDUINO_ARCH_ESP32)
    static uint8_t mbuffer[1];
    spi_flash_read(0x200000 + address, mbuffer, 1);
    return mbuffer[0];
#else
#if defined(_24LC_256) || defined(_24LC_512)
    uint8_t rdata = 0xFF;
    Wire.beginTransmission(0x50);
    Wire.write((int)(address >> 8));   // MSB
    Wire.write((int)(address & 0xFF)); // LSB
    Wire.endTransmission();
    Wire.requestFrom(0x50, 1);
    if (Wire.available())
        rdata = Wire.read();
    return rdata;
#else
    return EEPROM[address];
#endif
#endif
}

uint8_t nextByte()
{

    if (currentOffset == (ProgrammSize))
    {
        while (callInterrupt() == 0)
        {
        };
    }
    if (running)
    {
        uint8_t auxByte = readByte(currentOffset);
        currentOffset++;
        return auxByte;
    }
    else
    {
        return 0;
    }
}

void loadEepromDescriptor(VolatileMemory * _volatileMemory)
{
    if (readByte(0) == 1)
    {
        running = true;

        ProgrammSize = (readByte(1) * 10000) + (readByte(2) * 100) + readByte(3);
        initdirection = (readByte(4) * 10000) + (readByte(5) * 100) + readByte(6);
        volatileMemory=_volatileMemory;
        servoEepromLoad(volatileMemory);
    }
}

int32_t getValue()
{
    uint8_t valueBytes[4];
    for (uint8_t i = 0; i < 4; i++)
    {
        valueBytes[i] = nextByte();
    }
    int32_t positiveValue = (valueBytes[1] * 10000) + (valueBytes[2] * 100) + valueBytes[3];
    return valueBytes[0] == 0 ? positiveValue : (positiveValue * -1);
}

int32_t getInputValue(uint8_t firstByte)
{
    switch (firstByte)
    {
    case valueCommand:
        return getValue();
    case variableCommand:
        return getVariableValue();
    case comparatorCommand:
        return getComparatorValue();
    case logicCommand:
        return getLogicValue();
    case notCommand:
        return getNotValue();
    case aritmeticCommand:
        return getAritmeticValue();
    case mapCommand:
        return getMapValue();
    case analogicCommand:
        return analogicEepromRead(volatileMemory);
    case digitalCommand:
        return digitalInEepromRead(volatileMemory);
    case ultrasonicCommand:
        return ultrasonicEepromRead(volatileMemory);
    case randomCommand:
        return getRandomValue();
    default:
        return 0;
    }
}

#if defined(__AVR_ATmega32U4__) || defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_STM32)

void freeVolatileMemory()
{
    if (running)
    {
        ProgrammSize = (readByte(1) * 10000) + (readByte(2) * 100) + readByte(3);
        freeVariables();
        freeRepeatBegins();
    }
}

#endif

#if defined(__AVR_ATmega32U4__) || (ARDUINO_ARCH_ESP32) || (ARDUINO_ARCH_STM32)
void restartRunFromEeprom()
{   
    currentOffset=7;
    
    clearVolatileMemory(&volatileMemory,true);
    freeVolatileMemory();
    running = false;
    loadedServos = false;
    loadedMotors = false;
    loadedDigitalOuts = false;
    loadedFrequencies = false;
    loadedDigitalIns = false;
    loadedAnalogics = false;
    loadedUltrasonics = false;
    loadedVariables = false;
}
#endif

uint8_t callInterrupt()
{
    if (running)
    {
        uint8_t it = 0;

#if defined(ARDUINO_ARCH_ESP32)

        if (bleAvailable())
        {
            it = bleRead();
        }
#else

#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)
        int serialAvailable = Serial.available();
        int serial1Available = Serial1.available();
        if (serialAvailable > 0 || serial1Available > 0)
        {
            if (serialAvailable > 0)
            {
                it = Serial.read();
            }
            else if (serial1Available > 0)
            {
                it = Serial1.read();
            }
        }

#else

        if (Serial.available())
        {
            it = Serial.read();
        }

#endif

#endif
        if (it == versionCommand)
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
        else if (it == projectInit)
        {
#if defined(__AVR_ATmega32U4__) || (ARDUINO_ARCH_ESP32) || (ARDUINO_ARCH_STM32)
            restartRunFromEeprom();

            return 1;
#else
            asm volatile("jmp 0");
#endif
        }
        else
        {
            return 0;
        }
    }
    else
    {
        return 1;
    }
}

void nairdaRunMachineState()
{
    currentOffset = initdirection;
    while (callInterrupt() == 0)
    {
#if defined(ARDUINO_ARCH_ESP32)
#endif
        uint8_t auxByte = nextByte();

        switch (auxByte)
        {
        case delayCommand:
            runDelay();
            break;
        case setVarValueCommand:
            runSetVarValue(nextByte());
            break;
        case frequencyCommand:
            frequencyEepromRun(nextByte(),volatileMemory);
            break;
        case neopixelCommand:
            neoPixelEepromRun(nextByte(),volatileMemory);
            break;
        case servoCommand:
            servoEepromRun(nextByte(),volatileMemory);
            break;
        case motorDcCommand:
            motorEepromRun(nextByte(),volatileMemory);
            break;
        case ledCommand:
            digitalOutEepromRun(nextByte(),volatileMemory);
            break;
        case ifCommand:
            runIf();
            break;
        case goToFunctionCommand:
            runGoToFunction();
            break;
        case endFunctionCommand:
            runEndOfFunction();
            break;
        case repeatCommand:
            runRepeat();
            break;
        case endRepeatCommand:
            runEndRepeat();
            break;
        case breakCommand:
            runBreak();
            break;
        }
    }
}

#endif

uint8_t getMapedPin(uint8_t pin)
{
    return (pin >= 70) ? (A0 + (pin - 70)) : pin;
}