#include "load_from_eeprom.h"
#include "nairda.h"
#include <EEPROM.h>
#include <Wire.h>

#ifndef __AVR_ATmega168__

class variable
{
public:
    int32_t value;
    variable(int32_t cValue)
    {
        value = cValue;
    }
    void setvalue(int32_t newValue)
    {
        value = (newValue > 999999) ? 999999 : (newValue < -999999) ? -999999
                                                                    : newValue;
    }
};

class repeatBegin
{
public:
    uint32_t offsetStart;
    uint32_t offsetEnd;
    uint8_t loop;
    uint32_t times;
    repeatBegin(uint32_t cOffsetStart, uint32_t cOffsetEnd, uint8_t cLoop, uint32_t cTimes)
    {
        offsetStart = cOffsetStart;
        offsetEnd = cOffsetEnd;
        loop = cLoop;
        times = cTimes;
    }
};

uint8_t readByte(uint32_t address);
void nairdaRunMachineState();

uint8_t callInterrupt();

extern LinkedList<component *> listServos;
extern LinkedList<component *> listMotors;
extern LinkedList<component *> listDigitalOuts;
extern LinkedList<component *> listFrequencies;
extern LinkedList<component *> listNeopixels;
extern LinkedList<component *> listAnalogics;
extern LinkedList<component *> listDigitalIns;
extern LinkedList<component *> listUltrasonics;
extern bool running;
LinkedList<variable *> listVariables = LinkedList<variable *>();
LinkedList<repeatBegin *> listRepeatBegins = LinkedList<repeatBegin *>();
LinkedList<int> directionsStack = LinkedList<int>();

uint32_t currentOffset = 7;
uint32_t ProgrammSize = 0;
uint32_t initdirection = 0;
extern uint16_t descArgsBuffer[5];
extern uint32_t execBuffer[6];
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

void loadEepromDescriptor()
{
    if (readByte(0) == 1)
    {
        running = true;

        ProgrammSize = (readByte(1) * 10000) + (readByte(2) * 100) + readByte(3);
        initdirection = (readByte(4) * 10000) + (readByte(5) * 100) + readByte(6);

        servoEepromLoad();
    }
}

void nextVariable()
{
    uint8_t currentByte;
    while (!loadedVariables)
    {
        currentByte = nextByte();
        if (currentByte == endVariables)
        {
            loadedVariables = true;
        }
        else
        {
            uint8_t varBytes[4];
            varBytes[0] = currentByte;
            for (uint8_t i = 1; i < 4; i++)
            {
                varBytes[i] = nextByte();
            }
            int32_t positiveValue = (varBytes[1] * 10000) + (varBytes[2] * 100) + varBytes[3];
            variable *tempVariable = new variable(varBytes[0] == 0 ? positiveValue : (positiveValue * -1));
            listVariables.add(tempVariable);
        }
    }
    nairdaRunMachineState();
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

int32_t getVariableValue()
{
    int32_t val = listVariables.get(nextByte())->value;
    return val;
}

int32_t getComparatorValue()
{
    int32_t firstValue = getInputValue(nextByte());
    uint8_t operation = nextByte();
    int32_t secondByte = getInputValue(nextByte());
    switch (operation)
    {
    case 0:
        return (firstValue == secondByte) ? 1 : 0;
    case 1:
        return (firstValue != secondByte) ? 1 : 0;
    case 2:
        return (firstValue < secondByte) ? 1 : 0;
    case 3:
        return (firstValue <= secondByte) ? 1 : 0;
    case 4:
        return (firstValue > secondByte) ? 1 : 0;
    case 5:
        return (firstValue >= secondByte) ? 1 : 0;
    default:
        return 0;
    }
}

int32_t getLogicValue()
{
    uint8_t firstValue = getInputValue(nextByte()) == 0 ? 0 : 1;
    uint8_t operation = nextByte();
    uint8_t secondByte = getInputValue(nextByte()) == 0 ? 0 : 1;
    switch (operation)
    {
    case 0:
        return (firstValue && secondByte) ? 1 : 0;
    case 1:
        return (firstValue || secondByte) ? 1 : 0;
    default:
        return 0;
    }
}

int32_t getNotValue()
{
    return (getInputValue(nextByte()) == 0) ? 1 : 0;
}

int32_t getAritmeticValue()
{
    int32_t firstValue = getInputValue(nextByte());
    uint8_t operation = nextByte();
    int32_t secondByte = getInputValue(nextByte());
    switch (operation)
    {
    case 0:
        return firstValue + secondByte;
    case 1:
        return firstValue - secondByte;
    case 2:
        return firstValue * secondByte;
    case 3:
        return firstValue / secondByte;
    case 4:
        return firstValue % secondByte;
    default:
        return 0;
    }
}

int32_t getMapValue()
{
    int32_t source = getInputValue(nextByte());
    int32_t inMin = getInputValue(nextByte());
    int32_t inMax = getInputValue(nextByte());
    int32_t outMin = getInputValue(nextByte());
    int32_t outMax = getInputValue(nextByte());
    return map(source, inMin, inMax, outMin, outMax);
}

int32_t getRandomValue()
{
    int32_t from = getInputValue(nextByte());
    int32_t to = getInputValue(nextByte());
    return random(from, to);
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
        return analogicEepromRead();
    case digitalCommand:
        return digitalInEepromRead();
    case ultrasonicCommand:
        return ultrasonicEepromRead();
    case randomCommand:
        return getRandomValue();
    default:
        return 0;
    }
}

void runDelay()
{
    uint32_t delayTime = getInputValue(nextByte());
#if defined(ARDUINO_ARCH_ESP32)
    for (uint64_t i = 0; i < delayTime * 2500; i++)
    {
        if (i % 2500 == 0)
        {
            if (callInterrupt() == 1)
            {
                Serial.println("Interrupt called");
                break;
            }
        }
    }
#else
    uint32_t currentTime = millis();
    while ((millis() - currentTime) < delayTime && callInterrupt() == 0)
    {
    }
#endif
}

void runSetVatValue(uint8_t id)
{
    listVariables.get(id)->setvalue(getInputValue(nextByte()));
}



void runIf()
{
    int32_t conditionValue = getInputValue(nextByte());
    uint8_t eosBytes[3];
    for (uint8_t i = 0; i < 3; i++)
    {
        eosBytes[i] = nextByte();
    }
    uint32_t eos = (eosBytes[0] * 10000) + (eosBytes[1] * 100) + eosBytes[2];
    if (conditionValue != 0)
    {
    }
    else
    {
        currentOffset = eos;
    }
}

repeatBegin *findRepeatBegin(uint32_t repeatStartOffset)
{
    for (uint8_t i = 0; i < listRepeatBegins.size(); i++)
    {
        if (listRepeatBegins.get(i)->offsetStart == repeatStartOffset)
        {
            return listRepeatBegins.get(i);
        }
    }
    return NULL;
}

uint8_t findRepeatBeginIndex(uint32_t repeatStartOffset)
{
    for (uint8_t i = 0; i < listRepeatBegins.size(); i++)
    {
        if (listRepeatBegins.get(i)->offsetStart == repeatStartOffset)
        {
            return i;
        }
    }
    return 0;
}

void runRepeat()
{
    uint32_t sos = currentOffset - 1;
    repeatBegin *currentBegin = findRepeatBegin(sos);
    uint8_t loop = nextByte();
    int32_t times = getInputValue(nextByte());
    uint8_t eosBytes[3];
    for (uint8_t i = 0; i < 3; i++)
    {
        eosBytes[i] = nextByte();
    }
    if (currentBegin == NULL)
    {
        uint32_t eos = (eosBytes[0] * 10000) + (eosBytes[1] * 100) + eosBytes[2];
        currentBegin = new repeatBegin(sos, eos, loop, times);
        // Serial.print("eos: ");
        // Serial.println(currentBegin->offsetEnd);
        listRepeatBegins.add(currentBegin);
    }

    // Serial.print("Repeat times: ");
    // Serial.println(currentBegin->times);

    if (currentBegin->loop == 0)
    {

        if (currentBegin->times == 0)
        {
            currentOffset = currentBegin->offsetEnd;
            listRepeatBegins.remove(findRepeatBeginIndex(currentBegin->offsetStart));
            free(currentBegin);
            // listRepeatBegins.remove(listRepeatBegins.size()-1);
        }
        else
        {
            currentBegin->times = currentBegin->times - 1;
        }
    }
}

void runEndRepeat()
{
    currentOffset = listRepeatBegins.get(listRepeatBegins.size() - 1)->offsetStart;
    // Serial.print("repeats: ");
    // Serial.println(listRepeatBegins.size());
    // Serial.print("Start repeat: ");
    // Serial.println(listRepeatBegins.get(0)->offsetStart);
}

void runBreak()
{
    repeatBegin *breakBegin = listRepeatBegins.get(listRepeatBegins.size() - 1);
    currentOffset = breakBegin->offsetEnd;
    listRepeatBegins.remove(listRepeatBegins.size() - 1);
    free(breakBegin);
}

void runGoToFunction()
{
    uint8_t jumBytes[3];
    for (uint8_t i = 0; i < 3; i++)
    {
        jumBytes[i] = nextByte();
    }
    uint16_t jump = (jumBytes[0] * 10000) + (jumBytes[1] * 100) + jumBytes[2];
    bool contains = false;
    for (uint16_t i = 0; i < directionsStack.size(); i++)
    {
        if (directionsStack.get(i) == currentOffset)
        {
            contains = true;
        }
    }
    if (!contains)
    {
        directionsStack.add(currentOffset);
    }
    currentOffset = jump;
}

void runEndOfFunction()
{
    uint16_t jump = directionsStack.get(directionsStack.size() - 1);
    directionsStack.remove(directionsStack.size() - 1);
    currentOffset = jump;
}

void freeVariables()
{
    for (int i = 0; i < listVariables.size(); i++)
    {
        free(listVariables.get(i));
    }
    listVariables.clear();
}

void freeRepeatBegins()
{
    for (int i = 0; i < listRepeatBegins.size(); i++)
    {
        free(listRepeatBegins.get(i));
    }
    listRepeatBegins.clear();
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

    // resetLeonardoMemory();
}

#endif

#if defined(__AVR_ATmega32U4__) || (ARDUINO_ARCH_ESP32) || (ARDUINO_ARCH_STM32)
void restartRunFromEeprom()
{   
    currentOffset=7;
    resetMemory();
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
            runSetVatValue(nextByte());
            break;
        case frequencyCommand:
            frequencyEepromRun(nextByte());
            break;
        case neopixelCommand:
            neoPixelEepromRun(nextByte());
            break;
        case servoCommand:
            servoEepromRun(nextByte());
            break;
        case motorDcCommand:
            motorEepromRun(nextByte());
            break;
        case ledCommand:
            digitalOutEepromRun(nextByte());
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