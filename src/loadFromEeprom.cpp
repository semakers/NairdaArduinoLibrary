#include "loadFromEeprom.h"
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
        value = (newValue > 999999) ? 999999 : (newValue < -999999) ? -999999 : newValue;
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

void nextServo();
void nextDC();
void nextLed();
void nextAnalogic();
void nextDigital();
void nextUltra();
void nextVariable();
uint8_t readByte(uint32_t address);
void nairdaRunMachineState();
int32_t getInputValue(uint8_t firstByte);
uint8_t callInterrupt();

extern LinkedList<component *> listServos;
extern LinkedList<component *> listDC;
extern LinkedList<component *> listLeds;
extern LinkedList<component *> listAnalogics;
extern LinkedList<component *> listDigitals;
extern LinkedList<component *> listUltrasonics;
extern bool running;
LinkedList<variable *> listVariables = LinkedList<variable *>();
LinkedList<repeatBegin *> listRepeatBegins = LinkedList<repeatBegin *>();

uint32_t currentOffset = 4;
uint32_t ProgrammSize = 0;
extern uint16_t descArgsBuffer[5];
extern uint32_t execBuffer[2];
bool loadedServos = false;
bool loadedMotors = false;
bool loadedLeds = false;
bool loadedDigitals = false;
bool loadedAnalogics = false;
bool loadedUltrasonics = false;
bool loadedVariables = false;

uint8_t currentChannel=0;

uint8_t getCurrentChannel(){
    return currentChannel;
}

void nextCurrentChannel(){
    currentChannel++;
}

void clearCurrentChannel(){
    currentChannel=0;
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
    EEPROM.update(address, byte);
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
    if (currentOffset == (ProgrammSize + 4))
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

        nextServo();
    }
}

void nextServo()
{
    uint8_t currentByte;
    while (!loadedServos)
    {
        currentByte = nextByte();
        if (currentByte == endServos)
        {
            loadedServos = true;
        }
        else
        {
            uint8_t servoBytes[7];
            servoBytes[0] = currentByte;
            for (uint8_t i = 1; i < 7; i++)
            {
                servoBytes[i] = nextByte();
            }
            descArgsBuffer[0] = SERVO;
            descArgsBuffer[1] = getMapedPin(servoBytes[0]);
            descArgsBuffer[2] = (servoBytes[1] * 100) + servoBytes[2];
            descArgsBuffer[3] = (servoBytes[3] * 100) + servoBytes[4];
            descArgsBuffer[4] = (servoBytes[5] * 100) + servoBytes[6];
            component *tempServo = new component(descArgsBuffer);

            listServos.add(tempServo);
        }
    }
    nextDC();
}

void nextDC()
{
    uint8_t currentByte;
    while (!loadedMotors)
    {
        currentByte = nextByte();
        if (currentByte == endDC)
        {
            loadedMotors = true;
        }
        else
        {
            uint8_t dcBytes[3];
            dcBytes[0] = currentByte;
            for (uint8_t i = 1; i < 3; i++)
            {
                dcBytes[i] = nextByte();
            }

            descArgsBuffer[0] = MOTOR;
            descArgsBuffer[1] = getMapedPin(dcBytes[0]);
            descArgsBuffer[2] = getMapedPin(dcBytes[1]);
            descArgsBuffer[3] = getMapedPin(dcBytes[2]);

            component *tempDC = new component(descArgsBuffer);

            listDC.add(tempDC);
        }
    }
    nextLed();
}

void nextLed()
{
    uint8_t currentByte;
    while (!loadedLeds)
    {
        currentByte = nextByte();
        if (currentByte == endLeds)
        {
            loadedLeds = true;
        }
        else
        {
            descArgsBuffer[0] = LED;
            descArgsBuffer[1] = getMapedPin(currentByte);
            component *tempLed = new component(descArgsBuffer);
            listLeds.add(tempLed);
        }
    }
    nextAnalogic();
}

void nextAnalogic()
{
    uint8_t currentByte;
    while (!loadedAnalogics)
    {
        currentByte = nextByte();
        if (currentByte == endAnalogics)
        {
            loadedAnalogics = true;
        }
        else
        {
            descArgsBuffer[0] = ANALOGIC;
            descArgsBuffer[1] = getMapedPin(currentByte);
            component *tempAnalogic = new component(descArgsBuffer);
            listAnalogics.add(tempAnalogic);
        }
    }
    nextDigital();
}

void nextDigital()
{
    uint8_t currentByte;
    while (!loadedDigitals)
    {
        currentByte = nextByte();
        if (currentByte == endDigitals)
        {
            loadedDigitals = true;
        }
        else
        {
            descArgsBuffer[0] = DIGITAL;
            descArgsBuffer[1] = getMapedPin(currentByte);
            component *tempDigital = new component(descArgsBuffer);
            listDigitals.add(tempDigital);
        }
    }
    nextUltra();
}

void nextUltra()
{

    uint8_t currentByte;
    while (!loadedUltrasonics)
    {
        currentByte = nextByte();
        if (currentByte == endUltrasonics)
        {
            loadedUltrasonics = true;
        }
        else
        {
            uint8_t ultraBytes[2];
            ultraBytes[0] = currentByte;
            for (uint8_t i = 1; i < 2; i++)
            {
                ultraBytes[i] = nextByte();
            }
            descArgsBuffer[0] = ULTRASONIC;
            descArgsBuffer[1] = getMapedPin(ultraBytes[0]);
            descArgsBuffer[2] = getMapedPin(ultraBytes[1]);
            component *tempUltrasonic = new component(descArgsBuffer);
            listUltrasonics.add(tempUltrasonic);
        }
    }
    nextVariable();
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
    return listVariables.get(nextByte())->value;
}

int32_t getComparatorValue()
{
    int32_t firstValue = getInputValue(nextByte());
    uint8_t operation = nextByte();
    int32_t secondByte = getInputValue(nextByte());
    //Serial.println("=====================");
    //Serial.println(firstValue);
    //Serial.println(operation);
    //Serial.println(secondByte);
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

int32_t getAnalogicValue()
{
    return listAnalogics.get(nextByte())->getSensVal(ANALOGIC);
}

int32_t getDigitalValue()
{
    return listDigitals.get(nextByte())->getSensVal(DIGITAL);
}

int32_t getUltraValue()
{
    return listUltrasonics.get(nextByte())->getSensVal(ULTRASONIC);
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
        return getAnalogicValue();
    case digitalCommand:
        return getDigitalValue();
    case ultrasonicCommand:
        return getUltraValue();
    default:
        return 0;
    }
}

void runDelay()
{
    uint32_t currentTime = millis();
    uint32_t delayTime = getInputValue(nextByte());
    while ((millis() - currentTime) < delayTime && callInterrupt() == 0)
    {
    }
}

void runSetVatValue(uint8_t id)
{
    listVariables.get(id)->setvalue(getInputValue(nextByte()));
}

void runServo(uint8_t id)
{
    execBuffer[0] = getInputValue(nextByte());
    listServos.get(id)->execAct(execBuffer, SERVO);
}

void runMotorDc(uint8_t id)
{
    int32_t vel = getInputValue(nextByte());
    vel = (vel < 0) ? 0 : (vel > 100) ? 100 : vel;

    execBuffer[0] = vel;
    execBuffer[1] = nextByte();

    listDC.get(id)->execAct(execBuffer, MOTOR);
}

void runLed(uint8_t id)
{
    int32_t intensity = getInputValue(nextByte());
    intensity = (intensity < 0) ? 0 : (intensity > 100) ? 100 : intensity;
    execBuffer[0] = intensity;
    listLeds.get(id)->execAct(execBuffer, LED);
}

void runIf()
{
    int32_t conditionValue = getInputValue(nextByte());
    uint8_t eosBytes[3];
    for (uint8_t i = 0; i < 3; i++)
    {
        eosBytes[i] = nextByte();
    }
    uint32_t eos = (eosBytes[0] * 10000) + (eosBytes[1] * 100) + eosBytes[2] + 4;
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
        uint32_t eos = (eosBytes[0] * 10000) + (eosBytes[1] * 100) + eosBytes[2] + 4;
        currentBegin = new repeatBegin(sos, eos, loop, times);
        //Serial.print("eos: ");
        //Serial.println(currentBegin->offsetEnd);
        listRepeatBegins.add(currentBegin);
    }

    //Serial.print("Repeat times: ");
    //Serial.println(currentBegin->times);

    if (currentBegin->loop == 0)
    {

        if (currentBegin->times == 0)
        {
            currentOffset = currentBegin->offsetEnd;
            listRepeatBegins.remove(findRepeatBeginIndex(currentBegin->offsetStart));
            free(currentBegin);
            //listRepeatBegins.remove(listRepeatBegins.size()-1);
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
    //Serial.print("repeats: ");
    //Serial.println(listRepeatBegins.size());
    //Serial.print("Start repeat: ");
    //Serial.println(listRepeatBegins.get(0)->offsetStart);
}

void runBreak()
{
    repeatBegin *breakBegin = listRepeatBegins.get(listRepeatBegins.size() - 1);
    currentOffset = breakBegin->offsetEnd;
    listRepeatBegins.remove(listRepeatBegins.size() - 1);
    free(breakBegin);
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

#if defined(__AVR_ATmega32U4__) || defined(ARDUINO_ARCH_ESP32)

void freeVolatileMemory()
{
    if (running)
    {
        currentOffset = 4;
        ProgrammSize = (readByte(1) * 10000) + (readByte(2) * 100) + readByte(3);
        freeVariables();
        freeRepeatBegins();
    }

    // resetLeonardoMemory();
}

#endif

uint8_t callInterrupt()
{
    if (running)
    {
        uint8_t it;

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
#if defined(__AVR_ATmega32U4__) || (ARDUINO_ARCH_ESP32)
            resetMemory();
            freeVolatileMemory();
            running = false;
             loadedServos = false;
             loadedMotors = false;
             loadedLeds = false;
             loadedDigitals = false;
             loadedAnalogics = false;
             loadedUltrasonics = false;
             loadedVariables = false;
            
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
    while (callInterrupt() == 0)
    {
        switch (nextByte())
        {
        case delayCommand:
            runDelay();
            break;
        case setVarValueCommand:
            runSetVatValue(nextByte());
            break;
        case servoCommand:
            runServo(nextByte());
            break;
        case motorDcCommand:
            runMotorDc(nextByte());
            break;
        case ledCommand:
            runLed(nextByte());
            break;
        case ifCommand:
            runIf();
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