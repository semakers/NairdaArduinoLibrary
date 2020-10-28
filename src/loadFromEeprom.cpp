#include "loadFromEeprom.h"
#include "nairda.h"
#include <EEPROM.h>



#ifndef __AVR_ATmega168__

class variable
{
public:
  int32_t value;
  variable(int32_t cValue){
    value = cValue;
  }
  void setvalue(int32_t newValue){
      value=(newValue>999999)?999999:(newValue<-999999)?-999999:newValue;
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
void nairdaRunMachineState(uint8_t firstByte);
int32_t getInputValue(uint8_t firstByte);
uint8_t callInterrupt();

extern LinkedList<component *> listServos;
extern LinkedList<component *> listDC;
extern LinkedList<component *> listLeds;
extern LinkedList<component *> listAnalogics;
extern LinkedList<component *> listDigitals;
extern LinkedList<component *> listUltrasonics;
LinkedList<variable *> listVariables = LinkedList<variable *>();
LinkedList<repeatBegin *> listRepeatBegins = LinkedList<repeatBegin *>();

uint32_t currentOffset = 4;
uint32_t ProgrammSize = 0;
extern uint16_t descArgsBuffer[5];
extern uint32_t execBuffer[2];




void writeByte(unsigned long int address, unsigned short int byte)
{
    EEPROM.update(address, byte);
}

uint8_t readByte(uint32_t address)
{
    return EEPROM[address];
}

uint8_t nextByte()
{
    if (currentOffset == (ProgrammSize + 4))
    {
        while (1)
        {
            callInterrupt();
        };
    }
    uint8_t auxByte = readByte(currentOffset);
    //Serial.println(auxByte);
    //delay(250);
    currentOffset++;
    return auxByte;
}

void loadEepromDescriptor()
{
    if (readByte(0) == 1)
    {
        ProgrammSize = (readByte(1) * 10000) + (readByte(2) * 100) + readByte(3);
        nextServo();
    }
}

void nextServo()
{
    uint8_t firstByte = nextByte();
    if (firstByte == endServos)
    {
        nextDC();
    }
    else
    {
        uint8_t servoBytes[7];
        servoBytes[0] = firstByte;
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
        nextServo();
    }
}

void nextDC()
{
    uint8_t firstByte = nextByte();
    if (firstByte == endDC)
    {
        nextLed();
    }
    else
    {
        uint8_t dcBytes[3];
        dcBytes[0] = firstByte;
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
        nextDC();
    }
}

void nextLed()
{
    uint8_t firstByte = nextByte();
    if (firstByte == endLeds)
    {
        nextAnalogic();
    }
    else
    {
        descArgsBuffer[0] = LED;
        descArgsBuffer[1] = getMapedPin(firstByte);
        component *tempLed = new component(descArgsBuffer);
        listLeds.add(tempLed);
        nextLed();
    }
}

void nextAnalogic()
{
    uint8_t firstByte = nextByte();
    if (firstByte == endAnalogics)
    {
        nextDigital();
    }
    else
    {
        descArgsBuffer[0] = ANALOGIC;
        descArgsBuffer[1] = getMapedPin(firstByte);
        component *tempAnalogic = new component(descArgsBuffer);
        listAnalogics.add(tempAnalogic);
        nextAnalogic();
    }
}

void nextDigital()
{
    uint8_t firstByte = nextByte();
    if (firstByte == endDigitals)
    {
        nextUltra();
    }
    else
    {
        descArgsBuffer[0] = DIGITAL;
        descArgsBuffer[1] = getMapedPin(firstByte);
        component *tempDigital = new component(descArgsBuffer);
        listDigitals.add(tempDigital);
        nextDigital();
    }
}

void nextUltra()
{
    uint8_t firstByte = nextByte();
    if (firstByte == endUltrasonics)
    {
        nextVariable();
    }
    else
    {
        uint8_t ultraBytes[2];
        ultraBytes[0] = firstByte;
        for (uint8_t i = 1; i < 2; i++)
        {
            ultraBytes[i] = nextByte();
        }
        descArgsBuffer[0] = ULTRASONIC;
        descArgsBuffer[1] = getMapedPin(ultraBytes[0]);
        descArgsBuffer[2] = getMapedPin(ultraBytes[1]);
        component *tempUltrasonic = new component(descArgsBuffer);
        listUltrasonics.add(tempUltrasonic);
        nextUltra();
    }
}

void nextVariable()
{
    uint8_t firstByte = nextByte();
    if (firstByte == endVariables)
    {
        nairdaRunMachineState(nextByte());
    }
    else
    {
        uint8_t varBytes[4];
        varBytes[0] = firstByte;
        for (uint8_t i = 1; i < 4; i++)
        {
            varBytes[i] = nextByte();
        }
        int32_t positiveValue = (varBytes[1] * 10000) + (varBytes[2] * 100) + varBytes[3];
        variable *tempVariable = new variable(varBytes[0] == 0 ? positiveValue : (positiveValue * -1));
        listVariables.add(tempVariable);
        nextVariable();
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
    while ((millis() - currentTime) < delayTime)
    {
        callInterrupt();
    }
    nairdaRunMachineState(nextByte());
}

void runSetVatValue(uint8_t id)
{
    listVariables.get(id)->setvalue(getInputValue(nextByte()));
    nairdaRunMachineState(nextByte());
}

void runServo(uint8_t id)
{
    execBuffer[0]=getInputValue(nextByte());
    listServos.get(id)->execAct(execBuffer,SERVO);
    nairdaRunMachineState(nextByte());
}

void runMotorDc(uint8_t id)
{
    int32_t vel = getInputValue(nextByte());
    vel = (vel < 0) ? 0 : (vel > 100) ? 100 : vel;

    execBuffer[0]=vel;
    execBuffer[1]=nextByte();
    
    listDC.get(id)->execAct(execBuffer,MOTOR);
    nairdaRunMachineState(nextByte());
}

void runLed(uint8_t id)
{
    int32_t intensity = getInputValue(nextByte());
    intensity = (intensity < 0) ? 0 : (intensity > 100) ? 100 : intensity;
    execBuffer[0]=intensity;
    listLeds.get(id)->execAct(execBuffer,LED);
    nairdaRunMachineState(nextByte());
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
        nairdaRunMachineState(nextByte());
    }
    else
    {
        currentOffset = eos;
        nairdaRunMachineState(nextByte());
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

    nairdaRunMachineState(nextByte());
}

void runEndRepeat()
{
    currentOffset = listRepeatBegins.get(listRepeatBegins.size() - 1)->offsetStart;
    //Serial.print("repeats: ");
    //Serial.println(listRepeatBegins.size());
    //Serial.print("Start repeat: ");
    //Serial.println(listRepeatBegins.get(0)->offsetStart);
    nairdaRunMachineState(nextByte());
}

void runBreak()
{
    repeatBegin *breakBegin = listRepeatBegins.get(listRepeatBegins.size() - 1);
    currentOffset = breakBegin->offsetEnd;
    listRepeatBegins.remove(listRepeatBegins.size() - 1);
    free(breakBegin);

    nairdaRunMachineState(nextByte());
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

#ifdef __AVR_ATmega32U4__

void freeVolatileMemory()
{
    currentOffset = 4;
    ProgrammSize = (readByte(1) * 10000) + (readByte(2) * 100) + readByte(3);
    freeVariables();
    freeRepeatBegins();
    resetLeonardoMemory();
}

#endif

uint8_t callInterrupt()
{
    uint8_t it;
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
    if (it == versionCommand)
    {
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)
        Serial1.write(((char)CURRENT_VERSION));
#endif
        Serial.write(((char)CURRENT_VERSION));
    }
    else if (it == projectInit)
    {
#ifdef __AVR_ATmega32U4__
        resetMemory();
        freeVolatileMemory();
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

void nairdaRunMachineState(uint8_t firstByte)
{
    if (callInterrupt() == 0)
    {
        switch (firstByte)
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

uint8_t getMapedPin(uint8_t pin){
  return (pin >= 70) ? (A0 + (pin - 70)) : pin;
}