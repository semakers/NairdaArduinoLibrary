#include "loadFromEeprom.h"
#include "nairda.h"

typedef struct
{
    uint32_t offsetStart;
    uint32_t offsetEnd;
    uint8_t loop;
    uint32_t times;
} repeatBegin;

void nextServo();
void nextDC();
void nextLed();
void nextAnalogic();
void nextDigital();
void nextUltra();
void nextVariable();
void nairdaRunMachineState(uint8_t firstByte);
int32_t getInputValue(uint8_t firstByte);

LinkedList<servo *> listEepromServos = LinkedList<servo *>();
LinkedList<dc *> listEepromDC = LinkedList<dc *>();
LinkedList<led *> listEepromLeds = LinkedList<led *>();
LinkedList<analogic *> listEepromAnalogics = LinkedList<analogic *>();
LinkedList<digital *> listEepromDigitals = LinkedList<digital *>();
LinkedList<ultrasonic *> listEepromUltrasonics = LinkedList<ultrasonic *>();
LinkedList<variable *> listEepromVariables = LinkedList<variable *>();
LinkedList<repeatBegin *> listRepeatBegins = LinkedList<repeatBegin *>();

uint32_t currentOffset = 0;
uint8_t memory[10] = {};

uint8_t nextByte()
{
    static uint8_t auxByte = memory[currentOffset];
    currentOffset++;
    return auxByte;
}

void loaddEepromDescriptor()
{
    nextServo();
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
        servo *tempServo = new servo(getMapedPin(servoBytes[0]), (servoBytes[1] * 100) + servoBytes[2], (servoBytes[3] * 100) + servoBytes[4], (servoBytes[5] * 100) + servoBytes[6]);
        listEepromServos.add(tempServo);
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
        dc *tempDC = new dc(getMapedPin(dcBytes[0]), getMapedPin(dcBytes[1]), getMapedPin(dcBytes[2]));
        listEepromDC.add(tempDC);
        nextDC();
    }
}

void nextLed()
{
    uint8_t firstByte = nextByte();
    if (firstByte == endLeds)
    {
    }
    else
    {
        led *tempLed = new led(getMapedPin(firstByte));
        listEepromLeds.add(tempLed);
        nextLed();
    }
}

void nextAnalogic()
{
    uint8_t firstByte = nextByte();
    if (firstByte == endAnalogics)
    {
    }
    else
    {
        analogic *tempAnalogic = new analogic(getMapedPin(firstByte));
        listEepromAnalogics.add(tempAnalogic);
        nextAnalogic();
    }
}

void nextDigital()
{
    uint8_t firstByte = nextByte();
    if (firstByte == endDigitals)
    {
    }
    else
    {
        digital *tempDigital = new digital(getMapedPin(firstByte));
        listEepromDigitals.add(tempDigital);
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
        ultrasonic *tempUltrasonic = new ultrasonic(getMapedPin(ultraBytes[0]), getMapedPin(ultraBytes[1]));
        listEepromUltrasonics.add(tempUltrasonic);
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
        listEepromVariables.add(tempVariable);
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
    listEepromVariables.get(nextByte())->value;
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

int32_t getAnalogicValue()
{
   return listEepromAnalogics.get(nextByte())->getValue();
}

int32_t getDigitalValue()
{
   return listEepromDigitals.get(nextByte())->getValue();
}

int32_t getUltraValue()
{
    return listEepromUltrasonics.get(nextByte())->getValue();
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
    delay(getInputValue(nextByte()));
    nairdaRunMachineState(nextByte());
}

void runSetVatValue(uint8_t id)
{
    listEepromVariables.get(id)->setvalue(getInputValue(nextByte()));
    nairdaRunMachineState(nextByte());
}

void runServo(uint8_t id)
{
    listEepromServos.get(id)->setRawPos(getInputValue(nextByte()));
    nairdaRunMachineState(nextByte());
}

void runMotorDc(uint8_t id)
{
    int32_t vel = getInputValue(nextByte());
    vel = (vel < 0) ? 0 : (vel > 100) ? 100 : vel;
    listEepromDC.get(id)->setVel(vel);
    listEepromDC.get(id)->setMove(nextByte());
    nairdaRunMachineState(nextByte());
}

void runLed(uint8_t id)
{
    int32_t intensity = getInputValue(nextByte());
    intensity = (intensity < 0) ? 0 : (intensity > 100) ? 100 : intensity;
    listEepromLeds.get(id)->setPWM(intensity);
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
    uint32_t eos = (eosBytes[0] * 10000) + (eosBytes[1] * 100) + eosBytes[2];
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

uint8_t findRepeatBeginIndex(uint32_t repeatStartOffset){
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
    repeatBegin *currentBegin = findRepeatBegin(currentOffset);
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
        currentBegin->offsetStart = currentOffset;
        currentBegin->loop = loop;
        currentBegin->times = times;
        currentBegin->offsetEnd = eos;
        listRepeatBegins.add(currentBegin);
    }

        if(currentBegin->loop ==0){
            if(currentBegin->times==0){
            currentOffset=currentBegin->offsetStart;
            listRepeatBegins.remove(findRepeatBeginIndex(currentBegin->offsetStart));
            free(currentBegin);
            //listRepeatBegins.remove(listRepeatBegins.size()-1);


        }else{
            currentBegin->times=currentBegin->times-1;  
        }
        }
        
    nairdaRunMachineState(nextByte());

}


void runEndRepeat(){
    currentOffset=listRepeatBegins.get(listRepeatBegins.size()-1)->offsetStart;
    nairdaRunMachineState(nextByte());
}

void runBreak(){
    repeatBegin * breakBegin= listRepeatBegins.get(listRepeatBegins.size()-1);
    currentOffset=breakBegin->offsetEnd;
    listRepeatBegins.remove(listRepeatBegins.size()-1);
    free(breakBegin);
    
    nairdaRunMachineState(nextByte());
}

void nairdaRunMachineState(uint8_t firstByte)
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
    break;
    }
}