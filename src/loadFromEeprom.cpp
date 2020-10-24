#include "loadFromEeprom.h"
#include "nairda.h"
#include <EEPROM.h>



class repeatBegin
{
public:
    uint32_t offsetStart;
    uint32_t offsetEnd;
    uint8_t loop;
    uint32_t times;
    repeatBegin(uint32_t cOffsetStart,uint32_t cOffsetEnd,uint8_t cLoop,uint32_t cTimes)
    {
        offsetStart=cOffsetStart;
        offsetEnd=cOffsetEnd;
        loop=cLoop;
        times=cTimes;
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

LinkedList<servo *> listEepromServos = LinkedList<servo *>();
LinkedList<dc *> listEepromDC = LinkedList<dc *>();
LinkedList<led *> listEepromLeds = LinkedList<led *>();
LinkedList<analogic *> listEepromAnalogics = LinkedList<analogic *>();
LinkedList<digital *> listEepromDigitals = LinkedList<digital *>();
LinkedList<ultrasonic *> listEepromUltrasonics = LinkedList<ultrasonic *>();
LinkedList<variable *> listEepromVariables = LinkedList<variable *>();
LinkedList<repeatBegin *> listRepeatBegins = LinkedList<repeatBegin *>();

uint32_t currentOffset = 4;
uint32_t eepromProgrammSize=0;
//uint8_t memory[91] = {101, 102, 13, 103, 104, 105, 106, 0, 0, 0, 0, 108, 126, 1, 109, 0, 0, 0, 0, 0, 0, 91, 120, 109, 0, 0, 10, 0, 121, 0, 114, 110, 0, 0, 109, 0, 0, 0, 1, 126, 0, 109, 0, 0, 0, 2, 0, 0, 76, 124, 0, 109, 0, 0, 1, 0, 120, 109, 0, 0, 5, 0, 124, 0, 109, 0, 0, 0, 0, 120, 109, 0, 0, 5, 0, 127, 125, 111, 110, 0, 0, 109, 0, 0, 0, 3, 0, 0, 90, 128, 127};
//uint8_t memory[45] = {101, 102, 13, 103, 104, 105, 106, 108, 126, 0, 109, 0, 0, 0, 5, 0, 0, 45, 124, 0, 109, 0, 0, 0, 50, 120, 109, 0, 0, 10, 0, 124, 0, 109, 0, 0, 0, 0,120, 109, 0, 0, 10, 0, 127};


void writeByte(unsigned long int address,unsigned short int byte){
 EEPROM.update(address, byte);
}

uint8_t readByte(uint32_t address){
    return EEPROM[address];
}


uint8_t nextByte()
{
    if (currentOffset == (eepromProgrammSize+4))
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

void loaddEepromDescriptor()
{
    
    if(readByte(0)==1){
        eepromProgrammSize=(readByte(1)*10000)+(readByte(2)*100)+readByte(3);
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
        nextAnalogic();
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
        nextDigital();
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
        nextUltra();
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
    return listEepromVariables.get(nextByte())->value;
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
    uint32_t currentTime=millis();
    uint32_t delayTime=getInputValue(nextByte());
    while((millis()-currentTime)<delayTime){
        callInterrupt();
    }
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
    uint32_t sos=currentOffset-1;
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
        currentBegin= new repeatBegin(sos,eos,loop,times);
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

void freeEepromServos()
{
  for (int i = 0; i < listEepromServos.size(); i++)
  {
    listEepromServos.get(i)->off();
    free(listEepromServos.get(i));
  }
  listEepromServos.clear();
}

void freeEepromDc()
{
  for (int i = 0; i < listEepromDC.size(); i++)
  {
    listEepromDC.get(i)->off();
    free(listEepromDC.get(i));
  }
  listEepromDC.clear();
}

void freeEepromLeds()
{
  for (int i = 0; i < listEepromLeds.size(); i++)
  {
    listEepromLeds.get(i)->off();
    free(listEepromLeds.get(i));
  }
  listEepromLeds.clear();
}

void freeEepromAnalogics()
{
  for (int i = 0; i < listEepromAnalogics.size(); i++)
  {
    free(listEepromAnalogics.get(i));
  }
  listEepromAnalogics.clear();
}

void freeEepromUltrasonics()
{
  for (int i = 0; i < listEepromUltrasonics.size(); i++)
  {
    listEepromUltrasonics.get(i)->off();
    free(listEepromUltrasonics.get(i));
  }
  listEepromUltrasonics.clear();
}

void freeEepromDigitals()
{
  for (int i = 0; i < listEepromDigitals.size(); i++)
  {
    free(listEepromDigitals.get(i));
  }
  listEepromDigitals.clear();
}

void freeEepromVariables()
{
  for (int i = 0; i < listEepromVariables.size(); i++)
  {
    free(listEepromVariables.get(i));
  }
  listEepromVariables.clear();
}

void freeRepeatBegins()
{
  for (int i = 0; i < listRepeatBegins.size(); i++)
  {
    free(listRepeatBegins.get(i));
  }
  listRepeatBegins.clear();
}

void freeEEpromVolatileMemory(){
    freeEepromServos();
    freeEepromDc();
    freeEepromLeds();
    freeEepromAnalogics();
    freeEepromUltrasonics();
    freeEepromDigitals();
    freeEepromVariables();
    freeRepeatBegins();
}

uint8_t callInterrupt(){
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
    }else if(it==projectInit){
        #ifdef __AVR_ATmega32U4__
        resetMemory();
      freeEEpromVolatileMemory();
      return 1;
#else
      asm volatile("jmp 0");
#endif
    }else{
        return 0;
    }
}

void nairdaRunMachineState(uint8_t firstByte)
{
    if(callInterrupt()==0){
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