#include "nairda.h"
#include <EEPROM.h>
#include <Wire.h>
#include "load_from_eeprom.h"
#include "components/component.h"

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

#if defined(ARDUINO_ARCH_ESP32)


extern "C" void espShow(
    uint16_t pin, uint8_t *pixels, uint32_t numBytes, uint8_t type);

BLECharacteristic *pCharacteristic;
uint8_t bleBuffer[255];
uint8_t bleIndex = 0;

#define SERVICE_UUID "0000ffe0-0000-1000-8000-00805f9b34fb" // UART service UUID
#define CHARACTERISTIC_UUID "0000ffe1-0000-1000-8000-00805f9b34fb"



class MyServerCallbacks : public BLEServerCallbacks
{
  void onConnect(BLEServer *pServer)
  {
    preInit = true;
    restartRunFromEeprom();
  };
  void onDisconnect(BLEServer *pServer)
  {
    preInit = false;
    resetMemory();
    BLEDevice::startAdvertising();
  };
};

class MyCallbacks : public BLECharacteristicCallbacks
{
  void onWrite(BLECharacteristic *pCharacteristic)
  {
    std::string rxValue = pCharacteristic->getValue();
    if (rxValue.length() > 0)
    {
      for (int i = 0; i < rxValue.length(); i++)
      {
        if (!running)
        {
          nairdaDebug(rxValue[i]);
        }
        else
        {
          bleBuffer[bleIndex] = (uint8_t)rxValue[(rxValue.length() - 1) - i];
          bleIndex++;
        }
      }
    }
  }
};

bool bleAvailable()
{
  if (bleIndex > 0)
  {
    return true;
  }
  else
  {
    return false;
  }
}

uint8_t bleRead()
{
  if (bleAvailable())
  {
    bleIndex--;
    return bleBuffer[bleIndex];
  }
  else
  {
    return 0;
  }
}

void bleWrite(uint8_t byte)
{
  std::string myStringForUnit8((char *)&byte, 1);
  pCharacteristic->setValue(myStringForUnit8);
  pCharacteristic->notify();
  // pCharacteristic->indicate();
}


#endif

enum
{
  noMemory,
  memory1k,
  memory4k,
  memory256k,
  memory512k
};

bool declaratedDescriptor = false;
bool declaratedServos = false;
bool declaratedDC = false;
bool declaratedLeds = false;
bool declaratedFrequencies = false;
bool declaratedNeopixels =false;
bool declaratedAnalogics = false;
bool declaratedDigitals = false;
bool declaratedUltrasonics = false;

bool executeServo = false;
bool executeDC = false;
bool executeLed = false;
bool executeFrequency = false;
bool executeNeopixel =false;

uint8_t i;
uint8_t tempValue;
int runProgrammTimeOut = 0;

bool executeDCBoolean[3];
uint8_t executeDCBuffer[3];


bool executeFrequencyBoolean[7];
uint8_t executeFrequencyBuffer[7];

bool executeNeopixelBoolean[5];
uint8_t executeNeopixelBuffer[5];

bool servoBoolean[7];
bool dcBoolean[3];
bool ultraosnicBoolean;
bool neopixelBoolean;

uint8_t servoBuffer[7];
uint8_t dcBuffer[3];

uint16_t descArgsBuffer[5];
uint32_t execBuffer[6];


#if defined(__AVR_ATmega32U4__)
uint32_t asmOperations = 0;

void (*resetFunc)(void) = 0;

void resetLeonardoMemory()
{
  resetFunc();
}

#endif

LinkedList<component_t *> listServos = LinkedList<component_t *>();
LinkedList<component_t *> listMotors = LinkedList<component_t *>();
LinkedList<component_t *> listDigitalOuts = LinkedList<component_t *>();
LinkedList<component_t *> listFrequencies = LinkedList<component_t *>();
LinkedList<component_t *> listNeopixels = LinkedList<component_t *>();
LinkedList<component_t *> listAnalogics = LinkedList<component_t *>();
LinkedList<component_t *> listDigitalIns = LinkedList<component_t *>();
LinkedList<component_t *> listUltrasonics = LinkedList<component_t *>();

uint8_t firstValue(uint32_t value)
{
  return (uint8_t)(value / 10000);
}

uint8_t secondValue(uint32_t value)
{
  uint8_t first = firstValue(value);
  return (uint8_t)(first > 0) ? ((value - (first * 10000)) / 100) : (value / 100);
}

uint8_t thirdValue(uint32_t value)
{
  return uint8_t(value - ((uint8_t(value / 100)) * 100));
}

void freeCompList(LinkedList<component_t *> *list, uint8_t type)
{
  for (int i = 0; i < list->size(); i++)
  {
    off(type,list->get(i));
    free(list->get(i));
  }
  list->clear();
}

void cleanServoBoolean()
{
  for (int j = 0; j < 7; j++)
  {
    servoBoolean[j] = false;
  }
}

void cleanDCBoolean()
{
  for (int j = 0; j < 3; j++)
  {
    dcBoolean[j] = false;
  }
}

#ifndef __AVR_ATmega168__
void cleanSavingBoolean()
{
  for (int j = 0; j < 4; j++)
  {
    savingBoolean[j] = false;
  }
}
#endif

void cleanExecuteDCBoolean()
{
  for (int j = 0; j < 3; j++)
  {
    executeDCBoolean[j] = false;
  }
}

void cleanExecuteFrequencyBoolean(){
  for (int j = 0; j < 7; j++)
  {
    executeFrequencyBoolean[j] = false;
  }
}

void cleanExecuteNeopixelBoolean(){
  for (int j = 0; j < 5; j++)
  {
    executeNeopixelBoolean[j] = false;
  }
}

#if defined(__AVR_ATmega32U4__) || (ARDUINO_ARCH_ESP32) || (ARDUINO_ARCH_STM32)
void resetMemory()
{

  runProgrammTimeOut = millis();
  declaratedDescriptor = false;
  declaratedServos = false;
  declaratedDC = false;
  declaratedLeds = false;
  declaratedFrequencies =false;
  declaratedNeopixels =false;
  declaratedAnalogics = false;
  declaratedDigitals = false;
  declaratedUltrasonics = false;
  executeServo = false;
  executeDC = false;
  executeLed = false;
  executeFrequency =false;
  executeNeopixel =false;
  clearCurrentChannel();
  cleanServoBoolean();
  cleanDCBoolean();
  cleanExecuteDCBoolean();
  cleanExecuteFrequencyBoolean();
  cleanExecuteNeopixelBoolean();
  cleanSavingBoolean();
  freeCompList(&listServos, SERVO);
  freeCompList(&listMotors, MOTOR);
  freeCompList(&listDigitalOuts, DIGITAL_OUT);
  freeCompList(&listFrequencies, FREQUENCY);
  freeCompList(&listAnalogics, ANALOGIC);
  freeCompList(&listDigitalIns, DIGITAL_IN);
  freeCompList(&listUltrasonics, ULTRASONIC);
  freeCompList(&listNeopixels,NEOPIXEL);
}
#else
void resetMemory()
{
  freeCompList(&listMotors, MOTOR);
  freeCompList(&listDigitalOuts, DIGITAL_OUT);
}

#endif

#if defined(ARDUINO_ARCH_ESP32)
void nairdaBegin(const char *deviceName)
{

  BLEDevice::init(deviceName); // Give it a name

  // Create the BLE Server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  pCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_NOTIFY |
          BLECharacteristic::PROPERTY_WRITE |
          BLECharacteristic::PROPERTY_WRITE_NR |
          BLECharacteristic::PROPERTY_READ);
  pCharacteristic->addDescriptor(new BLE2902());

  pCharacteristic->setCallbacks(new MyCallbacks());

  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06); // functions that help with iPhone connections issue
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();

 // initBleIndicator();



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

#ifdef __AVR_ATmega32U4__

resetOffset:
  asmOperations = 0;
  resetFunc = &&resetOffset;

#endif
  runProgrammTimeOut = millis();
}



void nairdaLoop()
{

  /**/
#ifndef __AVR_ATmega168__
#ifdef __AVR_ATmega32U4__

  if (asmOperations > 250000 && declaratedServos == false)
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
  if ((millis() - runProgrammTimeOut) > 2500 && preInit == false)
  {
#else
  if ((millis() - runProgrammTimeOut) > 2500 && declaratedServos == false)
  {
#endif
    
    loadEepromDescriptor();
    runProgrammTimeOut = millis();
  }

#endif
#endif

#if defined(ARDUINO_ARCH_ESP32)

 

  if (bleAvailable())
  {

    tempValue = bleRead();
#else

#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)
  int serialAvailable = Serial.available();
  int serial1Available = Serial1.available();
  if (serialAvailable > 0 || serial1Available > 0)
  {
    if (serialAvailable > 0)
    {
      tempValue = Serial.read();
    }
    else if (serial1Available > 0)
    {
      tempValue = Serial1.read();
    }

#else

  if (Serial.available())
  {
    tempValue = Serial.read();
    // Serial.println(tempValue);

#endif

    nairdaDebug(tempValue);

#endif
  }
}

void nairdaDebug(uint8_t tempValue)
{
  if (tempValue == projectInit)
  {
#if defined(__AVR_ATmega32U4__) || (ARDUINO_ARCH_ESP32) || (ARDUINO_ARCH_STM32)
    resetMemory();
#else
    resetMemory();
    asm volatile("jmp 0");
#endif
    //Serial.println("Se limpriaron las listas");
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

#if defined(ARDUINO_ARCH_ESP32)
    spi_flash_erase_range(0x200000, 4096 * 128);
    char cleanBuffer[22];
    memset(cleanBuffer, 0, 22);
    pCharacteristic->setValue(cleanBuffer);
    char myString[4];
    myString[0] = (char)firstValue(memorySize);
    myString[1] = (char)secondValue(memorySize);
    myString[2] = (char)thirdValue(memorySize);
    myString[3] = 0;
    pCharacteristic->setValue(myString);
    pCharacteristic->notify();

#else

#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)
    Serial1.write((char)firstValue(memorySize));
    Serial1.write((char)secondValue(memorySize));
    Serial1.write((char)thirdValue(memorySize));
#endif
    Serial.write((char)firstValue(memorySize));
    Serial.write((char)secondValue(memorySize));
    Serial.write((char)thirdValue(memorySize));
#endif
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
      programmSize = (savingBuffer[1] * 10000) + (savingBuffer[2] * 100) + savingBuffer[3]-currentProgramOffset;
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
  else if (tempValue == endServos)
  {
    declaratedServos = true;
  }
  else if (tempValue == endDC)
  {
    declaratedDC = true;
  }
  else if (tempValue == endLeds)
  {
    declaratedLeds = true;
  }
  else if (tempValue == endFrequencies)
  {
    declaratedFrequencies = true;
  }
  else if(tempValue == endNeopixels)
  {
    declaratedNeopixels =true;
  }
  else if (tempValue == endAnalogics)
  {
    declaratedAnalogics = true;
  }
  else if (tempValue == endDigitals)
  {
    declaratedDigitals = true;
  }
  else if (tempValue == endUltrasonics)
  {
    declaratedUltrasonics = true;
    declaratedDescriptor = true;
  }

  else if (declaratedDescriptor == false && tempValue < 100)
  {
    if (declaratedServos == false)
    {
      //pasos para declarar un servo
      if (!servoBoolean[0])
      {
        servoBoolean[0] = true;
        servoBuffer[0] = tempValue;
      }
      else if (!servoBoolean[1])
      {
        servoBoolean[1] = true;
        servoBuffer[1] = tempValue;
      }
      else if (!servoBoolean[2])
      {
        servoBoolean[2] = true;
        servoBuffer[2] = tempValue;
      }
      else if (!servoBoolean[3])
      {
        servoBoolean[3] = true;
        servoBuffer[3] = tempValue;
      }
      else if (!servoBoolean[4])
      {
        servoBoolean[4] = true;
        servoBuffer[4] = tempValue;
      }
      else if (!servoBoolean[5])
      {
        servoBoolean[5] = true;
        servoBuffer[5] = tempValue;
      }
      else if (!servoBoolean[6])
      {
        servoBoolean[6] = true;
        servoBuffer[6] = tempValue;

        descArgsBuffer[0] = SERVO;
        descArgsBuffer[1] = getMapedPin(servoBuffer[0]);
        descArgsBuffer[2] = (servoBuffer[1] * 100) + servoBuffer[2];
        descArgsBuffer[3] = (servoBuffer[3] * 100) + servoBuffer[4];
        descArgsBuffer[4] = (servoBuffer[5] * 100) + servoBuffer[6];
        component_t *tempServo = newComponent(descArgsBuffer);
        listServos.add(tempServo);
        cleanServoBoolean();
      }
    }
    else if (declaratedDC == false && tempValue < 100)
    {
      //pasos para declarar un motor DC
      if (!dcBoolean[0])
      {
        dcBoolean[0] = true;
        dcBuffer[0] = tempValue;
      }
      else if (!dcBoolean[1])
      {
        dcBoolean[1] = true;
        dcBuffer[1] = tempValue;
      }
      else if (!dcBoolean[2])
      {
        dcBoolean[2] = true;
        dcBuffer[2] = tempValue;

        descArgsBuffer[0] = MOTOR;
        descArgsBuffer[1] = getMapedPin(dcBuffer[0]);
        descArgsBuffer[2] = getMapedPin(dcBuffer[1]);
        descArgsBuffer[3] = getMapedPin(dcBuffer[2]);

        component_t *tempDC = newComponent(descArgsBuffer);
        listMotors.add(tempDC);
        cleanDCBoolean();
      }
    }
    else if (declaratedLeds == false && tempValue < 100)
    {
      descArgsBuffer[0] = DIGITAL_OUT;
      descArgsBuffer[1] = getMapedPin(tempValue);
      component_t *tempLed = newComponent(descArgsBuffer);
      listDigitalOuts.add(tempLed);
    }
    else if (declaratedFrequencies == false && tempValue < 100)
    {
      descArgsBuffer[0] = FREQUENCY;
      descArgsBuffer[1] = getMapedPin(tempValue);
      component_t *tempFrequency = newComponent(descArgsBuffer);
      listFrequencies.add(tempFrequency);
    }
    else if (declaratedNeopixels == false && tempValue <100)
    {
      if(!neopixelBoolean){
        neopixelBoolean=true;
        i=tempValue;
      }else{
        descArgsBuffer[0]=NEOPIXEL;
        descArgsBuffer[1]=getMapedPin(i);
        descArgsBuffer[2]=tempValue;
        component_t *tempNeopixel = newComponent(descArgsBuffer);
        listNeopixels.add(tempNeopixel);
      }
    }
    else if (declaratedAnalogics == false && tempValue < 100)
    {
      descArgsBuffer[0] = ANALOGIC;
      descArgsBuffer[1] = getMapedPin(tempValue);
      component_t *tempAnalogic = newComponent(descArgsBuffer);
      listAnalogics.add(tempAnalogic);
    }
    else if (declaratedDigitals == false && tempValue < 100)
    {
      descArgsBuffer[0] = DIGITAL_IN;
      descArgsBuffer[1] = getMapedPin(tempValue);
      component_t *tempDigital = newComponent(descArgsBuffer);
      listDigitalIns.add(tempDigital);
    }
    else if (declaratedUltrasonics == false && tempValue < 100)
    {
      if (!ultraosnicBoolean)
      {
        ultraosnicBoolean = true;
        i = tempValue;
      }
      else
      {
        descArgsBuffer[0] = ULTRASONIC;
        descArgsBuffer[1] = getMapedPin(i);
        descArgsBuffer[2] = getMapedPin(tempValue);
        component_t *tempUltrasonic = newComponent(descArgsBuffer);
        listUltrasonics.add(tempUltrasonic);
        ultraosnicBoolean = false;
      }
    }
  }
  else
  {
    if (executeServo == false && executeDC == false && executeLed == false && executeFrequency ==false && executeNeopixel==false)
    {
      int indexServos = listServos.size();
      int indexMotors = indexServos + listMotors.size();
      int indexLeds = indexMotors + listDigitalOuts.size();
      int indexFrequencies = indexLeds + listFrequencies.size();
      int indexNeopixels = indexFrequencies + listNeopixels.size();
      int indexAnalogics = indexNeopixels + listAnalogics.size();
      int indexDigitals = indexAnalogics + listDigitalIns.size();
      int indexUltraosnics = indexDigitals + listUltrasonics.size();
      

      if (tempValue >= 0 && tempValue < indexServos && listServos.size() > 0)
      {
        i = tempValue;
        executeServo = true;
      }
      else if (tempValue >= indexServos && tempValue < indexMotors && listMotors.size() > 0)
      {
        executeDCBuffer[0] = tempValue - indexServos;
        executeDC = true;
      }
      else if (tempValue >= indexMotors && tempValue < indexLeds && listDigitalOuts.size() > 0)
      {
        i = tempValue - indexMotors;
        executeLed = true;
      }
      else if (tempValue >= indexLeds && tempValue < indexFrequencies && listFrequencies.size() > 0)
      {
        executeFrequencyBuffer[0]= tempValue - indexLeds;
        executeFrequency = true;
      }
      else if (tempValue >= indexFrequencies && tempValue < indexNeopixels && listNeopixels.size() > 0)
      {
        executeNeopixelBuffer[0]= tempValue - indexFrequencies;
        executeNeopixel = true;
      }
      else if (tempValue >= indexNeopixels && tempValue < indexAnalogics && listAnalogics.size() > 0)
      {
        int tempPin = tempValue - indexNeopixels;
        sendSensVal(ANALOGIC,listAnalogics.get(tempPin));
      }
      else if (tempValue >= indexAnalogics && tempValue < indexDigitals && listDigitalIns.size() > 0)
      {
        int tempPin = tempValue - indexAnalogics;
        sendSensVal(DIGITAL_IN,listDigitalIns.get(tempPin));
      }
      else if (tempValue >= indexDigitals && tempValue < indexUltraosnics && listUltrasonics.size() > 0)
      {
        int tempPin = tempValue - indexDigitals;
        sendSensVal(ULTRASONIC,listUltrasonics.get(tempPin));
      }
    }
    else
    {
      if (executeServo == true)
      {
        //adquirir valores para la ejecucion del servo
        execBuffer[0] = map(tempValue, 0, 99, 0, 180);
        execAct(execBuffer, SERVO,listServos.get(i));
        executeServo = false;
      }
      else if (executeDC == true)
      {
        //adquirir valores para la ejecucion del motor
        if (!executeDCBoolean[0])
        {
          executeDCBoolean[0] = true;
          executeDCBuffer[1] = tempValue;
        }
        else if (!executeDCBoolean[1])
        {
          executeDCBoolean[1] = true;
          executeDCBuffer[2] = tempValue;

          execBuffer[0] = executeDCBuffer[1];
          execBuffer[1] = executeDCBuffer[2];

          execAct(execBuffer, MOTOR,listMotors.get(executeDCBuffer[0]));
          cleanExecuteDCBoolean();
          executeDC = false;
        }
      }
      else if (executeLed == true)
      {
        execBuffer[0] = tempValue;
        execAct(execBuffer, DIGITAL_OUT,listDigitalOuts.get(i));
        executeLed = false;
      }
      else if (executeFrequency == true)
      {
         if (!executeFrequencyBoolean[0]){
            executeFrequencyBoolean[0] = true;
            executeFrequencyBuffer[1] = tempValue;
          }else if (!executeFrequencyBoolean[1]){
            executeFrequencyBoolean[1] = true;
            executeFrequencyBuffer[2] = tempValue;
          }else if (!executeFrequencyBoolean[2]){
            executeFrequencyBoolean[2] = true;
            executeFrequencyBuffer[3] = tempValue;
          }else if (!executeFrequencyBoolean[3]){
            executeFrequencyBoolean[3] = true;
            executeFrequencyBuffer[4] = tempValue;
          }else if (!executeFrequencyBoolean[4]){
            executeFrequencyBoolean[4] = true;
            executeFrequencyBuffer[5] = tempValue;
          }else if (!executeFrequencyBoolean[5])
        {
          executeFrequencyBoolean[5] = true;
          executeFrequencyBuffer[6] = tempValue;

          for(uint8_t i=0;i<6;i++)execBuffer[i] = executeFrequencyBuffer[i+1];

          execAct(execBuffer, FREQUENCY,listFrequencies.get(executeFrequencyBuffer[0]));
          cleanExecuteFrequencyBoolean();
          executeFrequency = false;
        }
      }
      else if(executeNeopixel == true){
        if(!executeNeopixelBoolean[0]){
          executeNeopixelBoolean[0]=true;
          executeNeopixelBuffer[1] = map(tempValue,0,99,0,255);
        }else if(!executeNeopixelBoolean[1]){
          executeNeopixelBoolean[1]=true;
          executeNeopixelBuffer[2] =  map(tempValue,0,99,0,255);
        }else if(!executeNeopixelBoolean[2]){
          executeNeopixelBoolean[2]=true;
          executeNeopixelBuffer[3] =  map(tempValue,0,99,0,255);
        }else if(!executeNeopixelBoolean[3]){
          executeNeopixelBoolean[3]=true;
          executeNeopixelBuffer[4] = tempValue;

          for(uint8_t i=0;i<4;i++)execBuffer[i] = executeNeopixelBuffer[i+1];
            execAct(execBuffer,NEOPIXEL,listNeopixels.get(executeNeopixelBuffer[0]));
            cleanExecuteNeopixelBoolean();
            executeNeopixel=false;
        }
      }
    }
  }
}
