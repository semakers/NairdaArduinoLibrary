#include "nairda.h"
#include <EEPROM.h>
#include <Wire.h>
#include "loadFromEeprom.h"


void nairdaDebug(uint8_t tempValue);
bool running = false;
int32_t programmSize = 0;
bool startSaving = false;
#ifndef __AVR_ATmega168__

uint32_t currentProgramOffset = 0;
bool savingBoolean[4];
uint8_t savingBuffer[4];
#endif

void cleanSavingBoolean();

#if defined(ARDUINO_ARCH_ESP32)

#define BLE_INDICATOR_PIN 19

extern "C" void espShow(
  uint16_t pin, uint8_t *pixels, uint32_t numBytes, uint8_t type);

BLECharacteristic *pCharacteristic;
uint8_t bleBuffer[255];
uint8_t bleIndex = 0;
int bleIndicatorFragmentTime=0;
uint8_t bleIndicator[3]={0,0,0};
bool bleIndicatorFlag=false;

#define SERVICE_UUID "0000ffe0-0000-1000-8000-00805f9b34fb" // UART service UUID
#define CHARACTERISTIC_UUID "0000ffe1-0000-1000-8000-00805f9b34fb"




void setBleIndicatorColor(uint8_t red,uint8_t green,uint8_t blue){
  bleIndicator[0]=green;
  bleIndicator[1]=red;
  bleIndicator[2]=blue;
  espShow(BLE_INDICATOR_PIN,bleIndicator,3,false);
}

void initBleIndicator(){
  pinMode(BLE_INDICATOR_PIN,OUTPUT);
}

void deinitBleIndicator(){
  setBleIndicatorColor(0,0,0);
  pinMode(BLE_INDICATOR_PIN,INPUT);
}

class MyServerCallbacks : public BLEServerCallbacks
{
  void onConnect(BLEServer *pServer){
    bleIndicatorFlag=true;
restartRunFromEeprom();
  deinitBleIndicator();
  };
  void onDisconnect(BLEServer *pServer){
    bleIndicatorFlag=false;
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
  std::string myStringForUnit8((char*)&byte, 1);
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
bool declaratedAnalogics = false;
bool declaratedDigitals = false;
bool declaratedUltrasonics = false;

bool executeServo = false;
bool executeDC = false;
bool executeLed = false;

uint8_t i;
uint8_t tempValue;
int runProgrammTimeOut = 0;

bool executeDCBoolean[3];
uint8_t executeDCBuffer[3];

bool servoBoolean[7];
bool dcBoolean[3];
bool ultraosnicBoolean;

uint8_t servoBuffer[7];
uint8_t dcBuffer[3];

uint16_t descArgsBuffer[5];
uint32_t execBuffer[2];

#if defined(__AVR_ATmega32U4__)
uint32_t asmOperations = 0;

void (*resetFunc)(void) = 0;

void resetLeonardoMemory()
{
  resetFunc();
}

#endif

LinkedList<component *> listServos = LinkedList<component *>();
LinkedList<component *> listDC = LinkedList<component *>();
LinkedList<component *> listLeds = LinkedList<component *>();
LinkedList<component *> listAnalogics = LinkedList<component *>();
LinkedList<component *> listDigitals = LinkedList<component *>();
LinkedList<component *> listUltrasonics = LinkedList<component *>();

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

void freeCompList(LinkedList<component *> *list, uint8_t type)
{
  for (int i = 0; i < list->size(); i++)
  {
    list->get(i)->off(type);
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

#if defined(__AVR_ATmega32U4__) || (ARDUINO_ARCH_ESP32) || (ARDUINO_ARCH_STM32)
void resetMemory()
{
  runProgrammTimeOut = millis();
  declaratedDescriptor = false;
  declaratedServos = false;
  declaratedDC = false;
  declaratedLeds = false;
  declaratedAnalogics = false;
  declaratedDigitals = false;
  declaratedUltrasonics = false;
  executeServo = false;
  executeDC = false;
  executeLed = false;
  clearCurrentChannel();
  cleanServoBoolean();
  cleanDCBoolean();
  cleanExecuteDCBoolean();
  cleanSavingBoolean();
  freeCompList(&listServos, SERVO);
  freeCompList(&listDC, MOTOR);
  freeCompList(&listLeds, LED);
  freeCompList(&listAnalogics, ANALOGIC);
  freeCompList(&listDigitals, DIGITAL);
  freeCompList(&listUltrasonics, ULTRASONIC);
}
#else
void resetMemory()
{
  freeCompList(&listDC, MOTOR);
  freeCompList(&listLeds, LED);
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

  initBleIndicator();
  
  

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

  if ((millis() - runProgrammTimeOut) > 1000 && declaratedServos == false)
  {
    loadEepromDescriptor();
    runProgrammTimeOut = millis();
  }

#endif
#endif

#if defined(ARDUINO_ARCH_ESP32)
static int16_t indicatorIntensity=0;
static bool upDownIntensity=true;
if(bleIndicatorFlag==false){
  if(millis() - bleIndicatorFragmentTime>25){
    bleIndicatorFragmentTime=millis();
    if(indicatorIntensity>205)upDownIntensity=false;
    if(indicatorIntensity<10)upDownIntensity=true;

    if(upDownIntensity)indicatorIntensity=indicatorIntensity>205?255:indicatorIntensity+15;
    else indicatorIntensity=indicatorIntensity<10?0:indicatorIntensity-15;

    setBleIndicatorColor(0,0,indicatorIntensity);
  }
}


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
    memset(cleanBuffer,0,22);
    pCharacteristic->setValue(cleanBuffer);
    char myString[4];
    myString[0]=(char) firstValue(memorySize);
    myString[1]=(char)secondValue(memorySize);
    myString[2]=(char)thirdValue(memorySize);
    myString[3]=0;
    pCharacteristic->setValue(myString);
    pCharacteristic->notify();

#else

#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)
    Serial1.write((char) firstValue(memorySize));
    Serial1.write((char)secondValue(memorySize));
    Serial1.write((char)thirdValue(memorySize));
#endif
    Serial.write((char) firstValue(memorySize));
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
      savingBoolean[3] = true;
      savingBuffer[3] = tempValue;
      writeByte(0, savingBuffer[0]);
      writeByte(1, savingBuffer[1]);
      writeByte(2, savingBuffer[2]);
      writeByte(3, savingBuffer[3]);
      programmSize = (savingBuffer[1] * 10000) + (savingBuffer[2] * 100) + savingBuffer[3];
      currentProgramOffset = 4;
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

    // Serial.println("Se han agregado todos los servos");
  }
  else if (tempValue == endDC)
  {
    declaratedDC = true;

    // Serial.println("Se han agregado todos los motores DC");
  }
  else if (tempValue == endLeds)
  {
    declaratedLeds = true;
    //  Serial.println("Se han agregado todos los leds");
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
        component *tempServo = new component(descArgsBuffer);
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

        component *tempDC = new component(descArgsBuffer);
        listDC.add(tempDC);
        cleanDCBoolean();
      }
    }
    else if (declaratedLeds == false && tempValue < 100)
    {
      descArgsBuffer[0] = LED;
      descArgsBuffer[1] = getMapedPin(tempValue);
      component *tempLed = new component(descArgsBuffer);
      listLeds.add(tempLed);
    }
    else if (declaratedAnalogics == false && tempValue < 100)
    {
      descArgsBuffer[0] = ANALOGIC;
      descArgsBuffer[1] = getMapedPin(tempValue);
      component *tempAnalogic = new component(descArgsBuffer);
      listAnalogics.add(tempAnalogic);
    }
    else if (declaratedDigitals == false && tempValue < 100)
    {
      descArgsBuffer[0] = DIGITAL;
      descArgsBuffer[1] = getMapedPin(tempValue);
      component *tempDigital = new component(descArgsBuffer);
      listDigitals.add(tempDigital);
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
        component *tempUltrasonic = new component(descArgsBuffer);
        listUltrasonics.add(tempUltrasonic);
        ultraosnicBoolean = false;
      }
    }
  }
  else
  {
    if (executeServo == false && executeDC == false && executeLed == false)
    {
      int indexServos = listServos.size();
      int indexMotors = indexServos + listDC.size();
      int indexLeds = indexMotors + listLeds.size();
      int indexAnalogics = indexLeds + listAnalogics.size();
      int indexDigitals = indexAnalogics + listDigitals.size();
      int indexUltraosnics = indexDigitals + listUltrasonics.size();

      if (tempValue >= 0 && tempValue < indexServos && listServos.size() > 0)
      {
        i = tempValue;
        executeServo = true;
      }
      else if (tempValue >= indexServos && tempValue < indexMotors && listDC.size() > 0)
      {
        executeDCBuffer[0] = tempValue - indexServos;
        executeDC = true;
      }
      else if (tempValue >= indexMotors && tempValue < indexLeds && listLeds.size() > 0)
      {
        i = tempValue - indexMotors;
        executeLed = true;
      }
      else if (tempValue >= indexLeds && tempValue < indexAnalogics && listAnalogics.size() > 0)
      {
        int tempPin = tempValue - indexLeds;
        listAnalogics.get(tempPin)->sendSensVal(ANALOGIC);
      }
      else if (tempValue >= indexAnalogics && tempValue < indexDigitals && listDigitals.size() > 0)
      {
        int tempPin = tempValue - indexAnalogics;
        listDigitals.get(tempPin)->sendSensVal(DIGITAL);
      }
      else if (tempValue >= indexDigitals && tempValue < indexUltraosnics && listUltrasonics.size() > 0)
      {
        int tempPin = tempValue - indexDigitals;
        listUltrasonics.get(tempPin)->sendSensVal(ULTRASONIC);
      }
    }
    else
    {
      if (executeServo == true)
      {
        //adquirir valores para la ejecucion del servo
        execBuffer[0] = map(tempValue, 0, 99, 0, 180);
        listServos.get(i)->execAct(execBuffer, SERVO);
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

          listDC.get(executeDCBuffer[0])->execAct(execBuffer, MOTOR);
          cleanExecuteDCBoolean();
          executeDC = false;
        }
      }
      else if (executeLed == true)
      {
        execBuffer[0] = tempValue;
        listLeds.get(i)->execAct(execBuffer, LED);
        executeLed = false;
      }
    }
  }
}
