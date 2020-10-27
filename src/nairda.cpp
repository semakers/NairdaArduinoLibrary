#include "nairda.h"
#include "loadFromEeprom.h"

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
bool startSaving = false;
bool executeServo = false;
bool executeDC = false;
bool executeLed = false;
int i;
int tempValue;
int runProgrammTimeOut = 0;

bool executeDCBoolean[3];
short int executeDCBuffer[3];

bool servoBoolean[7];
bool dcBoolean[3];
bool ultraosnicBoolean;
bool savingBoolean[4];

short int servoBuffer[7];
short int dcBuffer[3];
short int savingBuffer[4];

uint16_t descArgsBuffer[5];
int32_t execBuffer[2];

int32_t programmSize = 0;
uint32_t currentProgramOffset = 0;

#ifdef __AVR_ATmega32U4__
uint32_t asmOperations = 0;

void (*resetFunc)(void) = 0;

void resetLeonardoMemory()
{
  resetFunc();
}

#endif

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

void cleanSavingBoolean()
{
  for (int j = 0; j < 4; j++)
  {
    savingBoolean[j] = false;
  }
}

void cleanExecuteDCBoolean()
{
  for (int j = 0; j < 3; j++)
  {
    executeDCBoolean[j] = false;
  }
}

LinkedList<component *> listServos = LinkedList<component *>();
LinkedList<component *> listDC = LinkedList<component *>();
LinkedList<component *> listLeds = LinkedList<component *>();
LinkedList<component *> listAnalogics = LinkedList<component *>();
LinkedList<component *> listDigitals = LinkedList<component *>();
LinkedList<component *> listUltrasonics = LinkedList<component *>();

void freeCompList(LinkedList<component *> list, uint8_t type)
{
  for (int i = 0; i < list.size(); i++)
  {
    list.get(i)->off(type);
    free(list.get(i));
  }
  list.clear();
}

void resetMemory()
{
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
  cleanServoBoolean();
  cleanDCBoolean();
  cleanExecuteDCBoolean();
  cleanSavingBoolean();
  freeCompList(listServos, SERVO);
  freeCompList(listDC, MOTOR);
  freeCompList(listLeds, LED);
  freeCompList(listAnalogics, ANALOGIC);
  freeCompList(listDigitals, DIGITAL);
  freeCompList(listUltrasonics, ULTRASONIC);
}

void nairdaBegin(long int bauds)
{

#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)
  Serial1.begin(bauds);
#endif
  Serial.begin(bauds);

#ifdef __AVR_ATmega32U4__

resetOffset:
  asmOperations = 0;
  resetFunc = &&resetOffset;

#endif
  runProgrammTimeOut = millis();
  SoftPWMBegin();
}

uint8_t getMapedPin(uint8_t pin)
{

  return (pin >= 70) ? (A0 + (pin - 70)) : pin;
}

void nairdaLoop()
{
  /**/

#ifdef __AVR_ATmega32U4__

  if (asmOperations > 200000 && declaratedServos == false)
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

  if ((millis() - runProgrammTimeOut) > 800 && declaratedServos == false)
  {
    loadEepromDescriptor();
  }

#endif

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

#endif

    if (tempValue == projectInit)
    {
#ifdef __AVR_ATmega32U4__
      resetMemory();
      //resetFunc();
#else
      asm volatile("jmp 0");
#endif
      //Serial.println("Se limpriaron las listas");
    }
    if (tempValue == saveCommand)
    {
      uint8_t memoryType;
#if defined(__AVR_ATmega168__)
      startSaving = true;
      memoryType = noMemory;
#endif

#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega328P__)
      startSaving = true;
      memoryType = memory1k;
#endif

#if defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)
      startSaving = true;
      memoryType = memory4k;
#endif

#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)
      Serial1.write(((char)memoryType));
#endif
      Serial.write(((char)memoryType));
    }
    else if (startSaving)
    {
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
    }
    else if (tempValue == versionCommand)
    {
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)
      Serial1.write(((char)CURRENT_VERSION));
#endif
      Serial.write(((char)CURRENT_VERSION));
    }
    else if (tempValue == endServos)
    {

      declaratedServos = true;

      //Serial.println("Se han agregado todos los servos");
    }
    else if (tempValue == endDC)
    {
      declaratedDC = true;

      //Serial.println("Se han agregado todos los motores DC");
    }
    else if (tempValue == endLeds)
    {
      declaratedLeds = true;
      //Serial.println("Se han agregado todos los leds");
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
          descArgsBuffer[1] = getMapedPin(servoBuffer[0]);
          descArgsBuffer[2] = getMapedPin(servoBuffer[1]);
          descArgsBuffer[3] = getMapedPin(servoBuffer[2]);

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
          execBuffer[0]=map(tempValue, 0, 99, 0, 180);
          listServos.get(i)->execAct(execBuffer,SERVO);
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

            execBuffer[0]=executeDCBuffer[1];
            execBuffer[1]=executeDCBuffer[2];

            listDC.get(executeDCBuffer[0])->execAct(execBuffer,MOTOR);
            cleanExecuteDCBoolean();
            executeDC = false;
          }
        }
        else if (executeLed == true)
        {
          execBuffer[0]=tempValue;
          listLeds.get(i)->execAct(execBuffer,LED);
          executeLed = false;
        }
      }
    }
  }
}
