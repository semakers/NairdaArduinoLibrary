/*
Libejemplo.cpp -Descripción cpp
Creada por Nombre Autor, Fecha
Lanzado bajo licencia---
*/

//#include "arduino.h"
#include "nairda.h"
#include "loadFromEeprom.h"
#include "linked/LinkedList.h"
#include "softpwm/SoftPWM.h"

#ifndef __AVR_ATmega168__
#include "ping/NewPing.h"
#endif
#if defined(ARDUINO_ARCH_AVR)
#include "avr/Servo.h"
#elif defined(ARDUINO_ARCH_SAM)
#include "sam/Servo.h"
#else
#error "This library only supports boards with an AVR or SAM processor."
#endif

enum {noMemory, memory1k, memory4k, memory256k,memory512k};

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
int runProgrammTimeOut=0;

bool executeDCBoolean[3];
short int executeDCBuffer[3];

bool servoBoolean[7];
bool dcBoolean[3];
bool ultraosnicBoolean;
bool savingBoolean[4];

short int servoBuffer[7];
short int dcBuffer[3];
short int savingBuffer[4];
uint32_t programmSize=0;
uint32_t currentProgramOffset=0;

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

void cleanExecuteDCBoolean()
{
  for (int j = 0; j < 3; j++)
  {
    executeDCBoolean[j] = false;
  }
}

LinkedList<servo *> listServos = LinkedList<servo *>();
LinkedList<dc *> listDC = LinkedList<dc *>();
LinkedList<led *> listLeds = LinkedList<led *>();
LinkedList<analogic *> listAnalogics = LinkedList<analogic *>();
LinkedList<digital *> listDigitals = LinkedList<digital *>();
LinkedList<ultrasonic *> listUltrasonics = LinkedList<ultrasonic *>();

#ifndef __AVR_ATmega168__

void freeServos()
{
  for (int i = 0; i < listServos.size(); i++)
  {
    listServos.get(i)->off();
    free(listServos.get(i));
  }
  listServos.clear();
}

void freeDc()
{
  for (int i = 0; i < listDC.size(); i++)
  {
    listDC.get(i)->off();
    free(listDC.get(i));
  }
  listDC.clear();
}

void freeLeds()
{
  for (int i = 0; i < listLeds.size(); i++)
  {
    listLeds.get(i)->off();
    free(listLeds.get(i));
  }
  listLeds.clear();
}

void freeAnalogics()
{
  for (int i = 0; i < listAnalogics.size(); i++)
  {
    free(listAnalogics.get(i));
  }
  listAnalogics.clear();
}

void freeUltrasonics()
{
  for (int i = 0; i < listUltrasonics.size(); i++)
  {
    listUltrasonics.get(i)->off();
    free(listUltrasonics.get(i));
  }
  listUltrasonics.clear();
}

void freeDigitals()
{
  for (int i = 0; i < listDigitals.size(); i++)
  {
    free(listDigitals.get(i));
  }
  listDigitals.clear();
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
  freeServos();
  freeDc();
  freeLeds();
  freeAnalogics();
  freeUltrasonics();
  freeDigitals();
   runProgrammTimeOut=millis();
}
#endif

void nairdaBegin(long int bauds)
{
  runProgrammTimeOut=millis();

#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)
  Serial1.begin(bauds);
#endif
  Serial.begin(bauds);
  SoftPWMBegin();
  
}

uint8_t getMapedPin(uint8_t pin)
{

  return (pin >= 70) ? (A0 + (pin - 70)) : pin;
}

void nairdaLoop()
{

  if((millis()-runProgrammTimeOut)>650 && declaratedServos==false){
    loaddEepromDescriptor();
  }

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

       if (startSaving)
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
        
      }else if (!savingBoolean[3])
      {
        savingBoolean[3] = true;
        savingBuffer[3] = tempValue;
        writeByte(0, savingBuffer[0]);
        writeByte(1, savingBuffer[1]);
        writeByte(2, savingBuffer[2]);
        writeByte(3, savingBuffer[3]);
        programmSize = (savingBuffer[1] * 10000) + (savingBuffer[2] * 100) + savingBuffer[3];
        currentProgramOffset=4;
      }else{
        if(programmSize>0){
           writeByte(currentProgramOffset, tempValue);
           currentProgramOffset++;
           programmSize--;
        }else{
          startSaving=false;
        }
        
      }
    }

    if (tempValue == projectInit)
    {
#ifdef __AVR_ATmega32U4__
      resetMemory();
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
      memoryType=noMemory;
      #endif

      #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega328P__)
      startSaving = true;
      memoryType=memory1k;
      #endif

      #if defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__) 
      startSaving = true;
      memoryType=memory4k;
      #endif

      #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)
      Serial1.write(((char)memoryType));
#endif
      Serial.write(((char)memoryType));

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
          servo *tempServo = new servo(getMapedPin(servoBuffer[0]), (servoBuffer[1] * 100) + servoBuffer[2], (servoBuffer[3] * 100) + servoBuffer[4], (servoBuffer[5] * 100) + servoBuffer[6]);
          listServos.add(tempServo);
          cleanServoBoolean();

          /*Serial.print(tempServo->pin);
          Serial.print(" : ");
          Serial.print(tempServo->pmin);
          Serial.print(" : ");
          Serial.print(tempServo->pmax);
          Serial.print(" : ");
          Serial.println(tempServo->dpos);*/
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
          dc *tempDC = new dc(getMapedPin(dcBuffer[0]), getMapedPin(dcBuffer[1]), getMapedPin(dcBuffer[2]));
          listDC.add(tempDC);
          cleanDCBoolean();

          /*Serial.print("se agrego el motor DC ");
          Serial.print(tempDC->a);
          Serial.print(" : ");
          Serial.print(tempDC->b);
          Serial.print(" : ");
          Serial.println(tempDC->pwm);*/
        }
      }
      else if (declaratedLeds == false && tempValue < 100)
      {
        led *tempLed = new led(getMapedPin(tempValue));
        listLeds.add(tempLed);
        //Serial.print("se agrego el led ");
        //Serial.println(tempLed->pin);
      }
      else if (declaratedAnalogics == false && tempValue < 100)
      {
        analogic *tempAnalogic = new analogic(getMapedPin(tempValue));
        listAnalogics.add(tempAnalogic);
      }
      else if (declaratedDigitals == false && tempValue < 100)
      {
        digital *tempDigital = new digital(getMapedPin(tempValue));
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
          ultrasonic *tempUltrasonic = new ultrasonic(getMapedPin(i), getMapedPin(tempValue));
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
          listAnalogics.get(tempPin)->sendValue();
        }
        else if (tempValue >= indexAnalogics && tempValue < indexDigitals && listDigitals.size() > 0)
        {
          int tempPin = tempValue - indexAnalogics;
          listDigitals.get(tempPin)->sendValue();
        }
        else if (tempValue >= indexDigitals && tempValue < indexUltraosnics && listUltrasonics.size() > 0)
        {
          int tempPin = tempValue - indexDigitals;
          listUltrasonics.get(tempPin)->sendValue();
        }
      }
      else
      {
        if (executeServo == true)
        {
          //adquirir valores para la ejecucion del servo
          listServos.get(i)->setPos(tempValue);
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
            listDC.get(executeDCBuffer[0])->setVel(executeDCBuffer[1]);
            listDC.get(executeDCBuffer[0])->setMove(executeDCBuffer[2]);
            /*Serial.print(" se ejecuto el motor");
            Serial.print(" : ");
            Serial.print(executeDCBuffer[1]);
            Serial.print(" : ");
            Serial.println(executeDCBuffer[2]); */
            cleanExecuteDCBoolean();
            executeDC = false;
          }
        }
        else if (executeLed == true)
        {
          listLeds.get(i)->setPWM(tempValue);
          //Serial.print((char)okResponse);
          //Serial.print(" se ejecuto");
          executeLed = false;
        }
      }
    }
  }
}
