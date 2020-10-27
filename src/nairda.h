#ifndef nairda_h
#define nairda_h
#include "linked/LinkedList.h"
#include "softpwm/SoftPWM.h"

#include "ping/NewPing.h"
#if defined(ARDUINO_ARCH_AVR)
#include "avr/Servo.h"
#elif defined(ARDUINO_ARCH_SAM)
#include "sam/Servo.h"
#else
#error "This library only supports boards with an AVR or SAM processor."
#endif

#ifdef __AVR_ATmega168__
#error "Nairda dont support atmega 168. "
#endif

#define projectInit 100
#define endServos 101
#define endDC 102
#define endLeds 103
#define endAnalogics 104
#define endDigitals 105
#define endUltrasonics 106
#define versionCommand 107
#define endVariables 108
#define valueCommand 109
#define variableCommand 110
#define comparatorCommand 111
#define logicCommand 112
#define notCommand 113
#define aritmeticCommand 114
#define mapCommand 115
#define analogicCommand 116
#define digitalCommand 117
#define ultrasonicCommand 118
#define delayCommand 120
#define setVarValueCommand 121
#define servoCommand 122
#define motorDcCommand 123
#define ledCommand 124
#define ifCommand 125
#define repeatCommand 126
#define endRepeatCommand 127
#define breakCommand 128
#define saveCommand 129

#define CURRENT_VERSION 1

enum
{
  SERVO = 0,
  MOTOR,
  LED,
  DIGITAL,
  ANALOGIC,
  ULTRASONIC
};

class component
{
public:
  //analogc digital led
  uint8_t pin;
  //ultrasonic
  NewPing *sonar;
  //servo
  Servo servo;
  //dcMotor
  uint8_t a, b, pwm, vel;

  component(uint16_t *args)
  {
    switch (args[0])
    {
    case SERVO:
      servo.attach(args[1], args[2], args[3]);
      servo.write(args[4]);
      break;
    case MOTOR:
      pinMode(args[1], OUTPUT);
      pinMode(args[2], OUTPUT);
      SoftPWMSet(args[3], 0);
      a = args[1];
      b = args[2];
      pwm = args[2];
      break;
    case LED:
      pinMode(args[1], OUTPUT);
      SoftPWMSet(args[1], 0);
      pin = args[1];
      break;
    case DIGITAL:
      pin = args[1];
      pinMode(pin, INPUT);
      break;
    case ANALOGIC:
      pin = args[1];
      break;
    case ULTRASONIC:
      sonar = new NewPing(args[1], args[2], 100);
      break;
    }
  }

  void execAct(int32_t *execArgs, uint8_t type)
  {
    switch (type)
    {
    case SERVO:
      servo.write((execArgs[0] < 0) ? 0 : (execArgs[0] > 180) ? 180 : execArgs[0]);
      break;
    case MOTOR:
      vel = (execArgs[0] < 0) ? 0 : (execArgs[0] > 100) ? 100 : execArgs[0];
      switch (execArgs[1])
      {
      case 0:
        digitalWrite(a, HIGH);
        digitalWrite(b, LOW);
        SoftPWMSetPercent(pwm, vel);

        break;
      case 1:
        digitalWrite(a, LOW);
        digitalWrite(b, LOW);
        SoftPWMSetPercent(pwm, 0);
        break;
      case 2:
        digitalWrite(a, LOW);
        digitalWrite(b, HIGH);
        SoftPWMSetPercent(pwm, vel);
        break;
      }
      break;
    case LED:
      SoftPWMSetPercent(pin, (execArgs[0] < 0) ? 0 : (execArgs[0] > 100) ? 100 : execArgs[0]);
      break;
    }
  }

  uint8_t getSensVal(uint8_t type)
  {
    uint8_t tempRead;
    switch (type)
    {
    case DIGITAL:
      tempRead = digitalRead(pin);

      break;
    case ANALOGIC:
      tempRead = map(analogRead(pin), 0, 1023, 0, 100);
      break;
    case ULTRASONIC:
      tempRead = sonar->ping_cm();
      break;
    }
    return (tempRead < 0) ? 0 : (tempRead > 100) ? 100 : tempRead;
  }

  void sendSensVal(uint8_t type)
  {
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)
    Serial1.write((char)getSensVal(type));
#endif
    Serial.write((char)getSensVal(type));
  }

  void off(uint8_t type)
  {
    switch (type)
    {
    case SERVO:
      servo.detach();
      break;
    case ULTRASONIC:
      free(sonar);
      break;
    case MOTOR:
      SoftPWMEnd(a);
      SoftPWMEnd(b);
      SoftPWMEnd(pwm);
      break;
    case LED:
      SoftPWMSet(pin, 0);
      SoftPWMEnd(pin);
      break;
    }
  }
};

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
    if (newValue > 999999)
    {
      value = 999999;
    }
    else if (newValue < -999999)
    {
      value = -999999;
    }
    else
    {
      value = newValue;
    }
  }
};

#ifdef __AVR_ATmega32U4__

void resetLeonardoMemory();

#endif
void resetMemory();
uint8_t getMapedPin(uint8_t pin);
void nairdaBegin(long int bauds);
void nairdaLoop();

#endif
