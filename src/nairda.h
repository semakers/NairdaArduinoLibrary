#ifndef nairda_h
#define nairda_h
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

class component{
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

  void execAct(uint8_t *execArgs, uint8_t type)
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
      tempRead = map(analogRead(pin), 0, 1023, 0, 100);
      break;
    case ANALOGIC:
      tempRead = digitalRead(pin);
      break;
    case ULTRASONIC:
      tempRead = sonar->ping_cm();
      break;
    }
    return (tempRead < 0) ? 0 : (tempRead > 100) ? 100 : tempRead;
  }

  void sendSenseVal(uint8_t type)
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

class analogic
{
public:
  int pin;

  analogic(int cpin)
  {
    pin = cpin;
  }

  uint8_t getValue()
  {
    return map(analogRead(pin), 0, 1023, 0, 100);
  }

  void sendValue()
  {
#ifndef __AVR_ATmega168__
    char tempread = map(analogRead(pin), 0, 1023, 0, 100);
#else
    char tempread = analogRead(pin) / 10;
#endif

#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)
    Serial1.write((tempread >= 100) ? 100 : (tempread < 0) ? 0 : tempread);
#endif

    Serial.write((tempread >= 100) ? 100 : (tempread < 0) ? 0 : tempread);
    /*delay(5);
     #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)
      Serial1.write((tempread >= 100) ? 100 : tempread);
    #endif
    Serial.write((tempread >= 100) ? 100 : tempread);*/
  }
};

#ifdef __AVR_ATmega168__

class ultrasonic
{
public:
  int trigger;
  int echo;

  ultrasonic(int ctrigger, int cecho)
  {
    trigger = ctrigger;
    echo = cecho;
    pinMode(trigger, OUTPUT);
    pinMode(echo, INPUT);
  }

  uint8_t getValue(){
    digitalWrite(trigger, LOW); //Por cuestión de estabilización del sensor
    delayMicroseconds(5);
    digitalWrite(trigger, HIGH); //envío del pulso ultrasónico
    delayMicroseconds(10);
    int tiempo = pulseIn(echo, HIGH); //funcion para medir el tiempo y guardarla en la variable "tiempo"
    int tempRead = tiempo / 58;
    if (tempRead > 100)
      tempRead = 100;
    return tempRead;
  }

  void sendValue()
  {
    digitalWrite(trigger, LOW); //Por cuestión de estabilización del sensor
    delayMicroseconds(5);
    digitalWrite(trigger, HIGH); //envío del pulso ultrasónico
    delayMicroseconds(10);
    int tiempo = pulseIn(echo, HIGH); //funcion para medir el tiempo y guardarla en la variable "tiempo"
    int tempRead = tiempo / 58;
    if (tempRead > 100)
      tempRead = 100;
    Serial.write((char)tempRead);
    /*delay(5);
    Serial.write((char)tempRead);*/
  }

  void off()
  {
    //free(sonar);
    //sonar->timer_stop();
  }
};
#else

class ultrasonic
{
public:
  NewPing *sonar;

  ultrasonic(int trigger, int echo)
  {
    sonar = new NewPing(trigger, echo, 100);
  }

  uint8_t getValue(){
    long int tempRead = sonar->ping_cm();
    if (tempRead > 100)
      tempRead = 100;
    return tempRead;
  }

  void sendValue()
  { //funcion para medir el tiempo y guardarla en la variable "tiempo"
    long int tempRead = sonar->ping_cm();
    if (tempRead > 100)
      tempRead = 100;
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)
    Serial1.write((char)tempRead);
#endif

    Serial.write((char)tempRead);
    /*delay(5);

    #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)
      Serial1.write((char)tempRead);
    #endif
    Serial.write((char)tempRead);*/
  }

  void off()
  {
    free(sonar);
    //sonar->timer_stop();
  }
};

#endif

class digital
{
public:
  int pin;

  digital(int cpin)
  {
    pin = cpin;
    pinMode(pin, INPUT);
  }

  uint8_t getValue(){
    return digitalRead(pin);
  }

  void sendValue()
  {
    int tempRead = digitalRead(pin);
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)
    Serial1.write((char)tempRead);
#endif
    Serial.write((char)tempRead);
    /*delay(5);
    #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)
      Serial1.write((char)tempRead);
    #endif
    Serial.write((char)tempRead);*/
  }
};

class servo
{
public:
  Servo cservo;

  servo(int cpin, int cpmin, int cpmax, int cdpos)
  {
    cservo.attach(cpin, cpmin, cpmax);
    cservo.write(cdpos);
  }

  void setPos(int pos)
  {
    cservo.write(map(pos, 0, 99, 0, 180));
  }

  void setRawPos(int32_t pos){
    cservo.write((pos<0)?0:(pos>180)?180:pos);
  }

  void off()
  {
    cservo.detach();
  }
};

class dc
{
public:
  int a, b, pwm;
  int vel;

  dc(int ca, int cb, int cpwm)
  {
    pinMode(ca, OUTPUT);
    pinMode(cb, OUTPUT);
    SoftPWMSet(cpwm, 0);
    a = ca;
    b = cb;
    pwm = cpwm;
  }

  void setVel(int cvel)
  {
    vel = (cvel<0)?0:(cvel>100)?100:cvel;
  }

  void setMove(int mode)
  {
    switch (mode)
    {
    case 0:
      digitalWrite(a, HIGH);
      digitalWrite(b, LOW);
      SoftPWMSetPercent(pwm, vel);
      //Serial.print("izquierda");

      break;
    case 1:
      digitalWrite(a, LOW);
      digitalWrite(b, LOW);
      SoftPWMSetPercent(pwm, 0);
      //Serial.print("detener");
      break;
    case 2:
      digitalWrite(a, LOW);
      digitalWrite(b, HIGH);
      SoftPWMSetPercent(pwm, vel);
      //Serial.print("izquierda");
      break;
    }
  }

  void off()
  {
    SoftPWMEnd(a);
    SoftPWMEnd(b);
    SoftPWMEnd(pwm);
  }
};

class led
{
public:
  int pin;
  led(int cpin)
  {
    pinMode(cpin, OUTPUT);
    SoftPWMSet(cpin, 0);
    pin = cpin;
  }

  void setPWM(int pwm)
  {
    SoftPWMSetPercent(pin, (pwm<0)?0:(pwm>100)?100:pwm);
  }

  void off()
  {
    SoftPWMSet(pin, 0);
    SoftPWMEnd(pin);
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

  void setvalue(int32_t newValue){
    if(newValue>999999){
      value=999999;
    }else if(newValue<-999999){
      value=-999999;
    }else{
      value=newValue;
    }
  }
};

void freeServos();
void freeDc();
void freeLeds();
void freeAnalogics();
void freeUltrasonics();
void freeDigitals();

#ifdef __AVR_ATmega32U4__

void resetLeonardoMemory();

#endif
void resetMemory();
uint8_t getMapedPin(uint8_t pin);
void nairdaBegin(long int bauds);
void nairdaLoop();


#endif
