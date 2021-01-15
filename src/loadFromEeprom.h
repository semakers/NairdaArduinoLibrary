#include "linked/LinkedList.h"
#include <Arduino.h>
#include <Servo.h>
#include <stdint.h>
#include "nairda.h"

#if defined(ARDUINO_ARCH_STM32)
//#include "stm32Servo/Servo.h"
#include "softPwmSTM32/softPwmStm32.h"
#include <SoftwareSerial.h>
#else
#if defined(ARDUINO_ARCH_ESP32)

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "esp_spi_flash.h"
#include "esp32Servo/ServoESP32.h"

#else

#include "softpwm/SoftPWM.h"

#include "ping/NewPing.h"
/*#if defined(ARDUINO_ARCH_AVR)
#include "avr/Servo.h"
#elif defined(ARDUINO_ARCH_SAM)
#include "sam/Servo.h"
#else
#error "This library only supports boards with an AVR or SAM processor."
#endif

#ifdef __AVR_ATmega168__
#warning "Nairda dont suport save & load bynary from eeprom in Atmega168."
#endif*/

#endif
#endif

//#define _24LC_512

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

#define CURRENT_VERSION 2

uint8_t getCurrentChannel();
void nextCurrentChannel();
void clearCurrentChannel();

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

#if defined(ARDUINO_ARCH_STM32)
      //servo
      Servo servo;
      uint8_t trigger, echo;

#else
#if defined(ARDUINO_ARCH_ESP32)
      //servo
      Servo servo;
      int8_t ledcChannel = -1;
      uint8_t trigger, echo;
#else
      //servo
      Servo servo;
      //ultrasonic
      NewPing *sonar;
#endif

#endif
      uint8_t a, b, pwm, vel, intensity;

      component(uint16_t *args)
      {
            switch (args[0])
            {
            case SERVO:
#if defined(ARDUINO_ARCH_STM32)
                  servo.attach(args[1], args[2], args[3]);
                  servo.write(args[4]);
#else
#if defined(ARDUINO_ARCH_ESP32)

                  if (getCurrentChannel() < 16)
                  {
                        servo.attach(args[1], getCurrentChannel(), 0, 180, args[2], args[3]);
                        servo.write(args[4]);
                        ledcChannel = getCurrentChannel();
                        nextCurrentChannel();
                  }

#else

                  servo.attach(args[1], args[2], args[3]);
                  servo.write(args[4]);

#endif
#endif
                  break;
            case MOTOR:

                  a = args[1];
                  b = args[2];
                  pwm = args[3];
#if defined(ARDUINO_ARCH_STM32)
                  pinMode(args[1], OUTPUT);
                  pinMode(args[2], OUTPUT);
                  softPwmSTM32Attach(args[3], 0);
#else
#if defined(ARDUINO_ARCH_ESP32)
                  pinMode(args[1], OUTPUT);
                  pinMode(args[2], OUTPUT);
                  if (getCurrentChannel() < 16)
                  {
                        ledcSetup(getCurrentChannel(), 50, 16);
                        ledcAttachPin(args[3], getCurrentChannel());
                        ledcChannel = getCurrentChannel();
                        nextCurrentChannel();
                  }
                  else
                  {
                        pinMode(args[3], OUTPUT);
                  }

#else
                  pinMode(args[1], OUTPUT);
                  pinMode(args[2], OUTPUT);
                  SoftPWMSet(args[3], 0);

#endif
#endif
                  break;
            case LED:
                  pin = args[1];
#if defined(ARDUINO_ARCH_STM32)
                  softPwmSTM32Attach(args[1], 0);
#else
#if defined(ARDUINO_ARCH_ESP32)
                  if (getCurrentChannel() < 16)
                  {
                        ledcSetup(getCurrentChannel(), 50, 16);
                        ledcAttachPin(args[1], getCurrentChannel());
                        ledcChannel = getCurrentChannel();
                        nextCurrentChannel();
                  }
                  else
                  {
                        pinMode(args[1], OUTPUT);
                  }
#else
                  pinMode(args[1], OUTPUT);
                  SoftPWMSet(args[1], 0);

#endif
#endif
                  break;
            case DIGITAL:
                  pin = args[1];
#if defined(ARDUINO_ARCH_STM32)
                  pinMode(pin, INPUT);
#else
#if defined(ARDUINO_ARCH_ESP32)
                  pinMode(pin, INPUT);
#else

                  pinMode(pin, INPUT);
#endif
#endif
                  break;
            case ANALOGIC:
                  pin = args[1];
#if defined(ARDUINO_ARCH_STM32)

#else
#if defined(ARDUINO_ARCH_ESP32)

#else

#endif
#endif
                  break;
            case ULTRASONIC:
#if defined(ARDUINO_ARCH_STM32)
                  trigger = args[1];
                  echo = args[2];
                  pinMode(args[1], OUTPUT);
                  pinMode(args[2], INPUT);
#else
#if defined(ARDUINO_ARCH_ESP32)
                  trigger = args[1];
                  echo = args[2];
                  pinMode(args[1], OUTPUT);
                  pinMode(args[2], INPUT);
#else
                  sonar = new NewPing(args[1], args[2], 100);
#endif
#endif
                  break;
            }
      }

      void execAct(uint32_t *execArgs, uint8_t type)
      {
            switch (type)
            {
            case SERVO:
#if defined(ARDUINO_ARCH_STM32)
                  servo.write((execArgs[0] < 0) ? 0 : (execArgs[0] > 180) ? 180 : execArgs[0]);
#else
#if defined(ARDUINO_ARCH_ESP32)
                  servo.write((execArgs[0] < 0) ? 0 : (execArgs[0] > 180) ? 180 : execArgs[0]);
#else
                  servo.write((execArgs[0] < 0) ? 0 : (execArgs[0] > 180) ? 180 : execArgs[0]);
#endif
#endif
                  break;
            case MOTOR:
                  vel = (execArgs[0] < 0) ? 0 : (execArgs[0] > 100) ? 100 : execArgs[0];
#if defined(ARDUINO_ARCH_STM32)
                  switch (execArgs[1])
                  {
                  case 0:
                        digitalWrite(a, HIGH);
                        digitalWrite(b, LOW);
                        softPwmSTM32Set(pwm, vel);

                        break;
                  case 1:
                        digitalWrite(a, LOW);
                        digitalWrite(b, LOW);
                        softPwmSTM32Set(pwm, 0);
                        break;
                  case 2:
                        digitalWrite(a, LOW);
                        digitalWrite(b, HIGH);
                        softPwmSTM32Set(pwm, vel);
                        break;
                  }
#else
#if defined(ARDUINO_ARCH_ESP32)
                  switch (execArgs[1])
                  {
                  case 0:
                        digitalWrite(a, HIGH);
                        digitalWrite(b, LOW);
                        if (ledcChannel != -1)
                        {
                              ledcWrite(ledcChannel, map(vel, 0, 100, 0, 65535));
                        }
                        else
                        {
                              if (vel >= 0 && vel <= 50)
                              {
                                    digitalWrite(pwm, LOW);
                              }
                              else if (vel > 50 && vel <= 100)
                              {
                                    digitalWrite(pwm, HIGH);
                              }
                        }

                        break;
                  case 1:
                        digitalWrite(a, LOW);
                        digitalWrite(b, LOW);
                        ledcWrite(ledcChannel, 0);
                        break;
                  case 2:
                        digitalWrite(a, LOW);
                        digitalWrite(b, HIGH);
                        if (ledcChannel != -1)
                        {
                              ledcWrite(ledcChannel, map(vel, 0, 100, 0, 65535));
                        }
                        else
                        {
                              if (vel >= 0 && vel <= 50)
                              {
                                    digitalWrite(pwm, LOW);
                              }
                              else if (vel > 50 && vel <= 100)
                              {
                                    digitalWrite(pwm, HIGH);
                              }
                        }
                        break;
                  }

#else
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
#endif
#endif
                  break;
            case LED:
#if defined(ARDUINO_ARCH_STM32)
                  softPwmSTM32Set(pin, (execArgs[0] < 0) ? 0 : (execArgs[0] > 100) ? 100 : execArgs[0]);
#else
#if defined(ARDUINO_ARCH_ESP32)
                  /*Serial.print(pin);
                  Serial.print(": ");
                  Serial.println((execArgs[0] < 0) ? 0 : (execArgs[0] > 100) ? 100 : execArgs[0]);*/
                  intensity = (execArgs[0] < 0) ? 0 : (execArgs[0] > 100) ? 100 : execArgs[0];
                  if (ledcChannel != -1)
                  {
                        ledcWrite(ledcChannel, map(intensity, 0, 100, 0, 65535));
                  }
                  else
                  {
                        if (intensity >= 0 && intensity <= 50)
                        {
                              digitalWrite(pin, LOW);
                        }
                        else if (intensity > 50 && intensity <= 100)
                        {
                              digitalWrite(pin, HIGH);
                        }
                  }

#else
                  SoftPWMSetPercent(pin, (execArgs[0] < 0) ? 0 : (execArgs[0] > 100) ? 100 : execArgs[0]);
#endif
#endif
                  break;
            }
      }

      uint8_t getSensVal(uint8_t type)
      {
            uint8_t tempRead;
            switch (type)
            {
            case DIGITAL:
#if defined(ARDUINO_ARCH_STM32)
                  tempRead = digitalRead(pin);
#else
#if defined(ARDUINO_ARCH_ESP32)
                  tempRead = digitalRead(pin);
#else
                  tempRead = digitalRead(pin);
#endif
#endif
                  break;
            case ANALOGIC:
#if defined(ARDUINO_ARCH_STM32)
                  tempRead = map(analogRead(pin), 0, 1023, 0, 100);
#else
#if defined(ARDUINO_ARCH_ESP32)
                  tempRead = map(analogRead(pin), 0, 4095, 0, 100);
#else
                  tempRead = map(analogRead(pin), 0, 1023, 0, 100);
#endif
#endif
                  break;
            case ULTRASONIC:
#if defined(ARDUINO_ARCH_STM32)
                  digitalWrite(trigger, LOW);
                  delayMicroseconds(2);
                  digitalWrite(trigger, HIGH);
                  delayMicroseconds(10);
                  digitalWrite(trigger, LOW);
                  tempRead = pulseIn(echo, HIGH) / 27.6233 / 2;
#else
#if defined(ARDUINO_ARCH_ESP32)
                  digitalWrite(trigger, LOW);
                  delayMicroseconds(2);
                  digitalWrite(trigger, HIGH);
                  delayMicroseconds(10);
                  digitalWrite(trigger, LOW);
                  tempRead = pulseIn(echo, HIGH) / 27.6233 / 2;
#else
                  tempRead = sonar->ping_cm();
#endif
#endif
                  break;
            }
            return (tempRead < 0) ? 0 : (tempRead > 100) ? 100 : tempRead;
      }

      void sendSensVal(uint8_t type)
      {
#if defined(ARDUINO_ARCH_STM32)
            Serial.write((char)getSensVal(type));
#else
#if defined(ARDUINO_ARCH_ESP32)
            bleWrite((char)getSensVal(type));
#else

#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)
            Serial1.write((char)getSensVal(type));
#endif
            Serial.write((char)getSensVal(type));

#endif
#endif
      }

      void off(uint8_t type)
      {
            switch (type)
            {
            case SERVO:
#if defined(ARDUINO_ARCH_STM32)
                  servo.detach();
#else
#if defined(ARDUINO_ARCH_ESP32)
                  servo.detach();
#else
                  servo.detach();
#endif
#endif
                  break;
            case ULTRASONIC:
#if defined(ARDUINO_ARCH_STM32)

#else
#if defined(ARDUINO_ARCH_ESP32)

#else
                  free(sonar);
#endif
#endif
                  break;
            case MOTOR:
#if defined(ARDUINO_ARCH_STM32)
                  digitalWrite(a, LOW);
                  digitalWrite(b, LOW);
                  softPwmSTM32Set(pwm, 0);
                  softPwmSTM32Dettach(pwm);
#else
#if defined(ARDUINO_ARCH_ESP32)
                  digitalWrite(a, LOW);
                  digitalWrite(b, LOW);
                  ledcWrite(ledcChannel, 0);
                  ledcDetachPin(pwm);
#else
                  digitalWrite(a, LOW);
                  digitalWrite(b, LOW);
                  SoftPWMSet(pwm, 0);
                  SoftPWMEnd(pwm);
#endif
#endif
                  break;
            case LED:
#if defined(ARDUINO_ARCH_STM32)
                  softPwmSTM32Set(pin, 0);
                  softPwmSTM32Dettach(pin);
#else
#if defined(ARDUINO_ARCH_ESP32)

                  ledcWrite(ledcChannel, 0);
                  ledcDetachPin(pin);

#else
                  SoftPWMSet(pin, 0);
                  SoftPWMEnd(pin);
#endif
#endif
                  break;
            }
      }
};

uint8_t getMapedPin(uint8_t pin);
void loadEepromDescriptor();
void writeByte(uint32_t address, uint8_t byte);