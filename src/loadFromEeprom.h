#include "linked/LinkedList.h"
#include <Arduino.h>
#include <stdint.h>

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
#if defined(ARDUINO_ARCH_AVR)
#include "avr/Servo.h"
#elif defined(ARDUINO_ARCH_SAM)
#include "sam/Servo.h"
#else
#error "This library only supports boards with an AVR or SAM processor."
#endif

#ifdef __AVR_ATmega168__
#warning "Nairda dont suport save & load bynary from eeprom in Atmega168."
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

static uint8_t usedChannels = 0;

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

#if defined(ARDUINO_ARCH_ESP32)
      //servo
      Servo servo;
      int8_t ledcChannel = -1;

#else

      //servo
      Servo servo;

      //ultrasonic
      NewPing *sonar;

#endif
      uint8_t a, b, pwm, vel;

      component(uint16_t *args)
      {
            switch (args[0])
            {
            case SERVO:
#if defined(ARDUINO_ARCH_ESP32)
                  if (usedChannels < 16)
                  {
                        servo.attach(args[1], usedChannels, 0, 180, args[2], args[3]);
                        servo.write(args[4]);
                        usedChannels++;
                  }

#else

                  servo.attach(args[1], args[2], args[3]);
                  servo.write(args[4]);

#endif
                  break;
            case MOTOR:

                  a = args[1];
                  b = args[2];
                  pwm = args[3];

#if defined(ARDUINO_ARCH_ESP32)
                  pinMode(args[1], OUTPUT);
                  pinMode(args[2], OUTPUT);
                  if (usedChannels < 16)
                  {
                        ledcSetup(usedChannels, 5000, 8);
                        ledcAttachPin(args[3], usedChannels);
                        ledcChannel = usedChannels;
                        usedChannels++;
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
                  break;
            case LED:
                  pin = args[1];
#if defined(ARDUINO_ARCH_ESP32)
                  if (usedChannels < 16)
                  {
                        ledcSetup(usedChannels, 5000, 8);
                        ledcAttachPin(args[1], usedChannels);
                        ledcChannel = usedChannels;
                        usedChannels++;
                  }
                  else
                  {
                        pinMode(args[1], OUTPUT);
                  }
#else
                  pinMode(args[1], OUTPUT);
                  SoftPWMSet(args[1], 0);

#endif
                  break;
            case DIGITAL:
                  pin = args[1];
#if defined(ARDUINO_ARCH_ESP32)

#else

                  pinMode(pin, INPUT);
#endif
                  break;
            case ANALOGIC:
                  pin = args[1];
#if defined(ARDUINO_ARCH_ESP32)

#else

#endif
                  break;
            case ULTRASONIC:
#if defined(ARDUINO_ARCH_ESP32)

#else
                  sonar = new NewPing(args[1], args[2], 100);
#endif
                  break;
            }
      }

      void execAct(uint32_t *execArgs, uint8_t type)
      {
            switch (type)
            {
            case SERVO:
#if defined(ARDUINO_ARCH_ESP32)
                  servo.write((execArgs[0] < 0) ? 0 : (execArgs[0] > 180) ? 180 : execArgs[0]);
#else
                  servo.write((execArgs[0] < 0) ? 0 : (execArgs[0] > 180) ? 180 : execArgs[0]);
#endif
                  break;
            case MOTOR:
                  vel = (execArgs[0] < 0) ? 0 : (execArgs[0] > 100) ? 100 : execArgs[0];

#if defined(ARDUINO_ARCH_ESP32)
                  switch (execArgs[1])
                  {
                  case 0:
                        digitalWrite(a, HIGH);
                        digitalWrite(b, LOW);
                        if (ledcChannel != -1)
                        {
                              ledcWrite(ledcChannel, map(vel, 0, 100, 0, 255));
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
                              ledcWrite(ledcChannel, map(vel, 0, 100, 0, 255));
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
                  break;
            case LED:
#if defined(ARDUINO_ARCH_ESP32)
                  Serial.print(pin);
                  Serial.print(": ");
                  Serial.println((execArgs[0] < 0) ? 0 : (execArgs[0] > 100) ? 100 : execArgs[0]);

                  if (ledcChannel != -1)
                  {
                        ledcWrite(ledcChannel, map((execArgs[0] < 0) ? 0 : (execArgs[0] > 100) ? 100 : execArgs[0], 0, 100, 0, 255));
                  }
                  else
                  {
                        if (vel >= 0 && vel <= 50)
                        {
                              digitalWrite(pin, LOW);
                        }
                        else if (vel > 50 && vel <= 100)
                        {
                              digitalWrite(pin, HIGH);
                        }
                  }

#else
                  SoftPWMSetPercent(pin, (execArgs[0] < 0) ? 0 : (execArgs[0] > 100) ? 100 : execArgs[0]);
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
#if defined(ARDUINO_ARCH_ESP32)

#else
                  tempRead = digitalRead(pin);
#endif
                  break;
            case ANALOGIC:
#if defined(ARDUINO_ARCH_ESP32)

#else
                  tempRead = map(analogRead(pin), 0, 1023, 0, 100);
#endif
                  break;
            case ULTRASONIC:
#if defined(ARDUINO_ARCH_ESP32)

#else
                  tempRead = sonar->ping_cm();
#endif
                  break;
            }
            return (tempRead < 0) ? 0 : (tempRead > 100) ? 100 : tempRead;
      }

      void sendSensVal(uint8_t type)
      {
#if defined(ARDUINO_ARCH_ESP32)

#else

#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)
            Serial1.write((char)getSensVal(type));
#endif
            Serial.write((char)getSensVal(type));

#endif
      }

      void off(uint8_t type)
      {
            switch (type)
            {
            case SERVO:
#if defined(ARDUINO_ARCH_ESP32)
                  if (usedChannels > 0)
                  {
                        usedChannels--;
                        servo.detach();
                  }

#else

                  servo.detach();
#endif
                  break;
            case ULTRASONIC:
#if defined(ARDUINO_ARCH_ESP32)

#else
                  free(sonar);
#endif
                  break;
            case MOTOR:
#if defined(ARDUINO_ARCH_ESP32)
                  digitalWrite(a, LOW);
                  digitalWrite(b, LOW);

                  if (usedChannels > 0)
                  {
                        usedChannels--;
                        ledcWrite(ledcChannel, 0);
                        ledcDetachPin(pwm);
                  }
#else
                  digitalWrite(a, LOW);
                  digitalWrite(b, LOW);
                  SoftPWMSet(pwm, 0);
                  SoftPWMEnd(pwm);
#endif
                  break;
            case LED:
#if defined(ARDUINO_ARCH_ESP32)

                  if (usedChannels > 0)
                  {
                        usedChannels--;
                        ledcWrite(ledcChannel, 0);
                        ledcDetachPin(pin);
                  }

#else
                  SoftPWMSet(pin, 0);
                  SoftPWMEnd(pin);
#endif
                  break;
            }
      }
};

uint8_t getMapedPin(uint8_t pin);
void loadEepromDescriptor();
void writeByte(uint32_t address, uint8_t byte);