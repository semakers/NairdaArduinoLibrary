#include "linked/LinkedList.h"
#include <Arduino.h>

#include <stdint.h>
#include "nairda.h"

#include "freeTone/TimerFreeTone.h"
#include "NeoPixel/Adafruit_NeoPixel.h"

#if defined(ARDUINO_ARCH_STM32)
#include <Servo.h>
#include "softPwmSTM32/softPwmStm32.h"
#else
#if defined(ARDUINO_ARCH_ESP32)

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "esp_spi_flash.h"
#include "esp32Servo/ServoESP32.h"

#else
#include <Servo.h>
#include "softpwm/SoftPWM.h"
#include "ping/NewPing.h"
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
#define randomCommand 130
#define endFrequencies 131
#define frequencyCommand 132
#define endNeopixels 133
#define neopixelCommand 134
#define endFunctionCommand 135
#define goToFunctionCommand 136

#define CURRENT_VERSION 2

uint8_t getCurrentChannel();
void nextCurrentChannel();
void clearCurrentChannel();

enum
{
      SERVO = 0,
      MOTOR,
      LED,
      FREQUENCY,
      NEOPIXEL,
      DIGITAL,
      ANALOGIC,
      ULTRASONIC
};

class component
{
public:
      // analogc digital led
      
      Adafruit_NeoPixel *neopixel;
      

#if defined(ARDUINO_ARCH_STM32)
      // servo
      Servo servo;

#else
#if defined(ARDUINO_ARCH_ESP32)
      // servo
      Servo servo;
      int8_t ledcChannel = -1;
#else
      // servo
      Servo servo;
      // ultrasonic
      NewPing *sonar;
      // neopixel

#endif

#endif
      uint8_t pins[5]={0,0,0,0,0};
      uint8_t values[5]={0,0,0,0,0};

      component(uint16_t *args)
      {
            switch (args[0])
            {
            case SERVO:
            pins[0]=args[1];
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

                  pins[0] = args[1];
                  pins[1] = args[2];
                  pins[2] = args[3];
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
                  pins[0] = args[1];
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
            case FREQUENCY:
                  pins[0] = args[1];
                  break;

            case NEOPIXEL:
                  pins[0] = args[1];
                  neopixel = new Adafruit_NeoPixel(args[2], pins[0], NEO_GRB + NEO_KHZ800);
                  neopixel->begin();
                  break;
            case DIGITAL:
                  pins[0] = args[1];
#if defined(ARDUINO_ARCH_STM32)
                  pinMode(pins[0] , INPUT);
#else
#if defined(ARDUINO_ARCH_ESP32)
                  pinMode(pins[0] , INPUT);
#else

                  pinMode(pins[0] , INPUT);
#endif
#endif
                  break;

            case ANALOGIC:
                  pins[0] = args[1];
#if defined(ARDUINO_ARCH_STM32)

#else
#if defined(ARDUINO_ARCH_ESP32)

#else

#endif
#endif
                  break;
            case ULTRASONIC:
            pins[0] = args[1];
            pins[1] = args[2];
#if defined(ARDUINO_ARCH_STM32)
                 
            pinMode(args[1], OUTPUT);
            pinMode(args[2], INPUT);
#else
#if defined(ARDUINO_ARCH_ESP32)
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
                  servo.write((execArgs[0] < 0) ? 0 : (execArgs[0] > 180) ? 180
                                                                          : execArgs[0]);
#else
#if defined(ARDUINO_ARCH_ESP32)
                  servo.write((execArgs[0] < 0) ? 0 : (execArgs[0] > 180) ? 180
                                                                          : execArgs[0]);
#else
                  servo.write((execArgs[0] < 0) ? 0 : (execArgs[0] > 180) ? 180
                                                                          : execArgs[0]);
#endif
#endif
                  break;
            case MOTOR:
                  values[0] = (execArgs[0] < 0) ? 0 : (execArgs[0] > 100) ? 100
                                                                    : execArgs[0];
#if defined(ARDUINO_ARCH_STM32)
                  switch (execArgs[1])
                  {
                  case 0:
                        digitalWrite(pins[0], HIGH);
                        digitalWrite(pins[1], LOW);
                        softPwmSTM32Set(pins[2], values[0]);

                        break;
                  case 1:
                        digitalWrite(pins[0], LOW);
                        digitalWrite(pins[1], LOW);
                        softPwmSTM32Set(pins[2], 0);
                        break;
                  case 2:
                        digitalWrite(pins[0], LOW);
                        digitalWrite(pins[1], HIGH);
                        softPwmSTM32Set(pins[2], values[0]);
                        break;
                  }
#else
#if defined(ARDUINO_ARCH_ESP32)
                  switch (execArgs[1])
                  {
                  case 0:
                        digitalWrite(pins[0], HIGH);
                        digitalWrite(pins[1], LOW);
                        if (ledcChannel != -1)
                        {
                              ledcWrite(ledcChannel, map(values[0], 0, 100, 0, 65535));
                        }
                        else
                        {
                              if (values[0] >= 0 && values[0] <= 50)
                              {
                                    digitalWrite(pins[2], LOW);
                              }
                              else if (values[0] > 50 && values[0] <= 100)
                              {
                                    digitalWrite(pins[2], HIGH);
                              }
                        }

                        break;
                  case 1:
                        digitalWrite(pins[0], LOW);
                        digitalWrite(pins[1], LOW);
                        ledcWrite(ledcChannel, 0);
                        break;
                  case 2:
                        digitalWrite(pins[0], LOW);
                        digitalWrite(pins[1], HIGH);
                        if (ledcChannel != -1)
                        {
                              ledcWrite(ledcChannel, map(values[0], 0, 100, 0, 65535));
                        }
                        else
                        {
                              if (values[0] >= 0 && values[0] <= 50)
                              {
                                    digitalWrite(pins[2], LOW);
                              }
                              else if (values[0] > 50 && values[0] <= 100)
                              {
                                    digitalWrite(pins[2], HIGH);
                              }
                        }
                        break;
                  }

#else
                  switch (execArgs[1])
                  {
                  case 0:
                        digitalWrite(pins[0], HIGH);
                        digitalWrite(pins[1], LOW);
                        SoftPWMSetPercent(pins[2], values[0]);

                        break;
                  case 1:
                        digitalWrite(pins[0], LOW);
                        digitalWrite(pins[1], LOW);
                        SoftPWMSetPercent(pins[2], 0);
                        break;
                  case 2:
                        digitalWrite(pins[0], LOW);
                        digitalWrite(pins[1], HIGH);
                        SoftPWMSetPercent(pins[2], values[0]);
                        break;
                  }
#endif
#endif
                  break;
            case LED:
#if defined(ARDUINO_ARCH_STM32)
                  softPwmSTM32Set(pins[0], (execArgs[0] < 0) ? 0 : (execArgs[0] > 100) ? 100
                                                                                   : execArgs[0]);
#else
#if defined(ARDUINO_ARCH_ESP32)
                  values[0] = (execArgs[0] < 0) ? 0 : (execArgs[0] > 100) ? 100
                                                                          : execArgs[0];
                  if (ledcChannel != -1)
                  {
                        ledcWrite(ledcChannel, map(values[0], 0, 100, 0, 65535));
                  }
                  else
                  {
                        if (values[0] >= 0 && values[0] <= 50)
                        {
                              digitalWrite(pins[0], LOW);
                        }
                        else if (values[0] > 50 && values[0] <= 100)
                        {
                              digitalWrite(pins[0], HIGH);
                        }
                  }

                  /* Serial.print(pin);
                    Serial.print(": ");
                    Serial.println((execArgs[0] < 0) ? 0 : (execArgs[0] > 100) ? 100 : execArgs[0]);*/

#else
                  SoftPWMSetPercent(pins[0], (execArgs[0] < 0) ? 0 : (execArgs[0] > 100) ? 100
                                                                                     : execArgs[0]);
#endif
#endif
                  break;
            case FREQUENCY:
                  TimerFreeTone(pins[0], (execArgs[0] * 100) + execArgs[1], (execArgs[2] * 10000) + (execArgs[3] * 100) + execArgs[4], execArgs[5]);
                  break;
            case NEOPIXEL:
                  if(pins[0]==BLE_INDICATOR_PIN) neopixel->setPixelColor(execArgs[3], execArgs[1], execArgs[0], execArgs[2]);
                  else neopixel->setPixelColor(execArgs[3], execArgs[0], execArgs[1], execArgs[2]);
                  
                  neopixel->show();
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
                  tempRead = digitalRead(pins[0]);
#else
#if defined(ARDUINO_ARCH_ESP32)
                  tempRead = digitalRead(pins[0]);
#else
                  tempRead = digitalRead(pins[0]);
#endif
#endif
                  break;
            case ANALOGIC:
#if defined(ARDUINO_ARCH_STM32)
                  tempRead = map(analogRead(pins[0]), 0, 1023, 0, 100);
#else
#if defined(ARDUINO_ARCH_ESP32)
                  tempRead = map(analogRead(pins[0]), 0, 4095, 0, 100);
#else
                  tempRead = map(analogRead(pins[0]), 0, 1023, 0, 100);
#endif
#endif
                  break;
            case ULTRASONIC:
#if defined(ARDUINO_ARCH_STM32)
                  digitalWrite(pins[0], LOW);
                  delayMicroseconds(2);
                  digitalWrite(pins[0], HIGH);
                  delayMicroseconds(10);
                  digitalWrite(pins[0], LOW);
                  tempRead = pulseIn(pins[1], HIGH) / 27.6233 / 2;
#else
#if defined(ARDUINO_ARCH_ESP32)
                  digitalWrite(pins[0], LOW);
                  delayMicroseconds(2);
                  digitalWrite(pins[0], HIGH);
                  delayMicroseconds(10);
                  digitalWrite(pins[0], LOW);
                  tempRead = pulseIn(pins[1], HIGH) / 27.6233 / 2;
#else

                  tempRead = sonar->ping_cm();
#endif
#endif
                  static int lastValue;
                  static int zeroCounter = 0;

                  if (zeroCounter == 3)
                  {
                        if (tempRead == 0)
                              tempRead = 99;
                        else
                              zeroCounter = 0;
                  }
                  else
                  {
                        if (tempRead == 0)
                              zeroCounter++;
                        lastValue = (tempRead == 0) ? lastValue : tempRead;
                        tempRead = (tempRead == 0) ? lastValue : tempRead;
                  }
                  break;
            }
            return (tempRead < 0) ? 0 : (tempRead > 100) ? 100
                                                         : tempRead;
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
            case NEOPIXEL:
                  free(neopixel);
                  break;
            case MOTOR:
#if defined(ARDUINO_ARCH_STM32)
                  digitalWrite(pins[0], LOW);
                  digitalWrite(pins[1], LOW);
                  softPwmSTM32Set(pins[2], 0);
                  softPwmSTM32Dettach(pins[2]);
#else
#if defined(ARDUINO_ARCH_ESP32)
                  digitalWrite(pins[0], LOW);
                  digitalWrite(pins[1], LOW);
                  ledcWrite(ledcChannel, 0);
                  ledcDetachPin(pins[2]);
#else
                  digitalWrite(pins[0], LOW);
                  digitalWrite(pins[1], LOW);
                  SoftPWMSet(pins[2], 0);
                  SoftPWMEnd(pins[2]);
#endif
#endif
                  break;
            case LED:
#if defined(ARDUINO_ARCH_STM32)
                  softPwmSTM32Set(pins[0], 0);
                  softPwmSTM32Dettach(pins[0]);
#else
#if defined(ARDUINO_ARCH_ESP32)

                  ledcWrite(ledcChannel, 0);
                  ledcDetachPin(pins[0]);

#else
                  SoftPWMSet(pins[0], 0);
                  SoftPWMEnd(pins[0]);
#endif
#endif
                  break;
            }
      }
};

uint8_t getMapedPin(uint8_t pin);
void loadEepromDescriptor();
void writeByte(uint32_t address, uint8_t byte);

#if defined(__AVR_ATmega32U4__) || (ARDUINO_ARCH_ESP32) || (ARDUINO_ARCH_STM32)
void restartRunFromEeprom();
#endif