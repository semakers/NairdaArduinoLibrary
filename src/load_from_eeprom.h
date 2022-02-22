#include "extern_libraries/linked_list/linked_list.h"
#include <Arduino.h>

#include <stdint.h>
#include "nairda.h"

#include "components/outputs/servo/servo_component.h"
#include "components/outputs/motor/motor_component.h"
#include "components/outputs/digital_out/digital_out_component.h"
#include "components/outputs/frequency/frequency_component.h"
#include "components/outputs/neo_pixel/neo_pixel_component.h"

#include "components/inputs/analogic/analogic_component.h"
#include "components/inputs/digital_in/digital_in_component.h"
#include "components/inputs/ultrasonic/ultrasonic_component.h"

#if defined(ARDUINO_ARCH_STM32)
#include <Servo.h>
#elif defined(ARDUINO_ARCH_ESP32)

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "esp_spi_flash.h"

#include "extern_libraries/esp32_servo/esp32_servo.h"

#else
#include "extern_libraries/new_ping/new_ping.h"
#include "extern_libraries/soft_pwm/soft_pwm.h"
#include <Servo.h>
#endif

#include "extern_libraries/neo_pixel/Adafruit_NeoPixel.h"

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

#define CURRENT_VERSION 3

uint8_t getCurrentChannel();
void nextCurrentChannel();
void clearCurrentChannel();

enum
{
      SERVO = 0,
      MOTOR,
      DIGITAL_OUT,
      FREQUENCY,
      NEOPIXEL,
      DIGITAL_IN,
      ANALOGIC,
      ULTRASONIC
};

class component
{
public:
      int8_t ledcChannel = -1;
      uint8_t pins[5] = {0, 0, 0, 0, 0};
      uint8_t values[5] = {0, 0, 0, 0, 0};

      Adafruit_NeoPixel *neopixel;
      Servo *servo;

#if !defined(ARDUINO_ARCH_STM32) && !defined(ARDUINO_ARCH_ESP32)
      NewPing *sonar;
#endif

      component(uint16_t *args)
      {
            switch (args[0])
            {
            case SERVO:
                  servoCreate(args, pins, &ledcChannel, servo);
                  break;
            case MOTOR:
                  motorCreate(args, pins, &ledcChannel);
                  break;
            case DIGITAL_OUT:
                  digitalOutCreate(args, pins, &ledcChannel);
                  break;
            case FREQUENCY:
                  frequencyCreate(args, pins);
                  break;
            case NEOPIXEL:
                  neoPixelCreate(args, pins, neopixel);
                  break;
            case DIGITAL_IN:
                  digitalInCreate(args, pins);
                  break;
            case ANALOGIC:
                  analogicCreate(args, pins);
                  break;
            case ULTRASONIC:
#if !defined(ARDUINO_ARCH_STM32) && !defined(ARDUINO_ARCH_ESP32)
                  ultrasonicCreate(args, pins, sonar);
#else
                  ultrasonicCreate(args, pins);
#endif
                  break;
            }
      }

      void execAct(uint32_t *execArgs, uint8_t type)
      {
            switch (type)
            {
            case SERVO:
                  servoExec(execArgs, servo);
                  break;
            case MOTOR:
                  motorExec(execArgs, pins, values, &ledcChannel);
                  break;
            case DIGITAL_OUT:
                  digitalOutExec(execArgs, pins, values, &ledcChannel);
                  break;
            case FREQUENCY:
                  frequencyExec(execArgs, pins);
                  break;
            case NEOPIXEL:
                  neoPixelExec(execArgs, neopixel);
                  break;
            }
      }

      uint8_t getSensVal(uint8_t type)
      {
            uint8_t tempRead;
            switch (type)
            {
            case DIGITAL_IN:
                  digitalInSense(pins, &tempRead);
                  break;
            case ANALOGIC:
                  analogicSense(pins, &tempRead);
                  break;
            case ULTRASONIC:
            #if !defined(ARDUINO_ARCH_STM32) && !defined(ARDUINO_ARCH_ESP32)
                   ultrasonicSense(pins, &tempRead,sonar);
#else
                   ultrasonicSense(pins, &tempRead);
#endif
                 
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
            case NEOPIXEL:
                  neoPixelOff(neopixel);
                  break;
            case SERVO:
                  servoOff(servo);
                  break;
            case ULTRASONIC:
                  #if !defined(ARDUINO_ARCH_STM32) && !defined(ARDUINO_ARCH_ESP32)
                   ultrasonicOff(sonar);
                   #endif
                  break;
            case MOTOR:
                  motorOff(pins, &ledcChannel);
                  break;
            case DIGITAL_OUT:
                  digitalOutOff(pins, &ledcChannel);
                  break;
            }
      }
};

void nextVariable();

uint8_t nextByte();
int32_t getInputValue(uint8_t firstByte);
uint8_t getMapedPin(uint8_t pin);
void loadEepromDescriptor();
void writeByte(uint32_t address, uint8_t byte);

#if defined(__AVR_ATmega32U4__) || (ARDUINO_ARCH_ESP32) || (ARDUINO_ARCH_STM32)
void restartRunFromEeprom();
#endif