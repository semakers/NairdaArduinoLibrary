#include <EEPROM.h>
#include <Wire.h>
#include <Arduino.h>

#include "volatile_memory/volatile_memory.h"

#include "components/outputs/servo/servo_component.h"
#include "components/outputs/motor/motor_component.h"
#include "components/outputs/digital_out/digital_out_component.h"
#include "components/outputs/frequency/frequency_component.h"
#include "components/outputs/neo_pixel/neo_pixel_component.h"
#include "components/outputs/dynamixel/dynamixel_component.h"

#include "components/inputs/analogic/analogic_component.h"
#include "components/inputs/digital_in/digital_in_component.h"
#include "components/inputs/ultrasonic/ultrasonic_component.h"

#include "blocks_instructions/follow/follow_instructions.h"
#include "blocks_instructions/logic/loginc_instructions.h"
#include "blocks_instructions/functions/function_instructions.h"
#include "blocks_instructions/variables/variables_instructions.h"
#include "blocks_instructions/math/math_instructions.h"
#include "nairda.h"

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
// Dynamixel
#define dynamixelColor 137
#define dynamixelWrite 138
#define dynamixelRead 139

#define CURRENT_VERSION 3

uint8_t getCurrentChannel();
void nextCurrentChannel();
void nairdaRunMachineState();
uint8_t callInterrupt();
uint8_t nextByte();
int32_t getInputValue(uint8_t firstByte);
uint8_t getMapedPin(uint8_t pin);
void loadEepromDescriptor();
void writeByte(uint32_t address, uint8_t byte);

#if defined(__AVR_ATmega32U4__) || (ARDUINO_ARCH_ESP32) || (ARDUINO_ARCH_STM32)
void restartRunFromEeprom();
#endif
