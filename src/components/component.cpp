#include "component.h"
#include "load_from_eeprom.h"

component_t *newComponent(uint16_t *args)
{
    component_t *component;
    component->ledcChannel[0] = -1;
    switch (args[0])
    {
    case SERVO:
        servoCreate(args, component->pins, component->ledcChannel, component->servo);
        break;
    case MOTOR:
        motorCreate(args, component->pins, component->ledcChannel);
        break;
    case DIGITAL_OUT:
        digitalOutCreate(args, component->pins, component->ledcChannel);
        break;
    case FREQUENCY:
        frequencyCreate(args, component->pins);
        break;
    case NEOPIXEL:
        neoPixelCreate(args, component->pins, component->neopixel);
        break;
    case DIGITAL_IN:
        digitalInCreate(args, component->pins);
        break;
    case ANALOGIC:
        analogicCreate(args, component->pins);
        break;
    case ULTRASONIC:
#if !defined(ARDUINO_ARCH_STM32) && !defined(ARDUINO_ARCH_ESP32)
        ultrasonicCreate(args, component->pins, component->sonar);
#else
        ultrasonicCreate(args, component->pins);
#endif
        break;
    }
    return component;
}

void execAct(uint32_t *execArgs, uint8_t type, component_t *component)
{
    switch (type)
    {
    case SERVO:
        servoExec(execArgs, component->servo);
        break;
    case MOTOR:
        motorExec(execArgs, component->pins, component->values, component->ledcChannel);
        break;
    case DIGITAL_OUT:
        digitalOutExec(execArgs, component->pins, component->values, component->ledcChannel);
        break;
    case FREQUENCY:
        frequencyExec(execArgs, component->pins);
        break;
    case NEOPIXEL:
        neoPixelExec(execArgs, component->neopixel);
        break;
    }
}

uint8_t getSensVal(uint8_t type, component_t *component)
{
    uint8_t tempRead;
    switch (type)
    {
    case DIGITAL_IN:
        digitalInSense(component->pins, &tempRead);
        break;
    case ANALOGIC:
        analogicSense(component->pins, &tempRead);
        break;
    case ULTRASONIC:
#if !defined(ARDUINO_ARCH_STM32) && !defined(ARDUINO_ARCH_ESP32)
        ultrasonicSense(component->pins, &tempRead, component->sonar);
#else
        ultrasonicSense(component->pins, &tempRead);
#endif

        break;
    }
    return (tempRead < 0) ? 0 : (tempRead > 100) ? 100
                                                 : tempRead;
}

void sendSensVal(uint8_t type,component_t * component)
{
#if defined(ARDUINO_ARCH_STM32)
    Serial.write((char)getSensVal(type,component));
#else
#if defined(ARDUINO_ARCH_ESP32)
    bleWrite((char)getSensVal(type,component));
#else

#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)
    Serial1.write((char)getSensVal(type,component));
#endif
    Serial.write((char)getSensVal(type,component));

#endif
#endif
}

void off(uint8_t type, component_t *component)
{
    switch (type)
    {
    case NEOPIXEL:
        neoPixelOff(component->neopixel);
        break;
    case SERVO:
        servoOff(component->servo);
        break;
    case ULTRASONIC:
#if !defined(ARDUINO_ARCH_STM32) && !defined(ARDUINO_ARCH_ESP32)
        ultrasonicOff(component->sonar);
#endif
        break;
    case MOTOR:
        motorOff(component->pins, component->ledcChannel);
        break;
    case DIGITAL_OUT:
        digitalOutOff(component->pins, component->ledcChannel);
        break;
    }
}