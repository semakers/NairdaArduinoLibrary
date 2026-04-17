#ifndef nairda_h
#define nairda_h
#include <stdint.h>
#include "components/component.h"
#include "nairda_table/nairda_table.h"
#include "nairda_string/nairda_string.h"

// ── outputs ────────────────────────────────────────────────────────
void setupDigitalOut(component_t *component, int pin);
void runDigitalOut(component_t *component, int value);

void setupServo(component_t *component, int pin, int minPulse, int maxPulse, int initialAngle);
void runServo(component_t *component, int angle);

void setupMotor(component_t *component, int pin1, int pin2, int pinSpeed);
void runMotor(component_t *component, int speed, int direction);

void setupNeoPixel(component_t *component, int pin, int numPixels);
void runNeoPixel(component_t *component, int r, int g, int b, int index);

void setupFrequency(component_t *component, int pin);
void runFrequency(component_t *component, int frequency, int duration, int volume);

// ── inputs ─────────────────────────────────────────────────────────
void setupDigitalIn(component_t *component, int pin);
uint8_t readDigitalIn(component_t *component);

void setupAnalogic(component_t *component, int pin);
uint8_t readAnalogic(component_t *component);

void setupUltrasonic(component_t *component, int triggerPin, int echoPin);
uint8_t readUltrasonic(component_t *component);

// ── kit definitions ────────────────────────────────────────────────
#define NO_KIT 0
#define ROBBUS_KIDSY_KIT 1
#define LK32_KIT 2
#define ROBBUS_ZEEGO_KIT 3

// ── core ───────────────────────────────────────────────────────────
#if defined(ARDUINO_ARCH_ESP32)
void nairdaBegin(const char *deviceName, long int bauds);
#else
void nairdaBegin(long int bauds);
#endif
void setKit(uint8_t kitCode);
void nairdaLoop();
void nairdaDelay(unsigned long ms);
#endif
