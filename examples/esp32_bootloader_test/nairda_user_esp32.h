// nairda_user_esp32.h
// Header for ESP32 user-space programs that call the Nairda kernel via the
// Jump Table at RTC slow memory (0x50000000).
//
// Slot count and signatures must match esp32SetupJumpTable() in
// NairdaArduinoLibrary/src/platform/esp32/esp32_flash.cpp (19 slots, 4 bytes
// each). Hardware setup/run primitives take an opaque component handle of
// NAIRDA_COMP_SIZE bytes that the kernel populates.

#ifndef NAIRDA_USER_ESP32_H
#define NAIRDA_USER_ESP32_H

typedef unsigned char uint8_t;

#define JUMP_TABLE_ADDR 0x50000000
#define JT_READ(slot)   (*((void**)(JUMP_TABLE_ADDR + (slot) * 4)))

#define NAIRDA_COMP_SIZE 16

// ── Outputs ────────────────────────────────────────────────────────────────
//  slot 0  setupDigitalOut(comp, pin)
//  slot 1  runDigitalOut(comp, value)         value 0..100 (PWM)
//  slot 2  setupServo(comp, pin, minPulse, maxPulse, initialAngle)
//  slot 3  runServo(comp, angle)              angle 0..180
//  slot 4  setupMotor(comp, pin1, pin2, pinSpeed)
//  slot 5  runMotor(comp, speed, direction)   direction: 0 fwd, 1 stop, 2 rev
//  slot 6  setupNeoPixel(comp, pin, numPixels)
//  slot 7  runNeoPixel(comp, r, g, b, index)
//  slot 8  setupFrequency(comp, pin)
//  slot 9  runFrequency(comp, freq, duration, volume)

#define nairda_setupDigitalOut(comp, pin) \
    ((void(*)(void*, int))JT_READ(0))((comp), (pin))
#define nairda_runDigitalOut(comp, value) \
    ((void(*)(void*, int))JT_READ(1))((comp), (value))

#define nairda_setupServo(comp, pin, minPulse, maxPulse, angle) \
    ((void(*)(void*, int, int, int, int))JT_READ(2))((comp), (pin), (minPulse), (maxPulse), (angle))
#define nairda_runServo(comp, angle) \
    ((void(*)(void*, int))JT_READ(3))((comp), (angle))

#define nairda_setupMotor(comp, pin1, pin2, pinSpeed) \
    ((void(*)(void*, int, int, int))JT_READ(4))((comp), (pin1), (pin2), (pinSpeed))
#define nairda_runMotor(comp, speed, direction) \
    ((void(*)(void*, int, int))JT_READ(5))((comp), (speed), (direction))

#define nairda_setupNeoPixel(comp, pin, numPixels) \
    ((void(*)(void*, int, int))JT_READ(6))((comp), (pin), (numPixels))
#define nairda_runNeoPixel(comp, r, g, b, index) \
    ((void(*)(void*, int, int, int, int))JT_READ(7))((comp), (r), (g), (b), (index))

#define nairda_setupFrequency(comp, pin) \
    ((void(*)(void*, int))JT_READ(8))((comp), (pin))
#define nairda_runFrequency(comp, freq, duration, volume) \
    ((void(*)(void*, int, int, int))JT_READ(9))((comp), (freq), (duration), (volume))

// ── Inputs ─────────────────────────────────────────────────────────────────
//  slot 10 setupDigitalIn(comp, pin)
//  slot 11 readDigitalIn(comp) → uint8_t (0/1)
//  slot 12 setupAnalogic(comp, pin)
//  slot 13 readAnalogic(comp) → uint8_t (0..100)
//  slot 14 setupUltrasonic(comp, triggerPin, echoPin)
//  slot 15 readUltrasonic(comp) → uint8_t (0..99 cm)

#define nairda_setupDigitalIn(comp, pin) \
    ((void(*)(void*, int))JT_READ(10))((comp), (pin))
#define nairda_readDigitalIn(comp) \
    ((uint8_t(*)(void*))JT_READ(11))((comp))

#define nairda_setupAnalogic(comp, pin) \
    ((void(*)(void*, int))JT_READ(12))((comp), (pin))
#define nairda_readAnalogic(comp) \
    ((uint8_t(*)(void*))JT_READ(13))((comp))

#define nairda_setupUltrasonic(comp, trigger, echo) \
    ((void(*)(void*, int, int))JT_READ(14))((comp), (trigger), (echo))
#define nairda_readUltrasonic(comp) \
    ((uint8_t(*)(void*))JT_READ(15))((comp))

// ── Utilities ──────────────────────────────────────────────────────────────
//  slot 16 nairdaDelay(ms)
//  slot 17 random(min, max) → long
//  slot 18 map(val, fromLo, fromHi, toLo, toHi) → long

#define nairda_delay(ms) \
    ((void(*)(unsigned long))JT_READ(16))((ms))
#define nairda_random(minVal, maxVal) \
    ((long(*)(long, long))JT_READ(17))((minVal), (maxVal))
#define nairda_map(val, fromLo, fromHi, toLo, toHi) \
    ((long(*)(long, long, long, long, long))JT_READ(18))((val), (fromLo), (fromHi), (toLo), (toHi))

#endif
