// nairda_user_esp32.h
// Header para apps de usuario ESP32 - equivalente a nairda_user.h de AVR
//
// Jump Table en RTC slow memory (0x50000000) - dirección fija
// Igual que AVR usa 0x5F80 en flash

#ifndef NAIRDA_USER_ESP32_H
#define NAIRDA_USER_ESP32_H

#define JUMP_TABLE_ADDR 0x50000000
#define JT_READ(slot) (*((void**)(JUMP_TABLE_ADDR + (slot) * 4)))

// [0] init(pin)   → pinMode(pin, OUTPUT)
// [1] on(pin)     → digitalWrite(pin, HIGH)
// [2] off(pin)    → digitalWrite(pin, LOW)
// [3] delay(ms)   → delay(ms)

#define nairda_init(pin)  ((void(*)(int))JT_READ(0))(pin)
#define nairda_on(pin)    ((void(*)(int))JT_READ(1))(pin)
#define nairda_off(pin)   ((void(*)(int))JT_READ(2))(pin)
#define nairda_delay(ms)  ((void(*)(int))JT_READ(3))(ms)

#endif
