// user_syscalls.h — Syscall wrappers for bootjacker_loader mini-kernel
// Jump table at 0x5F80 (same as Nairda kernel)
//   Slot 0: pin13_setup()        — void(void)
//   Slot 1: led_on()             — void(void)
//   Slot 2: led_off()            — void(void)
//   Slot 3: delay_ms(unsigned long) — void(unsigned long)

#ifndef USER_SYSCALLS_H
#define USER_SYSCALLS_H

#include <stdint.h>
#include <avr/io.h>
#include <avr/pgmspace.h>

typedef void (*fn_ptr)(void);
#define JT_ADDR 0x5F80
#define JT_READ(slot) ((fn_ptr)pgm_read_word(JT_ADDR + (slot) * 2))

#define pin13_setup()   (JT_READ(0))()
#define led_on()        (JT_READ(1))()
#define led_off()       (JT_READ(2))()
#define delay_ms(ms)    ((void(*)(unsigned long))JT_READ(3))((ms))

#endif
