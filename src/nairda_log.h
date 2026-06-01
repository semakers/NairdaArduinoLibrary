#ifndef NAIRDA_LOG_H
#define NAIRDA_LOG_H

#include <Arduino.h>

// ─────────────────────────────────────────────────────────────────────────────
// Nairda kernel trace logs — compile-time gated
//
// Master switch: 1 = enable verbose kernel trace logs over Serial.
//                0 = all NRD_* macros expand to nothing (zero overhead).
//
// Flip to 1, recompile + flash the firmware, open the USB serial monitor at
// the same baud as nairdaBegin() (typically 9600). You'll see boot phases,
// jump-table dispatches, peripheral inits, sensor reads, etc.
//
// Throttled variants exist for logs inside tight loops (avoid flooding the
// serial line). Each NRD_LOG_THROTTLED call site has its own internal
// timestamp, so different sites don't interfere.
// ─────────────────────────────────────────────────────────────────────────────

#define NAIRDA_DEBUG 0

// AVR's HardwareSerial has no printf; force-disable trace logs there even when
// the master switch is on. The logs are only useful on ESP32 anyway.
#if !defined(ARDUINO_ARCH_ESP32)
  #undef NAIRDA_DEBUG
  #define NAIRDA_DEBUG 0
#endif

#if NAIRDA_DEBUG
  #define NRD_LOG(...)      Serial.printf(__VA_ARGS__)
  #define NRD_LOGLN(s)      Serial.println(s)
  #define NRD_FLUSH()       Serial.flush()

  #define NRD_LOG_THROTTLED(period_ms, ...)        \
    do {                                            \
      static unsigned long _nrd_last = 0;           \
      if (millis() - _nrd_last > (period_ms)) {     \
        Serial.printf(__VA_ARGS__);                 \
        _nrd_last = millis();                       \
      }                                             \
    } while (0)
#else
  #define NRD_LOG(...)                      ((void)0)
  #define NRD_LOGLN(s)                      ((void)0)
  #define NRD_FLUSH()                       ((void)0)
  #define NRD_LOG_THROTTLED(period_ms, ...) ((void)0)
#endif

#endif // NAIRDA_LOG_H
