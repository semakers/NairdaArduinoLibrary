#ifndef PLATFORM_HAL_H
#define PLATFORM_HAL_H

#include <stdint.h>

struct VolatileMemory;

// ── Communication ──────────────────────────────────────────────────
void hal_sendByte(uint8_t byte);

// ── Debug handlers (called from nairdaDebug) ───────────────────────
void hal_handleProjectInit(VolatileMemory *volatileMemory);
void hal_handleBootloader(VolatileMemory *volatileMemory);
void hal_handleVersionCommand();

// ── Loop hook ──────────────────────────────────────────────────────
bool hal_checkRebootRequest();

#endif
