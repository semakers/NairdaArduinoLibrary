#if defined(ARDUINO_ARCH_ESP32)
#include <stdint.h>
#include "extern_libraries/veml6040/VEML6040.h"

#define BUTTON_A 34
#define BUTTON_B 35
#define BUTTON_C 36

// ── Touch pads ("flechas") ──────────────────────────────────────────
// Cuatro pads capacitivos leídos con touchRead(). En el ESP32 clásico el
// valor BAJA al tocar (reposo ~850-1150, tocado → tiende a 0).
#define KIDSY_TOUCH_COUNT 4

// Sector de flash cruda (sector 125) para los umbrales de touch. Está por
// encima de los 2MB de la tabla de particiones — separado de la calibración
// de color/piso (sector 127) y del nombre BLE (sector 126) para no pisarse.
#define KIDSY_TOUCH_FLASH_ADDR (0x200000 + (4096 * 125))

// Umbral de respaldo cuando un pad no está calibrado (flash = 0xFFFF) o no se
// tocó durante la calibración. Coincide con el valor fijo previo.
#define KIDSY_TOUCH_DEFAULT_THRESHOLD 200

// Caída reposo→presión mínima para aceptar una captura como toque real.
#define KIDSY_TOUCH_MIN_DELTA 300

// Umbrales por pad cargados desde flash y usados en digitalInSense().
extern uint16_t kidsyTouchThresholds[KIDSY_TOUCH_COUNT];

void calibrateKidsyColorSensor(VEML6040 RGBWSensor);
void calibrateKidsyTouchSensors(void);
void readKidsyTouchCalibration(void);
int8_t kidsyArrowIndex(uint8_t pin);
#endif
