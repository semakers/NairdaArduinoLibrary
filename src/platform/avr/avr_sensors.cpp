#if !defined(ARDUINO_ARCH_ESP32)

#include <Arduino.h>
#include "virtual_machine/virtual_machine.h"

// ── Mocks de sensores para auditar el protocolo (variante AVR) ─────
//
// Mismo banco que en esp32_sensors.cpp: con NAIRDA_MOCK_SENSORS=1 los
// sensores no tocan hardware y devuelven patrones deterministas para validar
// el protocolo desde la app a traves del modulo BLE serie (HC-08):
//   - Analogico:  diente de sierra 0..100, avanza 1 por lectura, offset=pin.
//                 Un valor repetido delata lectura rancia; el offset, cruce
//                 de claves.
//   - Digital in: alterna 0/1 en cada lectura.
//   - Ultrasonico (avr_ultrasonic.cpp): constante = pin de trigger.
// Apagar (0) para volver a los sensores reales.
#define NAIRDA_MOCK_SENSORS 0

#if NAIRDA_MOCK_SENSORS
static uint16_t mockReads[32]; // contador por pin (D0..D19 y A0..A7 mapeados)

static uint8_t mockIdx(uint8_t pin) { return pin & 31; }

static uint8_t mockAnalog(uint8_t pin)
{
    uint8_t idx = mockIdx(pin);
    uint8_t val = (uint8_t)((pin + mockReads[idx]) % 101);
    mockReads[idx]++;
    return val;
}

static uint8_t mockDigital(uint8_t pin)
{
    uint8_t idx = mockIdx(pin);
    uint8_t val = (uint8_t)(mockReads[idx] & 1);
    mockReads[idx]++;
    return val;
}
#endif

void analogicCreate(uint16_t *args, component_t *component)
{
    component->pins[0] = args[1];
}

void analogicSense(uint8_t *pins, uint8_t *tempRead)
{
#if NAIRDA_MOCK_SENSORS
    tempRead[0] = mockAnalog(pins[0]);
    return;
#endif
    tempRead[0] = map(analogRead(pins[0]), 0, 1023, 0, 100);
}

void digitalInSense(uint8_t *pins, uint8_t *tempRead)
{
#if NAIRDA_MOCK_SENSORS
    tempRead[0] = mockDigital(pins[0]);
    return;
#endif
    tempRead[0] = digitalRead(pins[0]);
}

#endif
