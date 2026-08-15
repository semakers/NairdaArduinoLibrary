#if defined(ARDUINO_ARCH_ESP32)

#include <Arduino.h>
#include "virtual_machine/virtual_machine.h"
#include "nairda.h"
#include "nairda_log.h"

#include "extern_libraries/veml6040/VEML6040.h"
#include "extern_libraries/dht11/DHT.h"
#include "kits/zeego.h"
#include "kits/kidsy.h"

extern VEML6040 RGBWSensor;
extern DHT dht;
extern int temp;
extern int hum;
extern uint8_t currentKit;

static unsigned long previousMillis = 0;

// ── Mocks de sensores para auditar el protocolo ────────────────────
//
// Con NAIRDA_MOCK_SENSORS=1 los sensores NO tocan hardware: devuelven
// patrones deterministas pensados para validar el protocolo de comunicacion
// de punta a punta desde la app, tratando al micro como un backend:
//
//   - Analogico:   diente de sierra 0..100 que avanza UNA posicion por
//                  lectura, con offset inicial = pin. Una lectura rancia
//                  (el caché GATT servido sin refrescar) se delata como
//                  valor REPETIDO; una lectura cruzada, por el offset.
//   - Digital in:  alterna 0/1 en cada lectura.
//   - (El ultrasonico tiene su propio mock en esp32_ultrasonic.cpp.)
//
// Apagar (0) para volver a los sensores reales.
#define NAIRDA_MOCK_SENSORS 0

#if NAIRDA_MOCK_SENSORS
static uint16_t mockReads[40];   // contador de lecturas por pin (GPIO 0..39)

static uint8_t mockAnalog(uint8_t pin)
{
    uint8_t idx = pin < 40 ? pin : 0;
    uint8_t val = (uint8_t)((pin + mockReads[idx]) % 101);
    mockReads[idx]++;
    NRD_LOG("[NRD/MOCK] analog pin=%u lectura#%u -> %u\n",
            pin, (unsigned)mockReads[idx], val);
    return val;
}

static uint8_t mockDigital(uint8_t pin)
{
    uint8_t idx = pin < 40 ? pin : 0;
    uint8_t val = (uint8_t)(mockReads[idx] & 1);
    mockReads[idx]++;
    NRD_LOG("[NRD/MOCK] digital pin=%u lectura#%u -> %u\n",
            pin, (unsigned)mockReads[idx], val);
    return val;
}
#endif

// ── Analogic ───────────────────────────────────────────────────────

// analogRead sobre un GPIO sin canal ADC hace que ESP-IDF imprima
// "adc_io_to_channel: invalid gpio number" EN CADA LECTURA. A 9600 baudios
// esa linea (~55 chars) bloquea el main task ~57 ms: con el ritmo de sondeo
// de la app el main loop queda estrangulado, deja de drenar el buffer BLE y
// la conexion entera se degrada hasta morir (visto en el banco fisico: un
// solo pin mal tecleado por el nino tumbaba el enlace). Validamos ANTES de
// llamar a analogRead: pin invalido → 0, un aviso una sola vez, y cero spam.
static uint8_t safeAnalogRead0to100(uint8_t pin)
{
    if (digitalPinToAnalogChannel(pin) < 0)
    {
        static uint8_t warned_pin = 0xFF;
        if (warned_pin != pin)
        {
            warned_pin = pin;
            NRD_LOG("[NRD] analogicSense: pin %u SIN canal ADC — leera 0 "
                    "(aviso unico)\n", pin);
        }
        return 0;
    }
    return map(analogRead(pin), 0, 4095, 0, 100);
}

void analogicCreate(uint16_t *args, component_t *component)
{
    NRD_LOG("[NRD] analogicCreate(pin=%u) kit=%d comp=%p\n", args[1], currentKit, component);
    component->pins[0] = args[1];
    if (currentKit == ROBBUS_KIDSY_KIT && (args[1] == 37 || args[1] == 38 || args[1] == 39))
    {
        NRD_LOGLN("[NRD]   → Kidsy color pin, calling RGBWSensor.nairdaBegin()");
        RGBWSensor.nairdaBegin();
        NRD_LOGLN("[NRD]   ← RGBWSensor.nairdaBegin() returned");
    }
}

void analogicSense(uint8_t *pins, uint8_t *tempRead)
{
#if NAIRDA_MOCK_SENSORS
    tempRead[0] = mockAnalog(pins[0]);
    return;
#endif
    if (currentKit == LK32_KIT)
    {
        if (pins[0] == 16 || pins[0] == 17)
        {
            unsigned long currentMillis = millis();
            if (currentMillis - previousMillis >= 2000)
            {
                delay(10);
                previousMillis = currentMillis;
                temp = (round(dht.readTemperature()));
                hum = (round(dht.readHumidity()));
            }
        }
        switch (pins[0])
        {
        case 16:
            tempRead[0] = temp < 0 ? 0 : temp > 100 ? 100 : temp;
            break;
        case 17:
            tempRead[0] = hum < 0 ? 0 : hum > 100 ? 100 : hum;
            break;
        default:
            tempRead[0] = safeAnalogRead0to100(pins[0]);
        }
    }
    else if (currentKit == ROBBUS_KIDSY_KIT)
    {
        NRD_LOG_THROTTLED(500, "[NRD] analogicSense pin=%u (Kidsy path)\n", pins[0]);
        if (pins[0] == 37 || pins[0] == 38 || pins[0] == 39)
        {
            RGBWSensor.readFixedColors();
        }
        switch (pins[0])
        {
        case 37:
            tempRead[0] = RGBWSensor.getFixedRed();
            break;
        case 38:
            tempRead[0] = RGBWSensor.getFixedGreen();
            break;
        case 39:
            tempRead[0] = RGBWSensor.getFixedBlue();
            break;
        default:
            tempRead[0] = safeAnalogRead0to100(pins[0]);
        }
        NRD_LOG_THROTTLED(500, "[NRD]   tempRead[0]=%u\n", tempRead[0]);
    }
    else if (currentKit == ROBBUS_ZEEGO_KIT)
    {
        if (pins[0] == 37 || pins[0] == 38)
        {
            switch (pins[0])
            {
            case 37:
                tempRead[0] = readFloorValue();
                break;
            case 38:
                tempRead[0] = readFloorNumValue();
                break;
            }
        }
        else
        {
            tempRead[0] = safeAnalogRead0to100(pins[0]);
        }
    }
    else
    {
        tempRead[0] = safeAnalogRead0to100(pins[0]);
    }
}

// ── Digital Input ──────────────────────────────────────────────────

void digitalInSense(uint8_t *pins, uint8_t *tempRead)
{
#if NAIRDA_MOCK_SENSORS
    tempRead[0] = mockDigital(pins[0]);
    return;
#endif
    if (currentKit == ROBBUS_KIDSY_KIT)
    {
#if !defined(CONFIG_IDF_TARGET_ESP32C3)
        int8_t idx = kidsyArrowIndex(pins[0]);
        if (idx >= 0) {
            // Kidsy touch pads: untouched ~850-1150, touched approaches 0.
            // Per-pad threshold from calibration (falls back to the default
            // when the pad was never calibrated). Pressed when value < threshold.
            uint32_t tval = touchRead(pins[0]);
            uint16_t thr = kidsyTouchThresholds[idx];
            NRD_LOG_THROTTLED(200, "[NRD/TOUCH] pin=%u raw=%u threshold=%u\n", pins[0], (unsigned)tval, thr);
            tempRead[0] = tval < thr ? 1 : 0;
        } else {
            tempRead[0] = digitalRead(pins[0]);
        }
#else
        tempRead[0] = digitalRead(pins[0]);
#endif
    }
    else
    {
        tempRead[0] = digitalRead(pins[0]);
    }
}

#endif
