#if defined(ARDUINO_ARCH_ESP32)

#include "kits/kidsy.h"
#include "virtual_machine/virtual_machine.h"
#include "esp_flash.h"
#include "nairda_log.h"
#include <Arduino.h>

void calibrateKidsyColorSensor(VEML6040 RGBWSensor)
{
    pinMode(BUTTON_A, INPUT);
    pinMode(BUTTON_C, INPUT);

    if (digitalRead(BUTTON_A) == HIGH || digitalRead(BUTTON_C) == HIGH)
    {
        bool noCalibrate = digitalRead(BUTTON_C) == HIGH;

        Adafruit_NeoPixel neoPixel = Adafruit_NeoPixel(1, 19, NEO_GRB + NEO_KHZ800);
        RGBWSensor.nairdaBegin();
        if (!noCalibrate)
        {

            pinMode(BUTTON_B, INPUT);
            uint16_t minValues[4];
            uint16_t maxValues[4];

            while (digitalRead(BUTTON_B) == LOW)
            {
                for (uint8_t i = 0; i < 2; i++)
                {
                    neoPixel.setPixelColor(0, i ? 80 : 0, i ? 80 : 0, i ? 80 : 0);
                    neoPixel.show();
                    minValues[0] = RGBWSensor.getRed();
                    minValues[1] = RGBWSensor.getGreen();
                    minValues[2] = RGBWSensor.getBlue();
                    minValues[3] = RGBWSensor.getWhite();
                    delay(100);
                }
            }
            neoPixel.setPixelColor(0, 80, 80, 80);
            neoPixel.show();
            delay(1000);
            while (digitalRead(BUTTON_B) == LOW)
            {

                maxValues[0] = RGBWSensor.getRed();
                maxValues[1] = RGBWSensor.getGreen();
                maxValues[2] = RGBWSensor.getBlue();
                maxValues[3] = RGBWSensor.getWhite();
                delay(100);
            }
            neoPixel.setPixelColor(0, 0, 0, 0);
            neoPixel.show();
            esp_flash_erase_region(esp_flash_default_chip, 0x200000 + (4096 * 127), 4096);
            delay(150);
            uint8_t buffer[16];
            for (uint8_t i = 0; i < 4; i++)
            {
                buffer[i * 2] = minValues[i] & 0xFF;
                buffer[(i * 2) + 1] = (minValues[i] >> 8) & 0xFF;
            }
            for (uint8_t i = 0; i < 4; i++)
            {
                buffer[8 + (i * 2)] = maxValues[i] & 0xFF;
                buffer[8 + ((i * 2) + 1)] = (maxValues[i] >> 8) & 0xFF;
            }
            esp_flash_write(esp_flash_default_chip, buffer, 0x200000 + (4096 * 127), 16);
            delay(150);
        }
        RGBWSensor.readCalibration();
        while (true)
        {
            RGBWSensor.readFixedColors();
            delay(100);
            neoPixel.setPixelColor(0, RGBWSensor.getFixedRed(), RGBWSensor.getFixedGreen(), RGBWSensor.getFixedBlue());
            neoPixel.show();
        }
    }
}

// ── Touch pads ("flechas") ──────────────────────────────────────────

// Umbral por pad. Se inicializa al valor por defecto para que, si nunca se
// llama a readKidsyTouchCalibration(), la detección siga funcionando.
uint16_t kidsyTouchThresholds[KIDSY_TOUCH_COUNT] = {
    KIDSY_TOUCH_DEFAULT_THRESHOLD, KIDSY_TOUCH_DEFAULT_THRESHOLD,
    KIDSY_TOUCH_DEFAULT_THRESHOLD, KIDSY_TOUCH_DEFAULT_THRESHOLD};

// GPIOs de los pads en orden de índice. El índice también elige el color del
// pad en el loop de confirmación (0=ROJO, 1=VERDE, 2=AZUL, 3=BLANCO).
static const uint8_t kidsyTouchPins[KIDSY_TOUCH_COUNT] = {12, 13, 15, 14};

int8_t kidsyArrowIndex(uint8_t pin)
{
    for (uint8_t i = 0; i < KIDSY_TOUCH_COUNT; i++)
    {
        if (pin == kidsyTouchPins[i])
            return (int8_t)i;
    }
    return -1;
}

void readKidsyTouchCalibration(void)
{
    uint8_t buffer[KIDSY_TOUCH_COUNT * 2];
    esp_err_t err = esp_flash_read(esp_flash_default_chip, buffer,
                                   KIDSY_TOUCH_FLASH_ADDR, sizeof(buffer));
    NRD_LOG("[NRD/TOUCH] readCalibration err=%d addr=0x%08X\n",
            err, (unsigned)KIDSY_TOUCH_FLASH_ADDR);
    for (uint8_t i = 0; i < KIDSY_TOUCH_COUNT; i++)
    {
        uint16_t v = (uint16_t)buffer[i * 2] | ((uint16_t)buffer[i * 2 + 1] << 8);
        // Flash sin programar (0xFFFF) o vacía → usar el umbral por defecto.
        if (v == 0xFFFF || v == 0)
            v = KIDSY_TOUCH_DEFAULT_THRESHOLD;
        kidsyTouchThresholds[i] = v;
        NRD_LOG("[NRD/TOUCH]   pad %u (pin %u) threshold=%u\n",
                i, kidsyTouchPins[i], v);
    }
}

void calibrateKidsyTouchSensors(void)
{
#if !defined(CONFIG_IDF_TARGET_ESP32C3)
    pinMode(BUTTON_B, INPUT);

    // Solo se entra si B está presionado al encender; si no, es un no-op.
    if (digitalRead(BUTTON_B) != HIGH)
        return;

    NRD_LOGLN("[NRD/TOUCH] entering touch calibration (BTN_B held)");

    Adafruit_NeoPixel neoPixel = Adafruit_NeoPixel(1, 19, NEO_GRB + NEO_KHZ800);
    neoPixel.begin();

    // Esperar a que se suelte la pulsación de entrada para que no se lea como
    // la pulsación de "terminar" del bucle de captura.
    while (digitalRead(BUTTON_B) == HIGH)
        delay(10);
    delay(200); // antirrebote

    // Extremos por pad durante la ventana de captura:
    //   minVal → lectura más baja (pad presionado a fondo)
    //   maxVal → lectura más alta (pad en reposo)
    uint16_t minVal[KIDSY_TOUCH_COUNT];
    uint16_t maxVal[KIDSY_TOUCH_COUNT];
    for (uint8_t i = 0; i < KIDSY_TOUCH_COUNT; i++)
    {
        minVal[i] = 0xFFFF;
        maxVal[i] = 0;
    }

    // Ventana de captura: el NeoPixel parpadea AZUL mientras se muestrean los
    // cuatro pads. El usuario toca cada pad (~1s c/u, en cualquier orden) y
    // termina presionando B. Como solo se toca un pad a la vez, los otros tres
    // registran su valor de reposo → obtenemos min (presión) y max (reposo)
    // por pad en una sola ventana, sin fase de reposo aparte.
    uint8_t tick = 0;
    bool ledOn = false;
    while (digitalRead(BUTTON_B) == LOW)
    {
        for (uint8_t i = 0; i < KIDSY_TOUCH_COUNT; i++)
        {
            uint16_t v = (uint16_t)touchRead(kidsyTouchPins[i]);
            if (v < minVal[i])
                minVal[i] = v;
            if (v > maxVal[i])
                maxVal[i] = v;
        }

        if ((tick++ % 5) == 0)
        {
            ledOn = !ledOn;
            neoPixel.setPixelColor(0, 0, 0, ledOn ? 80 : 0);
            neoPixel.show();
        }
        delay(50);
    }

    neoPixel.setPixelColor(0, 0, 0, 0);
    neoPixel.show();

    // Umbral adaptativo por pad. Un pad solo cuenta como calibrado si vio una
    // caída reposo→presión clara; si no, se conserva el umbral por defecto
    // para que un pad que el usuario olvidó tocar nunca quede "presionado".
    for (uint8_t i = 0; i < KIDSY_TOUCH_COUNT; i++)
    {
        uint16_t thr = KIDSY_TOUCH_DEFAULT_THRESHOLD;
        if (maxVal[i] > minVal[i] &&
            (uint16_t)(maxVal[i] - minVal[i]) >= KIDSY_TOUCH_MIN_DELTA)
        {
            // 40% hacia arriba desde el valor presionado hacia el reposo.
            uint16_t delta = maxVal[i] - minVal[i];
            thr = minVal[i] + (uint16_t)(((uint32_t)delta * 2) / 5);
        }
        kidsyTouchThresholds[i] = thr;
        NRD_LOG("[NRD/TOUCH] pad %u pin %u: min=%u max=%u -> thr=%u %s\n",
                i, kidsyTouchPins[i], minVal[i], maxVal[i], thr,
                (thr == KIDSY_TOUCH_DEFAULT_THRESHOLD) ? "(default)" : "");
    }

    // Persistir: 4 × uint16 LE en el sector dedicado.
    esp_flash_erase_region(esp_flash_default_chip, KIDSY_TOUCH_FLASH_ADDR, 4096);
    delay(150);
    uint8_t buffer[KIDSY_TOUCH_COUNT * 2];
    for (uint8_t i = 0; i < KIDSY_TOUCH_COUNT; i++)
    {
        buffer[i * 2] = kidsyTouchThresholds[i] & 0xFF;
        buffer[i * 2 + 1] = (kidsyTouchThresholds[i] >> 8) & 0xFF;
    }
    esp_flash_write(esp_flash_default_chip, buffer, KIDSY_TOUCH_FLASH_ADDR, sizeof(buffer));
    delay(150);

    // Esperar a que se suelte la pulsación de "terminar" antes del loop.
    while (digitalRead(BUTTON_B) == HIGH)
        delay(10);

    // Loop de confirmación (reiniciar para salir): NeoPixel apagado cuando no
    // se toca nada; cada pad enciende su propio color para que el usuario
    // verifique que los cuatro responden. Colores combinables si se tocan
    // varios a la vez (p.ej. rojo + verde = amarillo).
    static const uint8_t padColors[KIDSY_TOUCH_COUNT][3] = {
        {80, 0, 0},   // pad 0 → rojo
        {0, 80, 0},   // pad 1 → verde
        {0, 0, 80},   // pad 2 → azul
        {80, 80, 80}, // pad 3 → blanco
    };
    while (true)
    {
        uint16_t r = 0, g = 0, b = 0;
        for (uint8_t i = 0; i < KIDSY_TOUCH_COUNT; i++)
        {
            uint16_t v = (uint16_t)touchRead(kidsyTouchPins[i]);
            if (v < kidsyTouchThresholds[i])
            {
                r += padColors[i][0];
                g += padColors[i][1];
                b += padColors[i][2];
            }
        }
        if (r > 255)
            r = 255;
        if (g > 255)
            g = 255;
        if (b > 255)
            b = 255;
        neoPixel.setPixelColor(0, (uint8_t)r, (uint8_t)g, (uint8_t)b);
        neoPixel.show();
        delay(50);
    }
#endif
}

#endif