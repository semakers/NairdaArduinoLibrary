# Nairda User API — Referencia para Programas de Usuario

## Introducción

Los programas de usuario se ejecutan desde el espacio Flash del ATmega328P (dirección 0x6002+). No tienen acceso directo al kernel de Nairda ni a las librerías de Arduino. En su lugar, usan la **Jump Table** — un array de punteros a función en Flash (0x5F80) — para llamar al kernel.

Cada función de la API recibe un **handle opaco** (`uint8_t comp[16]`) que representa un componente de hardware. El usuario declara el handle, las funciones `setup*` lo inicializan, y las funciones `run*`/`read*` lo usan para operar el hardware.

---

## Header: `nairda_user.h`

Todo programa de usuario debe incluir este header:

```c
#include "nairda_user.h"
```

Provee:
- Macros para todas las funciones de la API
- `NAIRDA_COMP_SIZE` (16) — tamaño del handle de componente
- `JT_READ(slot)` — macro interna que lee punteros desde Flash con `pgm_read_word()`

---

## Handle de Componente

Cada componente de hardware se representa como un array de 16 bytes:

```c
uint8_t mi_motor[NAIRDA_COMP_SIZE] = {0};
```

Internamente, el kernel interpreta estos 16 bytes como una estructura `component_t` que contiene pines, valores y punteros a objetos de hardware (Servo, NeoPixel, etc.). El usuario **nunca accede** a los bytes individuales — solo pasa el array a las funciones de la API.

### Reglas

1. **Siempre inicializar con `= {0}`** antes del primer `setup*`
2. **Un handle por componente** — no reusar el mismo handle para dos componentes distintos
3. **Llamar `setup*` antes de `run*`/`read*`** — el setup configura el hardware y llena el handle
4. **No copiar handles** — contienen punteros internos que no deben duplicarse

---

## API: Salidas Digitales

### `nairda_setupDigitalOut(comp, pin)`

Configura un pin como salida digital con PWM.

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `comp` | `uint8_t[16]` | Handle del componente (debe estar inicializado en ceros) |
| `pin` | `int` | Número de pin Arduino (2-13, A0=70, A1=71, ...) |

```c
uint8_t led[NAIRDA_COMP_SIZE] = {0};
nairda_setupDigitalOut(led, 13);  // LED integrado
```

### `nairda_runDigitalOut(comp, value)`

Escribe un valor en la salida digital.

| Parámetro | Tipo | Rango | Descripción |
|-----------|------|-------|-------------|
| `comp` | `uint8_t[16]` | — | Handle configurado con `setupDigitalOut` |
| `value` | `int` | 0 - 100 | 0 = apagado, 100 = encendido a máxima intensidad. Valores intermedios controlan PWM. |

```c
nairda_runDigitalOut(led, 100);  // Encender al 100%
nairda_runDigitalOut(led, 50);   // 50% de intensidad (PWM)
nairda_runDigitalOut(led, 0);    // Apagar
```

---

## API: Servomotores

### `nairda_setupServo(comp, pin, minPulse, maxPulse, initialAngle)`

Configura un servomotor.

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `comp` | `uint8_t[16]` | Handle del componente |
| `pin` | `int` | Pin de señal del servo |
| `minPulse` | `int` | Ancho de pulso mínimo en microsegundos (típico: 544) |
| `maxPulse` | `int` | Ancho de pulso máximo en microsegundos (típico: 2400) |
| `initialAngle` | `int` | Ángulo inicial en grados (0-180) |

```c
uint8_t servo[NAIRDA_COMP_SIZE] = {0};
nairda_setupServo(servo, 9, 544, 2400, 90);  // Servo en pin 9, centrado
```

### `nairda_runServo(comp, angle)`

Mueve el servo a un ángulo específico.

| Parámetro | Tipo | Rango | Descripción |
|-----------|------|-------|-------------|
| `comp` | `uint8_t[16]` | — | Handle configurado con `setupServo` |
| `angle` | `int` | 0 - 180 | Ángulo en grados |

```c
nairda_runServo(servo, 0);    // Girar a 0°
nairda_delay(1000);
nairda_runServo(servo, 180);  // Girar a 180°
```

---

## API: Motores DC

### `nairda_setupMotor(comp, pin1, pin2, pinSpeed)`

Configura un motor DC con puente H.

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `comp` | `uint8_t[16]` | Handle del componente |
| `pin1` | `int` | Pin de dirección 1 |
| `pin2` | `int` | Pin de dirección 2 |
| `pinSpeed` | `int` | Pin PWM para velocidad. Si es 0, pin1 y pin2 se usan como PWM bidireccional. |

```c
uint8_t motor[NAIRDA_COMP_SIZE] = {0};
nairda_setupMotor(motor, 3, 4, 5);  // IN1=3, IN2=4, ENA=5
```

### `nairda_runMotor(comp, speed, direction)`

Controla velocidad y dirección del motor.

| Parámetro | Tipo | Rango | Descripción |
|-----------|------|-------|-------------|
| `comp` | `uint8_t[16]` | — | Handle configurado con `setupMotor` |
| `speed` | `int` | 0 - 100 | Velocidad en porcentaje |
| `direction` | `int` | 0, 1, 2 | 0 = adelante, 1 = detenido, 2 = reversa |

```c
nairda_runMotor(motor, 80, 0);   // 80% adelante
nairda_delay(2000);
nairda_runMotor(motor, 0, 1);    // Detenido
nairda_delay(500);
nairda_runMotor(motor, 60, 2);   // 60% reversa
```

---

## API: NeoPixel (LEDs RGB direccionables)

### `nairda_setupNeoPixel(comp, pin, numPixels)`

Configura una tira de NeoPixels.

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `comp` | `uint8_t[16]` | Handle del componente |
| `pin` | `int` | Pin de datos |
| `numPixels` | `int` | Cantidad de LEDs en la tira |

```c
uint8_t strip[NAIRDA_COMP_SIZE] = {0};
nairda_setupNeoPixel(strip, 6, 8);  // 8 NeoPixels en pin 6
```

### `nairda_runNeoPixel(comp, r, g, b, index)`

Establece el color de un pixel individual.

| Parámetro | Tipo | Rango | Descripción |
|-----------|------|-------|-------------|
| `comp` | `uint8_t[16]` | — | Handle configurado con `setupNeoPixel` |
| `r` | `int` | 0 - 255 | Componente rojo |
| `g` | `int` | 0 - 255 | Componente verde |
| `b` | `int` | 0 - 255 | Componente azul |
| `index` | `int` | 0 - (numPixels-1) | Índice del pixel |

```c
nairda_runNeoPixel(strip, 255, 0, 0, 0);    // Pixel 0 = rojo
nairda_runNeoPixel(strip, 0, 255, 0, 1);    // Pixel 1 = verde
nairda_runNeoPixel(strip, 0, 0, 255, 2);    // Pixel 2 = azul
```

---

## API: Frecuencia / Buzzer

### `nairda_setupFrequency(comp, pin)`

Configura un pin para generar tonos.

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `comp` | `uint8_t[16]` | Handle del componente |
| `pin` | `int` | Pin del buzzer/speaker |

```c
uint8_t buzzer[NAIRDA_COMP_SIZE] = {0};
nairda_setupFrequency(buzzer, 8);
```

### `nairda_runFrequency(comp, frequency, duration, volume)`

Genera un tono.

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `comp` | `uint8_t[16]` | Handle configurado con `setupFrequency` |
| `frequency` | `int` | Frecuencia en Hz (e.g., 440 = La4) |
| `duration` | `int` | Duración en milisegundos |
| `volume` | `int` | Volumen (depende de implementación) |

```c
nairda_runFrequency(buzzer, 440, 500, 10);   // La4, 500ms
nairda_delay(100);
nairda_runFrequency(buzzer, 523, 500, 10);   // Do5, 500ms
```

---

## API: Entrada Digital

### `nairda_setupDigitalIn(comp, pin)`

Configura un pin como entrada digital.

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `comp` | `uint8_t[16]` | Handle del componente |
| `pin` | `int` | Número de pin |

```c
uint8_t boton[NAIRDA_COMP_SIZE] = {0};
nairda_setupDigitalIn(boton, 2);  // Botón en pin 2
```

### `nairda_readDigitalIn(comp)` → `uint8_t`

Lee el estado del pin digital.

| Retorno | Descripción |
|---------|-------------|
| `0` | Pin en LOW |
| `1` | Pin en HIGH |

```c
uint8_t estado = nairda_readDigitalIn(boton);
if (estado == 1) {
    nairda_runDigitalOut(led, 100);  // Encender LED
}
```

---

## API: Entrada Analógica

### `nairda_setupAnalogic(comp, pin)`

Configura un pin como entrada analógica.

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `comp` | `uint8_t[16]` | Handle del componente |
| `pin` | `int` | Pin analógico (70=A0, 71=A1, ...) |

```c
uint8_t potenciometro[NAIRDA_COMP_SIZE] = {0};
nairda_setupAnalogic(potenciometro, 70);  // A0
```

### `nairda_readAnalogic(comp)` → `uint8_t`

Lee el valor analógico.

| Retorno | Rango | Descripción |
|---------|-------|-------------|
| `uint8_t` | 0 - 100 | Valor mapeado a porcentaje (0-1023 → 0-100) |

```c
uint8_t valor = nairda_readAnalogic(potenciometro);
nairda_runMotor(motor, valor, 0);  // Velocidad controlada por potenciómetro
```

---

## API: Sensor Ultrasónico

### `nairda_setupUltrasonic(comp, triggerPin, echoPin)`

Configura un sensor ultrasónico HC-SR04.

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `comp` | `uint8_t[16]` | Handle del componente |
| `triggerPin` | `int` | Pin de trigger |
| `echoPin` | `int` | Pin de echo |

```c
uint8_t sensor[NAIRDA_COMP_SIZE] = {0};
nairda_setupUltrasonic(sensor, 7, 8);  // Trigger=7, Echo=8
```

### `nairda_readUltrasonic(comp)` → `uint8_t`

Lee la distancia medida.

| Retorno | Rango | Descripción |
|---------|-------|-------------|
| `uint8_t` | 0 - 99 | Distancia en centímetros (máximo ~100cm). 99 si no detecta objeto. |

```c
uint8_t distancia = nairda_readUltrasonic(sensor);
if (distancia < 20) {
    nairda_runMotor(motor, 0, 1);  // Detenerse si hay obstáculo
}
```

---

## API: Utilidades

### `nairda_delay(ms)`

Pausa la ejecución por un tiempo determinado. Durante el delay, el kernel sigue ejecutando `nairdaLoop()` internamente para mantener la comunicación BLE activa.

| Parámetro | Tipo | Descripción |
|-----------|------|-------------|
| `ms` | `unsigned long` | Milisegundos de espera |

```c
nairda_delay(1000);  // Esperar 1 segundo
```

---

## Estructura de un Programa de Usuario

```c
#include "nairda_user.h"

void __attribute__((section(".text"))) _start(void) {
    // Declarar componentes
    uint8_t motor_izq[NAIRDA_COMP_SIZE] = {0};
    uint8_t motor_der[NAIRDA_COMP_SIZE] = {0};
    uint8_t sensor[NAIRDA_COMP_SIZE] = {0};

    // Inicializar hardware
    nairda_setupMotor(motor_izq, 3, 4, 5);
    nairda_setupMotor(motor_der, 6, 7, 9);
    nairda_setupUltrasonic(sensor, 10, 11);

    // Loop principal
    while (1) {
        uint8_t dist = nairda_readUltrasonic(sensor);

        if (dist < 15) {
            // Obstáculo: girar
            nairda_runMotor(motor_izq, 60, 2);
            nairda_runMotor(motor_der, 60, 0);
            nairda_delay(500);
        } else {
            // Camino libre: avanzar
            nairda_runMotor(motor_izq, 70, 0);
            nairda_runMotor(motor_der, 70, 0);
        }

        nairda_delay(100);
    }
}
```

### Reglas del programa

1. **Punto de entrada**: La función `_start` con `__attribute__((section(".text")))`. No usar `naked`.
2. **Loop infinito**: El programa debe tener un `while(1)` — si retorna, el comportamiento es indefinido.
3. **Sin Arduino.h**: No usar `Serial`, `digitalWrite`, `analogRead`, etc. Solo la API de Nairda.
4. **Sin librerías**: No incluir librerías externas. Todo se accede vía Jump Table.
5. **Máximo 8190 bytes**: El espacio de usuario es de ~8KB.

---

## Compilación

```bash
avr-gcc -mmcu=atmega328p -Os -nostartfiles \
    -I/ruta/a/nairda_user.h \
    -Wl,--section-start=.text=0x6002 \
    -o app.elf app.c

avr-objcopy -O binary -j .text app.elf app.bin
```

El binario resultante (`app.bin`) se envía al Arduino prepuesto con 2 bytes de padding: `[0x00, 0xFF] + app.bin`.

---

## Mapeo de Pines

| Valor | Pin Arduino |
|-------|-------------|
| 0-13 | Pines digitales D0-D13 |
| 70 | A0 |
| 71 | A1 |
| 72 | A2 |
| 73 | A3 |
| 74 | A4 |
| 75 | A5 |

---

## Jump Table — Referencia de Slots

| Slot | Dirección | Función | Firma del wrapper |
|------|-----------|---------|-------------------|
| 0 | 0x5F80 | setupDigitalOut | `void (void*, int)` |
| 1 | 0x5F82 | runDigitalOut | `void (void*, int)` |
| 2 | 0x5F84 | setupServo | `void (void*, int, int, int, int)` |
| 3 | 0x5F86 | runServo | `void (void*, int)` |
| 4 | 0x5F88 | setupMotor | `void (void*, int, int, int)` |
| 5 | 0x5F8A | runMotor | `void (void*, int, int)` |
| 6 | 0x5F8C | setupNeoPixel | `void (void*, int, int)` |
| 7 | 0x5F8E | runNeoPixel | `void (void*, int, int, int, int)` |
| 8 | 0x5F90 | setupFrequency | `void (void*, int)` |
| 9 | 0x5F92 | runFrequency | `void (void*, int, int, int)` |
| 10 | 0x5F94 | setupDigitalIn | `void (void*, int)` |
| 11 | 0x5F96 | readDigitalIn | `uint8_t (void*)` |
| 12 | 0x5F98 | setupAnalogic | `void (void*, int)` |
| 13 | 0x5F9A | readAnalogic | `uint8_t (void*)` |
| 14 | 0x5F9C | setupUltrasonic | `void (void*, int, int)` |
| 15 | 0x5F9E | readUltrasonic | `uint8_t (void*)` |
| 16 | 0x5FA0 | nairdaDelay | `void (unsigned long)` |

Cada slot almacena un **word address** (2 bytes). Los punteros se leen con `pgm_read_word()` porque la tabla está en Flash (PROGMEM), no en RAM.
