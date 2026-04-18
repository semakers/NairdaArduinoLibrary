# Configuración del Linker — Fijar `.nairda_vectors` en `0x0100`

## Contexto: Direcciones en AVR

En el ATmega328P, la Flash se direcciona en **palabras de 16 bits** (word-addressed) por la CPU, pero `avr-gcc` / `avr-ld` trabajan en **direcciones de byte**. La dirección byte `0x0100` equivale a la palabra `0x0080`, que cae justo después del vector de interrupciones del ATmega328P (26 vectores × 2 words = 52 words = `0x0068`, con algo de margen).

El flag clave es:

```
-Wl,--section-start=.nairda_vectors=0x0100
```

---

## 1. PlatformIO (`platformio.ini`)

```ini
[env:uno]
platform  = atmelavr
board     = uno
framework = arduino

; Inyectar el flag de linker
build_flags =
    -Wl,--section-start=.nairda_vectors=0x0100
```

Si ya tienes otros `build_flags`, simplemente agrega la línea al bloque existente. PlatformIO pasa los `-Wl,*` directamente a `avr-gcc` en la fase de enlazado.

---

## 2. Arduino IDE (`boards.local.txt`)

Crea (o edita) el archivo `boards.local.txt` en el directorio del paquete de hardware del board. En macOS típicamente:

```
~/Library/Arduino15/packages/arduino/hardware/avr/<version>/boards.local.txt
```

Contenido:

```properties
uno.compiler.ldflags=-Wl,--section-start=.nairda_vectors=0x0100
```

Luego necesitas que `platform.txt` use esa variable. Si la receta de link del core AVR no incluye `{compiler.ldflags}` por defecto, crea también un `platform.local.txt` en el mismo directorio y sobreescribe la receta de link:

```properties
## Sobreescribir la receta de link para inyectar ldflags
recipe.c.combine.pattern="{compiler.path}{compiler.c.elf.cmd}" {compiler.c.elf.flags} -mmcu={build.mcu} {compiler.c.elf.extra_flags} {compiler.ldflags} -o "{build.path}/{build.project_name}.elf" {object_files} "{build.path}/{archive_file}" "-L{build.path}" -lm
```

Reinicia el Arduino IDE después de cualquier cambio a estos archivos.

---

## 3. Makefile (comando directo)

```makefile
MCU       = atmega328p
F_CPU     = 16000000UL
CC        = avr-gcc
OBJCOPY   = avr-objcopy

CFLAGS    = -Os -mmcu=$(MCU) -DF_CPU=$(F_CPU) -std=gnu11
CXXFLAGS  = -Os -mmcu=$(MCU) -DF_CPU=$(F_CPU) -std=gnu++17
LDFLAGS   = -mmcu=$(MCU) -Wl,--section-start=.nairda_vectors=0x0100

# Ejemplo de regla de link
firmware.elf: $(OBJECTS)
	$(CC) $(LDFLAGS) -o $@ $^ -lm

firmware.hex: firmware.elf
	$(OBJCOPY) -O ihex -R .eeprom $< $@
```

---

## Verificación post-build

Después de compilar, confirma que la sección quedó en la dirección correcta:

```bash
avr-objdump -h firmware.elf | grep nairda_vectors
```

Deberías ver algo como:

```
  X .nairda_vectors  000000XX  00000100  00000100  000XXXXX  2**1
```

Donde `00000100` confirma la dirección base `0x0100`.

También puedes verificar el símbolo de la tabla:

```bash
avr-nm -n firmware.elf | grep nairda_jump_table
```

Deberías ver una sola entrada en `00000100` apuntando al array.

---

## Arquitectura: Jump Table como array de punteros a función

La sección `.nairda_vectors` contiene un **único array** (`nairda_jump_table[]`) de punteros a función, colocado en la dirección fija `0x0100`. Cada entrada del array ocupa exactamente **2 bytes** (el tamaño de un puntero en AVR con Flash ≤64KB).

### ¿Por qué las direcciones son estables?

La tabla es solo **datos** (punteros), no código. El tamaño de cada slot es siempre 2 bytes, sin importar cuánto código tenga la función a la que apunta. Por lo tanto, la dirección de cada slot se calcula con una fórmula fija:

```
dirección_del_slot = 0x0100 + (índice × 2)
```

| Slot | Índice | Dirección fija | Función                |
|------|--------|----------------|------------------------|
| 0    | 0      | `0x0100`       | `real_setupDigitalOut` |
| 1    | 1      | `0x0102`       | `real_runDigitalOut`   |
| 2    | 2      | `0x0104`       | `real_setupServo`      |
| 3    | 3      | `0x0106`       | `real_runServo`        |
| 4    | 4      | `0x0108`       | `real_setupMotor`      |
| 5    | 5      | `0x010A`       | `real_runMotor`        |
| 6    | 6      | `0x010C`       | `real_setupNeoPixel`   |
| 7    | 7      | `0x010E`       | `real_runNeoPixel`     |
| 8    | 8      | `0x0110`       | `real_setupFrequency`  |
| 9    | 9      | `0x0112`       | `real_runFrequency`    |
| 10   | 10     | `0x0114`       | `real_setupDigitalIn`  |
| 11   | 11     | `0x0116`       | `real_readDigitalIn`   |
| 12   | 12     | `0x0118`       | `real_setupAnalogic`   |
| 13   | 13     | `0x011A`       | `real_readAnalogic`    |
| 14   | 14     | `0x011C`       | `real_setupUltrasonic` |
| 15   | 15     | `0x011E`       | `real_readUltrasonic`  |
| 16   | 16     | `0x0120`       | `real_nairdaDelay`     |

Si `real_setupMotor` crece de 50 a 500 bytes de código, su puntero en el slot 4 sigue viviendo en `0x0108`. Solo cambia el **valor** almacenado (la dirección a la que apunta), pero la **posición del slot** nunca se mueve.

### Tamaño total de la tabla

```
17 slots × 2 bytes = 34 bytes de Flash
```

### Cómo llama el espacio de usuario

El espacio de usuario solo necesita conocer la dirección base (`0x0100`) y el índice del slot:

```c
#include "components/component.h"

typedef void (*fn_ptr)(void);
#define JUMP_TABLE ((const fn_ptr *)0x0100)

// Ejemplo: llamar a setupMotor (slot 4)
((void (*)(component_t*, int, int, int)) JUMP_TABLE[4])(comp, pin1, pin2, pinSpeed);

// Ejemplo: llamar a readUltrasonic (slot 15)
uint8_t dist = ((uint8_t (*)(component_t*)) JUMP_TABLE[15])(sensor);
```

### Reglas de compatibilidad ABI

1. **NUNCA** reordenar slots existentes.
2. **NUNCA** eliminar un slot — si una función se depreca, dejar el puntero (puede apuntar a un stub vacío).
3. Nuevas funciones se agregan **siempre al final** del array.
4. Los índices del enum `NairdaSlot` en `NairdaKernel_Proxy.cpp` son la fuente de verdad.
