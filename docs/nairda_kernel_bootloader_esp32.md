---
name: Nairda Kernel Architecture - ESP32 (Probado en DOIT DevKit V1)
description: Arquitectura completa del sistema kernel/usuario para ESP32 WROOM. Jump Table en RTC memory, partición userapp, bootloader 150, compilación con xtensa-gcc, y ejecución desde IRAM. TODO FUNCIONA para ESP32.
type: project
---

## Resumen

Adaptación del microkernel Nairda para ESP32 WROOM. El kernel se instala una vez vía arduino-cli. Los programas del usuario se compilan con xtensa-gcc a binario crudo, se envían por serial (o BLE) con el mismo protocolo chunked del AVR, se almacenan en una partición de flash, y se ejecutan desde IRAM.

## Estado: FUNCIONA para ESP32 (DOIT DevKit V1)

Probado end-to-end:
- Blink pin 2 vía Jump Table ✓
- Blink pin 13 vía Jump Table ✓
- Dual blink (pin 2 + pin 13) ✓
- Cambio de delay (100ms, 250ms, 500ms, 1000ms) sin recompilar kernel ✓
- Carga vía serial con protocolo chunked ✓
- Auto-ejecución al boot (ventana de 4s) ✓
- Flag de integridad anti-corrupción ✓
- Jump Table en dirección fija (RTC memory 0x50000000) ✓

## Mapa de memoria Flash ESP32 (virtual 2MB)

```
0x000000 ┌──────────────────────────┐
         │ Bootloader (2nd stage)    │
0x001000 ├──────────────────────────┤
         │ Partition Table           │
0x008000 ├──────────────────────────┤
         │ NVS (20KB)               │
0x009000 ├──────────────────────────┤
         │ OTA data (8KB)           │
0x00E000 ├──────────────────────────┤
         │ Kernel app (~286KB, 14%) │
         │ - Jump Table functions   │
         │ - setupJumpTable()       │
         │ - enterBootloaderMode()  │
         │ - executeUserCode()      │
         │ - Ventana 4s boot        │
0x010000 ├──────────────────────────┤
         │ (espacio libre ~1.87MB)  │
0x1FE000 ├──────────────────────────┤
         │ Partición userapp (8KB)  │
         │ [flag][size_lo][size_hi] │
         │ [entry_offset][code...] │
0x200000 └──────────────────────────┘
```

## Mapa de RAM relevante

```
0x3FFB0000  DRAM (datos, heap)
0x4008F880  IRAM (código usuario, heap_caps_malloc MALLOC_CAP_EXEC)
0x50000000  RTC slow memory (8KB) ← Jump Table (dirección fija)
```

## Jump Table (4 slots, expandible)

Ubicación: **RTC slow memory @ 0x50000000** (dirección fija, equivalente a 0x5F80 en AVR)

El kernel escribe los punteros a funciones aquí al arrancar con `setupJumpTable()`. La app de usuario los lee con macros desde `nairda_user_esp32.h`. Si el kernel se recompila, los punteros se actualizan automáticamente al boot — las apps de usuario existentes siguen funcionando sin recompilar.

| Slot | Dirección | Función |
|------|-----------|---------|
| 0 | 0x50000000 | `jt_init(pin)` → `pinMode(pin, OUTPUT)` |
| 1 | 0x50000004 | `jt_on(pin)` → `digitalWrite(pin, HIGH)` |
| 2 | 0x50000008 | `jt_off(pin)` → `digitalWrite(pin, LOW)` |
| 3 | 0x5000000C | `jt_delay(ms)` → `delay(ms)` |

**Diferencia con AVR:** En AVR los slots son de 2 bytes (word address). En ESP32 son de 4 bytes (32-bit pointer). AVR lee con `pgm_read_word()` desde flash. ESP32 lee directamente desde RAM.

## nairda_user_esp32.h

```c
#define JUMP_TABLE_ADDR 0x50000000
#define JT_READ(slot) (*((void**)(JUMP_TABLE_ADDR + (slot) * 4)))

#define nairda_init(pin)  ((void(*)(int))JT_READ(0))(pin)
#define nairda_on(pin)    ((void(*)(int))JT_READ(1))(pin)
#define nairda_off(pin)   ((void(*)(int))JT_READ(2))(pin)
#define nairda_delay(ms)  ((void(*)(int))JT_READ(3))(ms)
```

## Bootloader Mode (comando 150)

### Protocolo (idéntico al AVR, unidireccional)

```
Host envía 150 → enterBootloaderMode()
Host envía chunks [len][checksum][data × len]
  len: 1-64 bytes
  checksum = sum(data) % 64
  Si checksum falla → chunk descartado (no aborta)
Host envía [0][0] → terminador
  Escribe todo a flash con flag=0x00
  Re-escribe con flag=0x01 (válido)
  Copia a IRAM → ejecuta
```

### Header en flash (4 bytes)

```
byte[0] = flag (0x01=válido, 0x00=corrupto/en progreso, 0xFF=vacío)
byte[1] = size_lo (tamaño total almacenado, low byte)
byte[2] = size_hi (tamaño total almacenado, high byte)
byte[3] = entry_offset (offset de _start dentro del binario)
byte[4+] = código máquina Xtensa
```

**¿Por qué entry_offset?** El compilador Xtensa genera una "literal pool" (constantes) antes de `_start`. El entry point no siempre está al inicio del binario. El kernel necesita saber a qué offset saltar.

### Flag de integridad

- `0x01` = válido → ejecutar al boot
- `0xFF` = vacío → no hay programa
- `0x00` = escritura en progreso → no ejecutar (protección anti-corrupción)

Si la transmisión se interrumpe, el flag nunca se actualiza a 0x01 porque eso solo ocurre después del terminador [0][0]. Al reiniciar, el kernel ve que no es válido y no ejecuta código corrupto.

**Nota flash NOR:** No se puede cambiar un bit de 0→1 sin borrar el sector completo. Por eso el kernel hace: escribir con flag=0x00, luego borrar todo el sector, y reescribir con flag=0x01.

### Ventana de 4 segundos (setup)

Después de inicializar, el kernel espera 4 segundos escuchando serial:
- Si recibe 150 → entra en bootloader
- Si no recibe nada → verifica flag en flash
  - Si flag=0x01 → ejecuta programa guardado
  - Si no → espera en loop

Conectar el USB reinicia el ESP32 (DTR/RTS), lo que abre la ventana para enviar el 150.

## Ejecución del código de usuario

### Flujo: Flash → DRAM → IRAM → Execute

```
1. esp_partition_read(userPartition, 4, tmpBuf, codeLen)  // Flash → DRAM
2. Copiar con escrituras de 32 bits alineadas               // DRAM → IRAM
3. entry = (entry_fn)(execMem + entryOffset)               // Saltar a _start
4. entry()                                                  // Ejecutar
```

**¿Por qué no ejecutar directo desde flash?** ESP32 requiere que el código pase por el MMU/cache para ser ejecutable. `spi_flash_mmap` no garantiza una dirección fija. La solución es copiar a IRAM (instruction RAM) con `heap_caps_malloc(MALLOC_CAP_EXEC)`.

**¿Por qué el paso intermedio DRAM?** IRAM no soporta acceso byte-a-byte. `esp_partition_read` internamente usa `memcpy` que puede hacer accesos de 1 byte, lo que causa `LoadStoreError` en IRAM. Se lee primero a DRAM y luego se copia a IRAM con escrituras de 32 bits alineadas.

## Compilación del kernel

```bash
arduino-cli compile --fqbn esp32:esp32:esp32doit-devkit-v1 \
  --build-property "build.partitions=partitions" \
  --build-property "upload.maximum_size=2027520" \
  esp32_bootloader_test.ino

arduino-cli upload --fqbn esp32:esp32:esp32doit-devkit-v1:UploadSpeed=115200 \
  --port /dev/cu.usbserial-XXXX \
  esp32_bootloader_test.ino
```

### partitions.csv

```csv
# Name,   Type, SubType, Offset,   Size,     Flags
nvs,      data, nvs,     0x9000,   0x5000,
otadata,  data, ota,     0xe000,   0x2000,
app0,     app,  ota_0,   0x10000,  0x1EE000,
userapp,  data, 0x40,    0x1FE000, 0x2000,
```

## Compilación de la app de usuario

### compile_user.sh

```bash
TOOLCHAIN=~/Library/Arduino15/packages/esp32/tools/esp-x32/2511/bin

$TOOLCHAIN/xtensa-esp32-elf-gcc \
  -Os \
  -nostartfiles \
  -nostdlib \
  -fno-builtin \
  -mtext-section-literals \
  -I"$SCRIPT_DIR" \
  -Wl,--entry=_start \
  -o user_app.elf \
  user_app.c

$TOOLCHAIN/xtensa-esp32-elf-objcopy -O binary user_app.elf user_app.bin
```

### Ejemplo de app de usuario

```c
#include "nairda_user_esp32.h"

void _start() {
  nairda_init(13);
  while (1) {
    nairda_on(13);
    nairda_delay(1000);
    nairda_off(13);
    nairda_delay(1000);
  }
}
```

Resultado: ~76 bytes de código máquina Xtensa.

## Upload (upload_test.py)

```bash
python3 upload_test.py /dev/cu.usbserial-XXXX user_app.bin 9600
```

El script:
1. Abre el puerto serial (resetea el ESP32 vía DTR/RTS)
2. Lee el entry_offset del ELF
3. Prepende `[0x00, entry_offset]` al binario
4. Envía 150 dentro de la ventana de 4s
5. Envía chunks de máx 64 bytes con checksum % 64
6. Envía terminador [0][0]

## Comparación AVR vs ESP32

| Aspecto | AVR (ATmega328P) | ESP32 (WROOM) |
|---------|-----------------|---------------|
| Flash total | 32KB | 2MB (virtual) |
| User app máx | 8190 bytes | 8192 bytes |
| Jump Table ubicación | Flash @ 0x5F80 | RTC RAM @ 0x50000000 |
| Jump Table slots | 31 × 2 bytes | 4 × 4 bytes (expandible) |
| Jump Table lectura | `pgm_read_word()` | lectura directa RAM |
| Ejecución | Directo desde flash (`jmp`) | Flash → DRAM → IRAM → call |
| Entry point | Dirección fija 0x6002 | Offset variable (literal pool) |
| Header | [flag][padding] (2 bytes) | [flag][size_lo][size_hi][entry_offset] (4 bytes) |
| Protocolo upload | [len][checksum][data×64] | Idéntico |
| Checksum | sum(data) % 64 | Idéntico |
| Terminador | [0][0] | Idéntico |
| Ventana boot | 2 segundos | 4 segundos |
| Compilador | avr-gcc | xtensa-esp32-elf-gcc |
| Flash write | do_spm (Optiboot) | esp_partition_write |
| ABI | AVR (call/ret) | Xtensa windowed (entry/retw/callx8) |
| Baudrate | 115200 (default) | 9600 |

## Archivos clave

| Archivo | Función |
|---------|---------|
| `esp32_bootloader_test.ino` | Kernel: JT, bootloader, ejecución |
| `nairda_user_esp32.h` | Header para apps de usuario (macros JT) |
| `user_app.c` | App de usuario ejemplo (blinker) |
| `compile_user.sh` | Script de compilación user app |
| `upload_test.py` | Script de upload por serial |
| `partitions.csv` | Tabla de particiones custom |

## Problemas resueltos durante desarrollo

1. **LoadStoreError al copiar a IRAM**: `esp_partition_read` directo a IRAM falla porque IRAM no soporta accesos byte-a-byte. Solución: leer a DRAM primero, luego copiar con escrituras de 32 bits.

2. **Crash al llamar funciones del kernel**: El código de usuario compilado standalone usa windowed ABI (entry/callx8). La llamada al entry point desde el kernel funciona correctamente porque ambos usan windowed ABI y el compilador pasa el argumento en A10 que se convierte en A2 tras `entry`.

3. **Detección de tamaño por trailing 0xFF**: El código máquina puede terminar en 0xFF legítimamente (ej. `movi a10, 250` = `FA A0 A2` seguido de instrucciones que contienen 0xFF). Solución: guardar el tamaño explícitamente en el header.

4. **Entry point no al inicio del binario**: Xtensa genera una literal pool (constantes como 0x50000000) antes de `_start`. El entry point tiene un offset variable. Solución: guardar el entry_offset en el header y el kernel salta a `execMem + entryOffset`.

5. **Flash NOR no permite 0→1**: Para actualizar el flag de 0x00 a 0x01 hay que borrar todo el sector y reescribir. El kernel guarda los datos en RAM, borra, y reescribe con el flag actualizado.

6. **Programa anterior sobrevive al re-flashear kernel**: `esptool` solo borra los sectores del app (0x10000-0x55FFF), no la partición userapp (0x1FE000). Un programa corrupto puede causar crash loop. Solución: la ventana de 4 segundos permite enviar el 150 antes de que se ejecute el programa guardado.
