---
name: Nairda Kernel Architecture - AVR (Funcionando en producción)
description: Arquitectura completa del sistema kernel/usuario para ATmega328P. Jump Table, do_spm, bootloader 150, compilador cloud, transpiler, y flujo Flutter. TODO FUNCIONA para AVR.
type: project
---

## Resumen

Nairda es un microkernel para microcontroladores. El firmware (kernel) se instala una vez (~18KB). Los programas del usuario se compilan en <1s en la nube y se cargan en <1s vía BLE/Serial sin tocar el kernel.

## Estado: FUNCIONA para AVR (Arduino Uno/Nano)

Probado end-to-end:
- Blink pin 13 vía Jump Table ✓
- Blink pin 11 vía Jump Table ✓
- Lectura analógica A0 → PWM pin 12 ✓
- Carga desde Flutter web ✓
- Carga desde upload.sh (bash) ✓
- Cloud compiler (Cloud Run) ✓

## Mapa de memoria Flash ATmega328P (32KB)

```
0x0000 ┌──────────────────────────┐
       │ Vector de interrupciones  │
       ├──────────────────────────┤
       │ Kernel (~18KB, 58%)       │
       │ - nairdaBegin/nairdaLoop  │
       │ - componentes (servo,     │
       │   motor, neopixel, etc.)  │
       │ - BLE/Serial              │
       │ - jt_* wrappers (31)      │
       │ - enterBootloaderMode     │
       │ - do_spm flash writer     │
0x5F80 ├──────────────────────────┤
       │ Jump Table (62 bytes)     │
       │ 31 slots × 2 bytes       │
0x6000 ├──────────────────────────┤
       │ Flag (1 byte) + Pad (1b) │
0x6002 │ Programa del usuario      │ Máx 8190 bytes
0x7DFF ├──────────────────────────┤
       │ Optiboot v8+ bootloader   │
0x7FFF └──────────────────────────┘
```

## Mapa de RAM (2KB)

```
0x0100  Kernel globals (767 bytes)
0x0600  User globals (.data del programa usuario)
0x08FF  Stack (crece hacia abajo)
```

Linker flag para user .data: `-Wl,--section-start=.data=0x800600`

## Jump Table (31 slots)

Archivo: `NairdaKernel_JumpTable.S` (assembly AVR, sobrevive LTO)
Sección: `.nairda_vectors` fijada en `0x5F80`

| Slots | Funciones |
|-------|-----------|
| 0-1 | setupDigitalOut, runDigitalOut |
| 2-3 | setupServo, runServo |
| 4-5 | setupMotor, runMotor |
| 6-7 | setupNeoPixel, runNeoPixel |
| 8-9 | setupFrequency, runFrequency |
| 10-11 | setupDigitalIn, readDigitalIn |
| 12-13 | setupAnalogic, readAnalogic |
| 14-15 | setupUltrasonic, readUltrasonic |
| 16-18 | nairdaDelay, nairdaRandom, nairdaMap |
| 19-21 | stringCompare, stringToLower, stringToUpper |
| 22-30 | tableCreate/Set/Get/Height/Width/AddRow/AddCol/RemRow/RemCol |

## Wrappers (NairdaKernel_Proxy.cpp)

Convierten `uint8_t[16]` del usuario → `component_t*` del kernel.
El usuario nunca ve `component_t`. Solo declara `uint8_t led[NAIRDA_COMP_SIZE] = {0}`.

## Bootloader Mode (comando 150)

### Protocolo (unidireccional, sin ACKs)

```
Flutter envía 150 → enterBootloaderMode()
Flutter envía [len][checksum][datos×64] → recibe chunk, valida, acumula
  checksum = sum(datos) % 64
  2 chunks = 128 bytes = 1 página Flash → flashWritePage()
Flutter envía [0][0] → terminador
  escribe primera página con flag=0x01
  jmp 0 → reinicia → timeout 2s → ejecuta programa
```

### Flag de integridad (0x6000)
- `0x01` = válido → ejecutar
- `0xFF` = vacío → nairdaLoop normal
- `0x00` = escritura en progreso → no ejecutar (protección anti-corrupción)

### Ventana de 2 segundos (nairdaBegin)
Después de inicializar, espera 2s. Si recibe 101→intérprete, 150→bootloader, timeout→verifica flag.

## Compilador Cloud

**URL**: `https://nairda-user-compiler-mpjkn2bhoq-uc.a.run.app/compile`
**Endpoint**: `POST /compile?target=avr` (o `?target=esp32`)
**Input**: código C (text/plain)
**Output**: binario crudo (application/octet-stream)

### compile.sh (AVR)
```bash
avr-gcc -mmcu=atmega328p -Os -nostartfiles \
    -Wl,--section-start=.text=0x6002 \
    -Wl,--section-start=.data=0x800600 \
    -o app.elf app.c
avr-objcopy -O binary -j .text app.elf app.bin
```

### nairda_user.h
```c
#define JUMP_TABLE_ADDR 0x5F80
#define JT_READ(slot) ((fn_ptr)pgm_read_word(JUMP_TABLE_ADDR + (slot) * 2))
#define NAIRDA_COMP_SIZE 16
#define nairda_setupDigitalOut(comp, pin) \
    ((void(*)(void*, int))JT_READ(0))((comp), (pin))
// ... 31 macros total
```

## Transpiler (nairda_ast_ai)

`NairdaToUserLayerTranspiler` convierte JSON de bloques → código C:
- Acepta `target: 'avr'` o `target: 'esp32'`
- Include correcto según target
- Variables como globals (`uint8_t led[NAIRDA_COMP_SIZE] = {0}`)
- `while(1)` solo si el usuario puso bloque WHILE
- Idle loop al final: `while(1) { nairda_delay(100); }`

## Flutter (firmware_upload_bloc.dart)

### Flujo
1. Transpile (bloques → C)
2. Compile (C → binario vía Cloud Run con `?target=`)
3. Prepend 2 bytes padding: `[0x00, 0xFF] + binario`
4. Enviar 100 (reset, solo AVR) + 150 (bootloader)
5. Enviar chunks byte a byte
6. Enviar [0][0] terminador
7. Desconectar

### Diferencia AVR vs ESP32 en Flutter
- AVR: envía 100 antes del 150 (jmp 0 reset)
- ESP32: NO envía 100 (clearVolatileMemory, mantiene BLE)

## Compilación del kernel
```bash
arduino-cli compile --fqbn arduino:avr:uno \
  --build-property "compiler.c.elf.extra_flags=-Wl,--section-start=.nairda_vectors=0x5F80 -Wl,--undefined=nairda_jump_table" \
  nairda_firmware.ino
```

## Upload script (upload.sh) - desarrollo
```bash
./upload.sh <puerto> <binario.bin> [baudrate]
# Default 115200 (AVR), usar 9600 para ESP32
```

## Archivos clave

| Archivo | Función |
|---------|---------|
| `NairdaKernel_JumpTable.S` | Jump Table assembly (solo AVR) |
| `NairdaKernel_Proxy.cpp` | Wrappers jt_* (arrays→component_t) |
| `flash_writer/flash_writer.h/.cpp` | do_spm + defines de direcciones |
| `nairda_debug/nairda_debug.cpp` | enterBootloaderMode + nairdaDebug |
| `nairda.cpp` | nairdaBegin con ventana 2s + ejecución offline |
| `virtual_machine/virtual_machine.h` | #define userBootloader 150 |
| `backend/user-app-compiler/` | Docker + Flask compiler |
| `nairda_ast_ai/.../nairda_to_user_layer_transpiler.dart` | Transpiler |
| `firmware_upload_bloc.dart` | BLoC Flutter upload |

## Nota sobre herramientas de test
- pyserial NO funciona con CH340 en macOS (envía 0x00)
- `stty` + shell (upload.sh) funciona perfecto para AVR
- `arduino-cli monitor` funciona
- Flutter web/iOS funciona vía BLE

## ESP32: Estado pendiente
La arquitectura está diseñada pero la carga vía Web Bluetooth falla con error GATT al enviar el comando bootloader. El modo intérprete en vivo funciona sin problemas. El kernel ESP32 compila y arranca correctamente. Ver `project_esp32_bootloader_debug.md` para detalles del bug.
