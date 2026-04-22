# BootJacker Loader — Documentacion Tecnica

## Resumen

BootJacker Loader es un mini-kernel para ATmega328P que permite escribir programas de usuario en la memoria Flash **sin programador ISP y sin Optiboot 8**, usando unicamente los gadgets ROP (Return-Oriented Programming) encontrados en el bootloader Optiboot 4.4 de fabrica que viene pre-instalado en todo Arduino Uno.

El sistema consta de tres componentes:

| Archivo | Funcion |
|---|---|
| `bootjacker_loader.ino` | Mini-kernel: UART bare-metal, syscalls, gadgets ROP, protocolo de comandos |
| `upload.py` | Script Python que orquesta la carga de programas via serial |
| `user_syscalls.h` | Header C para programas de usuario — macros que leen la Jump Table |

---

## Arquitectura de Memoria Flash (ATmega328P, 32KB)

```
0x0000 ┌─────────────────────────────┐
       │ bootjacker_loader           │
       │ (mini-kernel, ~1.2KB)       │
       │ - UART bare-metal           │
       │ - syscalls (pin13, led, delay)│
       │ - gadgets ROP               │
       │ - protocolo F/L/E/W/J/R/?   │
0x04B5 ├─────────────────────────────┤
       │ (espacio libre)             │
0x5F80 ├─────────────────────────────┤
       │ Jump Table (8 bytes usados) │
       │ Slot 0: pin13_setup (word)  │
       │ Slot 1: led_on (word)       │
       │ Slot 2: led_off (word)      │
       │ Slot 3: delay_ms (word)     │
       │ (resto: 0xFF padding)       │
0x5FFF ├─────────────────────────────┤
       │ User Space (8KB max)        │
0x6000 │ [flag] [pad] [codigo...]    │
       │  0x01 = programa valido     │
       │  codigo empieza en 0x6002   │
0x7DFF ├─────────────────────────────┤
       │ Optiboot 4.4 (512 bytes)    │
0x7E00 │ Bootloader de fabrica       │
       │ Contiene gadgets SPM        │
0x7FFF └─────────────────────────────┘
```

### RAM (SRAM, 2KB)

```
0x0100 ┌─────────────────────────────┐
       │ .data (variables init)      │
       │ OPTIBOOT_BUF: zona que el   │
       │ gadget lee para escribir    │
       │ Flash (128 bytes)           │
0x017F ├─────────────────────────────┤
       │ .bss (variables zero-init)  │
       ├─────────────────────────────┤
       │ .noinit                     │
       │ page_buf[128] — buffer de   │
       │ pagina (sobrevive WDT reset)│
       ├─────────────────────────────┤
       │ (espacio libre)             │
       ├─────────────────────────────┤
       │ Stack (crece hacia abajo)   │
0x08FF └─────────────────────────────┘
```

**Consumo total: 1206 bytes ROM (3%), 154 bytes RAM (7%)**

---

## El Problema: SPM desde Application Code

En ATmega328P, la instruccion `spm` (Store Program Memory) solo puede ejecutarse desde la **Boot Loader Section** (BLS, 0x7E00–0x7FFF). El codigo de aplicacion en la seccion RWW (0x0000–0x7DFF) no tiene permiso de ejecutar `spm` directamente.

Las soluciones convencionales son:
- **Optiboot 8+**: expone una funcion `do_spm` callable desde la aplicacion. Requiere reflashear el bootloader.
- **ISP Programmer**: escribe Flash desde un programador externo.

BootJacker no requiere ninguna de las dos. Usa **gadgets ROP** en el Optiboot 4.4 de fabrica.

---

## Gadgets ROP en Optiboot 4.4

Un "gadget" es una secuencia de instrucciones dentro del bootloader que podemos reusar saltando directamente a ella. Como estas instrucciones residen en la BLS, tienen permiso de ejecutar `spm`.

### Gadget de Erase+RWWSRE (byte 0x7F32, word 0x3F99)

Originalmente es la seccion de page-write del handler `STK_PROG_PAGE`. Lo reusamos para erase cargando `r10 = 0x03` (PGERS|SPMEN) en vez de `0x05` (PGWRT|SPMEN):

```asm
7F32: movw  Z, r12:r13       ; Z = direccion Flash objetivo
7F34: out   SPMCSR, r10      ; comando SPM (0x03 = erase)
7F36: spm                    ; ejecuta borrado de pagina
7F38: in    r0, SPMCSR       ;
7F3A: sbrc  r0, 0            ; busy-wait hasta completar
7F3C: rjmp  .-6              ;
7F3E: out   SPMCSR, r9       ; r9 = 0x11 = RWWSRE|SPMEN
7F40: spm                    ; re-habilita seccion RWW
7F42: rjmp  0x7F90           ; cae en loop del bootloader
```

**Registros requeridos:**
- `r12:r13` = direccion byte de la pagina Flash
- `r10` = 0x03 (PGERS | SPMEN)
- `r9` = 0x11 (RWWSRE | SPMEN)

### Gadget de Fill+Write+RWWSRE (byte 0x7F00, word 0x3F80)

Lee 128 bytes desde SRAM 0x0100, llena el buffer temporal de pagina word por word, escribe la pagina, y re-habilita RWW:

```asm
7F00: movw  r20, r12         ; direccion base
7F02: ldi   X, 0x0100        ; X = puntero a SRAM buffer
      ; --- loop de 64 iteraciones ---
7F06: ld    r18, X           ; byte bajo del word
      ld    r24, X+1         ; byte alto
      movw  Z, r20           ; direccion Flash actual
      movw  r0, r24:r18      ; word a escribir
7F20: out   SPMCSR, r8       ; r8 = 0x01 (SPMEN, fill)
7F22: spm                    ; llena buffer temporal
      r20 += 2, X += 2       ; avanza
      ; --- fin loop ---
7F32: movw  Z, r12           ; direccion base
7F34: out   SPMCSR, r10      ; r10 = 0x05 (PGWRT|SPMEN)
7F36: spm                    ; escribe pagina completa
7F38-7F3C: busy-wait
7F3E: out   SPMCSR, r9       ; RWWSRE
7F40: spm
7F42: rjmp  bootloader_loop
```

**Registros requeridos:**
- `r12:r13` = direccion byte base de la pagina
- `r8` = 0x01 (SPMEN, para fill)
- `r10` = 0x05 (PGWRT | SPMEN, para write)
- `r9` = 0x11 (RWWSRE | SPMEN)
- SRAM 0x0100–0x017F = 128 bytes de datos a escribir

---

## Mecanismo de Recuperacion: WDT Reset

### El Problema de las Interrupciones

Despues de ejecutar el gadget SPM + RWWSRE, el codigo cae en el loop principal de Optiboot (`getchar` bloqueante). La solucion obvia seria usar un Timer (como Timer2) para interrumpir y recuperar el control via ISR.

**Esto no funciona.** Los lock bits BLB0 del chip (configurados de fabrica) deshabilitan la entrega de interrupciones mientras el CPU ejecuta desde la Boot Loader Section. Verificado empiricamente:
- Timer2 hardware cuenta correctamente (flag OCF2A se activa)
- Pero el ISR nunca se dispara durante ejecucion en BLS

### La Solucion: Watchdog Timer Reset

El WDT es un mecanismo de reset por hardware que no depende de interrupciones. Funciona asi:

1. Habilitamos WDT con timeout de 15ms (`wdt_enable(WDTO_15MS)`)
2. Saltamos al gadget via `ijmp` (no `call` — no necesitamos retorno)
3. El gadget ejecuta SPM (~4.5ms) + RWWSRE (~0.1ms)
4. Cae en el loop de Optiboot (bloqueado en `getchar`)
5. A los ~15ms el WDT resetea el chip completo
6. Optiboot detecta `WDRF` en MCUSR (sin `EXTRF`) → salta a la app inmediatamente (<1ms, sin LED blink, sin timeout)
7. El kernel re-arranca, listo para el siguiente comando

**Tiempo total por operacion Flash: ~15ms (WDT) + ~1ms (boot) = ~16ms**

### Flujo de Escritura de una Pagina

```
                Script                          Arduino
                  │                                │
  1. Fill buffer  │── 'F' ────────────────────────►│ page_buf = 0xFF
                  │── 'L' 0 64 [data] ───────────►│ page_buf[0..63] = data
                  │── 'L' 64 64 [data] ──────────►│ page_buf[64..127] = data
                  │                                │
  2. Erase page   │── 'E' lo hi ─────────────────►│ cli(); wdt_enable(15ms);
                  │                                │ ijmp → gadget erase
                  │                                │ ... SPM erase ~4.5ms ...
                  │                                │ ... RWWSRE ...
                  │                                │ ... getchar blocks ...
                  │         (400ms delay)          │ ← WDT RESET (~15ms)
                  │                                │ Optiboot → jump to app
                  │◄── "BJ:READY" ─────────────────│ kernel re-arranca
                  │                                │
  3. Write page   │── 'W' lo hi ─────────────────►│ cli();
                  │                                │ copy page_buf → SRAM 0x0100
                  │                                │ wdt_enable(15ms);
                  │                                │ ijmp → gadget fill+write
                  │                                │ ... fill 64 words ~40µs ...
                  │                                │ ... SPM write ~4.5ms ...
                  │                                │ ... RWWSRE ...
                  │         (400ms delay)          │ ← WDT RESET
                  │◄── "BJ:READY" ─────────────────│
                  │                                │
  4. (repeat for next page)                        │
                  │                                │
  5. Execute      │── 'R' ───────────────────────►│ flag == 0x01?
                  │◄── "R:GO" ─────────────────────│ ijmp → 0x6002
```

---

## Implementacion del Kernel

### UART Bare-Metal

El kernel NO usa `Serial` de Arduino (que consume ~170 bytes RAM en buffers TX/RX). En su lugar implementa UART polling directo:

```c
static void uart_tx(uint8_t c) {
    while (!(UCSR0A & (1 << UDRE0)));  // espera buffer vacio
    UDR0 = c;                           // envia byte
}

static uint8_t uart_rx(void) {
    while (!(UCSR0A & (1 << RXC0)));   // espera byte recibido
    return UDR0;                        // lee byte
}
```

Configuracion: 115200 baud, 8N1, modo U2X (double speed).

### Syscalls y Jump Table

El kernel expone 4 funciones via una Jump Table en Flash 0x5F80:

| Slot | Funcion | Firma |
|---|---|---|
| 0 | `pin13_setup` | `void(void)` — `DDRB \|= (1<<5)` |
| 1 | `led_on` | `void(void)` — `PORTB \|= (1<<5)` |
| 2 | `led_off` | `void(void)` — `PORTB &= ~(1<<5)` |
| 3 | `delay_ms` | `void(unsigned long)` — busy-wait calibrado |

La JT se escribe a Flash via el mismo mecanismo E/W, orquestado por el script:

```
Script: 'J'                → kernel llena page_buf con word addresses
Script: 'E' 0x80 0x5F      → erase pagina 0x5F80
Script: 'J'                → re-llena page_buf (se perdio en WDT)
Script: 'W' 0x80 0x5F      → escribe JT a Flash
```

El programa de usuario lee la JT con `pgm_read_word`:

```c
// user_syscalls.h
#define JT_ADDR 0x5F80
#define JT_READ(slot) ((fn_ptr)pgm_read_word(JT_ADDR + (slot) * 2))
#define led_on()     (JT_READ(1))()
#define delay_ms(ms) ((void(*)(unsigned long))JT_READ(3))((ms))
```

### Invocacion de Gadgets

Los gadgets se invocan via inline assembly con `ijmp` (indirect jump al registro Z):

```c
static void bj_erase(uint16_t addr) __attribute__((noreturn));
static void bj_erase(uint16_t addr) {
    cli();                              // deshabilitar interrupciones
    wdt_enable(WDTO_15MS);             // armar watchdog
    asm volatile(
        "mov r12, %A[a] \n\t"          // r12:r13 = direccion Flash
        "mov r13, %B[a] \n\t"
        "ldi r30, 0x03  \n\t"          // PGERS | SPMEN
        "mov r10, r30   \n\t"
        "ldi r30, 0x11  \n\t"          // RWWSRE | SPMEN
        "mov r9,  r30   \n\t"
        "ldi r30, lo8(%[g]) \n\t"      // Z = word address del gadget
        "ldi r31, hi8(%[g]) \n\t"
        "ijmp \n\t"                     // salto al bootloader (no retorna)
        : : [a] "r" (addr), [g] "n" (GADGET_ERASE_W)
    );
    __builtin_unreachable();
}
```

**Nota sobre `ijmp` vs `icall`:** Se usa `ijmp` (no `icall`) porque no necesitamos return address — el WDT reseteara el chip. `icall` pushea un return address innecesario al stack.

### Buffer de Pagina en .noinit

```c
static volatile uint8_t page_buf[SPM_PAGESIZE]
    __attribute__((section(".noinit")));
```

- `volatile`: previene que LTO/GCC optimice escrituras como "dead stores" antes de llamadas `noreturn`
- `.noinit`: la seccion NO es inicializada por el startup code de AVR-GCC, asi que sobrevive WDT resets
- **Limitacion descubierta**: `memcpy` y casts a `(void*)` pierden el qualifier `volatile`, permitiendo a GCC eliminar stores. Por eso `bj_fill_write` usa un loop manual volatile para copiar a SRAM 0x0100

---

## Script de Carga (upload.py)

### Protocolo

El protocolo es **delay-based, sin ACK** (los ACK via serial causan problemas de sincronizacion). El script espera un tiempo fijo despues de cada WDT reset:

```python
WDT_DELAY = 0.4  # 400ms (el WDT real toma ~16ms, margen generoso)
```

### Flujo Completo

```python
1. Esperar "BJ:READY"
2. Escribir Jump Table:
   a. 'J'           → llenar page_buf con JT
   b. 'E' 0x80 0x5F → erase JT page → WDT → esperar READY
   c. 'J'           → re-llenar page_buf (perdido en WDT)
   d. 'W' 0x80 0x5F → write JT page → WDT → esperar READY
3. Por cada pagina del programa:
   a. 'F'           → page_buf = 0xFF
   b. 'L' off len data  → cargar chunks (max 64 bytes)
   c. 'E' lo hi     → erase → WDT → esperar READY
   d. 'W' lo hi     → write → WDT → esperar READY
4. 'R'              → ejecutar programa de usuario
```

### Header del Programa

El script antepone 2 bytes al binario:

```
[0x01] [0xFF] [codigo del programa...]
  │      │
  │      └─ padding
  └──────── flag de integridad (0x01 = valido)
```

El kernel verifica `pgm_read_byte(0x6000) == 0x01` antes de saltar a 0x6002.

---

## Compilacion de Programas de Usuario

```bash
avr-gcc -mmcu=atmega328p -Os -nostartfiles -fno-toplevel-reorder \
    -I<path-to-user_syscalls.h> \
    -Wl,--section-start=.text=0x6002 \
    -o program.elf program.c

avr-objcopy -O binary -j .text program.elf program.bin
```

Flags criticos:
- `-nostartfiles`: sin CRT startup (el kernel ya inicializo el hardware)
- `-fno-toplevel-reorder`: garantiza que `_start` quede en 0x6002 (primera funcion en el fuente)
- `--section-start=.text=0x6002`: codigo despues del header [flag, pad]

### Ejemplo: Blink con Syscalls

```c
#include "user_syscalls.h"

void __attribute__((section(".text"))) _start(void) {
    pin13_setup();
    while (1) {
        led_on();
        delay_ms(500);
        led_off();
        delay_ms(500);
    }
}
```

---

## Direcciones de los Gadgets

Verificadas via `avr-objdump` del `optiboot_atmega328.hex` (Arduino core 1.8.6):

| Gadget | Byte addr | Word addr | Funcion |
|---|---|---|---|
| Erase+RWWSRE | 0x7F32 | 0x3F99 | Page erase → busy-wait → RWWSRE → bootloop |
| Fill+Write+RWWSRE | 0x7F00 | 0x3F80 | Fill 64 words desde SRAM 0x0100 → page write → RWWSRE → bootloop |

Estos gadgets son especificos de Optiboot 4.4 tal como se distribuye con Arduino IDE (core `arduino:avr` version 1.8.6). Otros bootloaders o versiones tendran direcciones diferentes.

---

## Limitaciones Conocidas

1. **2 WDT resets por pagina**: cada pagina requiere erase (1 reset) + write (1 reset) = ~32ms + overhead de boot. Para 64 paginas (8KB): ~4 segundos de overhead.

2. **Lock bits BLB0**: los lock bits de fabrica deshabilitan interrupciones en la BLS. Esto impide usar Timer2 como mecanismo de rescate (el enfoque del walkthrough original). Si se corrigen los lock bits via ISP programmer, se podria usar Timer2 y eliminar los WDT resets.

3. **bj_fill_write solo funciona desde handlers UART**: por un bug no resuelto (posiblemente relacionado con el estado del hardware UART o timing post-boot), `bj_fill_write` no funciona cuando se llama desde codigo auto-disparado (setup, timers, primer iteracion del loop). Solo funciona de forma confiable cuando es invocado como respuesta a un comando serial recibido.

4. **Gadgets dependientes de Optiboot 4.4**: las direcciones 0x7F32 y 0x7F00 son especificas de esta version. Si el bootloader cambia, los gadgets deben re-verificarse con `avr-objdump`.

5. **Sin proteccion contra escritura parcial**: si el proceso se interrumpe entre erase y write, la pagina queda borrada (0xFF). El flag 0x01 se escribe como parte de los datos, no como paso separado de finalizacion.
