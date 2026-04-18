# Nairda Kernel Architecture — ATmega328P

## Mapa de Memoria Flash (32KB)

```
┌──────────────────────────────────┐ 0x0000
│  Vector de interrupciones (IVT)   │
│  26 vectores × 4 bytes = 104 bytes│
├──────────────────────────────────┤ 0x0100
│  Jump Table (34 bytes)            │
│  nairda_jump_table[17]            │
│  Cada slot = 2 bytes (puntero)    │
│  Dirección de slot = 0x0100 + N×2 │
├──────────────────────────────────┤ 0x0122
│                                   │
│  Kernel / Core (~24KB)            │
│  ├─ nairdaBegin()                 │
│  ├─ nairdaLoop()                  │
│  ├─ Implementaciones real_*       │
│  ├─ VM / intérprete               │
│  ├─ Módulo BLE (recepción)        │
│  ├─ Flash Writer (do_spm)         │
│  └─ Lógica de boot / timeout      │
│                                   │
├──────────────────────────────────┤ 0x6000 (24KB)
│  Espacio de Usuario (8KB)         │
│  ┌────────────────────────────┐   │
│  │ 0x6000: Flag de integridad │   │
│  │  0x01 = programa válido    │   │
│  │  cualquier otro = inválido │   │
│  ├────────────────────────────┤   │
│  │ 0x6001: Programa del       │   │
│  │  usuario (hasta 8191 bytes)│   │
│  │  Usa kernel vía Jump Table │   │
│  └────────────────────────────┘   │
├──────────────────────────────────┤ 0x7E00
│  Optiboot v8+ (512 bytes)        │
│  Bootloader de fábrica            │
│  Expone do_spm() para escritura   │
│  NO SE MODIFICA NUNCA             │
└──────────────────────────────────┘ 0x7FFF
```

---

## Jump Table — Contrato ABI

La Jump Table es un array único de punteros a función ubicado en la dirección fija `0x0100`. Cada entrada ocupa exactamente 2 bytes (tamaño de un puntero en AVR con Flash ≤64KB). El tamaño de las implementaciones reales no afecta la posición de los slots.

### Fórmula de direcciones

```
dirección_del_slot = 0x0100 + (índice × 2)
```

### Tabla de slots

| Slot | Índice | Dirección | Función                | Firma                                                  |
|------|--------|-----------|------------------------|--------------------------------------------------------|
| 0    | 0      | `0x0100`  | `real_setupDigitalOut` | `void (component_t*, int)`                             |
| 1    | 1      | `0x0102`  | `real_runDigitalOut`   | `void (component_t*, int)`                             |
| 2    | 2      | `0x0104`  | `real_setupServo`      | `void (component_t*, int, int, int, int)`              |
| 3    | 3      | `0x0106`  | `real_runServo`        | `void (component_t*, int)`                             |
| 4    | 4      | `0x0108`  | `real_setupMotor`      | `void (component_t*, int, int, int)`                   |
| 5    | 5      | `0x010A`  | `real_runMotor`        | `void (component_t*, int, int)`                        |
| 6    | 6      | `0x010C`  | `real_setupNeoPixel`   | `void (component_t*, int, int)`                        |
| 7    | 7      | `0x010E`  | `real_runNeoPixel`     | `void (component_t*, int, int, int, int)`              |
| 8    | 8      | `0x0110`  | `real_setupFrequency`  | `void (component_t*, int)`                             |
| 9    | 9      | `0x0112`  | `real_runFrequency`    | `void (component_t*, int, int, int)`                   |
| 10   | 10     | `0x0114`  | `real_setupDigitalIn`  | `void (component_t*, int)`                             |
| 11   | 11     | `0x0116`  | `real_readDigitalIn`   | `uint8_t (component_t*)`                               |
| 12   | 12     | `0x0118`  | `real_setupAnalogic`   | `void (component_t*, int)`                             |
| 13   | 13     | `0x011A`  | `real_readAnalogic`    | `uint8_t (component_t*)`                               |
| 14   | 14     | `0x011C`  | `real_setupUltrasonic` | `void (component_t*, int, int)`                        |
| 15   | 15     | `0x011E`  | `real_readUltrasonic`  | `uint8_t (component_t*)`                               |
| 16   | 16     | `0x0120`  | `real_nairdaDelay`     | `void (unsigned long)`                                 |

### Reglas de compatibilidad ABI

1. **NUNCA** reordenar slots existentes.
2. **NUNCA** eliminar un slot — si una función se depreca, el puntero apunta a un stub vacío.
3. Nuevas funciones se agregan **siempre al final** del array.
4. Los índices del enum `NairdaSlot` en `NairdaKernel_Proxy.cpp` son la fuente de verdad.

### Uso desde el espacio de usuario

```c
typedef void (*fn_ptr)(void);
#define JUMP_TABLE ((const fn_ptr *)0x0100)

// setupMotor (slot 4)
((void (*)(component_t*, int, int, int)) JUMP_TABLE[4])(comp, p1, p2, ps);

// readUltrasonic (slot 15)
uint8_t dist = ((uint8_t (*)(component_t*)) JUMP_TABLE[15])(sensor);

// nairdaDelay (slot 16)
((void (*)(unsigned long)) JUMP_TABLE[16])(1000);
```

---

## Flujo de Arranque

### Paso 1: Power ON / Reset físico

```
Arduino enciende o se resetea
│
└─→ Optiboot v8+ arranca automáticamente
    ├─ Escucha en hardware UART (pins 0/1) durante ~1 segundo
    ├─ ¿Recibe protocolo STK500v1? → Upload normal (Arduino IDE)
    └─ No recibe nada → JMP 0x0000 → Kernel arranca
```

### Paso 2: Kernel arranca

```
Kernel ejecuta nairdaBegin()
├─ Inicializa hardware (UART, BLE, pines)
├─ Inicializa la VM
└─ Entra en nairdaLoop()
```

### Paso 3: nairdaLoop() — bucle principal

```
nairdaLoop() corre permanentemente
│
├─ Procesa BLE, sensores, actuadores
│
└─ ¿Llegó un byte por BLE/UART?
   └─ ¿Es 100? → Reset por software (JMP 0x0000)
```

---

## Reset por Software (Comando 100)

Cuando el kernel recibe el byte `100` por BLE o UART, ejecuta un `JMP 0x0000` que reinicia el firmware. Tras el reinicio, el kernel entra en una **ventana de decisión de 2 segundos** donde espera el siguiente comando.

### Ventana de decisión (2 segundos)

```
JMP 0x0000 → Optiboot (no recibe STK500) → JMP 0x0000
│
│  Kernel arranca → nairdaBegin() → espera 2 segundos...
│
│  CASO A: Recibe 101 antes de 2s
│  └─ Modo Intérprete
│     Bytecode en tiempo real vía BLE (flujo actual existente)
│
│  CASO B: Recibe 150 antes de 2s
│  └─ Modo Bootloader
│     Recibe programa completo y lo escribe en Flash
│     (ver sección "Modo Bootloader")
│
│  CASO C: No recibe nada en 2 segundos
│  └─ Modo Ejecución Offline
│     (ver sección "Modo Ejecución Offline")
```

---

## Modo Bootloader (Comando 150)

### Principios de diseño

1. **Comunicación unidireccional**: El kernel solo recibe datos, nunca envía respuestas. Esto elimina problemas de lectura BLE desde la app móvil.
2. **Flag de integridad**: Un byte en `0x6000` indica si el programa almacenado es válido. Solo se marca como válido (`0x01`) cuando la transferencia completa termina exitosamente.
3. **do_spm del bootloader**: Optiboot v8+ expone una función `do_spm()` que permite escribir Flash desde la aplicación. Se llama como función C normal — no requiere hacks, timers ni manipulación de stack.
4. **Sin EEPROM**: Todo se procesa en RAM con un buffer de 128 bytes.

### Escritura de Flash vía do_spm

Optiboot v8+ expone un trampoline `do_spm` en la Boot Loader Section. Desde el código de aplicación se invoca como función C:

```c
typedef void (*do_spm_t)(uint16_t address, uint8_t command, uint16_t data);
#define DO_SPM ((do_spm_t)((FLASHEND - 511 + 2) >> 1))
// Para ATmega328P: (0x7FFF - 511 + 2) >> 1 = 0x3F01
```

Comandos SPM disponibles:

| Comando     | Valor  | Bits activos     | Descripción                    |
|------------|--------|------------------|--------------------------------|
| Page Erase | `0x03` | PGERS + SPMEN    | Borra una página de 128 bytes  |
| Page Fill  | `0x01` | SPMEN            | Escribe un word al temp buffer |
| Page Write | `0x05` | PGWRT + SPMEN    | Escribe temp buffer a Flash    |
| RWW Enable | `0x11` | RWWSRE + SPMEN   | Re-habilita lectura de Flash   |

**Nota**: `do_spm` ejecuta automáticamente RWW Enable después de page erase y page write. No es necesario llamarlo por separado.

### Secuencia para escribir una página completa

```c
void write_page(uint16_t page_addr, const uint8_t *data) {
    cli();

    // 1. Borrar la página
    DO_SPM(page_addr, 0x03, 0);

    // 2. Llenar el buffer temporal (64 words = 128 bytes)
    for (uint16_t i = 0; i < 128; i += 2) {
        uint16_t word = data[i] | ((uint16_t)data[i + 1] << 8);
        DO_SPM(page_addr + i, 0x01, word);
    }

    // 3. Escribir el buffer a Flash
    DO_SPM(page_addr, 0x05, 0);

    // 4. Re-habilitar lectura (por seguridad, do_spm ya lo hace)
    DO_SPM(0, 0x11, 0);

    sei();
}
```

**Tiempos**: Page erase ~4.5ms, page fill instantáneo, page write ~4.5ms. Escribir una página completa toma ~10ms. Escribir los 8KB del espacio de usuario (64 páginas) toma ~640ms.

### Protocolo de recepción de chunks

Cada chunk se transmite así:

```
┌─────────┬──────────┬─────────────────────────────┐
│ 1 byte  │ 1 byte   │ N bytes (máx 64)            │
│ len     │ checksum │ datos del chunk              │
└─────────┴──────────┴─────────────────────────────┘
```

- **len**: Cantidad de bytes de datos en este chunk. Nunca mayor a 64.
- **checksum**: Resultado de `(suma de todos los bytes de datos) % 64`.
- **datos**: Los bytes del programa del usuario.
- **Paquete terminador**: `len = 0`, `checksum = 0`. Indica fin de la transferencia.

### Validación de checksum

```
checksum_calculado = 0
por cada byte recibido en datos:
    checksum_calculado += byte
checksum_calculado = checksum_calculado % 64

¿checksum_calculado == checksum_recibido?
  SÍ → chunk válido, almacenar en buffer
  NO → chunk corrupto, descartar (la app reenviará por timeout)
```

Nota: Como la comunicación es unidireccional, el kernel no envía ACK/NACK. Si un chunk es corrupto, el kernel simplemente lo ignora. La app móvil implementa un timeout: si no detecta progreso, reinicia la transferencia completa desde el comando 100 → 150.

### Buffer RAM

```c
uint8_t page_buffer[128];       // Buffer de una página completa
uint8_t first_page_copy[128];   // Copia de la primera página (para reescribir el flag al final)
uint16_t current_addr = 0x6000; // Dirección actual de escritura en Flash
```

RAM total utilizada: **256 bytes** (de los 2048 disponibles).

### Flujo detallado del Modo Bootloader

```
Kernel recibe byte 150 → entra en Modo Bootloader
│
│  ┌─── PASO 1: Invalidar el flag ─────────────────────────┐
│  │                                                        │
│  │  El flag de integridad debe ser invalidado ANTES de    │
│  │  escribir cualquier dato. Sin embargo, no podemos      │
│  │  escribir solo 1 byte en Flash — la unidad mínima de   │
│  │  escritura es 1 página (128 bytes). Por lo tanto, el   │
│  │  flag se invalida cuando se escribe la primera página  │
│  │  con page_buffer[0] = 0x00.                            │
│  │                                                        │
│  │  current_addr = 0x6000                                 │
│  └────────────────────────────────────────────────────────┘
│
│  ┌─── PASO 2: Recibir y escribir página por página ──────┐
│  │                                                        │
│  │  LOOP {                                                │
│  │                                                        │
│  │    ── Chunk 1 (primera mitad de la página) ──          │
│  │    recibir 1 byte: len                                 │
│  │    recibir 1 byte: checksum                            │
│  │                                                        │
│  │    ¿len == 0 y checksum == 0?                          │
│  │      SÍ → ir a PASO 3 (fin de transferencia)           │
│  │      NO → continúa                                     │
│  │                                                        │
│  │    recibir len bytes → page_buffer[0..len-1]           │
│  │    si len < 64: rellenar page_buffer[len..63] con 0xFF │
│  │    validar checksum                                    │
│  │      FAIL → descartar chunk, esperar retransmisión     │
│  │      OK → continúa                                     │
│  │                                                        │
│  │    si es la primera página (current_addr == 0x6000):   │
│  │      page_buffer[0] = 0x00 (flag inválido forzado)     │
│  │      guardar copia: memcpy(first_page_copy,            │
│  │                             page_buffer, 64)           │
│  │                                                        │
│  │    ── Chunk 2 (segunda mitad de la página) ──          │
│  │    recibir 1 byte: len                                 │
│  │    recibir 1 byte: checksum                            │
│  │                                                        │
│  │    ¿len == 0 y checksum == 0?                          │
│  │      SÍ → rellenar page_buffer[64..127] con 0xFF      │
│  │           escribir página parcial (do_spm)             │
│  │           ir a PASO 3 (fin de transferencia)            │
│  │      NO → continúa                                     │
│  │                                                        │
│  │    recibir len bytes → page_buffer[64..64+len-1]       │
│  │    si len < 64: rellenar resto con 0xFF                │
│  │    validar checksum                                    │
│  │      FAIL → descartar chunk, esperar retransmisión     │
│  │      OK → continúa                                     │
│  │                                                        │
│  │    si es la primera página (current_addr == 0x6000):   │
│  │      guardar copia: memcpy(first_page_copy + 64,       │
│  │                             page_buffer + 64, 64)      │
│  │                                                        │
│  │    ── Escribir página con do_spm ──                    │
│  │    cli()                                               │
│  │    DO_SPM(current_addr, 0x03, 0)     // erase          │
│  │    for cada word:                                      │
│  │      DO_SPM(addr+i, 0x01, word)      // fill           │
│  │    DO_SPM(current_addr, 0x05, 0)     // write          │
│  │    sei()                                               │
│  │                                                        │
│  │    ── Verificar escritura ──                           │
│  │    leer página desde Flash con pgm_read_byte()         │
│  │    comparar byte a byte contra page_buffer[]           │
│  │      NO coincide → reintentar escritura de esta página │
│  │      SÍ coincide → continúa                            │
│  │                                                        │
│  │    current_addr += 128                                 │
│  │  }                                                     │
│  └────────────────────────────────────────────────────────┘
│
│  ┌─── PASO 3: Validar programa ─────────────────────────┐
│  │                                                        │
│  │  Transferencia completa. Ahora reescribir la primera   │
│  │  página con el flag de integridad activado.            │
│  │                                                        │
│  │  first_page_copy[0] = 0x01 (flag válido)              │
│  │                                                        │
│  │  Escribir first_page_copy en dirección 0x6000          │
│  │  usando do_spm (erase + fill + write)                  │
│  │                                                        │
│  │  Verificar con pgm_read_byte(0x6000) == 0x01           │
│  │    NO → reintentar escritura de primera página         │
│  │    SÍ → programa marcado como válido                   │
│  │                                                        │
│  │  JMP 0x0000 (reset por software)                       │
│  └────────────────────────────────────────────────────────┘
```

---

## Modo Ejecución Offline (Timeout 2 segundos)

Cuando pasan 2 segundos después del reset sin recibir ningún comando (ni 101 ni 150), el kernel verifica el flag de integridad y decide si ejecutar el programa del usuario.

```
Timeout de 2 segundos sin recibir comando
│
├─ Leer flag: pgm_read_byte(0x6000)
│
├─ flag == 0x01 (programa válido)
│  └─ JMP 0x6001 (inicio del programa del usuario)
│     │
│     │  El programa usa la Jump Table:
│     │  JUMP_TABLE[slot](argumentos...)
│     │
│     │  nairdaLoop() sigue ejecutándose para mantener
│     │  la comunicación BLE activa. Si llega el comando
│     │  100 por BLE → JMP 0x0000 (reset)
│     │
│     └─ El programa del usuario puede retornar al kernel
│        o correr indefinidamente en un loop propio
│
└─ flag != 0x01 (programa inválido o Flash vacía)
   └─ No ejecuta código de usuario
      nairdaLoop() corre normalmente
      Espera comandos por BLE/UART
      (Flash vacía = 0xFF, programa corrupto = 0x00)
```

---

## Resumen de los 3 Modos de Operación

| Secuencia         | Condición               | Modo              | Descripción                                           |
|-------------------|-------------------------|--------------------|-------------------------------------------------------|
| 100 → 101         | 101 llega antes de 2s   | Intérprete         | Bytecode en tiempo real vía BLE (flujo actual)        |
| 100 → 150         | 150 llega antes de 2s   | Bootloader         | Recibe chunks, escribe programa en Flash (do_spm)     |
| 100 → (silencio)  | Nada en 2 segundos      | Ejecución Offline  | Ejecuta programa almacenado si flag == 0x01           |

---

## Protocolo de Comunicación — Solo Escritura (App → Arduino)

La comunicación es **estrictamente unidireccional**: la app móvil envía datos, el Arduino los recibe. El Arduino nunca envía respuestas de ACK, NACK, ni confirmaciones.

### Formato de un chunk

```
┌─────────────────────┬────────────────────┬──────────────────────────────────┐
│ Byte 0: len         │ Byte 1: checksum   │ Bytes 2 a len+1: datos          │
│ (1-64, cant. datos) │ (suma datos % 64)  │ (bytes del programa del usuario) │
└─────────────────────┴────────────────────┴──────────────────────────────────┘
```

### Paquete terminador

```
┌──────────┬──────────┐
│ len = 0  │ check = 0│
└──────────┴──────────┘
```

### Manejo de errores sin lectura

Como el Arduino no responde, la app móvil debe implementar su propia lógica de confiabilidad:

- **Timeout por chunk**: La app espera un tiempo prudente entre chunks para asegurar que el Arduino tuvo tiempo de procesarlo (recepción + escritura Flash si aplica).
- **Reenvío por timeout**: Si la app detecta que la conexión BLE se interrumpió o que pasó demasiado tiempo, reinicia la transferencia completa desde el comando 100 → 150.
- **Sin respuestas parciales**: El Arduino nunca confirma chunks individuales. La garantía de integridad es el flag en 0x6000 — si la transferencia se interrumpió, el flag queda en 0x00 y el programa no se ejecuta.

### Flujo temporal de una transferencia completa

```
App Móvil                              Arduino (Kernel)
──────────                             ────────────────
envía: 100                      ───→   JMP 0x0000 (reset)
                                       kernel arranca, espera 2s...
envía: 150                      ───→   entra en Modo Bootloader
espera ~50ms
envía: [len1][chk1][datos...]   ───→   almacena en page_buffer[0..63]
espera ~50ms
envía: [len2][chk2][datos...]   ───→   almacena en page_buffer[64..127]
                                       do_spm escribe página (~10ms)
espera ~100ms
envía: [len3][chk3][datos...]   ───→   almacena en page_buffer[0..63]
espera ~50ms
envía: [len4][chk4][datos...]   ───→   almacena en page_buffer[64..127]
                                       do_spm escribe página (~10ms)
espera ~100ms
...
envía: [0][0]                   ───→   reescribe primera página con flag=0x01
                                       JMP 0x0000 (reset)
                                       kernel arranca, espera 2s...
                                       timeout → lee flag → 0x01
                                       JMP 0x6001 (ejecuta programa)
```

---

## Escenarios de Fallo y Seguridad

| Escenario                                   | Flag en 0x6000 | Resultado                              |
|---------------------------------------------|----------------|----------------------------------------|
| Transferencia completa exitosa              | 0x01           | Programa se ejecuta normalmente         |
| BLE se desconecta a mitad de transferencia  | 0x00           | No ejecuta, kernel espera comandos      |
| Arduino se apaga durante escritura          | 0x00 o 0xFF    | No ejecuta, kernel espera comandos      |
| Chunk corrupto (checksum falla)             | 0x00           | Kernel descarta chunk, espera más datos |
| do_spm falla en una página                  | 0x00           | Reintenta página; peor caso: no ejecuta |
| Flash vacía (nunca se subió programa)       | 0xFF           | No ejecuta, kernel espera comandos      |
| Programa anterior + nueva transferencia falla| 0x00          | Programa anterior se perdió, no ejecuta |

**Garantía fundamental**: El Arduino nunca se brickea. El kernel y el bootloader viven en zonas protegidas. Solo el espacio de usuario (0x6000+) puede quedar corrupto, y el flag de integridad impide que se ejecute código corrupto.

---

## Datos Técnicos del ATmega328P

| Parámetro            | Valor                          |
|----------------------|--------------------------------|
| Flash total          | 32KB (32768 bytes)             |
| Tamaño de página     | 128 bytes (64 words)           |
| SRAM                 | 2KB (2048 bytes)               |
| EEPROM               | 1KB (1024 bytes)               |
| Endurance Flash      | ~10,000 ciclos escritura/borrado por página |
| Bootloader (BLS)     | 512 bytes (0x7E00–0x7FFF)      |
| do_spm entry point   | Word 0x3F01 = Byte 0x7E02      |
| Lock bits (fábrica)  | 0x0F (BLB0 Mode 1, BLB1 Mode 3) |

### Lock Bits

| Bits       | Valor | Significado                                                       |
|-----------|-------|-------------------------------------------------------------------|
| LB[1:0]   | 11    | Sin restricción de lectura/escritura vía ISP                       |
| BLB0[1:0] | 11    | Sin restricción para SPM/LPM accediendo a la sección de aplicación |
| BLB1[1:0] | 00    | SPM no puede escribir en la BLS; LPM desde app no puede leer BLS  |

**Implicación**: SPM ejecutado desde la BLS puede escribir libremente en la sección de aplicación (0x0000–0x7DFF). El bootloader queda protegido contra escritura accidental.
