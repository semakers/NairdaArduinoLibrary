// ESP32 Bootloader Test - Step 4: Protocolo chunked (compatible AVR)
//
// Protocolo (idéntico al AVR):
//   1. Recibe 150 → entra en modo bootloader
//   2. Recibe chunks: [len][checksum][data × len]
//      - len: 1-64 bytes
//      - checksum: sum(data) % 64
//      - Si checksum falla, chunk descartado
//   3. Recibe [0][0] → termina, escribe a flash, ejecuta
//
// Partición "userapp": últimos 8KB del flash virtual 2MB (0x1FE000)
//
// Jump Table (4 funciones):
//   [0] init(pin)  → pinMode(pin, OUTPUT)
//   [1] on(pin)    → digitalWrite(pin, HIGH)
//   [2] off(pin)   → digitalWrite(pin, LOW)
//   [3] delay(ms)  → delay(ms)

#include "esp_partition.h"
#include "esp_heap_caps.h"

#define CMD_BOOTLOADER 150
#define USER_PARTITION_LABEL "userapp"
#define BUFFER_SIZE 8192
#define CHUNK_MAX 64
#define LED_PIN 2
#define USER_FLAG_VALID 0x01
#define USER_FLAG_INVALID 0x00
// Header: byte[0]=flag, byte[1]=size_lo, byte[2]=size_hi, byte[3]=entry_offset, byte[4+]=code
#define USER_HEADER_SIZE 4
#define USER_PROGRAM_OFFSET USER_HEADER_SIZE

// ---- Jump Table Functions (kernel) ----

void jt_init(int pin) {
  pinMode(pin, OUTPUT);
}

void jt_on(int pin) {
  digitalWrite(pin, HIGH);
}

void jt_off(int pin) {
  digitalWrite(pin, LOW);
}

void jt_delay(int ms) {
  delay(ms);
}

// Jump Table en RTC slow memory (dirección fija 0x50000000)
// Equivalente a 0x5F80 en AVR
#define JUMP_TABLE_ADDR ((volatile void**)0x50000000)

void setupJumpTable() {
  JUMP_TABLE_ADDR[0] = (void*)jt_init;
  JUMP_TABLE_ADDR[1] = (void*)jt_on;
  JUMP_TABLE_ADDR[2] = (void*)jt_off;
  JUMP_TABLE_ADDR[3] = (void*)jt_delay;
}

// ---- Bootloader ----

const esp_partition_t* userPartition = NULL;

// Espera bloqueante por un byte del serial
bool nextByte(uint8_t* out) {
  unsigned long timeout = millis() + 5000;
  while (!Serial.available()) {
    if (millis() > timeout) return false;
  }
  *out = Serial.read();
  return true;
}

void dumpFlash(uint16_t len) {
  uint8_t readBuf[16];
  Serial.println("\n--- FLASH DUMP ---");
  Serial.print("Particion: ");
  Serial.print(USER_PARTITION_LABEL);
  Serial.print(" @ 0x");
  Serial.println(userPartition->address, HEX);
  Serial.print("Bytes escritos: ");
  Serial.println(len);
  Serial.println();

  for (uint16_t i = 0; i < len; i += 16) {
    uint16_t chunkSize = min((uint16_t)16, (uint16_t)(len - i));
    esp_partition_read(userPartition, i, readBuf, chunkSize);

    if (i < 0x1000) Serial.print("0");
    if (i < 0x100) Serial.print("0");
    if (i < 0x10) Serial.print("0");
    Serial.print(i, HEX);
    Serial.print(": ");

    for (uint16_t j = 0; j < chunkSize; j++) {
      if (readBuf[j] < 0x10) Serial.print("0");
      Serial.print(readBuf[j], HEX);
      Serial.print(" ");
    }

    Serial.print(" |");
    for (uint16_t j = 0; j < chunkSize; j++) {
      Serial.print((readBuf[j] >= 32 && readBuf[j] <= 126) ? (char)readBuf[j] : '.');
    }
    Serial.println("|");
  }
  Serial.println("--- FIN DUMP ---");
}

// Verifica si hay un programa válido en flash
bool userProgramValid() {
  uint8_t flag;
  esp_partition_read(userPartition, 0, &flag, 1);
  return flag == USER_FLAG_VALID;
}

// Escribe el flag de integridad en byte[0] de la partición
void writeFlag(uint8_t flag) {
  // Hay que leer el sector completo, modificar el flag, y reescribir
  // porque esp_partition_write no puede escribir sobre datos ya escritos
  // sin borrar primero. Pero como el flag va de 0x00→0x01 (bits de 0→1 en
  // flash NOR), podemos escribir directamente sin borrar.
  esp_partition_write(userPartition, 0, &flag, 1);
}

void executeUserCode(uint16_t totalLen, uint8_t entryOffset) {
  uint16_t codeLen = totalLen - USER_HEADER_SIZE;
  Serial.print(">> Codigo ejecutable: ");
  Serial.print(codeLen);
  Serial.println(" bytes");

  uint16_t alignedLen = (codeLen + 3) & ~3;

  uint8_t* execMem = (uint8_t*)heap_caps_malloc(alignedLen, MALLOC_CAP_EXEC);
  if (!execMem) {
    Serial.println(">> ERROR: No se pudo asignar memoria ejecutable");
    return;
  }

  Serial.print(">> Memoria ejecutable en: 0x");
  Serial.println((uint32_t)execMem, HEX);

  // Flash → DRAM → IRAM (IRAM no soporta acceso byte-a-byte)
  uint8_t* tmpBuf = (uint8_t*)malloc(alignedLen);
  if (!tmpBuf) {
    Serial.println(">> ERROR: No se pudo asignar buffer temporal");
    heap_caps_free(execMem);
    return;
  }

  memset(tmpBuf, 0, alignedLen);
  // Leer desde offset 4 (saltar header)
  esp_partition_read(userPartition, USER_HEADER_SIZE, tmpBuf, codeLen);

  uint32_t* src = (uint32_t*)tmpBuf;
  uint32_t* dst = (uint32_t*)execMem;
  for (uint16_t i = 0; i < alignedLen / 4; i++) {
    dst[i] = src[i];
  }
  free(tmpBuf);

  Serial.print(">> Entry offset: ");
  Serial.println(entryOffset);
  Serial.println(">> Ejecutando user code...");
  typedef void (*entry_fn)(void);
  entry_fn entry = (entry_fn)(execMem + entryOffset);
  entry();

  heap_caps_free(execMem);
  Serial.println(">> User code terminado");
}

// Protocolo chunked: [len][checksum][data × len]
// Idéntico al AVR enterBootloaderMode
void enterBootloaderMode() {
  uint8_t dataBuffer[BUFFER_SIZE];
  uint16_t totalBytes = 0;
  uint8_t chunkData[CHUNK_MAX];
  uint8_t byte_val;

  Serial.println(">> Bootloader mode activado");
  Serial.println(">> Esperando chunks [len][checksum][data]...");

  // Borrar partición al inicio
  esp_partition_erase_range(userPartition, 0, userPartition->size);

  while (true) {
    // 1. Leer len
    if (!nextByte(&byte_val)) {
      Serial.println(">> Timeout esperando len");
      return;
    }
    uint8_t len = byte_val;

    // 2. Leer checksum
    if (!nextByte(&byte_val)) {
      Serial.println(">> Timeout esperando checksum");
      return;
    }
    uint8_t expected_checksum = byte_val;

    // 3. Terminador [0][0]
    if (len == 0 && expected_checksum == 0) {
      Serial.println(">> Terminador recibido");
      break;
    }

    // 4. Validar len
    if (len > CHUNK_MAX) {
      Serial.print(">> Chunk invalido, len=");
      Serial.println(len);
      continue;
    }

    // 5. Recibir data bytes
    uint8_t checksum_calc = 0;
    for (uint8_t i = 0; i < len; i++) {
      if (!nextByte(&byte_val)) {
        Serial.println(">> Timeout recibiendo data");
        return;
      }
      chunkData[i] = byte_val;
      checksum_calc += byte_val;
    }
    checksum_calc = checksum_calc % 64;

    // 6. Validar checksum
    if (checksum_calc != expected_checksum) {
      Serial.print(">> Checksum FAIL: esperado=");
      Serial.print(expected_checksum);
      Serial.print(" calculado=");
      Serial.println(checksum_calc);
      continue; // Descartar chunk, no abortar
    }

    // 7. Acumular en buffer
    if (totalBytes + len <= BUFFER_SIZE) {
      memcpy(dataBuffer + totalBytes, chunkData, len);
      totalBytes += len;
      Serial.print(">> Chunk OK: len=");
      Serial.print(len);
      Serial.print(" total=");
      Serial.println(totalBytes);
    } else {
      Serial.println(">> ERROR: Buffer lleno");
      break;
    }
  }

  Serial.print(">> Total bytes recibidos: ");
  Serial.println(totalBytes);

  if (totalBytes > 0) {
    // Flutter envía [0x00, 0xFF, ...code]
    // El kernel reemplaza los primeros 3 bytes con su header:
    //   [0] = flag (0x00 = inválido)
    //   [1] = totalBytes & 0xFF  (size low)
    //   [2] = totalBytes >> 8    (size high)
    //   [3] = entry_offset (de Flutter byte[2])
    //   [4+] = code
    // Flutter envía [0x00, entry_offset, ...code]
    // Mover code 2 posiciones adelante (de offset 2 a offset 4)
    uint8_t entryOffset = dataBuffer[1]; // Flutter envía entry_offset en byte[1]
    memmove(dataBuffer + USER_HEADER_SIZE, dataBuffer + 2, totalBytes - 2);
    uint16_t totalStored = totalBytes + 2; // 2 bytes más por el header expandido
    dataBuffer[0] = USER_FLAG_INVALID;
    dataBuffer[1] = totalStored & 0xFF;
    dataBuffer[2] = (totalStored >> 8) & 0xFF;
    dataBuffer[3] = entryOffset;

    Serial.println(">> Escribiendo en flash (flag=0x00)...");
    esp_err_t err = esp_partition_write(userPartition, 0, dataBuffer, totalStored);

    if (err == ESP_OK) {
      Serial.println(">> Escritura OK, actualizando flag...");
      dataBuffer[0] = USER_FLAG_VALID;
      esp_partition_erase_range(userPartition, 0, userPartition->size);
      esp_partition_write(userPartition, 0, dataBuffer, totalStored);
      Serial.println(">> Flag actualizado a 0x01 (valido)");
      dumpFlash(totalStored);
      executeUserCode(totalStored, entryOffset);
    } else {
      Serial.print(">> Error de escritura: ");
      Serial.println(err);
    }
  }
}

void setup() {
  Serial.begin(9600);
  setupJumpTable();
  Serial.println();
  Serial.println("=== ESP32 Bootloader Test ===");

  userPartition = esp_partition_find_first(
    ESP_PARTITION_TYPE_DATA,
    (esp_partition_subtype_t)0x40,
    USER_PARTITION_LABEL
  );

  if (userPartition) {
    Serial.print("Particion: ");
    Serial.print(USER_PARTITION_LABEL);
    Serial.print(" @ 0x");
    Serial.print(userPartition->address, HEX);
    Serial.print(", size: ");
    Serial.println(userPartition->size);
  } else {
    Serial.println("ERROR: Particion userapp NO encontrada");
  }

  Serial.print("Jump Table @ 0x");
  Serial.println((uint32_t)JUMP_TABLE_ADDR, HEX);

  if (!userPartition) return;

  // Ventana de 2 segundos: escuchar serial antes de ejecutar programa guardado
  Serial.println("Esperando 4s... (enviar 150 para bootloader)");
  unsigned long start = millis();
  bool bootloaderRequested = false;

  while (millis() - start < 4000) {
    if (Serial.available()) {
      uint8_t b = Serial.read();
      if (b == CMD_BOOTLOADER) {
        bootloaderRequested = true;
        break;
      }
    }
  }

  if (bootloaderRequested) {
    enterBootloaderMode();
  } else if (userProgramValid()) {
    Serial.println(">> Programa valido encontrado, ejecutando...");
    // Leer header: byte[1,2]=size, byte[3]=entry_offset
    uint8_t hdr[3];
    esp_partition_read(userPartition, 1, hdr, 3);
    uint16_t totalLen = hdr[0] | (hdr[1] << 8);
    uint8_t entryOffset = hdr[2];
    Serial.print(">> Tamano: ");
    Serial.print(totalLen);
    Serial.print(", entry_offset: ");
    Serial.println(entryOffset);
    executeUserCode(totalLen, entryOffset);
  } else {
    Serial.println("No hay programa valido en flash");
  }

  Serial.println("Esperando comando 150...");
}

void loop() {
  if (!Serial.available()) return;

  uint8_t b = Serial.read();

  if (b == CMD_BOOTLOADER && userPartition) {
    enterBootloaderMode();
    Serial.println("Esperando comando 150...");
  }
}
