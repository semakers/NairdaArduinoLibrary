// ============================================================================
// bootjacker_test.ino
// Sketch de prueba para verificar que BootJacker puede escribir Flash
// en la dirección 0x6000 del ATmega328P usando el bootloader de fábrica.
//
// Prueba:
//   1. Lee el byte en 0x6000 y lo muestra por Serial ("Antes: XX")
//   2. Prepara una página de 128 bytes con el valor 0x05 en el byte 0
//   3. Escribe la página en 0x6000 usando BootJacker
//   4. Lee de nuevo y muestra el resultado ("Después: XX")
//
// Primera ejecución esperada:  Antes: FF  /  Después: 05
// Segunda ejecución esperada:  Antes: 05  /  Después: 05
// ============================================================================

#include "bootjacker.h"
#include <avr/pgmspace.h>

// Dirección base del espacio de usuario
#define USER_SPACE_ADDR 0x6000

void setup() {
    Serial.begin(115200);
    while (!Serial);

    Serial.println(F("=== BootJacker Test ==="));
    Serial.println();

    // Inicializar BootJacker
    bj_init();

    // 1. Leer valor actual en 0x6000
    uint8_t before = pgm_read_byte(USER_SPACE_ADDR);
    Serial.print(F("Antes:   0x"));
    if (before < 0x10) Serial.print('0');
    Serial.println(before, HEX);

    // 2. Preparar página de 128 bytes
    //    Byte 0 = 0x05 (nuestro valor de prueba)
    //    Bytes 1-127 = 0xFF (Flash "vacía")
    uint8_t page[128];
    memset(page, 0xFF, sizeof(page));
    page[0] = 0x06;

    // 3. Escribir con BootJacker
    Serial.print(F("Escribiendo en 0x"));
    Serial.print(USER_SPACE_ADDR, HEX);
    Serial.println(F("..."));
    Serial.flush(); // FORZANDO ENVIO PARA DEBUGGAR DONDE SE CONGELA

    uint8_t result = bj_write_page(USER_SPACE_ADDR, page);

    if (result == 0) {
        Serial.println(F("Escritura OK"));
    } else {
        Serial.println(F("ERROR: Escritura fallida"));
    }

    // 4. Leer de nuevo
    uint8_t after = pgm_read_byte(USER_SPACE_ADDR);
    Serial.print(F("Despues: 0x"));
    if (after < 0x10) Serial.print('0');
    Serial.println(after, HEX);

    Serial.println();

    // Mostrar los primeros 16 bytes de la página para diagnóstico
    Serial.println(F("Primeros 16 bytes en 0x6000:"));
    for (uint8_t i = 0; i < 16; i++) {
        uint8_t b = pgm_read_byte(USER_SPACE_ADDR + i);
        if (b < 0x10) Serial.print('0');
        Serial.print(b, HEX);
        Serial.print(' ');
    }
    Serial.println();

    Serial.println();
    Serial.println(F("=== Test completado ==="));
}

void loop() {
    // Nada — el test corre una sola vez en setup()
}
