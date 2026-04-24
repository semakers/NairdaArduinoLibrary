#if defined(ARDUINO_ARCH_ESP32)

#include "flash_writer/flash_writer.h"
#include <Arduino.h>
#include "nairda.h"
#include <string.h>
#include <setjmp.h>

static jmp_buf kernelJmpBuf;
static bool jmpBufValid = false;
static uint8_t* execMemRef = NULL;

static const esp_partition_t* userPartition = NULL;

void esp32FlashInit(void) {
    userPartition = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA,
        (esp_partition_subtype_t)0x40,
        USER_PARTITION_LABEL
    );
}

bool flashUserProgramValid(void) {
    if (!userPartition) return false;
    uint8_t flag;
    esp_partition_read(userPartition, 0, &flag, 1);
    return flag == USER_FLAG_VALID;
}

bool esp32FlashWriteUserProgram(uint8_t *dataBuffer, uint16_t totalBytes) {
    if (!userPartition || totalBytes == 0) return false;

    uint8_t entryOffset = dataBuffer[1];
    memmove(dataBuffer + USER_HEADER_SIZE, dataBuffer + 2, totalBytes - 2);
    uint16_t totalStored = totalBytes + 2;
    dataBuffer[0] = 0x00;
    dataBuffer[1] = totalStored & 0xFF;
    dataBuffer[2] = (totalStored >> 8) & 0xFF;
    dataBuffer[3] = entryOffset;

    esp_partition_erase_range(userPartition, 0, userPartition->size);
    esp_partition_write(userPartition, 0, dataBuffer, totalStored);

    dataBuffer[0] = USER_FLAG_VALID;
    esp_partition_erase_range(userPartition, 0, userPartition->size);
    esp_err_t err = esp_partition_write(userPartition, 0, dataBuffer, totalStored);

    return err == ESP_OK;
}

void esp32AbortUserCode(void) {
    if (jmpBufValid) {
        jmpBufValid = false;
        longjmp(kernelJmpBuf, 1);
    }
}

void esp32ExecuteUserCode(void) {
    if (!userPartition) return;

    uint8_t hdr[3];
    esp_partition_read(userPartition, 1, hdr, 3);
    uint16_t totalLen = hdr[0] | (hdr[1] << 8);
    uint8_t entryOffset = hdr[2];

    uint16_t codeLen = totalLen - USER_HEADER_SIZE;
    if (codeLen == 0 || codeLen > USER_SPACE_SIZE) return;

    uint16_t alignedLen = (codeLen + 3) & ~3;

    uint8_t* execMem = (uint8_t*)heap_caps_malloc(alignedLen, MALLOC_CAP_EXEC);
    if (!execMem) return;

    uint8_t* tmpBuf = (uint8_t*)malloc(alignedLen);
    if (!tmpBuf) {
        heap_caps_free(execMem);
        return;
    }

    memset(tmpBuf, 0, alignedLen);
    esp_partition_read(userPartition, USER_HEADER_SIZE, tmpBuf, codeLen);

    uint32_t* src = (uint32_t*)tmpBuf;
    uint32_t* dst = (uint32_t*)execMem;
    for (uint16_t i = 0; i < alignedLen / 4; i++) {
        dst[i] = src[i];
    }
    free(tmpBuf);

    execMemRef = execMem;
    jmpBufValid = true;
    if (setjmp(kernelJmpBuf) != 0) {
        heap_caps_free(execMemRef);
        execMemRef = NULL;
        jmpBufValid = false;
        return;
    }

    typedef void (*entry_fn)(void);
    entry_fn entry = (entry_fn)(execMem + entryOffset);
    entry();

    heap_caps_free(execMem);
    execMemRef = NULL;
    jmpBufValid = false;
}

// ── Jump Table Wrappers ────────────────────────────────────────────

static inline component_t* asComp(void *arr) {
    return (component_t*)arr;
}

static void jt_setupDigitalOut(void *arr, int pin) { component_t *c = asComp(arr); memset(c, 0, sizeof(component_t)); setupDigitalOut(c, pin); }
static void jt_runDigitalOut(void *arr, int value) { runDigitalOut(asComp(arr), value); }
static void jt_setupServo(void *arr, int pin, int minP, int maxP, int angle) { component_t *c = asComp(arr); memset(c, 0, sizeof(component_t)); setupServo(c, pin, minP, maxP, angle); }
static void jt_runServo(void *arr, int angle) { runServo(asComp(arr), angle); }
static void jt_setupMotor(void *arr, int p1, int p2, int ps) { component_t *c = asComp(arr); memset(c, 0, sizeof(component_t)); setupMotor(c, p1, p2, ps); }
static void jt_runMotor(void *arr, int speed, int dir) { runMotor(asComp(arr), speed, dir); }
static void jt_setupNeoPixel(void *arr, int pin, int num) { component_t *c = asComp(arr); memset(c, 0, sizeof(component_t)); setupNeoPixel(c, pin, num); }
static void jt_runNeoPixel(void *arr, int r, int g, int b, int idx) { runNeoPixel(asComp(arr), r, g, b, idx); }
static void jt_setupFrequency(void *arr, int pin) { component_t *c = asComp(arr); memset(c, 0, sizeof(component_t)); setupFrequency(c, pin); }
static void jt_runFrequency(void *arr, int freq, int dur, int vol) { runFrequency(asComp(arr), freq, dur, vol); }
static void jt_setupDigitalIn(void *arr, int pin) { component_t *c = asComp(arr); memset(c, 0, sizeof(component_t)); setupDigitalIn(c, pin); }
static uint8_t jt_readDigitalIn(void *arr) { return readDigitalIn(asComp(arr)); }
static void jt_setupAnalogic(void *arr, int pin) { component_t *c = asComp(arr); memset(c, 0, sizeof(component_t)); setupAnalogic(c, pin); }
static uint8_t jt_readAnalogic(void *arr) { return readAnalogic(asComp(arr)); }
static void jt_setupUltrasonic(void *arr, int trig, int echo) { component_t *c = asComp(arr); memset(c, 0, sizeof(component_t)); setupUltrasonic(c, trig, echo); }
static uint8_t jt_readUltrasonic(void *arr) { return readUltrasonic(asComp(arr)); }

void esp32SetupJumpTable(void) {
    ESP32_JUMP_TABLE_ADDR[0] = (void*)jt_setupDigitalOut;
    ESP32_JUMP_TABLE_ADDR[1] = (void*)jt_runDigitalOut;
    ESP32_JUMP_TABLE_ADDR[2] = (void*)jt_setupServo;
    ESP32_JUMP_TABLE_ADDR[3] = (void*)jt_runServo;
    ESP32_JUMP_TABLE_ADDR[4] = (void*)jt_setupMotor;
    ESP32_JUMP_TABLE_ADDR[5] = (void*)jt_runMotor;
    ESP32_JUMP_TABLE_ADDR[6] = (void*)jt_setupNeoPixel;
    ESP32_JUMP_TABLE_ADDR[7] = (void*)jt_runNeoPixel;
    ESP32_JUMP_TABLE_ADDR[8] = (void*)jt_setupFrequency;
    ESP32_JUMP_TABLE_ADDR[9] = (void*)jt_runFrequency;
    ESP32_JUMP_TABLE_ADDR[10] = (void*)jt_setupDigitalIn;
    ESP32_JUMP_TABLE_ADDR[11] = (void*)jt_readDigitalIn;
    ESP32_JUMP_TABLE_ADDR[12] = (void*)jt_setupAnalogic;
    ESP32_JUMP_TABLE_ADDR[13] = (void*)jt_readAnalogic;
    ESP32_JUMP_TABLE_ADDR[14] = (void*)jt_setupUltrasonic;
    ESP32_JUMP_TABLE_ADDR[15] = (void*)jt_readUltrasonic;
    ESP32_JUMP_TABLE_ADDR[16] = (void*)nairdaDelay;
    ESP32_JUMP_TABLE_ADDR[17] = (void*)(long(*)(long, long))random;
    ESP32_JUMP_TABLE_ADDR[18] = (void*)(long(*)(long, long, long, long, long))map;
}

#endif
