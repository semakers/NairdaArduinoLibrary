#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328__)

#include "nairda.h"
#include <string.h>

static inline component_t* asComp(void *arr) {
    return (component_t*)arr;
}

extern "C" {

// ── Outputs ─────────────────────────────────────────────────────────────

__attribute__((used)) void jt_setupDigitalOut(void *arr, int pin) {
    component_t *c = asComp(arr);
    memset(c, 0, sizeof(component_t));
    setupDigitalOut(c, pin);
}
__attribute__((used)) void jt_runDigitalOut(void *arr, int value) {
    runDigitalOut(asComp(arr), value);
}

__attribute__((used)) void jt_setupServo(void *arr, int pin, int minPulse, int maxPulse, int initialAngle) {
    component_t *c = asComp(arr);
    memset(c, 0, sizeof(component_t));
    setupServo(c, pin, minPulse, maxPulse, initialAngle);
}
__attribute__((used)) void jt_runServo(void *arr, int angle) {
    runServo(asComp(arr), angle);
}

__attribute__((used)) void jt_setupMotor(void *arr, int pin1, int pin2, int pinSpeed) {
    component_t *c = asComp(arr);
    memset(c, 0, sizeof(component_t));
    setupMotor(c, pin1, pin2, pinSpeed);
}
__attribute__((used)) void jt_runMotor(void *arr, int speed, int direction) {
    runMotor(asComp(arr), speed, direction);
}

__attribute__((used)) void jt_setupNeoPixel(void *arr, int pin, int numPixels) {
    component_t *c = asComp(arr);
    memset(c, 0, sizeof(component_t));
    setupNeoPixel(c, pin, numPixels);
}
__attribute__((used)) void jt_runNeoPixel(void *arr, int r, int g, int b, int index) {
    runNeoPixel(asComp(arr), r, g, b, index);
}

__attribute__((used)) void jt_setupFrequency(void *arr, int pin) {
    component_t *c = asComp(arr);
    memset(c, 0, sizeof(component_t));
    setupFrequency(c, pin);
}
__attribute__((used)) void jt_runFrequency(void *arr, int frequency, int duration, int volume) {
    runFrequency(asComp(arr), frequency, duration, volume);
}

// ── Inputs ──────────────────────────────────────────────────────────────

__attribute__((used)) void jt_setupDigitalIn(void *arr, int pin) {
    component_t *c = asComp(arr);
    memset(c, 0, sizeof(component_t));
    setupDigitalIn(c, pin);
}
__attribute__((used)) uint8_t jt_readDigitalIn(void *arr) {
    return readDigitalIn(asComp(arr));
}

__attribute__((used)) void jt_setupAnalogic(void *arr, int pin) {
    component_t *c = asComp(arr);
    memset(c, 0, sizeof(component_t));
    setupAnalogic(c, pin);
}
__attribute__((used)) uint8_t jt_readAnalogic(void *arr) {
    return readAnalogic(asComp(arr));
}

__attribute__((used)) void jt_setupUltrasonic(void *arr, int triggerPin, int echoPin) {
    component_t *c = asComp(arr);
    memset(c, 0, sizeof(component_t));
    setupUltrasonic(c, triggerPin, echoPin);
}
__attribute__((used)) uint8_t jt_readUltrasonic(void *arr) {
    return readUltrasonic(asComp(arr));
}

// ── Strings ─────────────────────────────────────────────────────────────

__attribute__((used)) int jt_stringCompare(const char *a, const char *b) {
    return stringCompare(String(a), String(b));
}
__attribute__((used)) void jt_stringToLower(const char *src, char *dst, int maxLen) {
    String result = stringToLower(String(src));
    strncpy(dst, result.c_str(), maxLen - 1);
    dst[maxLen - 1] = '\0';
}
__attribute__((used)) void jt_stringToUpper(const char *src, char *dst, int maxLen) {
    String result = stringToUpper(String(src));
    strncpy(dst, result.c_str(), maxLen - 1);
    dst[maxLen - 1] = '\0';
}

// ── Tables ──────────────────────────────────────────────────────────────

__attribute__((used)) void jt_tableCreate(void *handle, int rows, int cols, int32_t defaultVal) {
    nairda_table_t *table = new nairda_table_t();
    tableCreate(table, rows, cols, defaultVal);
    *((nairda_table_t**)handle) = table;
}
__attribute__((used)) void jt_tableSet(void *handle, int row, int col, int32_t value) {
    nairda_table_t *table = *((nairda_table_t**)handle);
    if (table) tableSet(table, row, col, value);
}
__attribute__((used)) int32_t jt_tableGet(void *handle, int row, int col) {
    nairda_table_t *table = *((nairda_table_t**)handle);
    if (table) return tableGet(table, row, col);
    return 0;
}
__attribute__((used)) int jt_tableHeight(void *handle) {
    nairda_table_t *table = *((nairda_table_t**)handle);
    if (table) return tableHeight(table);
    return 0;
}
__attribute__((used)) int jt_tableWidth(void *handle) {
    nairda_table_t *table = *((nairda_table_t**)handle);
    if (table) return tableWidth(table);
    return 0;
}
__attribute__((used)) void jt_tableAddRow(void *handle) {
    nairda_table_t *table = *((nairda_table_t**)handle);
    if (table) tableAddRow(table);
}
__attribute__((used)) void jt_tableAddColumn(void *handle) {
    nairda_table_t *table = *((nairda_table_t**)handle);
    if (table) tableAddColumn(table);
}
__attribute__((used)) void jt_tableRemoveRow(void *handle) {
    nairda_table_t *table = *((nairda_table_t**)handle);
    if (table) tableRemoveRow(table);
}
__attribute__((used)) void jt_tableRemoveColumn(void *handle) {
    nairda_table_t *table = *((nairda_table_t**)handle);
    if (table) tableRemoveColumn(table);
}

// ── Utilidades ──────────────────────────────────────────────────────────

__attribute__((used)) void jt_nairdaDelay(unsigned long ms) {
    nairdaDelay(ms);
}
__attribute__((used)) long jt_nairdaRandom(long min, long max) {
    return random(min, max);
}
__attribute__((used)) long jt_nairdaMap(long value, long fromLow, long fromHigh, long toLow, long toHigh) {
    return map(value, fromLow, fromHigh, toLow, toHigh);
}

} // extern "C"

#endif
