#if defined(ARDUINO_ARCH_ESP32)

#include "extern_libraries/ssd1306/Adafruit_SSD1306.h"

#define JOYSTICK_UP 21
#define JOYSTICK_DOWN 26
#define FLOOR_1 33
#define FLOOR_2 35
#define FLOOR_3 34
#define FLOOR_4 39
#define FLOOR_5 36

uint8_t readFloorValue();
uint8_t readFloorNumValue();
void readFloorCalibration();
void calibrateZeegoFloorSensor(Adafruit_SSD1306 display);

#endif