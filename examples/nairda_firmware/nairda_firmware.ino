#include <nairda.h>

void setup()
{
#if defined(ARDUINO_ARCH_ESP32)
    nairdaBegin("NairdaESP32", 9600);
#else
    nairdaBegin(9600);
#endif
}

void loop()
{
    nairdaLoop();
}
