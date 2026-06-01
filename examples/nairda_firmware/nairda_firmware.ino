#include <nairda.h>

// Build variant select: pass `-DNAIRDA_KIDSY_BUILD` via
// `arduino-cli compile --build-property compiler.cpp.extra_flags=...` to
// produce the Kidsy binary (sets kit + custom BLE name). Without it, the
// build produces the generic ESP32 firmware.
void setup()
{
#if defined(ARDUINO_ARCH_ESP32)
  #ifdef NAIRDA_KIDSY_BUILD
    setKit(ROBBUS_KIDSY_KIT);
    nairdaBegin("NairdaKidsy", 9600);
  #else
    nairdaBegin("NairdaESP32", 9600);
  #endif
#else
    nairdaBegin(9600);
#endif
}

void loop()
{
    nairdaLoop();
#if defined(ARDUINO_ARCH_ESP32)
    // This path only runs when NO permanent user program is loaded — if there
    // is one, esp32ExecuteUserCode() never returns and Arduino loop() is never
    // reached. Without an explicit yield here, the main task spins tight and
    // could starve the BLE stack of CPU. delay(1) is the minimal vTaskDelay
    // tick that lets the BT controller/host tasks run on each loop iteration.
    delay(1);
#endif
}
