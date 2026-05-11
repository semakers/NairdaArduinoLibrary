// Blink user app for the current master kernel — pin 13, 1 second period.
//
// Uses the 19-slot jump table (esp32SetupJumpTable in esp32_flash.cpp).
// setupDigitalOut/runDigitalOut take an opaque 16-byte component handle that
// the kernel fills in; user code never inspects its bytes.

#include "nairda_user_esp32.h"

void _start() {
    uint8_t led[NAIRDA_COMP_SIZE];
    nairda_setupDigitalOut(led, 13);

    while (1) {
        nairda_runDigitalOut(led, 100);
        nairda_delay(1000);
        nairda_runDigitalOut(led, 0);
        nairda_delay(1000);
    }
}
