// User App - Blinker pin 13, 1 segundo

#include "nairda_user_esp32.h"

void _start() {
  nairda_init(13);

  while (1) {
    nairda_on(13);
    nairda_delay(1000);
    nairda_off(13);
    nairda_delay(1000);
  }
}
