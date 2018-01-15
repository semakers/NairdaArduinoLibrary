#include <nairda_serial2.h>

void setup() {
  // put your setup code here, to run once:
  nairdaBegin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  nairdaLoop();
}
